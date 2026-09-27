"""Adaptive parameter editing.

In *adaptive* mode an edit to one parameter moves the dependent parameters so
the design intent is kept. The functions return the new model, the list of
changed parameters and a derivation log of every adapted number.

Rules (per axle):
  h_rc_mm            -> inboard pivots (LCA_in, UCA_in) move vertically; ball
                        joints and front-view swing-arm length l_FVSA are kept.
                        The new instant centre lies on the line CP -> RC_target:
                            IC' = CP + s * l_FVSA * (RC - CP)/|RC - CP|
                        and each inboard pivot is put on the line BJ -> IC':
                            z_in' = z_BJ + (z_IC' - z_BJ) (y_in - y_BJ) / (y_IC' - y_BJ)
  tire radius, static camber
                     -> the upright (ball joints) moves rigidly with the wheel
                        centre frame; inboard pivots stay.
  roll_stiffness_Nm_deg (components mode)
                     -> spring rate solved for the target, ARB share kept:
                            k_tot = K / (1000 t^2/2),  k_ARB = share k_tot
                            k_s   = ((1-share) k_tot - F_s0 MR'(0)) / MR(0)^2
  arb_share          -> re-split at constant roll stiffness (same equations).
  always (sync)      -> h_rc_mm = kinematic static RC height;
                        roll_stiffness_Nm_deg = linearised roll stiffness (components);
                        arb_share = k_ARB / (k_w(0) + k_ARB).
"""
from __future__ import annotations

import math

import numpy as np

from . import kinematics2d as k2
from .derivation import Calc, fmt, paren
from .schemas import ModelIn
from .suspension import AxleSuspension
from .sweep import get_path, set_path

DEG = math.pi / 180


def _axle_ms(m: ModelIn, tag: str) -> float:
    V = m.vehicle
    if tag == "front":
        return V.mass_kg * V.weight_front - 2 * m.front.unsprung_mass_kg
    return V.mass_kg * (1 - V.weight_front) - 2 * m.rear.unsprung_mass_kg


def adapt_rc(d: dict, m: ModelIn, tag: str, h_target: float, calc: Calc) -> None:
    ax = getattr(m, tag)
    hp = ax.hardpoints
    S = f"Adapt {tag}: roll-centre height → {h_target:g} mm"
    s0 = k2.solve_pose(hp, hp, ax.track_mm)
    o = s0["sides"]["o"]
    cp = np.array(o["points"]["cp"])
    ic = o["ic"]
    if ic is None:
        raise ValueError("Parallel arms (instant centre at infinity): swing-arm length undefined - "
                         "edit hardpoints in absolute mode first")
    ic = np.array(ic)
    L = float(np.linalg.norm(ic - cp))
    rc_old = np.array([0.0, s0["rc_height_mm"]])
    side = 1.0 if np.dot(ic - cp, rc_old - cp) >= 0 else -1.0
    rc = np.array([0.0, h_target])
    u = (rc - cp) / np.linalg.norm(rc - cp)
    icn = cp + side * L * u
    calc.add(f"adapt.{tag}.L", r"l_{FVSA}", "Front-view swing-arm length (kept)",
             r"\left|IC-CP\right|", rf"\sqrt{{({fmt(ic[0])}-{fmt(cp[0])})^2+({fmt(ic[1])}-{fmt(cp[1])})^2}}",
             L, "mm", S)
    calc.add(f"adapt.{tag}.ic_y", r"y_{IC}'", "New instant centre y",
             r"y_{CP}+s\,l_{FVSA}\frac{y_{RC}-y_{CP}}{|RC-CP|}",
             rf"{fmt(cp[0])}+{fmt(side)}\cdot{fmt(L)}\cdot{fmt(u[0])}", icn[0], "mm", S)
    calc.add(f"adapt.{tag}.ic_z", r"z_{IC}'", "New instant centre z",
             r"z_{CP}+s\,l_{FVSA}\frac{z_{RC}-z_{CP}}{|RC-CP|}",
             rf"{fmt(cp[1])}+{fmt(side)}\cdot{fmt(L)}\cdot{paren(u[1])}", icn[1], "mm", S)
    for arm, bj in (("lca", hp.lca_out), ("uca", hp.uca_out)):
        pin = getattr(hp, f"{arm}_in")
        if abs(icn[0] - bj.y) < 1e-6:
            raise ValueError("Instant centre vertically above a ball joint - cannot place pivot")
        z_new = bj.z + (icn[1] - bj.z) * (pin.y - bj.y) / (icn[0] - bj.y)
        calc.add(f"adapt.{tag}.{arm}_in_z", rf"z_{{{arm.upper()},in}}'", f"New {arm.upper()} inboard pivot z",
                 r"z_{BJ}+(z_{IC}'-z_{BJ})\frac{y_{in}-y_{BJ}}{y_{IC}'-y_{BJ}}",
                 rf"{fmt(bj.z)}+({paren(icn[1])}-{fmt(bj.z)})\frac{{{fmt(pin.y)}-{fmt(bj.y)}}}{{{paren(icn[0])}-{fmt(bj.y)}}}",
                 z_new, "mm", S, note=f"was {pin.z:g} mm; y kept at {pin.y:g} mm")
        set_path(d, f"{tag}.hardpoints.{arm}_in.z", round(z_new, 4))


def adapt_wheel(d: dict, old: ModelIn, tag: str, calc: Calc) -> None:
    """Move the ball joints rigidly with the wheel-centre frame (tire radius / static camber edit)."""
    new = ModelIn.model_validate(d)
    ho, hn = getattr(old, tag).hardpoints, getattr(new, tag).hardpoints
    T = getattr(new, tag).track_mm / 2
    def frame(hp):
        g = hp.static_camber_deg * DEG
        u = np.array([math.sin(g), math.cos(g)])
        return np.array([T, 0.0]) + hp.tire_radius_mm * u, g
    wc0, g0 = frame(ho)
    wc1, g1 = frame(hn)
    dg = g1 - g0  # camber increase = rotation clockwise (top outward) in side coords
    R = np.array([[math.cos(-dg), -math.sin(-dg)], [math.sin(-dg), math.cos(-dg)]])
    S = f"Adapt {tag}: wheel centre moved"
    calc.add(f"adapt.{tag}.wc", r"\Delta WC", "Wheel-centre shift (y, z)",
             r"CP+r\,(\sin\gamma,\cos\gamma)\ \text{new}-\text{old}",
             rf"({fmt(wc1[0] - wc0[0])},\ {fmt(wc1[1] - wc0[1])})", float(np.linalg.norm(wc1 - wc0)), "mm", S)
    for bj in ("lca_out", "uca_out"):
        p = getattr(ho, bj)
        q = wc1 + R @ (np.array([p.y, p.z]) - wc0)
        calc.add(f"adapt.{tag}.{bj}", rf"P_{{{bj}}}'", f"{bj} moved with the upright",
                 r"WC'+R(-\Delta\gamma)\,(P-WC)", rf"({fmt(q[0])},\ {fmt(q[1])})", float(q[1]), "mm", S,
                 note=f"y {p.y:g} → {q[0]:.3f}, z {p.z:g} → {q[1]:.3f}")
        set_path(d, f"{tag}.hardpoints.{bj}.y", round(float(q[0]), 4))
        set_path(d, f"{tag}.hardpoints.{bj}.z", round(float(q[1]), 4))


def adapt_roll_stiffness(d: dict, m: ModelIn, tag: str, K_deg: float, share: float, calc: Calc) -> None:
    ax = getattr(m, tag)
    su = AxleSuspension(ax, _axle_ms(m, tag), m.maneuver.g)
    S = f"Adapt {tag}: roll stiffness → {K_deg:g} N·m/°, ARB share {share:g}"
    K = K_deg / DEG
    ktot = calc.add(f"adapt.{tag}.ktot", r"k_{tot}", "Required total wheel rate in roll",
                    r"\frac{K_\phi}{1000\,t^2/2}", rf"\frac{{{fmt(K)}}}{{1000\cdot{fmt(su.t)}^2/2}}",
                    K / (1000 * su.t ** 2 / 2), "N/mm", S)
    karb = calc.add(f"adapt.{tag}.karb", r"k_{ARB}", "ARB rate at the wheel", r"\eta_{ARB}\,k_{tot}",
                    rf"{fmt(share)}\cdot{fmt(ktot)}", share * ktot, "N/mm", S)
    dmr = su.mr.dmr(0.0)
    ks = calc.add(f"adapt.{tag}.ks", r"k_s", "Spring rate", r"\frac{(1-\eta_{ARB})k_{tot}-F_{s0}\,MR'(0)}{MR(0)^2}",
                  rf"\frac{{(1-{fmt(share)})\cdot{fmt(ktot)}-{fmt(su.Fs0)}\cdot{paren(dmr)}}}{{{fmt(su.mr0)}^2}}",
                  ((1 - share) * ktot - su.Fs0 * dmr) / su.mr0 ** 2, "N/mm", S)
    if ks <= 0:
        raise ValueError("Target roll stiffness too low for this MR progression / ARB share (spring rate ≤ 0)")
    set_path(d, f"{tag}.arb_rate_N_mm", round(karb, 6))
    set_path(d, f"{tag}.spring.rate_N_mm", round(ks, 6))


def sync(d: dict, calc: Calc | None = None) -> None:
    """Keep derived inputs consistent (adaptive mode)."""
    m = ModelIn.model_validate(d)
    for tag in ("front", "rear"):
        ax = getattr(m, tag)
        s0 = k2.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm)
        if s0["ok"] and not math.isnan(s0["rc_height_mm"]):
            set_path(d, f"{tag}.h_rc_mm", round(s0["rc_height_mm"], 4))
        if ax.roll_stiffness_source == "components":
            su = AxleSuspension(ax, _axle_ms(m, tag), m.maneuver.g)
            K = su.linear_roll_stiffness()
            set_path(d, f"{tag}.roll_stiffness_Nm_deg", round(K * DEG, 4))
            kw = su.wheel_rate(0.0)
            if kw + su.karb > 0:
                set_path(d, f"{tag}.arb_share", round(su.karb / (kw + su.karb), 6))


def diff(a, b, prefix=""):
    out = []
    if isinstance(a, dict):
        for k in a:
            out += diff(a[k], b.get(k) if isinstance(b, dict) else None, f"{prefix}.{k}" if prefix else k)
    elif a != b:
        out.append({"path": prefix, "old": a, "new": b})
    return out


def apply(model: ModelIn, path: str, value) -> tuple[ModelIn, list[dict], Calc]:
    calc = Calc()
    old = model
    d = model.model_dump()
    get_path(d, path)  # KeyError for unknown path
    if model.param_mode == "absolute" or path == "param_mode":
        set_path(d, path, value)
        if path == "param_mode" and value == "adaptive":
            sync(d)
        new = ModelIn.model_validate(d)
        return new, diff(old.model_dump(), new.model_dump()), calc
    parts = path.split(".")
    tag = parts[0] if parts[0] in ("front", "rear") else None
    if tag and parts[1:] == ["h_rc_mm"]:
        adapt_rc(d, model, tag, float(value), calc)
    elif tag and parts[1:] == ["roll_stiffness_Nm_deg"] and getattr(model, tag).roll_stiffness_source == "components":
        adapt_roll_stiffness(d, model, tag, float(value), getattr(model, tag).arb_share, calc)
    elif tag and parts[1:] == ["arb_share"]:
        adapt_roll_stiffness(d, model, tag, getattr(model, tag).roll_stiffness_Nm_deg, float(value), calc)
    elif tag and len(parts) == 3 and parts[1] == "hardpoints" and parts[2] in ("tire_radius_mm", "static_camber_deg"):
        set_path(d, path, value)
        adapt_wheel(d, model, tag, calc)
    else:
        set_path(d, path, value)
    ModelIn.model_validate(d)
    sync(d)
    new = ModelIn.model_validate(d)
    return new, diff(old.model_dump(), new.model_dump()), calc
