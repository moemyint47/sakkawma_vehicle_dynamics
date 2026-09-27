"""Force-based geometric load transfer with roll-centre migration and jacking.

After C. Rouelle (OptimumG), "Rolling about", Racecar Engineering, Dec 2021:
the tyre side force F_y of each wheel acts along the line contact patch -> instant
centre, so each wheel receives a vertical link force

    F_z,link,o = F_y,o * tan(theta_o)          (outer wheel, load increase)
    F_z,link,i = -F_y,i * tan(theta_i)         (inner wheel, load decrease)

theta = angle between the ground and the line CP -> IC (positive when the IC is
above the ground on the inboard side). Per axle (sprung-mass share of F_y):

    geometric LT (per wheel)  dF_g = (F_y,o tan th_o + F_y,i tan th_i) / 2
    jacking force (body up)   J    =  F_y,o tan th_o - F_y,i tan th_i

With th_o = th_i = atan(h_rc / (t/2)) this reduces exactly to dF_g = F_y h_rc / t.
The instant centres are taken from the front-view kinematics AT THE ROLLED POSE,
so roll-centre migration (height and lateral) enters the load transfer, and the
inner/outer side-force split comes from the tyre model. Roll angle, wheel loads,
tyre forces and the IC angles are iterated to a fixed point.

Effective roll-centre height used in the load-transfer equations:
    h_rc,eff = dF_g * t / (m_s,axle a_y) = t (r tan th_o + (1-r) tan th_i) / 2,
    r = F_y,o / (F_y,o + F_y,i)
"""
from __future__ import annotations

import json
import math
from functools import lru_cache

from . import kinematics2d as k2
from .derivation import Calc, NullCalc, fmt, paren
from .schemas import ModelIn, Hardpoints

DEG = math.pi / 180


@lru_cache(maxsize=4096)
def _tans_cached(hp_json: str, track: float, phi_deg: float):
    hp = Hardpoints.model_validate_json(hp_json)
    s = k2.solve_pose(hp, hp, track, phi_deg=phi_deg)
    if not s["ok"]:
        return float("nan"), float("nan"), None, None
    out = []
    for tag, sgn in (("o", 1.0), ("i", -1.0)):
        side = s["sides"][tag]
        cp = side["points"]["cp"]
        ic = side["ic"]
        if ic is None:  # parallel arms: direction of the lower arm
            p, q = side["points"]["lbj"], side["points"]["lca_in"]
            dy, dz = q[0] - p[0], q[1] - p[1]
        else:
            dy, dz = ic[0] - cp[0], ic[1] - cp[1]
        # inboard horizontal distance is -dy for the outer side, +dy for the inner side
        inb = -sgn * dy
        out.append(dz / inb if abs(inb) > 1e-9 else float("inf"))
    return out[0], out[1], s["sides"]["o"]["ic"], s["sides"]["i"]["ic"]


def ic_tangents(ax, phi_deg: float):
    """tan(theta_o), tan(theta_i) and the ICs (ground frame) at body roll phi (exact pose solve)."""
    return _tans_cached(ax.hardpoints.model_dump_json(), float(ax.track_mm), round(float(phi_deg), 6))


PHI_GRID = [x * 0.5 for x in range(-20, 21)]  # -10 .. 10 deg, cubic spline


@lru_cache(maxsize=256)
def _tan_table(hp_json: str, track: float):
    from scipy.interpolate import CubicSpline
    to, ti = [], []
    for p in PHI_GRID:
        a, b, _, _ = _tans_cached(hp_json, track, p)
        to.append(a); ti.append(b)
    import numpy as np
    to, ti = np.array(to), np.array(ti)
    ok = np.isfinite(to) & np.isfinite(ti)
    x = np.array(PHI_GRID)[ok]
    return CubicSpline(x, to[ok]), CubicSpline(x, ti[ok]), float(x.min()), float(x.max())


def tan_fast(ax, phi_deg: float):
    """Cubic-spline interpolation of tan(theta_o/i) over roll (0.25 deg grid) - used in iterations."""
    so, si, lo, hi = _tan_table(ax.hardpoints.model_dump_json(), float(ax.track_mm))
    p = min(max(phi_deg, lo), hi)
    return float(so(p)), float(si(p))


@lru_cache(maxsize=512)
def _static_rc(hp_json: str, track: float) -> float:
    hp = Hardpoints.model_validate_json(hp_json)
    s = k2.solve_pose(hp, hp, track)
    return s["rc_height_mm"] / 1000.0


def static_rc(ax) -> float:
    return _static_rc(ax.hardpoints.model_dump_json(), float(ax.track_mm))


def solve(m: ModelIn, ay_g: float | None, tol_mm: float = 1e-4, itmax: int = 40, h0: dict | None = None,
          exact: bool = True) -> dict:
    """Fixed point: roll -> wheel loads -> tyre force split -> IC angles -> h_rc,eff.

    exact=False uses the spline table of the IC angles (fast, for sweeps / limit search);
    exact=True solves the rolled pose directly in the final iterations.
    """
    from . import load_transfer
    from .cornering import solve_axle
    from . import tire as tiremod
    h = dict(h0) if h0 else {"f": static_rc(m.front), "r": static_rc(m.rear)}
    info = {"h": dict(h), "it": 0, "converged": True, "axles": {}}
    lt0 = load_transfer.compute(m, NullCalc(), ay_g=ay_g, hrc_override=h)
    if abs(lt0["ay"]) < 1e-9:
        info["lt"] = lt0
        for tag in "fr":
            info["axles"][tag] = dict(r=0.5, tan_o=float("nan"), tan_i=float("nan"), J=0.0, geo=0.0,
                                      Fyo=0.0, Fyi=0.0, phi_deg=0.0, ic_o=None, ic_i=None, fallback=False)
        return info
    for it in range(itmax):
        lt = load_transfer.compute(m, NullCalc(), ay_g=ay_g, hrc_override=h)
        ay, phi_deg = lt["ay"], lt["phi_deg"]
        if math.isnan(phi_deg):
            info["converged"] = False
            break
        new = {}
        for tag, ax in (("f", m.front), ("r", m.rear)):
            A = lt["axles"][tag]
            m_ax = lt["m_f"] if tag == "f" else lt["m_r"]
            ms_ax = m_ax - 2 * ax.unsprung_mass_kg
            alpha, _, _ = solve_axle(m_ax * ay, A["Fz_out"], A["Fz_in"], m.tire)
            fallback = math.isnan(alpha)
            if fallback:  # saturated axle: split in proportion to the wheel loads
                Fo_, Fi_ = max(A["Fz_out"], 0.0), max(A["Fz_in"], 0.0)
            else:
                Fo_, Fi_ = tiremod.fy(A["Fz_out"], alpha, m.tire), tiremod.fy(A["Fz_in"], alpha, m.tire)
            r = Fo_ / (Fo_ + Fi_) if (Fo_ + Fi_) > 0 else 0.5
            if exact:
                to, ti, ico, ici = ic_tangents(ax, phi_deg)
            else:
                (to, ti), ico, ici = tan_fast(ax, phi_deg), None, None
            t = ax.track_mm / 1000
            Fys = ms_ax * ay
            geo = Fys * (r * to + (1 - r) * ti) / 2
            J = Fys * (r * to - (1 - r) * ti)
            new[tag] = t * (r * to + (1 - r) * ti) / 2
            info["axles"][tag] = dict(r=r, tan_o=to, tan_i=ti, J=J, geo=geo, Fyo=r * Fys, Fyi=(1 - r) * Fys,
                                      phi_deg=phi_deg, ic_o=ico, ic_i=ici, fallback=fallback, alpha=alpha)
        dh = max(abs(new[k] - h[k]) for k in h) * 1000
        h = new
        info["it"] = it + 1
        if dh < tol_mm:
            break
    else:
        info["converged"] = False
    info["h"] = h
    info["lt"] = load_transfer.compute(m, NullCalc(), ay_g=ay_g, hrc_override=h)
    return info


def record(m: ModelIn, info: dict, calc: Calc) -> None:
    """Derivation of the force-based geometric load transfer (steady state)."""
    from .suspension import AxleSuspension
    S = "2b. Force-based geometric load transfer (IC angles, RC migration, jacking)"
    calc.add("geo.iter", r"n_{it}", "Fixed-point iterations (roll ↔ IC angles ↔ tyre split)",
             r"\text{until }|\Delta h_{rc,eff}|<10^{-4}\,mm", r"\text{converged}" if info["converged"] else r"\text{NOT converged}",
             info["it"], "-", S)
    if not info["converged"]:
        calc.warn("Force-based geometric LT did not converge - check kinematics range / roll stability.")
    g = m.maneuver.g
    for tag, ax, name in (("f", m.front, "Front"), ("r", m.rear, "Rear")):
        a = info["axles"][tag]
        if math.isnan(a["tan_o"]):
            continue
        t = ax.track_mm / 1000
        phi = a["phi_deg"]
        if a["ic_o"] is not None:
            calc.add(f"geo.{tag}.ic_o", rf"IC_o^{tag}", f"{name}: outer instant centre at φ = {phi:.3f}°",
                     r"\text{LCA}\cap\text{UCA (rolled pose, ground frame)}", rf"({fmt(a['ic_o'][0])},\ {fmt(a['ic_o'][1])})",
                     a["ic_o"][1], "mm", S, note="value = IC height; (y, z) in the substitution")
            calc.add(f"geo.{tag}.ic_i", rf"IC_i^{tag}", f"{name}: inner instant centre at φ = {phi:.3f}°",
                     r"\text{LCA}\cap\text{UCA (rolled pose, ground frame)}", rf"({fmt(a['ic_i'][0])},\ {fmt(a['ic_i'][1])})",
                     a["ic_i"][1], "mm", S)
        calc.add(f"geo.{tag}.tan_o", rf"\tan\theta_o^{tag}", f"{name}: outer CP→IC line slope",
                 r"\frac{z_{IC,o}-z_{CP,o}}{y_{CP,o}-y_{IC,o}}", fmt(a["tan_o"]), a["tan_o"], "-", S,
                 ref="OptimumG 'Rolling about' Fig. 6")
        calc.add(f"geo.{tag}.tan_i", rf"\tan\theta_i^{tag}", f"{name}: inner CP→IC line slope",
                 r"\frac{z_{IC,i}-z_{CP,i}}{y_{IC,i}-y_{CP,i}}", fmt(a["tan_i"]), a["tan_i"], "-", S)
        calc.add(f"geo.{tag}.r", rf"r^{tag}", f"{name}: outer share of axle side force",
                 r"\frac{F_{y,o}}{F_{y,o}+F_{y,i}}\ \text{(tyre model)}", fmt(a["r"]), a["r"], "-", S,
                 note="axle saturated: split by wheel load" if a["fallback"] else "from the TMeasy forces at the solved slip angle")
        calc.add(f"geo.{tag}.geo", rf"\Delta F_{{g}}^{tag}", f"{name}: geometric LT (force-based)",
                 r"\frac{F_{y,s}\left[r\tan\theta_o+(1-r)\tan\theta_i\right]}{2},\ F_{y,s}=m_{s,ax}a_y",
                 rf"\frac{{{fmt(a['Fyo'] + a['Fyi'])}\left[{fmt(a['r'])}\cdot{paren(a['tan_o'])}+{fmt(1 - a['r'])}\cdot{paren(a['tan_i'])}\right]}}{{2}}",
                 a["geo"], "N", S)
        calc.add(f"geo.{tag}.h_eff", rf"h_{{rc,eff}}^{tag}", f"{name}: effective roll-centre height (migrated)",
                 r"\frac{t}{2}\left[r\tan\theta_o+(1-r)\tan\theta_i\right]",
                 rf"\frac{{{fmt(t * 1000)}}}{{2}}\left[{fmt(a['r'])}\cdot{paren(a['tan_o'])}+{fmt(1 - a['r'])}\cdot{paren(a['tan_i'])}\right]",
                 info["h"][tag] * 1000, "mm", S, note=f"static kinematic RC = {static_rc(ax) * 1000:.2f} mm")
        J = calc.add(f"geo.{tag}.J", rf"J^{tag}", f"{name}: jacking force on the body (+ up)",
                     r"F_{y,o}\tan\theta_o-F_{y,i}\tan\theta_i",
                     rf"{fmt(a['Fyo'])}\cdot{paren(a['tan_o'])}-{fmt(a['Fyi'])}\cdot{paren(a['tan_i'])}", a["J"], "N", S,
                     ref="OptimumG 'Rolling about'")
        wf = m.vehicle.weight_front if tag == "f" else 1 - m.vehicle.weight_front
        su = AxleSuspension(ax, m.vehicle.mass_kg * wf - 2 * ax.unsprung_mass_kg, g)
        if su.direct:
            kw = su.K_direct / (1000 * t ** 2 / 2)
            src = r"K_\phi/(1000\,t^2/2)\ \text{(direct mode, incl. ARB)}"
        else:
            kw = su.wheel_rate(0.0)
            src = r"k_w(0)"
        calc.add(f"geo.{tag}.dz", rf"\Delta z_{{body}}^{tag}", f"{name}: ride-height change from jacking (+ up)",
                 rf"\frac{{J}}{{2\,k_w}},\ k_w={src}", rf"\frac{{{fmt(J)}}}{{2\cdot{fmt(kw)}}}", J / (2 * kw) if kw > 0 else float("nan"),
                 "mm", S, note="linear heave estimate; heave DOF not yet in the roll model")
