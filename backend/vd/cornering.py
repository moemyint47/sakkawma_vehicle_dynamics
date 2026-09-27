"""Steady-state cornering: axle lateral forces -> slip angles -> balance.

Assumptions (v1):
  * steady state (no yaw acceleration), lateral dynamics only (no drive/brake force, no aero)
  * both wheels of an axle run the same slip angle (no toe / Ackermann / compliance steer)
  * tire camber not included yet (enters once kinematics are coupled)
  * small-angle steer relation delta = l/R + alpha_f - alpha_r   (Rill eq. 9.14x, linear handling)

Required axle forces from moment equilibrium about each axle:
    F_y,f = m a_y * b / l = m_f a_y        F_y,r = m a_y * a / l = m_r a_y
"""
from __future__ import annotations

import math

from scipy.optimize import brentq

from . import load_transfer, tire as tiremod
from .derivation import Calc, NullCalc, fmt
from .schemas import ModelIn

DEG = math.pi / 180.0
ALPHA_SCAN_MAX = 30 * DEG


def axle_capacity(Fz_o, Fz_i, t):
    """Maximum of F_y,o(alpha)+F_y,i(alpha) and the slip angle where it occurs."""
    po, pi_ = tiremod.params_at(Fz_o, t), tiremod.params_at(Fz_i, t)
    G = lambda a: tiremod.force_from_slip(math.tan(a), po) + tiremod.force_from_slip(math.tan(a), pi_)
    n = 36
    best_a, best = 0.0, 0.0
    for k in range(1, n + 1):
        a = ALPHA_SCAN_MAX * k / n
        v = G(a)
        if v > best:
            best, best_a = v, a
    # golden-section refinement around the coarse maximum
    lo, hi = max(best_a - ALPHA_SCAN_MAX / n, 0.0), min(best_a + ALPHA_SCAN_MAX / n, ALPHA_SCAN_MAX)
    gr = (math.sqrt(5) - 1) / 2
    c, d = hi - gr * (hi - lo), lo + gr * (hi - lo)
    for _ in range(34):
        if G(c) > G(d):
            hi = d
        else:
            lo = c
        c, d = hi - gr * (hi - lo), lo + gr * (hi - lo)
    a_star = 0.5 * (lo + hi)
    return G(a_star), a_star, G


def solve_axle(F_req, Fz_o, Fz_i, t):
    Gmax, a_star, G = axle_capacity(Fz_o, Fz_i, t)
    if F_req <= 0:
        return 0.0, Gmax, a_star
    if F_req > Gmax:
        return float("nan"), Gmax, a_star
    a = brentq(lambda x: G(x) - F_req, 0.0, a_star, xtol=1e-10)
    return a, Gmax, a_star


def compute(m: ModelIn, calc: Calc | None = None, ay_g: float | None = None,
            with_gradient: bool = True) -> dict:
    calc = calc or NullCalc()
    geo = None
    if m.maneuver.geo_model == "ic_angles":
        from . import geo_coupling
        geo = geo_coupling.solve(m, ay_g)
        lt = load_transfer.compute(m, calc, ay_g=ay_g, hrc_override=geo["h"])
        if not isinstance(calc, NullCalc):
            geo_coupling.record(m, geo, calc)
    else:
        lt = load_transfer.compute(m, calc, ay_g=ay_g)
    t = m.tire
    ay = lt["ay"]
    if not isinstance(calc, NullCalc):
        tiremod.record_tire_inputs(t, calc)
    out = {"lt": lt, "axles": {}, "geo": geo}
    for tag, name in (("f", "Front"), ("r", "Rear")):
        S = f"6. {name} axle tires & slip angle"
        ax = lt["axles"][tag]
        m_ax = lt["m_f"] if tag == "f" else lt["m_r"]
        Freq = calc.add(f"{tag}.Fy_req", rf"F_{{y}}^{{{tag}}}", f"{name}: required axle lateral force",
                        rf"m_{tag}\,a_y", rf"{fmt(m_ax)}\cdot{fmt(ay)}", m_ax * ay, "N", S,
                        ref="steady state: moment balance about the other axle")
        alpha, Gmax, a_star = solve_axle(Freq, ax["Fz_out"], ax["Fz_in"], t)
        calc.add(f"{tag}.Fy_max", rf"F_{{y,\max}}^{{{tag}}}", f"{name}: axle lateral capacity",
                 r"\max_{\alpha}\left[F_y(F_{z,o},\alpha)+F_y(F_{z,i},\alpha)\right]",
                 rf"\text{{at }}\alpha={fmt(a_star / DEG)}^\circ", Gmax, "N", S,
                 note="numerical maximisation (grid + golden section)")
        util = calc.add(f"{tag}.util", rf"\mu_{{use}}^{{{tag}}}", f"{name}: grip utilisation",
                        rf"F_y^{{{tag}}}/F_{{y,\max}}^{{{tag}}}", rf"{fmt(Freq)}/{fmt(Gmax)}",
                        Freq / Gmax if Gmax > 0 else float("inf"), "-", S)
        if math.isnan(alpha):
            calc.warn(f"{name} axle saturated: required F_y exceeds capacity at this a_y.")
            calc.add(f"{tag}.alpha", rf"\alpha_{tag}", f"{name}: slip angle", r"\text{no solution}",
                     r"F_y > F_{y,\max}", float("nan"), "deg", S)
            out["axles"][tag] = dict(alpha_deg=float("nan"), Fy_req=Freq, Fy_max=Gmax, util=util,
                                     Fy_out=float("nan"), Fy_in=float("nan"))
            continue
        calc.add(f"{tag}.alpha", rf"\alpha_{tag}", f"{name}: slip angle (solved)",
                 r"\text{root of } F_y(F_{z,o},\alpha)+F_y(F_{z,i},\alpha)-F_y^{" + tag + "}=0",
                 r"\text{Brent's method on }[0,\ " + fmt(a_star / DEG) + r"^\circ]",
                 alpha / DEG, "deg", S)
        Fo = tiremod.fy_derivation(ax["Fz_out"], alpha, t, calc, f"{tag}.tire_o", f"{name} outer", S)
        Fi = tiremod.fy_derivation(ax["Fz_in"], alpha, t, calc, f"{tag}.tire_i", f"{name} inner", S)
        calc.add(f"{tag}.Fy_check", rf"F_{{y,o}}+F_{{y,i}}", f"{name}: check sum of tire forces",
                 rf"F_{{y,o}}^{{{tag}}}+F_{{y,i}}^{{{tag}}}", rf"{fmt(Fo)}+{fmt(Fi)}", Fo + Fi, "N", S,
                 note=f"must equal F_y^{tag} = {Freq:.2f} N")
        # load-sensitivity penalty: capacity lost by load transfer at this slip angle
        Fz0 = ax["Fz_static"]
        F_noLT = 2 * tiremod.fy(Fz0, alpha, t)
        calc.add(f"{tag}.lt_loss", rf"\Delta F_{{y,LT}}^{{{tag}}}", f"{name}: force lost to load transfer at α",
                 rf"2F_y(F_{{z,0}},\alpha)-\left(F_{{y,o}}+F_{{y,i}}\right)",
                 rf"2\cdot{fmt(F_noLT / 2)}-{fmt(Fo + Fi)}", F_noLT - (Fo + Fi), "N", S,
                 note="degressive tire load sensitivity: why more load transfer on an axle reduces its grip")
        out["axles"][tag] = dict(alpha_deg=alpha / DEG, Fy_req=Freq, Fy_max=Gmax, util=util,
                                 Fy_out=Fo, Fy_in=Fi, lt_loss=F_noLT - (Fo + Fi))

    S7 = "7. Balance"
    af, ar = out["axles"]["f"]["alpha_deg"], out["axles"]["r"]["alpha_deg"]
    l, R = lt["l"], m.maneuver.radius_m
    da = calc.add("dalpha", r"\Delta\alpha", "Slip-angle difference (understeer > 0)", r"\alpha_f-\alpha_r",
                  rf"{fmt(af)}-{fmt(ar)}", af - ar, "deg", S7)
    ack = calc.add("delta_ack", r"\delta_A", "Ackermann steer angle", r"\frac{l}{R}\cdot\frac{180}{\pi}",
                   rf"\frac{{{fmt(l)}}}{{{fmt(R)}}}\cdot\frac{{180}}{{\pi}}", l / R / DEG, "deg", S7)
    delta = calc.add("delta", r"\delta", "Required mean front steer angle", r"\delta_A+\alpha_f-\alpha_r",
                     rf"{fmt(ack)}+{fmt(da)}", ack + da, "deg", S7, note="small-angle bicycle-model relation")
    ay_g_used = ay / m.maneuver.g
    v = calc.add("v", r"v", "Speed on radius R", r"\sqrt{a_y R}", rf"\sqrt{{{fmt(ay)}\cdot{fmt(R)}}}",
                 math.sqrt(max(ay, 0) * R), "m/s", S7)
    calc.add("v_kmh", r"v", "Speed on radius R", r"3.6\,v", rf"3.6\cdot{fmt(v)}", 3.6 * v, "km/h", S7)
    out.update(dalpha_deg=da, delta_deg=delta, delta_ack_deg=ack, v=v)

    if with_gradient and not math.isnan(da):
        h_ = 0.01
        lo, hi = max(ay_g_used - h_, 0.0), ay_g_used + h_
        d_hi = compute(m, NullCalc(), ay_g=hi, with_gradient=False)["dalpha_deg"]
        d_lo = compute(m, NullCalc(), ay_g=lo, with_gradient=False)["dalpha_deg"]
        if not (math.isnan(d_hi) or math.isnan(d_lo)):
            K = calc.add("K_us", r"K_{us}", "Understeer gradient (local)",
                         r"\frac{\Delta\alpha(a_y+h)-\Delta\alpha(a_y-h)}{2h}",
                         rf"\frac{{{fmt(d_hi)}-{paren_(d_lo)}}}{{{fmt(hi - lo)}}}", (d_hi - d_lo) / (hi - lo),
                         "deg/g", S7, note="central finite difference, h = 0.01 g; >0 understeer, <0 oversteer")
            out["K_us"] = K
    return out


def paren_(x):
    s = fmt(x)
    return f"({s})" if x < 0 else s


_WARM = {}


def _feasible(m: ModelIn, ay_g: float):
    """Return (ok, reason) - ok if both axles can generate required force without wheel lift."""
    if m.maneuver.geo_model == "ic_angles":
        from . import geo_coupling
        key = id(m)
        info = geo_coupling.solve(m, ay_g, exact=False, tol_mm=0.05, h0=_WARM.get(key))
        _WARM.clear(); _WARM[key] = info["h"]  # warm start for the next bisection step
        lt = info["lt"]
    else:
        lt = load_transfer.compute(m, NullCalc(), ay_g=ay_g)
    if math.isnan(lt["phi"]):
        return False, "roll instability"
    for tag, name in (("f", "front"), ("r", "rear")):
        if lt["axles"][tag]["Fz_in"] < 0:
            return False, f"{name} inner wheel lift"
    for tag, name, us in (("f", "front", "limit understeer"), ("r", "rear", "limit oversteer")):
        ax = lt["axles"][tag]
        m_ax = lt["m_f"] if tag == "f" else lt["m_r"]
        Gmax, _, _ = axle_capacity(ax["Fz_out"], ax["Fz_in"], m.tire)
        if m_ax * lt["ay"] > Gmax:
            return False, f"{name} axle saturation ({us})"
    return True, ""


def limit_ay(m: ModelIn, calc: Calc | None = None, ay_hi: float = 4.0) -> dict:
    """Maximum steady-state a_y (bisection) and the mechanism that limits it."""
    calc = calc or NullCalc()
    lo, hi = 0.0, ay_hi
    ok, reason = _feasible(m, hi)
    if ok:
        return {"ay_max_g": hi, "reason": f"no limit found below {ay_hi} g"}
    for _ in range(32):
        mid = 0.5 * (lo + hi)
        ok, r = _feasible(m, mid)
        if ok:
            lo = mid
        else:
            hi, reason = mid, r
    S = "8. Cornering limit"
    ay_max = calc.add("ay_max", r"a_{y,\max}", "Maximum steady-state lateral acceleration",
                      r"\max a_y\ \text{s.t.}\ F_y^{f}\le F_{y,\max}^{f},\ F_y^{r}\le F_{y,\max}^{r},\ F_{z,i}\ge0",
                      r"\text{bisection on }a_y\in[0," + fmt(ay_hi) + r"]\,g", lo, "g", S,
                      note=f"limited by: {reason}")
    R, g = m.maneuver.radius_m, m.maneuver.g
    v = calc.add("v_max", r"v_{\max}", "Max speed on radius R", r"\sqrt{a_{y,\max} g R}",
                 rf"\sqrt{{{fmt(ay_max)}\cdot{fmt(g)}\cdot{fmt(R)}}}", math.sqrt(ay_max * g * R), "m/s", S)
    calc.add("v_max_kmh", r"v_{\max}", "Max speed on radius R", r"3.6\,v_{\max}", rf"3.6\cdot{fmt(v)}",
             3.6 * v, "km/h", S)
    # skidpad-style lap time on that radius (one circle)
    calc.add("lap_time", r"T_{lap}", "Time for one full circle of radius R", r"\frac{2\pi R}{v_{\max}}",
             rf"\frac{{2\pi\cdot{fmt(R)}}}{{{fmt(v)}}}", 2 * math.pi * R / v if v > 0 else float("nan"), "s", S)
    return {"ay_max_g": ay_max, "reason": reason, "v_max": v}
