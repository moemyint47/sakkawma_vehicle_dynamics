"""Corner replay: the car follows the road path exactly (path / "track" replay).

At every instant, from the road (curvature kappa(s), speed v(s)) and the roll model:

  kinematics      r = v kappa,   a_y = v^2 kappa,   a_x = dv/dt,   r' = dr/dt
  axle forces     F_yf + F_yr = m a_y                       (lateral equilibrium)
                  a F_yf - b F_yr = I_z r'                   (yaw equilibrium)
               -> F_yf = (m a_y b + I_z r') / l,   F_yr = (m a_y a - I_z r') / l
  wheel loads     F_z = F_z,static  +/- dF_z,lat(t)  -/+ m a_x h / (2 l)
                  dF_z,lat from the transient roll model (geometric + springs + ARB + dampers + unsprung)
  tyres           both wheels of an axle at one slip angle: F_y(F_z,L, alpha) + F_y(F_z,R, alpha) = F_y,axle
  sideslip/steer  beta = b r / v - alpha_r,     delta = alpha_f - alpha_r + l kappa     (small angles)
  balance         d_alpha = sgn(a_y) (alpha_f - alpha_r): > +band understeer, < -band oversteer
                  utilisation mu = |F_y,axle| / F_y,max(axle); an axle above 1 is sliding
Sign convention: x forward, y left, kappa > 0 = left-hand turn, a_y > 0 to the left.
The drive/brake forces are not included in the lateral/yaw balance (no F_x delta terms).
"""
from __future__ import annotations

import math

import numpy as np

from . import tire as tiremod
from .cornering import axle_capacity, solve_axle
from .derivation import Calc, NullCalc, fmt, paren
from .schemas import ModelIn, effective_ay_g
from .transient import road_profile, ay_profile, simulate

DEG = math.pi / 180


def _plan(m: ModelIn, tt: np.ndarray, ayf) -> dict:
    """Path quantities on the time grid tt."""
    T, g = m.maneuver.transient, m.maneuver.g
    if T.profile == "road":
        _, P = road_profile(m)
        pt = P["t"]
        f = lambda k: np.interp(tt, pt, P[k])
        return {"x": f("x"), "y": f("y"), "psi": f("psi"), "v": f("v"), "kappa": f("kappa"), "s": f("s"),
                "road": {"x": P["x"][::max(1, len(P["x"]) // 1500)].tolist(), "y": P["y"][::max(1, len(P["y"]) // 1500)].tolist(),
                         "edges": P["edges"].tolist()}}
    # other a_y(t) profiles: constant speed, curvature from a_y
    v0 = m.maneuver.speed_kmh / 3.6
    v = np.full_like(tt, v0)
    kap = np.array([ayf(t) * g for t in tt]) / v0 ** 2
    dt = np.diff(tt, prepend=tt[0])
    psi = np.cumsum(v * kap * dt)
    x = np.cumsum(v * np.cos(psi) * dt)
    y = np.cumsum(v * np.sin(psi) * dt)
    return {"x": x, "y": y, "psi": psi, "v": v, "kappa": kap, "s": np.cumsum(v * dt),
            "road": {"x": x.tolist(), "y": y.tolist(), "edges": []}}


def _axle(F, FzL, FzR, t):
    """Slip angle (signed, rad), per-wheel F_y, capacity and utilisation for one axle."""
    Fo, Fi = max(FzL, FzR), min(FzL, FzR)
    a, cap, _ = solve_axle(abs(F), Fo, Fi, t)
    if F == 0:
        a = 0.0
    sat = math.isnan(a)
    if sat:
        cap, a_star, _ = axle_capacity(Fo, Fi, t)
        a = a_star  # sliding: tyres at the peak of the combined characteristic
    a = math.copysign(a, F) if F != 0 else 0.0
    FyL, FyR = tiremod.fy(FzL, a, t), tiremod.fy(FzR, a, t)
    return a, FyL, FyR, cap, (abs(F) / cap if cap > 0 else float("inf")), sat


def run(m: ModelIn, dt: float = 0.01, band_deg: float = 0.1, calc: Calc | None = None) -> dict:
    calc = calc or NullCalc()
    V, M = m.vehicle, m.maneuver
    g = M.g
    tr = simulate(m)  # roll model: lateral LT per axle vs time
    t_tr = np.array(tr["t"])
    T_end = t_tr[-1]
    tt = np.arange(0.0, T_end + 1e-9, dt)
    ayf = ay_profile(m)
    P = _plan(m, tt, ayf)
    mt, wf, l, h = V.mass_kg, V.weight_front, V.wheelbase_mm / 1000, V.h_cg_mm / 1000
    a, b = l * (1 - wf), l * wf  # CG to front / rear axle
    Iz = V.yaw_inertia_kgm2
    v, kap = P["v"], P["kappa"]
    r = v * kap
    ay = v * v * kap
    ax = np.gradient(v, tt) if len(tt) > 2 else np.zeros_like(tt)
    rdot = np.gradient(r, tt) if len(tt) > 2 else np.zeros_like(tt)
    Fyf = (mt * ay * b + Iz * rdot) / l
    Fyr = (mt * ay * a - Iz * rdot) / l
    lat = {k: np.interp(tt, t_tr, tr["axles"][k]["total"]) for k in "fr"}
    phi = np.interp(tt, t_tr, tr["phi_deg"])
    Fz0 = {"f": mt * wf * g / 2, "r": mt * (1 - wf) * g / 2}
    dlong = mt * ax * h / (2 * l)  # per wheel, + = rear gains when accelerating
    # a_y > 0 (left turn): right wheels are outside -> +dF on the right
    Fz = {"FL": Fz0["f"] - lat["f"] - dlong, "FR": Fz0["f"] + lat["f"] - dlong,
          "RL": Fz0["r"] - lat["r"] + dlong, "RR": Fz0["r"] + lat["r"] + dlong}
    out = {k: [] for k in ("alpha_f", "alpha_r", "beta", "delta", "dalpha", "util_f", "util_r", "cap_f", "cap_r",
                           "FyFL", "FyFR", "FyRL", "FyRR", "status")}
    for i in range(len(tt)):
        af, fl, fr_, capf, uf, satf = _axle(Fyf[i], Fz["FL"][i], Fz["FR"][i], m.tire)
        ar, rl, rr, capr, ur, satr = _axle(Fyr[i], Fz["RL"][i], Fz["RR"][i], m.tire)
        vv = max(v[i], 0.1)
        beta = b * r[i] / vv - ar
        delta = af - ar + l * kap[i]
        sg = 1.0 if ay[i] >= 0 else -1.0
        da = sg * (af - ar) / DEG
        if abs(ay[i]) < 0.05 * g and abs(Iz * rdot[i]) < 50:
            st = "straight"
        elif satf and satr:
            st = "both sliding"
        elif satf:
            st = "front sliding (understeer)"
        elif satr:
            st = "rear sliding (oversteer)"
        elif min(Fz[k][i] for k in Fz) < 0:
            st = "wheel lift"
        elif da > band_deg:
            st = "understeer"
        elif da < -band_deg:
            st = "oversteer"
        else:
            st = "neutral"
        for k, val in (("alpha_f", af / DEG), ("alpha_r", ar / DEG), ("beta", beta / DEG), ("delta", delta / DEG),
                       ("dalpha", da), ("util_f", uf), ("util_r", ur), ("cap_f", capf), ("cap_r", capr),
                       ("FyFL", fl), ("FyFR", fr_), ("FyRL", rl), ("FyRR", rr), ("status", st)):
            out[k].append(val)
    res = {
        "t": tt.tolist(), "x": P["x"].tolist(), "y": P["y"].tolist(), "psi_deg": (P["psi"] / DEG).tolist(),
        "s": P["s"].tolist(), "v_kmh": (v * 3.6).tolist(), "kappa": kap.tolist(),
        "ay_g": (ay / g).tolist(), "ax_g": (ax / g).tolist(), "r_deg_s": (r / DEG).tolist(), "rdot_deg_s2": (rdot / DEG).tolist(),
        "Fyf": Fyf.tolist(), "Fyr": Fyr.tolist(), "Mz_yaw": (Iz * rdot).tolist(), "phi_deg": phi.tolist(),
        "Fz": {k: v_.tolist() for k, v_ in Fz.items()}, "road": P["road"],
        "geom": {"l": l, "a": a, "b": b, "tf": m.front.track_mm / 1000, "tr": m.rear.track_mm / 1000,
                 "Fz0f": Fz0["f"], "Fz0r": Fz0["r"], "band_deg": band_deg},
        **out,
    }
    res["summary"] = summary(res)
    probe(m, res, calc)
    return res


def summary(res) -> dict:
    st = res["status"]
    t = res["t"]
    n = len(t)
    dt = t[1] - t[0] if n > 1 else 0
    counts = {}
    for s in st:
        counts[s] = counts.get(s, 0) + dt
    uf, ur = np.array(res["util_f"]), np.array(res["util_r"])
    return {"time_in_state_s": counts, "max_util_f": float(np.nanmax(uf)), "max_util_r": float(np.nanmax(ur)),
            "max_dalpha": float(np.nanmax(res["dalpha"])), "min_dalpha": float(np.nanmin(res["dalpha"])),
            "min_wheel_load": float(min(min(v) for v in res["Fz"].values()))}


def probe(m: ModelIn, res: dict, calc: Calc) -> None:
    if isinstance(calc, NullCalc):
        return
    tq = m.maneuver.transient.t_probe_s
    t = np.array(res["t"])
    i = int(np.argmin(np.abs(t - tq)))
    V, g = m.vehicle, m.maneuver.g
    G = res["geom"]
    S = f"13. Corner replay at t = {t[i]:.3f} s"
    v = calc.add("rp.v", "v", "Speed on the path", r"v(s(t))", rf"{fmt(res['v_kmh'][i])}/3.6", res["v_kmh"][i] / 3.6, "m/s", S)
    k = calc.add("rp.kappa", r"\kappa", "Path curvature (+ left)", r"\kappa(s(t))", fmt(res["kappa"][i]), res["kappa"][i], "1/m", S,
                 note="clothoid transitions between road segments")
    r = calc.add("rp.r", "r", "Yaw rate", r"v\,\kappa", rf"{fmt(v)}\cdot{paren(k)}", v * k, "rad/s", S)
    ay = calc.add("rp.ay", "a_y", "Lateral acceleration", r"v^2\kappa", rf"{fmt(v)}^2\cdot{paren(k)}", v * v * k, "m/s²", S)
    ax = calc.add("rp.ax", "a_x", "Longitudinal acceleration", r"\frac{dv}{dt}\ \text{(central difference)}", fmt(res["ax_g"][i] * g),
                  res["ax_g"][i] * g, "m/s²", S)
    rd = calc.add("rp.rdot", r"\dot r", "Yaw acceleration", r"\frac{dr}{dt}\ \text{(central difference)}", fmt(res["rdot_deg_s2"][i] * DEG),
                  res["rdot_deg_s2"][i] * DEG, "rad/s²", S)
    mt, l, a, b, Iz = V.mass_kg, G["l"], G["a"], G["b"], V.yaw_inertia_kgm2
    calc.add("rp.Fyf", r"F_{y,f}", "Front axle lateral force", r"\frac{m a_y b + I_z \dot r}{l}",
             rf"\frac{{{fmt(mt)}\cdot{paren(ay)}\cdot{fmt(b)}+{fmt(Iz)}\cdot{paren(rd)}}}{{{fmt(l)}}}", res["Fyf"][i], "N", S,
             ref="lateral + yaw equilibrium")
    calc.add("rp.Fyr", r"F_{y,r}", "Rear axle lateral force", r"\frac{m a_y a - I_z \dot r}{l}",
             rf"\frac{{{fmt(mt)}\cdot{paren(ay)}\cdot{fmt(a)}-{fmt(Iz)}\cdot{paren(rd)}}}{{{fmt(l)}}}", res["Fyr"][i], "N", S)
    dl = mt * ax * V.h_cg_mm / 1000 / (2 * l)
    calc.add("rp.dlong", r"\Delta F_{z,x}", "Longitudinal load transfer per wheel (+ to rear)", r"\frac{m\,a_x\,h}{2\,l}",
             rf"\frac{{{fmt(mt)}\cdot{paren(ax)}\cdot{fmt(V.h_cg_mm / 1000)}}}{{2\cdot{fmt(l)}}}", dl, "N", S, note="no pitch motion (load transfer only)")
    for w, base, lat_ax, sgn_lat, sgn_long in (("FL", G["Fz0f"], "f", -1, -1), ("FR", G["Fz0f"], "f", 1, -1),
                                               ("RL", G["Fz0r"], "r", -1, 1), ("RR", G["Fz0r"], "r", 1, 1)):
        Fz = res["Fz"][w][i]
        latv = (Fz - base - sgn_long * dl) * sgn_lat
        calc.add(f"rp.Fz{w}", rf"F_{{z,{w}}}", f"{w} wheel load",
                 rf"F_{{z,0}}{'+' if sgn_lat > 0 else '-'}\Delta F_{{z,lat}}^{lat_ax}(t){'+' if sgn_long > 0 else '-'}\Delta F_{{z,x}}",
                 rf"{fmt(base)}{'+' if sgn_lat > 0 else '-'}{paren(latv)}{'+' if sgn_long > 0 else '-'}{paren(dl)}", Fz, "N", S,
                 note="lateral LT from the transient roll model (geometric+springs+ARB+dampers+unsprung)")
    calc.add("rp.alpha_f", r"\alpha_f", "Front slip angle", r"F_y(F_{z,FL},\alpha)+F_y(F_{z,FR},\alpha)=F_{y,f}",
             r"\text{Brent's method (TMeasy)}", res["alpha_f"][i], "deg", S)
    calc.add("rp.alpha_r", r"\alpha_r", "Rear slip angle", r"F_y(F_{z,RL},\alpha)+F_y(F_{z,RR},\alpha)=F_{y,r}",
             r"\text{Brent's method (TMeasy)}", res["alpha_r"][i], "deg", S)
    calc.add("rp.beta", r"\beta", "Vehicle sideslip at the CG", r"\frac{b\,r}{v}-\alpha_r",
             rf"\frac{{{fmt(b)}\cdot{paren(r)}}}{{{fmt(max(v, 0.1))}}}-{paren(res['alpha_r'][i] * DEG)}", res["beta"][i], "deg", S,
             note="value in deg; substitution in rad")
    calc.add("rp.delta", r"\delta", "Required mean front steer angle", r"\alpha_f-\alpha_r+l\,\kappa",
             rf"{paren(res['alpha_f'][i] * DEG)}-{paren(res['alpha_r'][i] * DEG)}+{fmt(l)}\cdot{paren(k)}", res["delta"][i], "deg", S,
             note="value in deg; substitution in rad")
    calc.add("rp.dalpha", r"\Delta\alpha", "Balance: sgn(a_y)(alpha_f - alpha_r)", r"\operatorname{sgn}(a_y)\,(\alpha_f-\alpha_r)",
             rf"\operatorname{{sgn}}({fmt(ay)})\,({fmt(res['alpha_f'][i])}-{paren(res['alpha_r'][i])})", res["dalpha"][i], "deg", S,
             note=f"> +{G['band_deg']}° understeer, < -{G['band_deg']}° oversteer -> {res['status'][i]}")
    calc.add("rp.util_f", r"\mu_f", "Front grip utilisation", r"\frac{|F_{y,f}|}{F_{y,\max,f}}",
             rf"\frac{{{fmt(abs(res['Fyf'][i]))}}}{{{fmt(res['cap_f'][i])}}}", res["util_f"][i], "-", S)
    calc.add("rp.util_r", r"\mu_r", "Rear grip utilisation", r"\frac{|F_{y,r}|}{F_{y,\max,r}}",
             rf"\frac{{{fmt(abs(res['Fyr'][i]))}}}{{{fmt(res['cap_r'][i])}}}", res["util_r"][i], "-", S)
