"""Transient roll response to an open-loop lateral-acceleration input.

Equation of motion (roll-only, rigid chassis, sprung mass rolling about the roll axis):

    I_ra * phi'' = m_s a_y(t) h1  [+ m_s g h1 phi]
                   - sum_axles t_i * ( dF_spring + dF_bump + dF_ARB + dF_damper )_i

    I_ra = I_xx,cg + m_s h1^2          (parallel-axis theorem)

Per axle and per wheel, at every instant:
    geometric  dF_g = m_s,axle a_y(t) h_rc / t      (instantaneous, links)
    unsprung   dF_u = 2 m_u a_y(t) h_u / t          (instantaneous, tires)
    spring / bump / ARB / damper from the element forces at z = +-(t/2) phi, z' = +-(t/2) phi'
This reproduces the OptimumG "Entry requirements" decomposition (geometric vs.
spring vs. ARB vs. damper load transfer against time).
"""
from __future__ import annotations

import math

import numpy as np
from scipy.integrate import solve_ivp

from .derivation import Calc, NullCalc, fmt, paren
from .load_transfer import solve_roll
from .schemas import ModelIn, effective_ay_g
from .suspension import AxleSuspension

DEG = math.pi / 180


def ay_profile(m: ModelIn):
    T = m.maneuver.transient
    A = effective_ay_g(m.maneuver)
    if T.profile == "road":
        return road_profile(m)[0]
    if T.profile == "step":
        return lambda t: A if t >= T.t0_s else 0.0
    if T.profile == "ramp":
        return lambda t: 0.0 if t < T.t0_s else (A * (t - T.t0_s) / T.rise_s if t < T.t0_s + T.rise_s else A)
    if T.profile == "sine":
        return lambda t: 0.0 if t < T.t0_s else A * math.sin(2 * math.pi * T.freq_hz * (t - T.t0_s))
    if T.profile == "csv":
        if not T.csv or len(T.csv) < 2:
            raise ValueError("CSV profile selected but no [[t, a_y], ...] data given")
        pts = sorted((float(a), float(b)) for a, b in T.csv)
        tt = np.array([p[0] for p in pts]); aa = np.array([p[1] for p in pts])
        return lambda t: float(np.interp(t, tt, aa))
    raise ValueError(T.profile)


def road_profile(m: ModelIn, ds: float = 0.02):
    """a_y(t) [g] from road segments: a_y = v(s)^2 kappa(s), s(t) = int v dt.

    Curvature kappa = 1/R per segment (0 on straights, sign = direction); between
    segments kappa ramps linearly over the clothoid length L_tr (centred on the joint).
    Speed is linearly blended over the same transition. Returns (ay_of_t, plan) where
    plan holds the x-y path for plotting.
    """
    T = m.maneuver.transient
    g = m.maneuver.g
    rows = [r for r in (T.road or []) if len(r) >= 3 and r[0] > 0]
    if not rows:
        raise ValueError("Road profile: give at least one segment [length m, radius m, speed km/h]")
    L = np.array([r[0] for r in rows], float)
    k = np.array([(1.0 / r[1]) if r[1] != 0 else 0.0 for r in rows])
    v = np.array([r[2] / 3.6 for r in rows])
    if np.any(v <= 0):
        raise ValueError("Road profile: speeds must be > 0")
    edges = np.concatenate([[0.0], np.cumsum(L)])
    s = np.arange(0.0, edges[-1] + ds / 2, ds)
    idx = np.clip(np.searchsorted(edges, s, side="right") - 1, 0, len(rows) - 1)
    kap, vel = k[idx].copy(), v[idx].copy()
    Ltr = T.road_transition_m
    if Ltr > 0:
        for j in range(1, len(rows)):
            a, b = edges[j] - Ltr / 2, edges[j] + Ltr / 2
            msk = (s >= a) & (s <= b)
            w = (s[msk] - a) / Ltr
            kap[msk] = k[j - 1] + (k[j] - k[j - 1]) * w
            vel[msk] = v[j - 1] + (v[j] - v[j - 1]) * w
    t = np.concatenate([[0.0], np.cumsum(ds / (0.5 * (vel[1:] + vel[:-1])))])
    ay = vel ** 2 * kap / g
    # plan view (heading = integral of curvature)
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (kap[1:] + kap[:-1]) * ds)])
    x = np.concatenate([[0.0], np.cumsum(np.cos(psi[:-1]) * ds)])
    y = np.concatenate([[0.0], np.cumsum(np.sin(psi[:-1]) * ds)])
    plan = {"s": s, "t": t, "x": x, "y": y, "ay_g": ay, "kappa": kap, "v": vel, "edges": edges}
    return (lambda tq: float(np.interp(tq, t, ay))), plan


def _setup(m: ModelIn):
    V, F, R, M = m.vehicle, m.front, m.rear, m.maneuver
    g = M.g
    mf, mr = V.mass_kg * V.weight_front, V.mass_kg * (1 - V.weight_front)
    msf, msr = mf - 2 * F.unsprung_mass_kg, mr - 2 * R.unsprung_mass_kg
    ms = msf + msr
    l = V.wheelbase_mm / 1000
    huf, hur = F.h_unsprung_mm / 1000, R.h_unsprung_mm / 1000
    hs = (V.mass_kg * V.h_cg_mm / 1000 - 2 * F.unsprung_mass_kg * huf - 2 * R.unsprung_mass_kg * hur) / ms
    a_s = l * msr / ms
    hrf, hrr = F.h_rc_mm / 1000, R.h_rc_mm / 1000
    h1 = hs - (hrf + (hrr - hrf) * a_s / l)
    susp = {"f": AxleSuspension(F, msf, g), "r": AxleSuspension(R, msr, g)}
    msax = {"f": msf, "r": msr}
    tr = {"f": F.track_mm / 1000, "r": R.track_mm / 1000}
    axes = {"f": F, "r": R}
    if M.geo_model == "ic_angles":
        from . import geo_coupling as gc
        hrf, hrr = gc.static_rc(F), gc.static_rc(R)
        h1 = hs - (hrf + (hrr - hrf) * a_s / l)
        # outer-force share r(a_y) from the steady coupled solution at a few a_y levels
        from .schemas import effective_ay_g
        amax = max(0.1, abs(effective_ay_g(M)), 0.1)
        if M.transient.profile in ("road", "csv"):
            try:
                amax = max(amax, float(np.max(np.abs([ay_profile(m)(t) for t in np.linspace(0, M.transient.t_end_s, 400)]))))
            except Exception:
                pass
        nodes = np.linspace(0.02, amax, 9)
        rt = {"f": [], "r": []}
        for a in nodes:
            info = gc.solve(m, float(a), exact=False, tol_mm=1e-3)
            for k in "fr":
                rt[k].append(info["axles"][k]["r"])

        def geo(tag, ay, phi):
            if abs(ay) < 1e-12:
                return 0.0, 0.0
            sg = 1.0 if ay > 0 else -1.0
            r = float(np.interp(abs(ay) / g, nodes, rt[tag]))
            to, ti = gc.tan_fast(axes[tag], sg * phi / DEG)
            Fys = msax[tag] * abs(ay)
            return sg * Fys * (r * to + (1 - r) * ti) / 2, Fys * (r * to - (1 - r) * ti)
    else:
        def geo(tag, ay, phi):
            return msax[tag] * ay * (hrf if tag == "f" else hrr) / tr[tag], 0.0
    return dict(g=g, ms=ms, msax=msax, h1=h1, hs=hs, susp=susp, geo=geo, tr=tr,
                hrc={"f": hrf, "r": hrr}, mu={"f": F.unsprung_mass_kg, "r": R.unsprung_mass_kg},
                hu={"f": huf, "r": hur}, I=V.roll_inertia_kgm2 + ms * h1 ** 2)


def _applied_moment(P, ay, phi, grav):
    """m_s a_y h_s - sum(geometric moments) [+ m_s g h1_eff phi]  (= m_s a_y h1 for the classic RC model)."""
    ms, g, hs = P["ms"], P["g"], P["hs"]
    Mg = sum(P["geo"](k, ay, phi)[0] * P["tr"][k] for k in "fr")
    h1e = hs - Mg / (ms * ay) if abs(ay) > 1e-9 else P["h1"]
    return ms * ay * hs - Mg + (ms * g * h1e * phi if grav else 0.0)


def _rhs_factory(P, ayf, grav):
    ms, h1, g, I, susp = P["ms"], P["h1"], P["g"], P["I"], P["susp"]

    def rhs(t, y):
        phi, w = y
        ay = ayf(t) * g
        M = _applied_moment(P, ay, phi, grav)
        for s in susp.values():
            c = s.components(phi, w)
            M -= s.t * (c["spring"] + c["bump"] + c["arb"] + c["damper"])
        return [w, M / I]
    return rhs


def simulate(m: ModelIn, calc: Calc | None = None, n_out: int | None = None) -> dict:
    calc = calc or NullCalc()
    T = m.maneuver.transient
    grav = m.maneuver.roll_gravity_term
    P = _setup(m)
    ayf = ay_profile(m)
    g, ms, h1 = P["g"], P["ms"], P["h1"]
    phi0 = solve_roll(P["susp"], ms, ayf(0.0) * g, g, h1, grav)
    if math.isnan(phi0):
        phi0 = 0.0
    rhs = _rhs_factory(P, ayf, grav)
    dt = max(0.002, T.t_end_s / 2000) if n_out is None else T.t_end_s / (n_out - 1)
    tt = np.arange(0.0, T.t_end_s + 1e-12, dt)
    sol = solve_ivp(rhs, (0.0, T.t_end_s), [phi0, 0.0], method="RK45", t_eval=tt, dense_output=True,
                    max_step=0.002, rtol=1e-7, atol=1e-10)
    if not sol.success:
        raise RuntimeError(f"transient integration failed: {sol.message}")
    out = {"t": tt.tolist(), "ay_g": [ayf(t) for t in tt], "phi_deg": (sol.y[0] / DEG).tolist(),
           "phidot_deg_s": (sol.y[1] / DEG).tolist(), "axles": {}}
    for tag in ("f", "r"):
        s = P["susp"][tag]
        cols = {k: [] for k in ("geo", "jack", "dz_jack", "unsprung", "spring", "bump", "arb", "damper", "elastic", "suspended",
                                "total", "Fz_out", "Fz_in", "v_d_out", "v_d_in", "z_out",
                                "pct_geo", "pct_spring", "pct_arb", "pct_damper", "pct_bump")}
        Fz0 = (P["msax"][tag] + 2 * P["mu"][tag]) * g / 2
        for i, t in enumerate(tt):
            ay = ayf(t) * g
            phi, w = sol.y[0, i], sol.y[1, i]
            c = s.components(phi, w)
            geo, jack = P["geo"](tag, ay, phi)
            kw = s.wheel_rate(0.0) if not s.direct else s.K_direct / (1000 * s.t ** 2 / 2)
            cols["jack"].append(jack)
            cols["dz_jack"].append(jack / (2 * kw) if kw > 0 else None)
            un = 2 * P["mu"][tag] * ay * P["hu"][tag] / s.t
            el = c["spring"] + c["bump"] + c["arb"] + c["damper"]
            sus = geo + el
            tot = sus + un
            for k, v in (("geo", geo), ("unsprung", un), ("spring", c["spring"]), ("bump", c["bump"]),
                         ("arb", c["arb"]), ("damper", c["damper"]), ("elastic", el), ("suspended", sus),
                         ("total", tot), ("Fz_out", Fz0 + tot), ("Fz_in", Fz0 - tot),
                         ("v_d_out", s.damper_mr(c["z_o"]) * c["v_o"]), ("v_d_in", s.damper_mr(c["z_i"]) * c["v_i"]),
                         ("z_out", c["z_o"])):
                cols[k].append(v)
            for k, v in (("pct_geo", geo), ("pct_spring", c["spring"]), ("pct_arb", c["arb"]),
                         ("pct_damper", c["damper"]), ("pct_bump", c["bump"])):
                cols[k].append(100 * v / sus if abs(sus) > 1e-6 else None)
        out["axles"][tag] = cols
    if T.profile == "road":
        plan = road_profile(m)[1]
        step = max(1, len(plan["s"]) // 800)
        out["road"] = {k: np.asarray(plan[k])[::step].tolist() for k in ("s", "t", "x", "y", "ay_g", "kappa", "v")}
        out["road"]["edges"] = plan["edges"].tolist()
        out["road"]["t_total"] = float(plan["t"][-1])
    out["metrics"] = metrics(m, P, sol, tt, out, calc)
    probe(m, P, ayf, sol, calc)
    return out


def metrics(m, P, sol, tt, out, calc):
    T = m.maneuver.transient
    S = "11. Transient metrics"
    phi = np.array(out["phi_deg"])
    res = {}
    ayv = np.abs(np.array(out["ay_g"]))
    if T.profile in ("road", "csv", "sine") and ayv.max() > 0:
        t0 = float(tt[np.argmax(ayv > 0.01 * ayv.max())])
        calc.add("tr.t0", r"t_0", "Input onset (|a_y| > 1 % of max)", r"\min\{t:|a_y(t)|>0.01\max|a_y|\}", fmt(t0), t0, "s", S)
    else:
        t0 = T.t0_s
    fin = calc.add("tr.phi_final", r"\phi(t_{end})", "Roll angle at end of run", r"\phi(t_{end})",
                   rf"t_{{end}}={fmt(T.t_end_s)}\,s", float(phi[-1]), "deg", S)
    ipk = int(np.argmax(np.abs(phi)))
    pk = calc.add("tr.phi_peak", r"\phi_{peak}", "Peak roll angle", r"\max_t|\phi(t)|",
                  rf"t={fmt(tt[ipk])}\,s", float(phi[ipk]), "deg", S)
    if abs(fin) > 0.2 * abs(pk) and abs(fin) > 1e-9:
        calc.add("tr.overshoot", r"OS_\phi", "Roll overshoot", r"\frac{\phi_{peak}-\phi(t_{end})}{\phi(t_{end})}\cdot100",
                 rf"\frac{{{fmt(pk)}-{fmt(fin)}}}{{{fmt(fin)}}}\cdot100", 100 * (pk - fin) / fin, "%", S)
    ref = fin if (abs(fin) > 0.2 * abs(pk) and abs(fin) > 1e-9) else pk
    idx = np.where((tt >= t0) & (np.abs(phi) >= 0.9 * abs(ref)))[0]
    if len(idx) and abs(ref) > 1e-9:
        calc.add("tr.t90", r"t_{90}", "Time to 90 % of final (or peak) roll, from input onset",
                 r"\min\{t:|\phi|\ge0.9|\phi_{ref}|\}-t_0", rf"{fmt(tt[idx[0]])}-{fmt(t0)}",
                 float(tt[idx[0]] - t0), "s", S)
    for tag, name in (("f", "Front"), ("r", "Rear")):
        A = out["axles"][tag]
        for dtm in (0.05, 0.10):
            tq = t0 + dtm
            if tq > T.t_end_s:
                continue
            i = int(np.argmin(np.abs(tt - tq)))
            ms_ = int(round(dtm * 1000))
            for comp, lab in (("geo", "geometric"), ("damper", "damper"), ("spring", "spring"), ("arb", "ARB")):
                v = A[f"pct_{comp}"][i]
                if v is None:
                    continue
                calc.add(f"tr.{tag}.pct_{comp}_{ms_}", rf"\eta_{{{comp}}}^{{{tag}}}(t_0+{ms_}ms)",
                         f"{name}: {lab} share of suspended LT at t0 + {ms_} ms",
                         rf"\frac{{\Delta F_{{{comp}}}}}{{\Delta F_{{geo}}+\Delta F_{{spring}}+\Delta F_{{bump}}+\Delta F_{{ARB}}+\Delta F_{{damper}}}}\cdot100",
                         rf"\frac{{{fmt(A[comp][i])}}}{{{fmt(A['suspended'][i])}}}\cdot100", v, "%", S,
                         note="as in OptimumG 'Entry requirements' Fig. 2/4")
        vd = np.abs(np.array(A["v_d_out"]))
        calc.add(f"tr.{tag}.vdmax", rf"|v_d|_{{\max}}^{{{tag}}}", f"{name}: peak damper velocity", r"\max_t|MR\,\dot z|",
                 rf"t={fmt(tt[int(np.argmax(vd))])}\,s", float(vd.max()), "mm/s", S)
        tot = np.array(A["total"])
        calc.add(f"tr.{tag}.dFz_peak", rf"\Delta F_{{z,\max}}^{{{tag}}}", f"{name}: peak total load transfer",
                 r"\max_t\Delta F_z(t)", rf"t={fmt(tt[int(np.argmax(np.abs(tot)))])}\,s", float(tot[np.argmax(np.abs(tot))]), "N", S)
    return {s.key: s.value for s in calc.steps if s.key.startswith("tr.")} if not isinstance(calc, NullCalc) else res


def probe(m, P, ayf, sol, calc):
    """Full derivation of every load-transfer component at t = t_probe."""
    if isinstance(calc, NullCalc):
        return
    T = m.maneuver.transient
    tp = min(max(T.t_probe_s, 0.0), T.t_end_s)
    S = f"10. Transient breakdown at t = {tp:g} s"
    g, ms, h1, I = P["g"], P["ms"], P["h1"], P["I"]
    V = m.vehicle
    phi, w = sol.sol(tp)
    ay = calc.add("p.ay", r"a_y(t)", "Input lateral acceleration", r"a_{y,[g]}(t)\cdot g",
                  rf"{fmt(ayf(tp))}\cdot{fmt(g)}", ayf(tp) * g, "m/s²", S, note=f"profile: {T.profile}")
    calc.add("p.I", r"I_{ra}", "Sprung roll inertia about the roll axis", r"I_{xx,cg}+m_s h_1^2",
             rf"{fmt(V.roll_inertia_kgm2)}+{fmt(ms)}\cdot{paren(h1)}^2", I, "kg·m²", S, ref="parallel-axis theorem")
    calc.add("p.phi", r"\phi(t)", "Roll angle (integrated)", r"\int\!\!\int\ddot\phi\,dt^2\ \ \text{(RK45)}",
             rf"\phi={fmt(phi)}\,rad", phi / DEG, "deg", S)
    calc.add("p.phidot", r"\dot\phi(t)", "Roll rate (integrated)", r"\int\ddot\phi\,dt\ \ \text{(RK45)}",
             rf"\dot\phi={fmt(w)}\,rad/s", w / DEG, "deg/s", S)
    Mtot = _applied_moment(P, ay, phi, m.maneuver.roll_gravity_term)
    coupled = m.maneuver.geo_model == "ic_angles"
    calc.add("p.Mapp", r"M_{app}(t)", "Applied roll moment on the springs/dampers",
             r"m_s a_y h_s-\sum_i t_i\,\Delta F_{g,i}" + (r"+m_s g h_{1,eff}\phi" if m.maneuver.roll_gravity_term else ""),
             rf"{fmt(ms)}\cdot{fmt(ay)}\cdot{fmt(P['hs'])}-\sum t_i\Delta F_{{g,i}}" + (r"+\dots" if m.maneuver.roll_gravity_term else ""),
             Mtot, "N·m", S, note="classic RC model: equals m_s a_y h_1 (+ m_s g h_1 phi)")
    parts = []
    for tag, name in (("f", "Front"), ("r", "Rear")):
        s = P["susp"][tag]
        c = s.components(phi, w)
        zo, vo = c["z_o"], c["v_o"]
        calc.add(f"p.{tag}.zo", rf"z_o^{tag},\ \dot z_o^{tag}", f"{name}: outer wheel travel / velocity",
                 r"\frac{t}{2}\phi,\ \frac{t}{2}\dot\phi", rf"\frac{{{fmt(s.t * 1000)}}}{{2}}\cdot{paren(phi)},\ \frac{{{fmt(s.t * 1000)}}}{{2}}\cdot{paren(w)}",
                 zo, "mm", S, note=f"z'_o = {vo:.4f} mm/s")
        gv, jv = P["geo"](tag, ay, phi)
        if coupled:
            geo = calc.add(f"p.{tag}.geo", rf"\Delta F_g^{tag}", f"{name}: geometric LT (IC angles at \u03c6(t))",
                           r"\frac{m_{s,ax}a_y\left[r\tan\theta_o(\phi)+(1-r)\tan\theta_i(\phi)\right]}{2}",
                           rf"\phi={fmt(phi / DEG)}^\circ,\ J={fmt(jv)}\,N", gv, "N", S,
                           note="r(a_y) from the steady coupled solution; tan(theta) from the rolled kinematics")
        else:
            geo = calc.add(f"p.{tag}.geo", rf"\Delta F_g^{tag}", f"{name}: geometric LT", r"\frac{m_{s,ax}\,a_y\,h_{rc}}{t}",
                           rf"\frac{{{fmt(P['msax'][tag])}\cdot{fmt(ay)}\cdot{paren(P['hrc'][tag])}}}{{{fmt(s.t)}}}",
                           gv, "N", S)
        if s.direct:
            calc.add(f"p.{tag}.spring", rf"\Delta F_{{spring}}^{tag}", f"{name}: springs+ARB LT (direct K)",
                     r"\frac{K_\phi\phi}{t}", rf"\frac{{{fmt(s.K_direct)}\cdot{paren(phi)}}}{{{fmt(s.t)}}}", c["spring"], "N", S)
        else:
            Fo, Fi = s.spring_wheel(zo), s.spring_wheel(-zo)
            calc.add(f"p.{tag}.spring", rf"\Delta F_{{spring}}^{tag}", f"{name}: spring LT",
                     r"\frac{[F_{s0}+k_s x_s(z_o)]MR(z_o)-[F_{s0}+k_s x_s(z_i)]MR(z_i)}{2}",
                     rf"\frac{{{fmt(Fo)}-{fmt(Fi)}}}{{2}}", c["spring"], "N", S)
            calc.add(f"p.{tag}.bump", rf"\Delta F_{{bump}}^{tag}", f"{name}: bump-stop LT",
                     r"\frac{k_b(z_o-g_b)^+-k_b(z_i-g_b)^+}{2}", fmt(c["bump"]), c["bump"], "N", S)
            calc.add(f"p.{tag}.arb", rf"\Delta F_{{ARB}}^{tag}", f"{name}: ARB LT", r"k_{ARB}\frac{z_o-z_i}{2}",
                     rf"{fmt(s.karb)}\cdot{fmt(zo)}", c["arb"], "N", S)
        mro, mri = s.damper_mr(zo), s.damper_mr(-zo)
        Fdo, Fdi = s.damper.force(mro * vo), s.damper.force(-mri * vo)
        calc.add(f"p.{tag}.damper", rf"\Delta F_{{damper}}^{tag}", f"{name}: damper LT",
                 r"\frac{F_d(MR_o\dot z_o)MR_o-F_d(MR_i\dot z_i)MR_i}{2}",
                 rf"\frac{{{fmt(Fdo)}\cdot{fmt(mro)}-({fmt(Fdi)})\cdot{fmt(mri)}}}{{2}}", c["damper"], "N", S,
                 note=f"damper velocity outer {mro * vo:.3f} mm/s (+ bump), inner {-mri * vo:.3f} mm/s")
        el = c["spring"] + c["bump"] + c["arb"] + c["damper"]
        calc.add(f"p.{tag}.susp", rf"\Delta F_{{susp}}^{tag}", f"{name}: suspended-mass LT",
                 r"\Delta F_g+\Delta F_{spring}+\Delta F_{bump}+\Delta F_{ARB}+\Delta F_{damper}",
                 rf"{fmt(geo)}+{paren(c['spring'])}+{paren(c['bump'])}+{paren(c['arb'])}+{paren(c['damper'])}",
                 geo + el, "N", S)
        parts.append((s.t, el, tag))
    Mel = sum(t * e for t, e, _ in parts)
    msum = "+".join(fmt(t) + r"\cdot" + paren(e) for t, e, _ in parts)
    calc.add("p.phiddot", r"\ddot\phi(t)", "Roll acceleration (equation of motion)",
             r"\frac{M_{app}-\sum t_i\,\Delta F_{el,i}}{I_{ra}}",
             rf"\frac{{{fmt(Mtot)}-\left({msum}\right)}}{{{fmt(I)}}}",
             (Mtot - Mel) / I / DEG, "deg/s²", S)
