"""Steady-state lateral load transfer, rigid chassis.

Split of the lateral load transfer of each axle into three paths
(Milliken & Milliken, *Race Car Vehicle Dynamics*, ch. 18; Rill, *Road
Vehicle Dynamics*, sec. 9.2.4-9.2.5):

    dFz_u  direct / unsprung  : 2 m_u a_y h_u / t              (through the tires only)
    dFz_g  geometric          : m_s,axle a_y h_rc / t          (through the links, instantaneous)
    dFz_e  elastic            : K_phi,axle * phi / t           (through springs/ARB, needs body roll)

with the body roll angle (sprung mass rolling about the roll axis)

    phi = m_s a_y h1 / (K_phi,f + K_phi,r - m_s g h1)          (gravity term optional)

Consequence (single axle, no unsprung mass): the geometric share is
h_rc / h_cg - RC at CG -> 100 % geometric, RC at ground -> 100 % elastic.
"""
from __future__ import annotations

import math

from .derivation import Calc, NullCalc, fmt, paren
from .schemas import ModelIn
from .suspension import AxleSuspension

DEG = math.pi / 180.0


def compute(m: ModelIn, calc: Calc | None = None, ay_g: float | None = None,
            hrc_override: dict | None = None) -> dict:
    calc = calc or NullCalc()
    from .derivation import quiet
    with quiet(isinstance(calc, NullCalc)):
        return _compute(m, calc, ay_g, hrc_override)


def _compute(m, calc, ay_g, hrc_override):
    V, F, R, M = m.vehicle, m.front, m.rear, m.maneuver
    S0, S1 = "1. Mass distribution", "2. Body roll"
    g = M.g
    if ay_g is None and M.ay_source == "corner":
        v = M.speed_kmh / 3.6
        ay = calc.add("ay", r"a_y", "Lateral acceleration (corner: speed & radius)", r"\frac{v^2}{R}",
                      rf"\frac{{({fmt(M.speed_kmh)}/3.6)^2}}{{{fmt(M.radius_m)}}}", v * v / M.radius_m, "m/s²", S0)
        ay_g = ay / g
        calc.add("ay_g", r"a_{y,[g]}", "Lateral acceleration in g", r"a_y/g", rf"{fmt(ay)}/{fmt(g)}", ay_g, "g", S0)
    else:
        ay_g = M.ay_g if ay_g is None else ay_g
        ay = calc.add("ay", r"a_y", "Lateral acceleration", r"a_{y,[g]}\cdot g",
                      rf"{fmt(ay_g)}\cdot{fmt(g)}", ay_g * g, "m/s²", S0)
    mt = V.mass_kg
    wf = V.weight_front
    l = calc.add("l", r"l", "Wheelbase", r"l_{[mm]}/1000", rf"{fmt(V.wheelbase_mm)}/1000",
                 V.wheelbase_mm / 1000, "m", S0)
    h = calc.add("h", r"h", "Total CG height", r"h_{[mm]}/1000", rf"{fmt(V.h_cg_mm)}/1000",
                 V.h_cg_mm / 1000, "m", S0)
    tf = calc.add("t_f", r"t_f", "Front track", r"t_{f,[mm]}/1000", rf"{fmt(F.track_mm)}/1000",
                  F.track_mm / 1000, "m", S0)
    tr = calc.add("t_r", r"t_r", "Rear track", r"t_{r,[mm]}/1000", rf"{fmt(R.track_mm)}/1000",
                  R.track_mm / 1000, "m", S0)
    mf = calc.add("m_f", r"m_f", "Front axle mass", r"m\,w_f", rf"{fmt(mt)}\cdot{fmt(wf)}", mt * wf, "kg", S0)
    mr = calc.add("m_r", r"m_r", "Rear axle mass", r"m\,(1-w_f)", rf"{fmt(mt)}\cdot(1-{fmt(wf)})",
                  mt * (1 - wf), "kg", S0)
    muf, mur = F.unsprung_mass_kg, R.unsprung_mass_kg
    huf = calc.add("h_uf", r"h_{u,f}", "Front unsprung CG height", r"h_{u,f,[mm]}/1000",
                   rf"{fmt(F.h_unsprung_mm)}/1000", F.h_unsprung_mm / 1000, "m", S0)
    hur = calc.add("h_ur", r"h_{u,r}", "Rear unsprung CG height", r"h_{u,r,[mm]}/1000",
                   rf"{fmt(R.h_unsprung_mm)}/1000", R.h_unsprung_mm / 1000, "m", S0)
    ms = calc.add("m_s", r"m_s", "Sprung mass", r"m-2m_{u,f}-2m_{u,r}",
                  rf"{fmt(mt)}-2\cdot{fmt(muf)}-2\cdot{fmt(mur)}", mt - 2 * muf - 2 * mur, "kg", S0)
    msf = calc.add("m_sf", r"m_{s,f}", "Sprung mass on front axle", r"m_f-2m_{u,f}",
                   rf"{fmt(mf)}-2\cdot{fmt(muf)}", mf - 2 * muf, "kg", S0,
                   note="unsprung masses assumed to sit on the axle lines")
    msr = calc.add("m_sr", r"m_{s,r}", "Sprung mass on rear axle", r"m_r-2m_{u,r}",
                   rf"{fmt(mr)}-2\cdot{fmt(mur)}", mr - 2 * mur, "kg", S0)
    a_s = calc.add("a_s", r"a_s", "Front axle to sprung CG", r"l\,\frac{m_{s,r}}{m_s}",
                   rf"{fmt(l)}\cdot\frac{{{fmt(msr)}}}{{{fmt(ms)}}}", l * msr / ms, "m", S0)
    hs = calc.add("h_s", r"h_s", "Sprung CG height",
                  r"\frac{m\,h-2m_{u,f}h_{u,f}-2m_{u,r}h_{u,r}}{m_s}",
                  rf"\frac{{{fmt(mt)}\cdot{fmt(h)}-2\cdot{fmt(muf)}\cdot{fmt(huf)}-2\cdot{fmt(mur)}\cdot{fmt(hur)}}}{{{fmt(ms)}}}",
                  (mt * h - 2 * muf * huf - 2 * mur * hur) / ms, "m", S0)

    if hrc_override:
        hrf = calc.add("h_rf", r"h_{rc,f}", "Front roll-centre height (force-based, migrated)",
                       r"h_{rc,eff}^{f}/1000\ 	ext{(section 2b)}", rf"{fmt(hrc_override['f'] * 1000)}/1000",
                       hrc_override["f"], "m", S1)
        hrr = calc.add("h_rr", r"h_{rc,r}", "Rear roll-centre height (force-based, migrated)",
                       r"h_{rc,eff}^{r}/1000\ 	ext{(section 2b)}", rf"{fmt(hrc_override['r'] * 1000)}/1000",
                       hrc_override["r"], "m", S1)
    else:
        hrf = calc.add("h_rf", r"h_{rc,f}", "Front roll-centre height", r"h_{rc,f,[mm]}/1000",
                       rf"{fmt(F.h_rc_mm)}/1000", F.h_rc_mm / 1000, "m", S1)
        hrr = calc.add("h_rr", r"h_{rc,r}", "Rear roll-centre height", r"h_{rc,r,[mm]}/1000",
                       rf"{fmt(R.h_rc_mm)}/1000", R.h_rc_mm / 1000, "m", S1)
    hra = calc.add("h_ra", r"h_{ra}", "Roll-axis height below sprung CG",
                   r"h_{rc,f}+\left(h_{rc,r}-h_{rc,f}\right)\frac{a_s}{l}",
                   rf"{fmt(hrf)}+\left({fmt(hrr)}-{paren(hrf)}\right)\frac{{{fmt(a_s)}}}{{{fmt(l)}}}",
                   hrf + (hrr - hrf) * a_s / l, "m", S1)
    h1 = calc.add("h1", r"h_1", "Roll moment arm (sprung CG above roll axis)", r"h_s-h_{ra}",
                  rf"{fmt(hs)}-{paren(hra)}", hs - hra, "m", S1)
    if h1 < 0:
        calc.warn("Roll axis lies ABOVE the sprung CG (h1 < 0): the body rolls into the turn.")
    susp = {"f": AxleSuspension(F, msf, g), "r": AxleSuspension(R, msr, g)}
    Ks = {}
    for tag, AX, name in (("f", F, "Front"), ("r", R, "Rear")):
        su = susp[tag]
        if su.direct:
            Ks[tag] = calc.add(f"K_{tag}", rf"K_{{\phi,{tag}}}", f"{name} roll stiffness (direct input)",
                               rf"K_{{\phi,{tag},[Nm/^\circ]}}\cdot\frac{{180}}{{\pi}}",
                               rf"{fmt(AX.roll_stiffness_Nm_deg)}\cdot\frac{{180}}{{\pi}}",
                               AX.roll_stiffness_Nm_deg / DEG, "N·m/rad", S1)
        else:
            Ks[tag] = record_rates(su, calc, tag, name)
    Kf, Kr = Ks["f"], Ks["r"]
    all_direct = susp["f"].direct and susp["r"].direct
    grav = M.roll_gravity_term
    if all_direct:
        if grav:
            den = Kf + Kr - ms * g * h1
            if den <= 0:
                calc.warn("Roll stiffness too low: K_f + K_r \u2264 m_s g h1 (statically unstable in roll).")
                den = float("nan")
            phi = calc.add("phi", r"\phi", "Body roll angle",
                           r"\frac{m_s\,a_y\,h_1}{K_{\phi,f}+K_{\phi,r}-m_s\,g\,h_1}",
                           rf"\frac{{{fmt(ms)}\cdot{fmt(ay)}\cdot{paren(h1)}}}{{{fmt(Kf)}+{fmt(Kr)}-{fmt(ms)}\cdot{fmt(g)}\cdot{paren(h1)}}}",
                           ms * ay * h1 / den, "rad", S1,
                           note="small-angle; includes the lateral shift of the sprung CG (m_s g h1 phi)")
        else:
            phi = calc.add("phi", r"\phi", "Body roll angle",
                           r"\frac{m_s\,a_y\,h_1}{K_{\phi,f}+K_{\phi,r}}",
                           rf"\frac{{{fmt(ms)}\cdot{fmt(ay)}\cdot{paren(h1)}}}{{{fmt(Kf)}+{fmt(Kr)}}}",
                           ms * ay * h1 / (Kf + Kr), "rad", S1, note="small-angle, gravity term neglected")
    else:
        phi = solve_roll(susp, ms, ay, g, h1, grav)
        if math.isnan(phi):
            calc.warn("No static roll equilibrium found within \u00b120\u00b0 (suspension too soft or bump stops missing).")
        lin = ms * ay * h1 / (Kf + Kr - (ms * g * h1 if grav else 0.0))
        calc.add("phi", r"\phi", "Body roll angle (nonlinear equilibrium)",
                 r"\text{root of }\ m_s a_y h_1" + (r"+m_s g h_1\phi" if grav else "") +
                 r"-M_f(\phi)-M_r(\phi)=0,\quad M(\phi)=t\,\left[\Delta F_{spring}+\Delta F_{bump}+\Delta F_{ARB}\right]",
                 rf"\text{{Brent's method; linear estimate }}\ \frac{{{fmt(ms)}\cdot{fmt(ay)}\cdot{paren(h1)}}}{{{fmt(Kf)}+{fmt(Kr)}" +
                 (rf"-{fmt(ms)}\cdot{fmt(g)}\cdot{paren(h1)}" if grav else "") + rf"}}={fmt(lin)}",
                 phi, "rad", S1, note="progressive springs / bump stops make M(phi) nonlinear")
    phi_deg = calc.add("phi_deg", r"\phi", "Body roll angle", r"\phi\cdot\frac{180}{\pi}",
                       rf"{fmt(phi)}\cdot\frac{{180}}{{\pi}}", phi / DEG, "deg", S1)
    if ay_g > 0:
        calc.add("roll_gradient", r"\partial\phi/\partial a_y", "Roll gradient",
                 r"\phi/a_{y,[g]}", rf"{fmt(phi_deg)}/{fmt(ay_g)}", phi_deg / ay_g, "deg/g", S1)

    res = {"ay": ay, "phi": phi, "phi_deg": phi_deg, "h1": h1, "hs": hs, "ms": ms, "l": l,
           "m_f": mf, "m_r": mr, "axles": {}}
    for tag, AX, t, m_ax, ms_ax, mu, hu, hrc, K in (
            ("f", F, tf, mf, msf, muf, huf, hrf, Kf), ("r", R, tr, mr, msr, mur, hur, hrr, Kr)):
        K = Ks[tag]
        name = "Front" if tag == "f" else "Rear"
        S = f"3. {name} axle load transfer"
        du = calc.add(f"{tag}.dFz_u", rf"\Delta F_{{z,u}}^{{{tag}}}", f"{name}: direct (unsprung) load transfer",
                      rf"\frac{{2m_{{u,{tag}}}\,a_y\,h_{{u,{tag}}}}}{{t_{tag}}}",
                      rf"\frac{{2\cdot{fmt(mu)}\cdot{fmt(ay)}\cdot{fmt(hu)}}}{{{fmt(t)}}}",
                      2 * mu * ay * hu / t, "N", S)
        dg = calc.add(f"{tag}.dFz_g", rf"\Delta F_{{z,g}}^{{{tag}}}", f"{name}: geometric load transfer",
                      rf"\frac{{m_{{s,{tag}}}\,a_y\,h_{{rc,{tag}}}}}{{t_{tag}}}",
                      rf"\frac{{{fmt(ms_ax)}\cdot{fmt(ay)}\cdot{paren(hrc)}}}{{{fmt(t)}}}",
                      ms_ax * ay * hrc / t, "N", S, note="transmitted by the suspension links, no roll needed")
        if susp[tag].direct:
            de = calc.add(f"{tag}.dFz_e", rf"\Delta F_{{z,e}}^{{{tag}}}", f"{name}: elastic load transfer",
                          rf"\frac{{K_{{\phi,{tag}}}\,\phi}}{{t_{tag}}}",
                          rf"\frac{{{fmt(K)}\cdot{paren(phi)}}}{{{fmt(t)}}}",
                          K * phi / t, "N", S, note="transmitted by springs/ARB, requires body roll")
            parts = {"spring": de, "bump": 0.0, "arb": 0.0}
        else:
            de, parts = record_elastic(susp[tag], phi, calc, tag, name, S)
        dt = calc.add(f"{tag}.dFz", rf"\Delta F_z^{{{tag}}}", f"{name}: total load transfer (per wheel)",
                      rf"\Delta F_{{z,u}}^{{{tag}}}+\Delta F_{{z,g}}^{{{tag}}}+\Delta F_{{z,e}}^{{{tag}}}",
                      rf"{fmt(du)}+{paren(dg)}+{paren(de)}", du + dg + de, "N", S)
        if dt != 0:
            geo_share = calc.add(f"{tag}.geo_share", rf"\eta_g^{{{tag}}}", f"{name}: geometric share",
                                 rf"\Delta F_{{z,g}}^{{{tag}}}/\Delta F_z^{{{tag}}}",
                                 rf"{fmt(dg)}/{fmt(dt)}", dg / dt, "-", S)
            el_share = calc.add(f"{tag}.el_share", rf"\eta_e^{{{tag}}}", f"{name}: elastic share",
                                rf"\Delta F_{{z,e}}^{{{tag}}}/\Delta F_z^{{{tag}}}",
                                rf"{fmt(de)}/{fmt(dt)}", de / dt, "-", S)
        else:
            geo_share = el_share = float("nan")
        S4 = "4. Wheel loads"
        fs = calc.add(f"{tag}.Fz_static", rf"F_{{z,0}}^{{{tag}}}", f"{name}: static wheel load",
                      rf"\frac{{m_{tag}\,g}}{{2}}", rf"\frac{{{fmt(m_ax)}\cdot{fmt(g)}}}{{2}}",
                      m_ax * g / 2, "N", S4)
        fo = calc.add(f"{tag}.Fz_out", rf"F_{{z,o}}^{{{tag}}}", f"{name}: OUTER wheel load",
                      rf"F_{{z,0}}^{{{tag}}}+\Delta F_z^{{{tag}}}", rf"{fmt(fs)}+{paren(dt)}", fs + dt, "N", S4)
        fi = calc.add(f"{tag}.Fz_in", rf"F_{{z,i}}^{{{tag}}}", f"{name}: INNER wheel load",
                      rf"F_{{z,0}}^{{{tag}}}-\Delta F_z^{{{tag}}}", rf"{fmt(fs)}-{paren(dt)}", fs - dt, "N", S4)
        if fi < 0:
            calc.warn(f"{name} inner wheel lifts (F_z,i < 0) - results beyond this a_y are not physical.")
        res["axles"][tag] = dict(dFz_u=du, dFz_g=dg, dFz_e=de, dFz=dt, dFz_e_spring=parts["spring"],
                                 dFz_e_bump=parts["bump"], dFz_e_arb=parts["arb"], geo_share=geo_share,
                                 el_share=el_share, Fz_static=fs, Fz_out=fo, Fz_in=fi, track=t)

    S5 = "5. Load transfer distribution"
    dfF, dfR = res["axles"]["f"]["dFz"], res["axles"]["r"]["dFz"]
    if dfF + dfR != 0:
        res["lltd_front"] = calc.add("lltd_front", r"\mathrm{LLTD}_f", "Lateral load transfer distribution (front)",
                                     r"\frac{\Delta F_z^f}{\Delta F_z^f+\Delta F_z^r}",
                                     rf"\frac{{{fmt(dfF)}}}{{{fmt(dfF)}+{fmt(dfR)}}}", dfF / (dfF + dfR), "-", S5,
                                     note=f"compare with static front weight fraction w_f = {wf:.3f}")
    else:
        res["lltd_front"] = float("nan")
    moment = calc.add("M_check", r"\sum \Delta F_z t", "Check: total overturning moment carried",
                      r"\Delta F_z^f t_f+\Delta F_z^r t_r",
                      rf"{fmt(dfF)}\cdot{fmt(tf)}+{fmt(dfR)}\cdot{fmt(tr)}", dfF * tf + dfR * tr, "N·m", S5)
    expected = mt * ay * h + (ms * g * h1 * phi if M.roll_gravity_term else 0.0)
    calc.add("M_expected", r"m\,a_y\,h\ (+m_s g h_1\phi)", "Check: overturning moment from CG",
             r"m\,a_y\,h" + (r"+m_s\,g\,h_1\,\phi" if M.roll_gravity_term else ""),
             rf"{fmt(mt)}\cdot{fmt(ay)}\cdot{fmt(h)}" +
             (rf"+{fmt(ms)}\cdot{fmt(g)}\cdot{paren(h1)}\cdot{paren(phi)}" if M.roll_gravity_term else ""),
             expected, "N·m", S5, note="must equal the line above (consistency check)")
    res["moment_check"] = (moment, expected)
    return res


def solve_roll(susp, ms, ay, g, h1, grav, lim=0.35):
    """Static roll equilibrium with nonlinear suspension moments."""
    from scipy.optimize import brentq
    if ay == 0:
        return 0.0
    f = lambda p: ms * ay * h1 + (ms * g * h1 * p if grav else 0.0) - susp["f"].static_moment(p) - susp["r"].static_moment(p)
    # fast path: bracket around the linearised estimate
    K0 = (susp["f"].static_moment(1e-4) + susp["r"].static_moment(1e-4)) / 1e-4 - (ms * g * h1 if grav else 0.0)
    if K0 > 0:
        pl = ms * ay * h1 / K0
        for lo_, hi_ in ((0.7 * pl, 1.3 * pl), (0.3 * pl, 1.8 * pl)):
            a_, b_ = min(lo_, hi_), max(lo_, hi_)
            if a_ != b_ and f(a_) * f(b_) <= 0:
                return brentq(f, a_, b_, xtol=1e-12)
    # search outward from 0 in the direction of the applied moment for a sign change
    sgn = 1.0 if ms * ay * h1 >= 0 else -1.0
    a, fa = 0.0, f(0.0)
    n = 70
    for k in range(1, n + 1):
        b = sgn * lim * k / n
        fb = f(b)
        if fa * fb <= 0:
            return brentq(f, min(a, b), max(a, b), xtol=1e-12)
        a, fa = b, fb
    return float("nan")


def record_rates(su, calc, tag, name):
    """Derivation of the linearised roll stiffness from spring, MR, ARB (returns N·m/rad)."""
    S = f"2a. {name} springs, motion ratio & ARB"
    ax = su.ax
    sp = ax.spring
    calc.add(f"{tag}.W_s", rf"W_{{s,{tag}}}", f"{name}: static sprung corner load", rf"\frac{{m_{{s,{tag}}}\,g}}{{2}}",
             rf"\frac{{{fmt(su.W * 2 / 9.81 if False else su.W * 2)}}}{{2}}", su.W, "N", S)
    mr0 = calc.add(f"{tag}.MR0", rf"MR_{{{tag}}}(0)", f"{name}: motion ratio at ride height",
                   r"c_0" if su.mr.mode == "poly" else r"\text{PCHIP}(z=0)", fmt(su.mr0), su.mr0, "-", S,
                   note="MR = d(spring travel)/d(wheel travel)")
    dmr0 = su.mr.dmr(0.0)
    calc.add(f"{tag}.dMR0", rf"MR'_{{{tag}}}(0)", f"{name}: motion-ratio slope at ride height",
             r"c_1" if su.mr.mode == "poly" else r"\frac{d\,\text{PCHIP}}{dz}(0)", fmt(dmr0), dmr0, "1/mm", S,
             note="> 0 = progressive (rising rate)")
    fs0 = calc.add(f"{tag}.Fs0", rf"F_{{s0,{tag}}}", f"{name}: spring preload (static equilibrium)",
                   rf"\frac{{W_{{s,{tag}}}}}{{MR(0)}}", rf"\frac{{{fmt(su.W)}}}{{{fmt(su.mr0)}}}", su.Fs0, "N", S)
    kw0 = su.wheel_rate(0.0)
    calc.add(f"{tag}.kw0", rf"k_{{w,{tag}}}(0)", f"{name}: wheel rate at ride height",
             r"k_s\,MR(0)^2+F_{s0}\,MR'(0)", rf"{fmt(su.ks)}\cdot{fmt(su.mr0)}^2+{fmt(fs0)}\cdot{paren(dmr0)}",
             kw0, "N/mm", S, ref="virtual work: F_w = F_s MR, k_w = dF_w/dz")
    calc.add(f"{tag}.karb", rf"k_{{ARB,{tag}}}", f"{name}: ARB rate at the wheel", r"\text{input}", fmt(su.karb),
             su.karb, "N/mm", S)
    K = calc.add(f"K_{tag}", rf"K_{{\phi,{tag}}}", f"{name}: linearised roll stiffness (at \u03c6 = 0)",
                 rf"\left(k_{{w,{tag}}}(0)+k_{{ARB,{tag}}}\right)\cdot1000\cdot\frac{{t_{tag}^2}}{{2}}",
                 rf"\left({fmt(kw0)}+{fmt(su.karb)}\right)\cdot1000\cdot\frac{{{fmt(su.t)}^2}}{{2}}",
                 su.linear_roll_stiffness(), "N·m/rad", S)
    calc.add(f"K_{tag}_deg", rf"K_{{\phi,{tag}}}", f"{name}: linearised roll stiffness", r"K_{\phi}\cdot\frac{\pi}{180}",
             rf"{fmt(K)}\cdot\frac{{\pi}}{{180}}", K * DEG, "N·m/deg", S)
    return K


def record_elastic(su, phi, calc, tag, name, S):
    """Elastic load transfer from the individual elements at roll angle phi."""
    c = su.components(phi)
    zo, zi = c["z_o"], c["z_i"]
    calc.add(f"{tag}.z_o", rf"z_{{o,{tag}}}", f"{name}: outer wheel travel (jounce +)",
             rf"\frac{{t_{tag}}}{{2}}\cdot1000\cdot\phi", rf"\frac{{{fmt(su.t)}}}{{2}}\cdot1000\cdot{paren(phi)}", zo, "mm", S)
    for side, z in (("o", zo), ("i", zi)):
        mr, xs, Fs = su.mr.mr(z), su.mr.xs(z), su.spring_force(z)
        calc.add(f"{tag}.Fw_{side}", rf"F_{{w,{side}}}^{{{tag}}}", f"{name}: {'outer' if side == 'o' else 'inner'} spring force at wheel",
                 r"\left[F_{s0}+k_s\,x_s(z)\right]MR(z),\ x_s=\int_0^z MR\,dz",
                 rf"\left[{fmt(su.Fs0)}+{fmt(su.ks)}\cdot{paren(xs)}\right]\cdot{fmt(mr)}", Fs * mr, "N", S,
                 note=f"z = {z:.3f} mm, MR(z) = {mr:.5f}, x_s = {xs:.4f} mm")
    Fo, Fi = su.spring_wheel(zo), su.spring_wheel(zi)
    sp = calc.add(f"{tag}.dFz_e_spring", rf"\Delta F_{{z,spring}}^{{{tag}}}", f"{name}: elastic LT via springs",
                  r"\frac{F_{w,o}-F_{w,i}}{2}", rf"\frac{{{fmt(Fo)}-{fmt(Fi)}}}{{2}}", c["spring"], "N", S)
    gap = su.gap
    bp = calc.add(f"{tag}.dFz_e_bump", rf"\Delta F_{{z,bump}}^{{{tag}}}", f"{name}: elastic LT via bump stops",
                  r"\frac{k_b\,(z_o-g_b)^+-k_b\,(z_i-g_b)^+}{2}",
                  (rf"\frac{{{fmt(su.kb)}\cdot({fmt(zo)}-{fmt(gap)})^+-{fmt(su.kb)}\cdot({paren(zi)}-{fmt(gap)})^+}}{{2}}"
                   if gap is not None else r"0\ (\text{no bump stop})"), c["bump"], "N", S)
    ab = calc.add(f"{tag}.dFz_e_arb", rf"\Delta F_{{z,ARB}}^{{{tag}}}", f"{name}: elastic LT via ARB",
                  r"k_{ARB}\,\frac{z_o-z_i}{2}", rf"{fmt(su.karb)}\cdot\frac{{{fmt(zo)}-{paren(zi)}}}{{2}}", c["arb"], "N", S)
    de = calc.add(f"{tag}.dFz_e", rf"\Delta F_{{z,e}}^{{{tag}}}", f"{name}: elastic load transfer",
                  r"\Delta F_{z,spring}+\Delta F_{z,bump}+\Delta F_{z,ARB}", rf"{fmt(sp)}+{paren(bp)}+{paren(ab)}",
                  sp + bp + ab, "N", S, note="transmitted by springs, bump stops and ARB; requires body roll")
    if phi != 0:
        calc.add(f"K_{tag}_eff", rf"K_{{\phi,{tag}}}^{{eff}}", f"{name}: effective (secant) roll stiffness at \u03c6",
                 rf"\frac{{\Delta F_{{z,e}}^{{{tag}}}\,t_{tag}}}{{\phi}}\cdot\frac{{\pi}}{{180}}",
                 rf"\frac{{{fmt(de)}\cdot{fmt(su.t)}}}{{{paren(phi)}}}\cdot\frac{{\pi}}{{180}}", de * su.t / phi * DEG,
                 "N·m/deg", S)
    return de, {"spring": sp, "bump": bp, "arb": ab}
