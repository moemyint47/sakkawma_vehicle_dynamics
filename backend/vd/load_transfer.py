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

DEG = math.pi / 180.0


def compute(m: ModelIn, calc: Calc | None = None, ay_g: float | None = None) -> dict:
    calc = calc or NullCalc()
    V, F, R, M = m.vehicle, m.front, m.rear, m.maneuver
    S0, S1 = "1. Mass distribution", "2. Body roll"
    g = M.g
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
    Kf = calc.add("K_f", r"K_{\phi,f}", "Front roll stiffness", r"K_{\phi,f,[Nm/^\circ]}\cdot\frac{180}{\pi}",
                  rf"{fmt(F.roll_stiffness_Nm_deg)}\cdot\frac{{180}}{{\pi}}",
                  F.roll_stiffness_Nm_deg / DEG, "N·m/rad", S1)
    Kr = calc.add("K_r", r"K_{\phi,r}", "Rear roll stiffness", r"K_{\phi,r,[Nm/^\circ]}\cdot\frac{180}{\pi}",
                  rf"{fmt(R.roll_stiffness_Nm_deg)}\cdot\frac{{180}}{{\pi}}",
                  R.roll_stiffness_Nm_deg / DEG, "N·m/rad", S1)
    if M.roll_gravity_term:
        den = Kf + Kr - ms * g * h1
        if den <= 0:
            calc.warn("Roll stiffness too low: K_f + K_r ≤ m_s g h1 (statically unstable in roll).")
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
    phi_deg = calc.add("phi_deg", r"\phi", "Body roll angle", r"\phi\cdot\frac{180}{\pi}",
                       rf"{fmt(phi)}\cdot\frac{{180}}{{\pi}}", phi / DEG, "deg", S1)
    if ay_g > 0:
        calc.add("roll_gradient", r"\partial\phi/\partial a_y", "Roll gradient",
                 r"\phi/a_{y,[g]}", rf"{fmt(phi_deg)}/{fmt(ay_g)}", phi_deg / ay_g, "deg/g", S1)

    res = {"ay": ay, "phi": phi, "phi_deg": phi_deg, "h1": h1, "hs": hs, "ms": ms, "l": l,
           "m_f": mf, "m_r": mr, "axles": {}}
    for tag, AX, t, m_ax, ms_ax, mu, hu, hrc, K in (
            ("f", F, tf, mf, msf, muf, huf, hrf, Kf), ("r", R, tr, mr, msr, mur, hur, hrr, Kr)):
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
        de = calc.add(f"{tag}.dFz_e", rf"\Delta F_{{z,e}}^{{{tag}}}", f"{name}: elastic load transfer",
                      rf"\frac{{K_{{\phi,{tag}}}\,\phi}}{{t_{tag}}}",
                      rf"\frac{{{fmt(K)}\cdot{paren(phi)}}}{{{fmt(t)}}}",
                      K * phi / t, "N", S, note="transmitted by springs/ARB, requires body roll")
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
        res["axles"][tag] = dict(dFz_u=du, dFz_g=dg, dFz_e=de, dFz=dt, geo_share=geo_share,
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
