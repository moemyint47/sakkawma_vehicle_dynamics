"""Derived kinematic numbers with derivations (static RC, gains, migration)."""
from __future__ import annotations

from . import kinematics2d as k2
from .derivation import Calc, fmt, paren
from .schemas import ModelIn


def compute(m: ModelIn, calc: Calc) -> dict:
    res = {}
    for tag, name in (("front", "Front"), ("rear", "Rear")):
        ax = getattr(m, tag)
        hp, t = ax.hardpoints, ax.track_mm
        S = f"9. {name} kinematics (front view)"
        s0 = k2.solve_pose(hp, hp, t)
        if not s0["ok"]:
            calc.warn(f"{name} kinematics: {s0.get('error')}")
            continue
        o = s0["sides"]["o"]
        L = o["lengths"]
        calc.add(f"k{tag}.L_lca", r"L_{LCA}", f"{name}: LCA length (front view)",
                 r"\left|P_{LBJ}-P_{LCA,in}\right|",
                 rf"\sqrt{{({fmt(hp.lca_out.y)}-{paren(hp.lca_in.y)})^2+({fmt(hp.lca_out.z)}-{paren(hp.lca_in.z)})^2}}",
                 L["lca"], "mm", S)
        calc.add(f"k{tag}.L_uca", r"L_{UCA}", f"{name}: UCA length (front view)",
                 r"\left|P_{UBJ}-P_{UCA,in}\right|",
                 rf"\sqrt{{({fmt(hp.uca_out.y)}-{paren(hp.uca_in.y)})^2+({fmt(hp.uca_out.z)}-{paren(hp.uca_in.z)})^2}}",
                 L["uca"], "mm", S)
        ic = o["ic"]
        cp = o["points"]["cp"]
        if ic is not None:
            calc.add(f"k{tag}.ic_y", r"y_{IC}", f"{name}: instant centre lateral position",
                     r"\text{intersection of LCA and UCA lines}", r"\text{(solved 2}\times\text{2 linear system)}",
                     ic[0], "mm", S, note="+ = outboard of the centreline on the same side")
            calc.add(f"k{tag}.ic_z", r"z_{IC}", f"{name}: instant centre height",
                     r"\text{intersection of LCA and UCA lines}", r"\text{(solved 2}\times\text{2 linear system)}",
                     ic[1], "mm", S)
            fvsa = calc.add(f"k{tag}.fvsa", r"l_{FVSA}", f"{name}: front-view swing-arm length",
                            r"\sqrt{(y_{CP}-y_{IC})^2+(z_{CP}-z_{IC})^2}",
                            rf"\sqrt{{({fmt(cp[0])}-{paren(ic[0])})^2+({fmt(cp[1])}-{paren(ic[1])})^2}}",
                            ((cp[0] - ic[0]) ** 2 + (cp[1] - ic[1]) ** 2) ** 0.5, "mm", S)
            hrc = calc.add(f"k{tag}.h_rc", r"h_{RC}", f"{name}: static roll-centre height (kinematic)",
                           r"z_{IC}\,\frac{y_{CP}}{y_{CP}-y_{IC}}",
                           rf"{paren(ic[1])}\cdot\frac{{{fmt(cp[0])}}}{{{fmt(cp[0])}-{paren(ic[0])}}}",
                           ic[1] * cp[0] / (cp[0] - ic[0]), "mm", S,
                           note="line CP–IC evaluated at the centreline (symmetric, static)")
        else:
            hrc = calc.add(f"k{tag}.h_rc", r"h_{RC}", f"{name}: static roll-centre height (kinematic)",
                           r"\text{parallel arms: line through CP parallel to arms}", r"-",
                           s0["rc_height_mm"], "mm", S)
        hv = k2.symmetric_rc_height_by_velocity(hp, t)
        calc.add(f"k{tag}.h_rc_check", r"h_{RC}", f"{name}: RC height, independent check",
                 r"\frac{t}{2}\,\frac{dy_{CP}}{dz_{CP}}", r"\text{virtual-velocity method}", hv, "mm", S,
                 note="must equal the line above")
        if abs(hrc - ax.h_rc_mm) > 2:
            calc.warn(f"{name}: kinematic RC height ({hrc:.1f} mm) differs from the load-transfer input "
                      f"({ax.h_rc_mm:.1f} mm). Use 'Copy kinematic RC' to align them.")
        d = 5.0
        hp_sw = k2.sweep(hp, hp, t, "heave", [-d, 0, d])
        c_lo, c_hi = hp_sw["camber_o_deg"][0], hp_sw["camber_o_deg"][2]
        if None not in (c_lo, c_hi):
            calc.add(f"k{tag}.camber_gain_heave", r"\partial\gamma/\partial z", f"{name}: camber gain in heave",
                     r"\frac{\gamma(+5\,mm)-\gamma(-5\,mm)}{10\,mm}", rf"\frac{{{fmt(c_hi)}-{paren(c_lo)}}}{{10}}",
                     (c_hi - c_lo) / (2 * d), "deg/mm", S, note="negative = camber goes negative in jounce")
            rh_lo, rh_hi = hp_sw["rc_height_mm"][0], hp_sw["rc_height_mm"][2]
            calc.add(f"k{tag}.rc_heave_gain", r"\partial h_{RC}/\partial z", f"{name}: RC height change in heave",
                     r"\frac{h_{RC}(+5\,mm)-h_{RC}(-5\,mm)}{10\,mm}", rf"\frac{{{fmt(rh_hi)}-{paren(rh_lo)}}}{{10}}",
                     (rh_hi - rh_lo) / (2 * d), "mm/mm", S)
        e = 1.0
        rl = k2.sweep(hp, hp, t, "roll", [-e, 0, e])
        if None not in rl["camber_o_deg"]:
            co = rl["camber_o_deg"]
            calc.add(f"k{tag}.roll_camber_o", r"\partial\gamma_o/\partial\phi", f"{name}: outer camber change per deg roll",
                     r"\frac{\gamma_o(+1^\circ)-\gamma_o(-1^\circ)}{2^\circ}", rf"\frac{{{fmt(co[2])}-{paren(co[0])}}}{{2}}",
                     (co[2] - co[0]) / 2, "deg/deg", S,
                     note="0 = perfect camber compensation, 1 = wheel leans fully with the body")
            ry = rl["rc_lateral_mm"]
            calc.add(f"k{tag}.rc_lat_mig", r"\partial y_{RC}/\partial\phi", f"{name}: RC lateral migration per deg roll",
                     r"\frac{y_{RC}(+1^\circ)-y_{RC}(-1^\circ)}{2^\circ}", rf"\frac{{{fmt(ry[2])}-{paren(ry[0])}}}{{2}}",
                     (ry[2] - ry[0]) / 2, "mm/deg", S, note="+ = RC moves toward the outer wheel")
            rz = rl["rc_height_mm"]
            calc.add(f"k{tag}.rc_roll_h", r"h_{RC}(1^\circ)", f"{name}: RC height at 1 deg roll",
                     r"\text{line intersection CP}_o\text{IC}_o\cap\text{CP}_i\text{IC}_i", r"\text{(solved)}",
                     rz[2], "mm", S)
        res[tag] = {"h_rc_mm": hrc}
    return res
