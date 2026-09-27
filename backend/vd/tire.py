"""TMeasy lateral tire characteristic (steady state, pure lateral slip).

Source: G. Rill, *Road Vehicle Dynamics - Fundamentals and Modeling with
MATLAB*, Sec. 3.5-3.7:
  * Listing 3.3  (tmy_fcombined)  - shape of the force characteristic
  * Eq. 3.121    - degressive (quadratic) wheel-load influence on dF0, F_M, F_S
  * Eq. 3.122    - linear wheel-load influence on slip locations s_M, s_S
  * Eq. 3.72     - lateral slip s_y = tan(alpha)

The interface is deliberately small:  ``fy(Fz, alpha)``.  Camber, combined
slip and relaxation can be added later without changing callers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .derivation import Calc, NullCalc, fmt, paren
from .schemas import TireIn

DEG = math.pi / 180.0


@dataclass
class TireAtLoad:
    Fz: float
    dF0: float   # N per unit slip
    sM: float
    FM: float
    sS: float
    FS: float


def _quad(Fz, FzN, yN, y2N):
    """Eq. 3.121 - quadratic through (0,0), (FzN,yN), (2FzN,y2N)."""
    r = Fz / FzN
    return r * (2 * yN - 0.5 * y2N - (yN - 0.5 * y2N) * r)


def _lin(Fz, FzN, xN, x2N):
    """Eq. 3.122 - linear inter/extrapolation of slip locations."""
    return xN + (x2N - xN) * (Fz / FzN - 1)


def nominal_sets(t: TireIn):
    """Return (N-set, 2N-set) in TMeasy units (dF0 [N/-], s [-], F [N])."""
    n = dict(dF0=t.C_alpha_nom_N_deg / DEG, sM=math.tan(t.alpha_M_nom_deg * DEG),
             FM=t.F_M_nom_N, sS=math.tan(t.alpha_S_nom_deg * DEG), FS=t.F_S_nom_N)
    if t.has_double_load:
        d = dict(dF0=t.C_alpha_2nom_N_deg / DEG, sM=math.tan(t.alpha_M_2nom_deg * DEG),
                 FM=t.F_M_2nom_N, sS=math.tan(t.alpha_S_2nom_deg * DEG), FS=t.F_S_2nom_N)
    else:  # linear load scaling of forces, constant slip locations
        d = dict(dF0=2 * n["dF0"], sM=n["sM"], FM=2 * n["FM"], sS=n["sS"], FS=2 * n["FS"])
    return n, d


def record_tire_inputs(t: TireIn, calc: Calc, section="6. Tire parameters (TMeasy)") -> None:
    """Log the conversion of user tire inputs into TMeasy parameters."""
    n, d = nominal_sets(t)
    calc.add("tire.dF0_N", r"dF_{0}^{N}", "Initial inclination at F_z^N",
             r"C_{\alpha}^{N}\cdot\frac{180}{\pi}",
             rf"{fmt(t.C_alpha_nom_N_deg)}\cdot\frac{{180}}{{\pi}}", n["dF0"], "N", section,
             ref="dF_y/ds_y = dF_y/dα at s_y = tanα ≈ α (Rill eq. 3.72)")
    calc.add("tire.sM_N", r"s_{M}^{N}", "Slip at peak force, F_z^N",
             r"\tan\alpha_{M}^{N}", rf"\tan({fmt(t.alpha_M_nom_deg)}^\circ)", n["sM"], "-", section)
    calc.add("tire.sS_N", r"s_{S}^{N}", "Slip at full sliding, F_z^N",
             r"\tan\alpha_{S}^{N}", rf"\tan({fmt(t.alpha_S_nom_deg)}^\circ)", n["sS"], "-", section)
    if t.has_double_load:
        calc.add("tire.dF0_2N", r"dF_{0}^{2N}", "Initial inclination at 2F_z^N",
                 r"C_{\alpha}^{2N}\cdot\frac{180}{\pi}",
                 rf"{fmt(t.C_alpha_2nom_N_deg)}\cdot\frac{{180}}{{\pi}}", d["dF0"], "N", section)
        calc.add("tire.sM_2N", r"s_{M}^{2N}", "Slip at peak force, 2F_z^N",
                 r"\tan\alpha_{M}^{2N}", rf"\tan({fmt(t.alpha_M_2nom_deg)}^\circ)", d["sM"], "-", section)
        calc.add("tire.sS_2N", r"s_{S}^{2N}", "Slip at full sliding, 2F_z^N",
                 r"\tan\alpha_{S}^{2N}", rf"\tan({fmt(t.alpha_S_2nom_deg)}^\circ)", d["sS"], "-", section)
    else:
        calc.warn("Tire: no double-load data given - forces scale linearly with F_z "
                  "(no degressive load sensitivity). Provide 2F_z^N data for realistic load transfer effects.")


def params_at(Fz: float, t: TireIn, calc: Calc | None = None, key="tire",
              section="Tire (TMeasy)") -> TireAtLoad:
    """Load-dependent TMeasy parameters at wheel load Fz (with derivation)."""
    calc = calc or NullCalc()
    n, d = nominal_sets(t)
    FzN = t.Fz_nom_N
    Fzc = max(Fz, 0.0)
    r = Fzc / FzN
    out = {}
    for name, sym in (("dF0", "dF_0"), ("FM", "F_M"), ("FS", "F_S")):
        v = _quad(Fzc, FzN, n[name], d[name])
        out[name] = v
        calc.add(f"{key}.{name}", rf"{sym}(F_z)", f"{sym} at F_z (eq. 3.121)",
                 rf"\frac{{F_z}}{{F_z^N}}\left[2{sym}^N-\tfrac12 {sym}^{{2N}}-\left({sym}^N-\tfrac12 {sym}^{{2N}}\right)\frac{{F_z}}{{F_z^N}}\right]",
                 rf"{fmt(r)}\left[2\cdot{fmt(n[name])}-\tfrac12\cdot{fmt(d[name])}-\left({fmt(n[name])}-\tfrac12\cdot{fmt(d[name])}\right)\cdot{fmt(r)}\right]",
                 v, "N", section, ref="Rill eq. 3.121")
    for name, sym in (("sM", "s_M"), ("sS", "s_S")):
        v = _lin(Fzc, FzN, n[name], d[name])
        out[name] = v
        calc.add(f"{key}.{name}", rf"{sym}(F_z)", f"{sym} at F_z (eq. 3.122)",
                 rf"{sym}^N+\left({sym}^{{2N}}-{sym}^N\right)\left(\frac{{F_z}}{{F_z^N}}-1\right)",
                 rf"{fmt(n[name])}+\left({fmt(d[name])}-{fmt(n[name])}\right)\left({fmt(r)}-1\right)",
                 v, "-", section, ref="Rill eq. 3.122")
    if Fz <= 0:
        calc.warn(f"{key}: wheel load ≤ 0 (wheel lift) - tire force set to zero.")
    return TireAtLoad(Fz=Fzc, dF0=max(out["dF0"], 0.0), sM=max(out["sM"], 1e-6),
                      FM=max(out["FM"], 0.0), sS=max(out["sS"], 1e-6), FS=max(out["FS"], 0.0))


def force_from_slip(s: float, p: TireAtLoad) -> float:
    """TMeasy characteristic F(s) - Rill Listing 3.3 (1-D, s >= 0 handled by sign)."""
    sign = 1.0 if s >= 0 else -1.0
    s = abs(s)
    df0, fm, sm, fs, ss = p.dF0, p.FM, p.sM, p.FS, p.sS
    if df0 <= 0 or fm <= 0 or s == 0:
        return 0.0
    smloc = max(2.0 * fm / df0, sm)
    ssloc = ss + (smloc - sm)
    if s > ssloc:
        f = fs
    elif s < smloc:
        pp = df0 * smloc / fm - 2.0
        sn = s / smloc
        f = df0 / (1.0 + (sn + pp) * sn) * s
    else:
        a = (fm / smloc) ** 2 / (df0 * smloc)
        sstar = smloc + (fm - fs) / (a * (ssloc - smloc)) if ssloc > smloc else smloc
        if sstar <= ss:
            if s <= sstar:
                f = fm - a * (s - smloc) ** 2
            else:
                b = a * (sstar - smloc) / (ssloc - sstar)
                f = fs + b * (ssloc - s) ** 2
        else:
            sn = (s - smloc) / (ssloc - smloc)
            f = fm - (fm - fs) * sn * sn * (3.0 - 2.0 * sn)
    return sign * f


def fy(Fz: float, alpha_rad: float, t: TireIn) -> float:
    """Lateral force [N] for wheel load Fz [N] and slip angle alpha [rad]."""
    return force_from_slip(math.tan(alpha_rad), params_at(Fz, t))


def fy_derivation(Fz: float, alpha_rad: float, t: TireIn, calc: Calc, key: str,
                  label: str, section: str) -> float:
    """Evaluate F_y with the full derivation logged (branch of Listing 3.3)."""
    p = params_at(Fz, t, calc, key, section)
    s = math.tan(abs(alpha_rad))
    calc.add(f"{key}.s", r"s_y", f"{label}: lateral slip", r"\tan\alpha",
             rf"\tan({fmt(alpha_rad / DEG)}^\circ)", s, "-", section, ref="Rill eq. 3.72")
    df0, fm, sm, fs, ss = p.dF0, p.FM, p.sM, p.FS, p.sS
    F = force_from_slip(s, p)
    if df0 <= 0 or fm <= 0:
        calc.add(f"{key}.Fy", r"F_y", f"{label}: lateral force", "0", "0", 0.0, "N", section)
        return 0.0
    smloc = max(2.0 * fm / df0, sm)
    ssloc = ss + (smloc - sm)
    if smloc != sm:
        calc.add(f"{key}.smloc", r"s_M^{*}", f"{label}: adjusted peak slip",
                 r"\max\left(\frac{2F_M}{dF_0},\ s_M\right)",
                 rf"\max\left(\frac{{2\cdot{fmt(fm)}}}{{{fmt(df0)}}},\ {fmt(sm)}\right)", smloc, "-",
                 section, note="dF0 < 2F_M/s_M would create a turning point; Listing 3.3 shifts s_M, s_S")
    if s > ssloc:
        calc.add(f"{key}.Fy", r"F_y", f"{label}: lateral force (full sliding, s > s_S)",
                 r"F_S", fmt(fs), F, "N", section, ref="Rill Listing 3.3")
    elif s < smloc:
        pp = df0 * smloc / fm - 2.0
        calc.add(f"{key}.Fy", r"F_y", f"{label}: lateral force (adhesion, s < s_M)",
                 r"\frac{dF_0\, s}{1+\frac{s}{s_M}\left(\frac{s}{s_M}+\frac{dF_0 s_M}{F_M}-2\right)}",
                 rf"\frac{{{fmt(df0)}\cdot{fmt(s)}}}{{1+{fmt(s / smloc)}\left({fmt(s / smloc)}+{fmt(pp)}\right)}}",
                 F, "N", section, ref="Rill Listing 3.3")
    else:
        a = (fm / smloc) ** 2 / (df0 * smloc)
        sstar = smloc + (fm - fs) / (a * (ssloc - smloc)) if ssloc > smloc else smloc
        if sstar <= ss and s <= sstar:
            calc.add(f"{key}.Fy", r"F_y", f"{label}: lateral force (transition, parabola 1)",
                     r"F_M-a(s-s_M)^2,\quad a=\frac{(F_M/s_M)^2}{dF_0 s_M}",
                     rf"{fmt(fm)}-{fmt(a)}\cdot({fmt(s)}-{fmt(smloc)})^2", F, "N", section,
                     ref="Rill Listing 3.3")
        elif sstar <= ss:
            b = a * (sstar - smloc) / (ssloc - sstar)
            calc.add(f"{key}.Fy", r"F_y", f"{label}: lateral force (transition, parabola 2)",
                     r"F_S+b(s_S-s)^2", rf"{fmt(fs)}+{fmt(b)}\cdot({fmt(ssloc)}-{fmt(s)})^2",
                     F, "N", section, ref="Rill Listing 3.3")
        else:
            sn = (s - smloc) / (ssloc - smloc)
            calc.add(f"{key}.Fy", r"F_y", f"{label}: lateral force (transition, cubic)",
                     r"F_M-(F_M-F_S)\sigma^2(3-2\sigma),\ \sigma=\frac{s-s_M}{s_S-s_M}",
                     rf"{fmt(fm)}-({fmt(fm)}-{fmt(fs)})\cdot{fmt(sn)}^2(3-2\cdot{fmt(sn)})",
                     F, "N", section, ref="Rill Listing 3.3")
    return F


def curves(t: TireIn, loads: list[float], alpha_max_deg: float = 25.0, n: int = 101):
    """F_y(alpha) curves at several wheel loads (for plotting)."""
    alphas = [alpha_max_deg * i / (n - 1) for i in range(n)]
    out = []
    for Fz in loads:
        p = params_at(Fz, t)
        out.append({"Fz": Fz, "alpha_deg": alphas,
                    "Fy": [force_from_slip(math.tan(a * DEG), p) for a in alphas]})
    return out


def load_curves(t: TireIn, alphas_deg: list[float], Fz_max: float, n: int = 81):
    """F_y(F_z) at several fixed slip angles - shows degressive load influence."""
    loads = [Fz_max * i / (n - 1) for i in range(n)]
    out = []
    for a in alphas_deg:
        out.append({"alpha_deg": a, "Fz": loads,
                    "Fy": [force_from_slip(math.tan(a * DEG), params_at(F, t)) for F in loads]})
    return out
