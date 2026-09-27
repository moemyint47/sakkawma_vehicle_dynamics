"""Suspension force elements per axle: spring through a (progressive) motion
ratio, bump stop, anti-roll bar and damper.

Conventions (per corner):
    z      wheel travel relative to the body, mm, + = jounce (bump)
    MR(z)  motion ratio = d(spring travel) / d(wheel travel)       [-]
    x_s(z) spring travel = integral_0^z MR dz                        [mm]
    F_s    spring force = F_s0 + k_s x_s                             [N]
    F_w    vertical force at the wheel = F_s * MR  (virtual work)    [N]
    k_w    wheel rate  = dF_w/dz = k_s MR^2 + F_s dMR/dz             [N/mm]
The preload F_s0 is solved so that F_w(0) equals the static sprung corner load.

Roll (roll-only model, rigid chassis): outer wheel z_o = +(t/2) phi,
inner wheel z_i = -(t/2) phi. Per-wheel elastic load transfer of an axle is
    dF = (F_o - F_i) / 2           and the roll moment is  M = dF * t.
Net vertical force from asymmetric elements (progressive springs, bump/rebound
damping) would cause heave; it is reacted in this roll-only model.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.interpolate import PchipInterpolator

from .schemas import AxleIn, DamperIn, SpringIn


class MRCurve:
    def __init__(self, sp: SpringIn):
        self.mode = sp.mr_mode
        if self.mode == "table" and sp.mr_table and len(sp.mr_table) >= 2:
            pts = sorted((float(a), float(b)) for a, b in sp.mr_table)
            z = np.array([p[0] for p in pts])
            m = np.array([p[1] for p in pts])
            if np.any(np.diff(z) <= 0):
                raise ValueError("MR table: wheel-travel values must be distinct")
            self._p = PchipInterpolator(z, m, extrapolate=True)
            self._dp = self._p.derivative()
            anti = self._p.antiderivative()
            self._x = lambda zz: float(anti(zz) - anti(0.0))
        else:
            self.mode = "poly"
            c0, c1, c2 = sp.mr_c0, sp.mr_c1, sp.mr_c2
            self.c = (c0, c1, c2)
            self._p = lambda zz: c0 + c1 * zz + c2 * zz * zz
            self._dp = lambda zz: c1 + 2 * c2 * zz
            self._x = lambda zz: c0 * zz + c1 * zz ** 2 / 2 + c2 * zz ** 3 / 3

    def mr(self, z):
        return float(self._p(z))

    def dmr(self, z):
        return float(self._dp(z))

    def xs(self, z):
        return float(self._x(z))


class Damper:
    def __init__(self, d: DamperIn):
        self.d = d
        self.table = None
        if d.use_table and d.table and len(d.table) >= 2:
            pts = sorted((float(a), float(b)) for a, b in d.table)
            self.tv = np.array([p[0] for p in pts])
            self.tf = np.array([p[1] for p in pts])
            self.table = True

    def force(self, v_mm_s: float) -> float:
        """Damper force [N] at damper velocity [mm/s]; + = bump."""
        if self.table:
            v, f = self.tv, self.tf
            if v_mm_s <= v[0]:
                s = (f[1] - f[0]) / (v[1] - v[0])
                return float(f[0] + s * (v_mm_s - v[0]))
            if v_mm_s >= v[-1]:
                s = (f[-1] - f[-2]) / (v[-1] - v[-2])
                return float(f[-1] + s * (v_mm_s - v[-1]))
            return float(np.interp(v_mm_s, v, f))
        d = self.d
        if v_mm_s >= 0:
            cl, ch, vk = d.c_ls_bump, d.c_hs_bump, d.v_knee_bump_mm_s
        else:
            cl, ch, vk = d.c_ls_reb, d.c_hs_reb, d.v_knee_reb_mm_s
        a = abs(v_mm_s)
        f = cl * a / 1000 if a <= vk else (cl * vk + ch * (a - vk)) / 1000
        return math.copysign(f, v_mm_s) if v_mm_s != 0 else 0.0


class AxleSuspension:
    """Force elements of one axle; forces in N, travels in mm, t in m."""

    def __init__(self, ax: AxleIn, m_s_axle: float, g: float):
        self.ax = ax
        self.t = ax.track_mm / 1000.0
        self.direct = ax.roll_stiffness_source == "direct"
        self.K_direct = ax.roll_stiffness_Nm_deg * 180 / math.pi  # N·m/rad
        self.W = m_s_axle * g / 2.0  # static sprung corner load
        self.mr = MRCurve(ax.spring)
        self.ks = ax.spring.rate_N_mm
        self.mr0 = self.mr.mr(0.0)
        if self.mr0 <= 0:
            raise ValueError("Motion ratio at z = 0 must be > 0")
        self.Fs0 = self.W / self.mr0
        self.gap = ax.spring.bump_gap_mm
        self.kb = ax.spring.bump_rate_N_mm
        self.karb = ax.arb_rate_N_mm
        self.damper = Damper(ax.damper)

    # ---- corner elements (wheel forces, N)
    def spring_force(self, z):
        return max(self.Fs0 + self.ks * self.mr.xs(z), 0.0)

    def spring_wheel(self, z):
        return self.spring_force(z) * self.mr.mr(z)

    def wheel_rate(self, z):
        Fs = self.spring_force(z)
        return self.ks * self.mr.mr(z) ** 2 + Fs * self.mr.dmr(z) if Fs > 0 else 0.0

    def bump_wheel(self, z):
        if self.gap is None or z <= self.gap:
            return 0.0
        return self.kb * (z - self.gap)

    def damper_mr(self, z):
        return self.mr.mr(z) if self.ax.damper.mr_mode == "spring" else self.ax.damper.mr_const

    def damper_wheel(self, z, zdot_mm_s):
        m = self.damper_mr(z)
        return self.damper.force(m * zdot_mm_s) * m

    # ---- axle load-transfer components in roll (per wheel, N)
    def components(self, phi, phidot=0.0):
        h = self.t * 1000 / 2  # mm
        zo, zi = h * phi, -h * phi
        vo, vi = h * phidot, -h * phidot
        if self.direct:
            sp, bp, arb = self.K_direct * phi / self.t, 0.0, 0.0
        else:
            sp = (self.spring_wheel(zo) - self.spring_wheel(zi)) / 2
            bp = (self.bump_wheel(zo) - self.bump_wheel(zi)) / 2
            arb = self.karb * (zo - zi) / 2
        dp = (self.damper_wheel(zo, vo) - self.damper_wheel(zi, vi)) / 2
        return {"spring": sp, "bump": bp, "arb": arb, "damper": dp, "z_o": zo, "z_i": zi,
                "v_o": vo, "v_i": vi}

    def static_moment(self, phi):
        c = self.components(phi, 0.0)
        return (c["spring"] + c["bump"] + c["arb"]) * self.t

    def linear_roll_stiffness(self):
        """dM/dphi at phi = 0 [N·m/rad]."""
        if self.direct:
            return self.K_direct
        kw = self.wheel_rate(0.0)  # N/mm = kN/m
        return (kw + self.karb) * 1000 * self.t ** 2 / 2

    # ---- curves for plotting
    def curves(self, zmin=-40.0, zmax=40.0, n=81):
        zs = np.linspace(zmin, zmax, n)
        out = {"z": zs.tolist(), "mr": [], "xs": [], "Fs": [], "Fw": [], "kw": [], "Fb": [], "mr_d": []}
        for z in zs:
            out["mr"].append(self.mr.mr(z))
            out["xs"].append(self.mr.xs(z))
            out["Fs"].append(self.spring_force(z))
            out["Fw"].append(self.spring_wheel(z) + self.bump_wheel(z))
            kb = self.kb if (self.gap is not None and z > self.gap) else 0.0
            out["kw"].append(self.wheel_rate(z) + kb)
            out["Fb"].append(self.bump_wheel(z))
            out["mr_d"].append(self.damper_mr(z))
        return out

    def roll_curves(self, phimax_deg=4.0, n=81):
        ph = np.linspace(-phimax_deg, phimax_deg, n)
        M, K = [], []
        e = 1e-4
        for p in ph:
            r = math.radians(p)
            M.append(self.static_moment(r))
            K.append((self.static_moment(r + e) - self.static_moment(r - e)) / (2 * e) * math.pi / 180)
        return {"phi_deg": ph.tolist(), "M": M, "K_Nm_deg": K}

    def damper_curve(self, vmax=400.0, n=161):
        vs = np.linspace(-vmax, vmax, n)
        return {"v": vs.tolist(), "F": [self.damper.force(v) for v in vs]}
