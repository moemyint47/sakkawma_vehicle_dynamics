"""Bell-crank (rocker) kinematics in the front view -> motion ratio MR(z).

Linkage per corner (side coordinates, mm, y outward, z up; static design pose):
    pushrod outboard end  P  : fixed to the LCA (rotates about the LCA inboard pivot)
                               or to the upright (moves with the upright)
    rocker pivot          O  : chassis
    rocker pushrod joint  A  : on the rocker
    rocker spring joint   B  : on the rocker
    spring chassis mount  C  : chassis
For a wheel-centre travel z (+ jounce):
    1. LCA angle theta(z) from the double-wishbone four-bar (same solver as the kinematics)
    2. P(z) moved with its member
    3. rocker angle beta: |O + R(beta)(A0-O) - P(z)| = |A0 - P0|   (pushrod is rigid)
    4. B(z) = O + R(beta)(B0-O);  spring length L_s(z) = |B(z) - C|
    5. spring travel x_s(z) = L_s(0) - L_s(z)  (compression +),  MR(z) = dx_s/dz
MR(z) is sampled every 2 mm and fitted with a C2 cubic spline (so the wheel rate
k_s MR^2 + F_s dMR/dz is continuous).
"""
from __future__ import annotations

import math
from functools import lru_cache

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq

from . import kinematics2d as k2

Z_GRID = np.arange(-50.0, 50.0 + 1e-9, 2.0)


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


class BellCrank:
    def __init__(self, hp, bc, track_mm: float):
        self.geo = k2.SideGeom.from_hardpoints(hp, track_mm)
        P = lambda p: np.array([p.y, p.z], dtype=float)
        self.on = bc.pushrod_on
        self.P0, self.O, self.A0, self.B0, self.C = (P(bc.pushrod_out), P(bc.rocker_pivot), P(bc.rocker_pushrod),
                                                    P(bc.rocker_spring), P(bc.spring_mount))
        self.Lpr = float(np.linalg.norm(self.A0 - self.P0))
        self.Ls0 = float(np.linalg.norm(self.B0 - self.C))
        if self.Lpr < 1 or self.Ls0 < 1:
            raise ValueError("Bell-crank: pushrod or spring length is zero")

    def theta_for_travel(self, z):
        g = self.geo
        wc0 = g.wc0[1]
        f = lambda th: (g.pose(th)["wc"][1] - wc0 - z) if g.pose(th) is not None else float("nan")
        a, b = g.theta0 - 0.5, g.theta0 + 0.5
        grid = np.linspace(a, b, 101)
        vals = [f(t) for t in grid]
        best = None
        for i in range(100):
            va, vb = vals[i], vals[i + 1]
            if not (np.isfinite(va) and np.isfinite(vb)):
                continue
            if va == 0 or va * vb < 0:
                r = grid[i] if va == 0 else brentq(f, grid[i], grid[i + 1], xtol=1e-13)
                if best is None or abs(r - g.theta0) < abs(best - g.theta0):
                    best = r
        if best is None:
            raise ValueError(f"Bell-crank: wheel travel {z:.1f} mm outside linkage range")
        return best

    def state(self, z):
        return self.state_theta(self.theta_for_travel(z))

    def state_theta(self, th):
        g = self.geo
        ps = g.pose(th)
        if self.on == "lca":
            Pz = g.lca_in + _rot(th - g.theta0) @ (self.P0 - g.lca_in)
        else:
            Pz = ps["lbj"] + _rot(ps["psi"]) @ (self.P0 - g.lbj0)
        best = self._beta(Pz, 0.0)
        if best is None:
            raise ValueError(f"Bell-crank: rocker cannot be assembled at LCA angle {math.degrees(th):.2f} deg")
        A = self.O + _rot(best) @ (self.A0 - self.O)
        B = self.O + _rot(best) @ (self.B0 - self.O)
        Ls = float(np.linalg.norm(B - self.C))
        return {"theta": th, "beta": best, "P": Pz, "A": A, "B": B, "Ls": Ls, "xs": self.Ls0 - Ls}

    def _beta(self, Pz, guess=None):
        f = lambda b: float(np.linalg.norm(self.O + _rot(b) @ (self.A0 - self.O) - Pz) - self.Lpr)
        if guess is not None:
            for w in (0.05, 0.2):
                a, b = guess - w, guess + w
                fa, fb = f(a), f(b)
                if fa * fb < 0:
                    return brentq(f, a, b, xtol=1e-13)
        grid = np.linspace(-1.2, 1.2, 241)
        vals = [f(b) for b in grid]
        best = None
        for i in range(240):
            if vals[i] == 0 or vals[i] * vals[i + 1] < 0:
                r = grid[i] if vals[i] == 0 else brentq(f, grid[i], grid[i + 1], xtol=1e-13)
                if best is None or abs(r) < abs(best):
                    best = r
        return best

    def table(self):
        """Spring travel x_s vs wheel-centre travel z, from a dense sweep of the LCA angle."""
        g = self.geo
        rows = []
        for sgn in (1.0, -1.0):  # march outwards from the design angle in both directions
            beta = 0.0
            for k in range(0, 161):
                th = g.theta0 + sgn * 0.004 * k
                ps = g.pose(th)
                if ps is None:
                    break
                if self.on == "lca":
                    Pz = g.lca_in + _rot(th - g.theta0) @ (self.P0 - g.lca_in)
                else:
                    Pz = ps["lbj"] + _rot(ps["psi"]) @ (self.P0 - g.lbj0)
                beta = self._beta(Pz, beta)
                if beta is None:
                    break
                B = self.O + _rot(beta) @ (self.B0 - self.O)
                z = float(ps["wc"][1] - g.wc0[1])
                rows.append((z, self.Ls0 - float(np.linalg.norm(B - self.C))))
                if abs(z) > 55:
                    break
        rows = sorted(set(rows))
        z = np.array([r[0] for r in rows]); x = np.array([r[1] for r in rows])
        if len(z) < 6 or z.min() > -5 or z.max() < 5 or np.any(np.diff(z) <= 0):
            raise ValueError("Bell-crank: linkage cannot move through the wheel-travel range")
        zg = Z_GRID[(Z_GRID >= z.min()) & (Z_GRID <= z.max())]
        # dense samples -> cubic resample on the 2 mm grid
        return zg, CubicSpline(z, x)(zg)


@lru_cache(maxsize=128)
def _spline(hp_json: str, bc_json: str, track: float):
    from .schemas import Hardpoints, BellCrankIn
    bcr = BellCrank(Hardpoints.model_validate_json(hp_json), BellCrankIn.model_validate_json(bc_json), track)
    z, x = bcr.table()
    return CubicSpline(z, x), float(z.min()), float(z.max())


def spline_for(ax):
    """C2 spline x_s(z) for an axle whose spring.mr_mode == 'bellcrank'."""
    return _spline(ax.hardpoints.model_dump_json(), ax.spring.bellcrank.model_dump_json(), float(ax.track_mm))


def pose_points(ax, z_wheel: float):
    """Pushrod / rocker / spring points (side coordinates) at wheel travel z (for drawing)."""
    bcr = BellCrank(ax.hardpoints, ax.spring.bellcrank, ax.track_mm)
    st = bcr.state(z_wheel)
    return {"P": st["P"], "O": bcr.O, "A": st["A"], "B": st["B"], "C": bcr.C, "xs": st["xs"], "beta": st["beta"]}


@lru_cache(maxsize=128)
def _fast(hp_json: str, bc_json: str, track: float):
    from .suspension import _FastCubic
    cs, lo, hi = _spline(hp_json, bc_json, track)
    return _FastCubic(cs, 0), _FastCubic(cs, 1), _FastCubic(cs, 2), lo, hi


def fast_for(ax):
    return _fast(ax.hardpoints.model_dump_json(), ax.spring.bellcrank.model_dump_json(), float(ax.track_mm))
