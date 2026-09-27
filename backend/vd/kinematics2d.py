"""Front-view (2D) double-wishbone kinematics.

Method: planar four-bar linkage per side (chassis - LCA - upright - UCA),
one degree of freedom (LCA angle theta), the same model as Chepkasov et al.,
"Suspension Kinematics Study of the Formula SAE Sports Car", Procedia Eng.
150 (2016) eqs. 1-20, solved here by circle-circle intersection instead of
the closed-form substitution (numerically identical, more robust).

Frames (mm, rad):
  body frame   : Y lateral (+ to the OUTER side of the turn = screen right),
                 Z up, origin on the vehicle centreline at static ground level.
  ground frame : body frame rotated by the roll angle phi (outer side down for
                 phi > 0) and lifted by the heave H:
                     p_g = Rot(-phi) p_b + (0, H)
Hardpoints are given for one side with y > 0 outward; side "o" (outer) uses
Y = +y, side "i" (inner) uses Y = -y (mirror image).

Contact point (CP): the point on the wheel centre plane at distance r_loaded
below the wheel centre (tire assumed radially rigid, as in the paper, eq. 17-18).

Instant centre (IC): intersection of the LCA and UCA lines (body frame).
Roll centre (RC): intersection of the lines CP_o-IC_o and CP_i-IC_i. It moves
laterally as soon as the two sides are at different travel (roll, one-wheel bump).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from .schemas import Hardpoints

DEG = math.pi / 180.0


def rot(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def circle_intersections(c0, r0, c1, r1):
    d = np.linalg.norm(c1 - c0)
    if d == 0 or d > r0 + r1 or d < abs(r0 - r1):
        return None
    a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(max(r0 ** 2 - a ** 2, 0.0))
    ex = (c1 - c0) / d
    ey = np.array([-ex[1], ex[0]])
    p = c0 + a * ex
    return p + h * ey, p - h * ey


def line_intersection(p1, d1, p2, d2):
    """Intersection of p1 + s d1 and p2 + t d2; None if (near) parallel."""
    A = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
    det = np.linalg.det(A)
    if abs(det) < 1e-12 * max(np.linalg.norm(d1) * np.linalg.norm(d2), 1e-30):
        return None
    s, _ = np.linalg.solve(A, p2 - p1)
    return p1 + s * d1


@dataclass
class SideGeom:
    """Static design of one side, in 'side coordinates' (y outward > 0)."""
    lca_in: np.ndarray
    uca_in: np.ndarray
    lbj0: np.ndarray
    ubj0: np.ndarray
    wc0: np.ndarray
    cp0: np.ndarray
    r: float
    gamma0: float
    L_l: float
    L_u: float
    L_k: float
    theta0: float
    branch: float  # sign of cross product that selects the correct UBJ solution
    tire_w: float

    @classmethod
    def from_hardpoints(cls, hp: Hardpoints, track_mm: float) -> "SideGeom":
        P = lambda p: np.array([p.y, p.z], dtype=float)
        lca_in, lbj0, uca_in, ubj0 = P(hp.lca_in), P(hp.lca_out), P(hp.uca_in), P(hp.uca_out)
        g0 = hp.static_camber_deg * DEG
        r = hp.tire_radius_mm
        cp0 = np.array([track_mm / 2.0, 0.0])
        u = np.array([math.sin(g0), math.cos(g0)])
        wc0 = cp0 + r * u
        L_l = float(np.linalg.norm(lbj0 - lca_in))
        L_u = float(np.linalg.norm(ubj0 - uca_in))
        L_k = float(np.linalg.norm(ubj0 - lbj0))
        theta0 = math.atan2(*(lbj0 - lca_in)[::-1])
        v1, v2 = ubj0 - lbj0, uca_in - lbj0
        branch = math.copysign(1.0, v1[0] * v2[1] - v1[1] * v2[0])
        return cls(lca_in, uca_in, lbj0, ubj0, wc0, cp0, r, g0, L_l, L_u, L_k, theta0, branch,
                   hp.tire_width_mm)

    def pose(self, theta: float):
        """Upright pose for LCA angle theta (side coordinates). None if not assemblable."""
        lbj = self.lca_in + self.L_l * np.array([math.cos(theta), math.sin(theta)])
        sols = circle_intersections(self.uca_in, self.L_u, lbj, self.L_k)
        if sols is None:
            return None
        ubj = None
        for s in sols:
            v1, v2 = s - lbj, self.uca_in - lbj
            if math.copysign(1.0, v1[0] * v2[1] - v1[1] * v2[0]) == self.branch:
                ubj = s
        if ubj is None:
            return None
        a0 = math.atan2(*(self.ubj0 - self.lbj0)[::-1])
        a1 = math.atan2(*(ubj - lbj)[::-1])
        psi = a1 - a0  # upright rotation (CCW +) -> top moves inward
        Rm = rot(psi)
        T = lambda p0: lbj + Rm @ (p0 - self.lbj0)
        return {"lbj": lbj, "ubj": ubj, "wc": T(self.wc0), "cp": T(self.cp0), "psi": psi,
                "gamma_body": self.gamma0 - psi}


def _to_body(p_side: np.ndarray, sign: int) -> np.ndarray:
    return np.array([sign * p_side[0], p_side[1]])


def _body_to_ground(p_b: np.ndarray, phi: float, H: float) -> np.ndarray:
    return rot(-phi) @ p_b + np.array([0.0, H])


def solve_side(geo: SideGeom, sign: int, phi: float, H: float, ground_z: float):
    """Find theta so that the contact point touches the ground (height ground_z)."""
    def f(th):
        ps = geo.pose(th)
        if ps is None:
            return None
        return _body_to_ground(_to_body(ps["cp"], sign), phi, H)[1] - ground_z

    th0 = geo.theta0
    span, n = 0.7, 140
    grid = [th0 + span * (2 * k / n - 1) for k in range(n + 1)]
    vals = [f(t) for t in grid]
    best = None
    for k in range(n):
        a, b = vals[k], vals[k + 1]
        if a is None or b is None:
            continue
        if a == 0:
            cand = grid[k]
        elif a * b < 0:
            cand = brentq(lambda t: f(t), grid[k], grid[k + 1], xtol=1e-12)
        else:
            continue
        if best is None or abs(cand - th0) < abs(best - th0):
            best = cand
    if best is None:
        return None
    return geo.pose(best) | {"theta": best}


def solve_pose(hp_o: Hardpoints, hp_i: Hardpoints, track_mm: float, *, phi_deg=0.0,
               heave_mm=0.0, bump_o_mm=0.0, bump_i_mm=0.0, h_cg_mm: float | None = None) -> dict:
    """Solve both sides for a body roll / heave / ground bump state.

    phi_deg   : body roll, + = outer side down
    heave_mm  : body heave relative to static (+ up). Wheel travel ~ -heave.
    bump_*_mm : ground height under the outer / inner wheel (one-wheel bump).
    """
    phi, H = phi_deg * DEG, heave_mm
    out = {"phi_deg": phi_deg, "heave_mm": heave_mm, "bump_o_mm": bump_o_mm, "bump_i_mm": bump_i_mm,
           "ok": True, "sides": {}}
    ics, cps, dirs = {}, {}, {}
    for tag, hp, sign, gz in (("o", hp_o, +1, bump_o_mm), ("i", hp_i, -1, bump_i_mm)):
        geo = SideGeom.from_hardpoints(hp, track_mm)
        ps = solve_side(geo, sign, phi, H, gz)
        if ps is None:
            out["ok"] = False
            out["error"] = f"{'outer' if tag == 'o' else 'inner'} side cannot reach the ground within linkage range"
            return out
        B = lambda p: _to_body(p, sign)
        pts_b = {"lca_in": B(geo.lca_in), "uca_in": B(geo.uca_in), "lbj": B(ps["lbj"]),
                 "ubj": B(ps["ubj"]), "wc": B(ps["wc"]), "cp": B(ps["cp"])}
        # instant centre (body frame)
        dl = pts_b["lbj"] - pts_b["lca_in"]
        du = pts_b["ubj"] - pts_b["uca_in"]
        ic = line_intersection(pts_b["lca_in"], dl, pts_b["uca_in"], du)
        ics[tag] = ic
        cps[tag] = pts_b["cp"]
        dirs[tag] = (ic - pts_b["cp"]) if ic is not None else dl  # IC at infinity -> parallel arms
        travel = float(ps["wc"][1] - geo.wc0[1])  # jounce + (body frame)
        # wheel plane unit 'up' vector in body then ground frame
        gb = ps["gamma_body"]
        u_b = np.array([sign * math.sin(gb), math.cos(gb)])
        u_g = rot(-phi) @ u_b
        gamma_g = math.atan2(sign * u_g[0], u_g[1])
        pts_g = {k: _body_to_ground(v, phi, H) for k, v in pts_b.items()}
        # tire outline (ground frame) - rectangle centred on the wheel centre
        w, r = geo.tire_w, geo.r
        e_up, e_lat = u_g, np.array([u_g[1], -u_g[0]])
        c = pts_g["wc"]
        tire = [c + e_lat * sx * w / 2 + e_up * sz * r for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        rim_r = 0.55 * r
        rim = [c + e_lat * sx * w * 0.42 + e_up * sz * rim_r for sx, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        out["sides"][tag] = {
            "points": {k: v.tolist() for k, v in pts_g.items()},
            "tire": [p.tolist() for p in tire], "rim": [p.tolist() for p in rim],
            "ic": _body_to_ground(ic, phi, H).tolist() if ic is not None else None,
            "camber_ground_deg": gamma_g / DEG,
            "camber_body_deg": gb / DEG,
            "travel_mm": travel,
            "theta_deg": ps["theta"] / DEG,
            "lengths": {"lca": geo.L_l, "uca": geo.L_u, "upright": geo.L_k},
        }
    rc_b = line_intersection(cps["o"], dirs["o"], cps["i"], dirs["i"])
    if rc_b is None:
        # parallel force lines: coincident -> RC anywhere on the line, take the midpoint
        # (e.g. equal-length parallel arms at static: RC on the ground); otherwise at infinity
        d = dirs["o"] / np.linalg.norm(dirs["o"])
        w = cps["i"] - cps["o"]
        if abs(w[0] * d[1] - w[1] * d[0]) < 1e-6:
            rc_b = 0.5 * (cps["o"] + cps["i"])
    rc_g = _body_to_ground(rc_b, phi, H) if rc_b is not None else None
    cpo_g = np.array(out["sides"]["o"]["points"]["cp"])
    cpi_g = np.array(out["sides"]["i"]["points"]["cp"])
    out["rc"] = rc_g.tolist() if rc_g is not None else None
    out["rc_height_mm"] = float(rc_g[1]) if rc_g is not None else float("nan")
    out["rc_lateral_mm"] = float(rc_g[0]) if rc_g is not None else float("nan")
    out["track_mm"] = float(cpo_g[0] - cpi_g[0])
    # body outline for drawing (ground frame): box spanning inboard pivots
    yb = max(abs(hp_o.lca_in.y), abs(hp_o.uca_in.y), abs(hp_i.lca_in.y), abs(hp_i.uca_in.y))
    zl = min(hp_o.lca_in.z, hp_i.lca_in.z, hp_o.uca_in.z, hp_i.uca_in.z) - 25
    zu = max(hp_o.lca_in.z, hp_i.lca_in.z, hp_o.uca_in.z, hp_i.uca_in.z) + 60
    body = [np.array(p) for p in ((-yb, zl), (yb, zl), (yb, zu), (-yb, zu))]
    out["body"] = [_body_to_ground(p, phi, H).tolist() for p in body]
    out["centreline"] = [_body_to_ground(np.array([0.0, -50.0]), phi, H).tolist(),
                         _body_to_ground(np.array([0.0, zu + 120]), phi, H).tolist()]
    if h_cg_mm is not None:
        out["cg"] = _body_to_ground(np.array([0.0, h_cg_mm]), phi, H).tolist()
    return out


def symmetric_rc_height_by_velocity(hp: Hardpoints, track_mm: float, dz: float = 0.01) -> float:
    """Independent check: h_rc = (t/2) * dY_cp/dZ_cp for symmetric heave (virtual velocities)."""
    geo = SideGeom.from_hardpoints(hp, track_mm)
    th = geo.theta0
    e = 1e-6
    p1, p2 = geo.pose(th - e), geo.pose(th + e)
    dy = p2["cp"][0] - p1["cp"][0]
    dzz = p2["cp"][1] - p1["cp"][1]
    return geo.cp0[0] * dy / dzz


def sweep(hp_o, hp_i, track_mm, mode: str, values, h_cg_mm=None):
    """Kinematic curves: mode in {'heave' (wheel travel, + jounce), 'roll' (deg), 'bump' (outer, mm)}."""
    rows = []
    for v in values:
        if mode == "heave":
            s = solve_pose(hp_o, hp_i, track_mm, heave_mm=-v, h_cg_mm=h_cg_mm)
        elif mode == "roll":
            s = solve_pose(hp_o, hp_i, track_mm, phi_deg=v, h_cg_mm=h_cg_mm)
        elif mode == "bump":
            s = solve_pose(hp_o, hp_i, track_mm, bump_o_mm=v, h_cg_mm=h_cg_mm)
        else:
            raise ValueError(mode)
        if not s["ok"]:
            rows.append(None)
            continue
        rows.append({
            "x": v,
            "camber_o_deg": s["sides"]["o"]["camber_ground_deg"],
            "camber_i_deg": s["sides"]["i"]["camber_ground_deg"],
            "camber_o_body_deg": s["sides"]["o"]["camber_body_deg"],
            "travel_o_mm": s["sides"]["o"]["travel_mm"],
            "travel_i_mm": s["sides"]["i"]["travel_mm"],
            "track_mm": s["track_mm"],
            "rc_height_mm": s["rc_height_mm"],
            "rc_lateral_mm": s["rc_lateral_mm"],
        })
    keys = ["camber_o_deg", "camber_i_deg", "camber_o_body_deg", "travel_o_mm", "travel_i_mm",
            "track_mm", "rc_height_mm", "rc_lateral_mm"]
    res = {"x": list(values)}
    for k in keys:
        res[k] = [(r[k] if r is not None else None) for r in rows]
    t0 = next((r["track_mm"] for r, v in zip(rows, values) if r is not None and v == 0), None)
    res["track_change_mm"] = [(r["track_mm"] - t0 if (r is not None and t0 is not None) else None) for r in rows]
    return res
