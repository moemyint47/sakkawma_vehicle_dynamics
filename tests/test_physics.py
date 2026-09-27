"""Correctness checks: identities, limiting cases and independent methods."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))

from vd import cornering, kinematics2d as k2, load_transfer, tire  # noqa: E402
from vd.derivation import Calc  # noqa: E402
from vd.schemas import ModelIn, Point2  # noqa: E402
from vd.sweep import with_param  # noqa: E402


def model(**over):
    m = ModelIn(param_mode="absolute")  # physics tests: every input independent
    for path, v in over.items():
        m = with_param(m, path.replace("__", "."), v)
    return m


# ------------------------------------------------------------------ load transfer
@pytest.mark.parametrize("grav", [True, False])
def test_moment_balance(grav):
    m = model(maneuver__roll_gravity_term=grav)
    r = load_transfer.compute(m, Calc())
    got, exp = r["moment_check"]
    assert got == pytest.approx(exp, rel=1e-9)  # exact up to the roll-equilibrium solver tolerance


def test_rc_at_cg_is_fully_geometric():
    # no unsprung mass, equal RC heights at the CG height -> h1 = 0 -> no roll, 100 % geometric
    m = model(front__unsprung_mass_kg=0, rear__unsprung_mass_kg=0, front__h_rc_mm=290, rear__h_rc_mm=290)
    r = load_transfer.compute(m)
    assert r["phi"] == pytest.approx(0, abs=1e-12)
    for t in "fr":
        assert r["axles"][t]["geo_share"] == pytest.approx(1.0)


def test_rc_at_ground_is_fully_elastic():
    m = model(front__unsprung_mass_kg=0, rear__unsprung_mass_kg=0, front__h_rc_mm=0, rear__h_rc_mm=0)
    r = load_transfer.compute(m)
    for t in "fr":
        assert r["axles"][t]["el_share"] == pytest.approx(1.0)


def test_geometric_share_equals_hrc_over_hcg():
    # equal RC heights, no unsprung, no gravity term: eta_g = h_rc / h_cg on each axle
    m = model(front__unsprung_mass_kg=0, rear__unsprung_mass_kg=0, front__h_rc_mm=80, rear__h_rc_mm=80,
              maneuver__roll_gravity_term=False, front__track_mm=1200, rear__track_mm=1200,
              front__roll_stiffness_Nm_deg=380, rear__roll_stiffness_Nm_deg=380 * (0.53 / 0.47),
              front__roll_stiffness_source="direct", rear__roll_stiffness_source="direct")  # K split = weight split
    r = load_transfer.compute(m)
    assert r["axles"]["f"]["geo_share"] == pytest.approx(80 / 290, rel=1e-9)
    assert r["axles"]["r"]["geo_share"] == pytest.approx(80 / 290, rel=1e-9)


def test_wheel_loads_sum_to_weight():
    m = model()
    r = load_transfer.compute(m)
    tot = sum(r["axles"][t]["Fz_out"] + r["axles"][t]["Fz_in"] for t in "fr")
    assert tot == pytest.approx(m.vehicle.mass_kg * m.maneuver.g)


# ------------------------------------------------------------------ tire
def test_tmeasy_reproduces_nominal_and_double_load_data():
    t = ModelIn().tire
    for Fz, FM, aM in ((t.Fz_nom_N, t.F_M_nom_N, t.alpha_M_nom_deg),
                       (2 * t.Fz_nom_N, t.F_M_2nom_N, t.alpha_M_2nom_deg)):
        p = tire.params_at(Fz, t)
        assert p.FM == pytest.approx(FM)
        assert tire.force_from_slip(math.tan(math.radians(aM)), p) == pytest.approx(FM)


def test_tmeasy_initial_slope_and_continuity():
    t = ModelIn().tire
    p = tire.params_at(t.Fz_nom_N, t)
    s = 1e-7
    assert tire.force_from_slip(s, p) / s == pytest.approx(p.dF0, rel=1e-5)
    for s0 in (p.sM, p.sS):
        assert tire.force_from_slip(s0 - 1e-9, p) == pytest.approx(tire.force_from_slip(s0 + 1e-9, p), abs=1e-3)
    assert tire.force_from_slip(-0.05, p) == pytest.approx(-tire.force_from_slip(0.05, p))


def test_tmeasy_degressive():
    t = ModelIn().tire
    a = math.radians(4)
    assert tire.fy(2 * t.Fz_nom_N, a, t) < 2 * tire.fy(t.Fz_nom_N, a, t)


def test_tmeasy_linear_without_double_load():
    t = ModelIn().tire.model_copy(update=dict(C_alpha_2nom_N_deg=None))
    a = math.radians(3)
    assert tire.fy(2 * t.Fz_nom_N, a, t) == pytest.approx(2 * tire.fy(t.Fz_nom_N, a, t))


# ------------------------------------------------------------------ cornering
def test_slip_angle_solution_satisfies_force_balance():
    m = model()
    r = cornering.compute(m, Calc())
    for t in "fr":
        a = r["axles"][t]
        assert a["Fy_out"] + a["Fy_in"] == pytest.approx(a["Fy_req"], rel=1e-8)


def test_limit_is_boundary_of_feasibility():
    m = model()
    L = cornering.limit_ay(m)
    assert cornering._feasible(m, L["ay_max_g"] - 1e-4)[0]
    assert not cornering._feasible(m, L["ay_max_g"] + 1e-4)[0]


# ------------------------------------------------------------------ kinematics
def test_rc_graphical_equals_virtual_velocity():
    for ax in (ModelIn().front, ModelIn().rear):
        s = k2.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm)
        assert s["rc_height_mm"] == pytest.approx(
            k2.symmetric_rc_height_by_velocity(ax.hardpoints, ax.track_mm), abs=1e-4)


def test_parallel_equal_arms():
    hp = ModelIn().front.hardpoints.model_copy(update=dict(
        lca_in=Point2(y=200, z=120), lca_out=Point2(y=580, z=120),
        uca_in=Point2(y=180, z=320), uca_out=Point2(y=560, z=320), static_camber_deg=0))
    # parallelogram: no camber change relative to body in heave, RC at ground
    sw = k2.sweep(hp, hp, 1220, "heave", [-20, 0, 20])
    for c in sw["camber_o_body_deg"]:
        assert c == pytest.approx(0, abs=1e-9)
    assert sw["rc_height_mm"][1] == pytest.approx(0, abs=1e-6)


def test_roll_is_antisymmetric_and_static_is_consistent():
    ax = ModelIn().front
    sw = k2.sweep(ax.hardpoints, ax.hardpoints, ax.track_mm, "roll", [-1.5, 0, 1.5])
    assert sw["rc_lateral_mm"][0] == pytest.approx(-sw["rc_lateral_mm"][2], abs=1e-6)
    assert sw["camber_o_deg"][1] == pytest.approx(ax.hardpoints.static_camber_deg, abs=1e-9)
    assert sw["track_mm"][1] == pytest.approx(ax.track_mm, abs=1e-9)


def test_links_keep_length():
    ax = ModelIn().front
    s = k2.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm, phi_deg=2.0)
    for side in s["sides"].values():
        P = side["points"]
        L = side["lengths"]
        d = lambda a, b: math.dist(P[a], P[b])
        assert d("lca_in", "lbj") == pytest.approx(L["lca"], abs=1e-6)
        assert d("uca_in", "ubj") == pytest.approx(L["uca"], abs=1e-6)
        assert d("lbj", "ubj") == pytest.approx(L["upright"], abs=1e-6)
        assert P["cp"][1] == pytest.approx(0, abs=1e-6)  # on the ground
