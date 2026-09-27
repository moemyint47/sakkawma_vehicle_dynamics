"""v2: suspension elements, transient roll, adaptive editing, workspaces."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backend"))

from vd import adapt, kinematics2d as k2, load_transfer, transient  # noqa: E402
from vd.derivation import Calc  # noqa: E402
from vd.schemas import ModelIn  # noqa: E402
from vd.suspension import AxleSuspension  # noqa: E402
from vd.sweep import with_param  # noqa: E402


def model(**over):
    m = ModelIn(param_mode="absolute")  # physics tests: every input independent
    for p, v in over.items():
        m = with_param(m, p.replace("__", "."), v)
    return m


def susp(m, tag="front"):
    ax = getattr(m, tag)
    wf = m.vehicle.weight_front if tag == "front" else 1 - m.vehicle.weight_front
    return AxleSuspension(ax, m.vehicle.mass_kg * wf - 2 * ax.unsprung_mass_kg, m.maneuver.g)


@pytest.mark.parametrize("mode", ["poly", "table"])
def test_wheel_rate_is_derivative_of_wheel_force(mode):
    m = model(front__spring__mr_mode=mode,
              front__spring__mr_table=[[-40, 0.68], [-10, 0.73], [0, 0.75], [20, 0.80], [40, 0.88]])
    s = susp(m)
    for z in (-30.0, -5.0, 0.0, 12.0, 35.0):
        e = 1e-4
        num = (s.spring_wheel(z + e) - s.spring_wheel(z - e)) / (2 * e)
        assert s.wheel_rate(z) == pytest.approx(num, rel=1e-6)


def test_static_equilibrium_preload():
    s = susp(ModelIn())
    assert s.spring_wheel(0.0) == pytest.approx(s.W)


def test_damper_piecewise_and_table():
    m = ModelIn()
    d = susp(m).damper
    D = m.front.damper
    assert d.force(D.v_knee_bump_mm_s) == pytest.approx(D.c_ls_bump * D.v_knee_bump_mm_s / 1000)
    assert d.force(-100) == pytest.approx(-(D.c_ls_reb * 50 + D.c_hs_reb * 50) / 1000)
    m2 = model(front__damper__use_table=True, front__damper__table=[[-200, -500], [0, 0], [100, 200]])
    d2 = susp(m2).damper
    assert d2.force(50) == pytest.approx(100)
    assert d2.force(-400) == pytest.approx(-1000)  # linear extrapolation


@pytest.mark.parametrize("src", ["components", "direct"])
def test_transient_settles_to_steady_state(src):
    m = model(front__roll_stiffness_source=src, rear__roll_stiffness_source=src,
              maneuver__transient__t_end_s=4.0)
    tr = transient.simulate(m)
    lt = load_transfer.compute(m)
    assert tr["phi_deg"][-1] == pytest.approx(lt["phi_deg"], rel=1e-5)
    for t in "fr":
        assert tr["axles"][t]["total"][-1] == pytest.approx(lt["axles"][t]["dFz"], rel=1e-5)


def test_transient_geometric_is_instantaneous():
    m = model(maneuver__transient__profile="step")
    tr = transient.simulate(m)
    i = next(k for k, t in enumerate(tr["t"]) if t >= m.maneuver.transient.t0_s)
    # right after the step the body has not rolled yet: all suspended LT is geometric
    assert tr["axles"]["f"]["pct_geo"][i] == pytest.approx(100.0, abs=1e-3)


def test_progressive_rate_raises_roll_stiffness_with_roll():
    s = susp(model(front__spring__mr_c1=0.004))
    r = s.roll_curves()
    k = r["K_Nm_deg"]
    assert k[-1] > k[len(k) // 2]


def test_adaptive_rc_exact_and_swing_arm_kept():
    m = ModelIn()
    ax = m.front
    s0 = k2.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm)
    L0 = math.dist(s0["sides"]["o"]["points"]["cp"], s0["sides"]["o"]["ic"])
    n, ch, _ = adapt.apply(m, "front.h_rc_mm", 70.0)
    s1 = k2.solve_pose(n.front.hardpoints, n.front.hardpoints, n.front.track_mm)
    assert s1["rc_height_mm"] == pytest.approx(70.0, abs=1e-2)
    assert n.front.h_rc_mm == pytest.approx(70.0, abs=1e-2)
    L1 = math.dist(s1["sides"]["o"]["points"]["cp"], s1["sides"]["o"]["ic"])
    assert L1 == pytest.approx(L0, rel=1e-4)
    assert n.front.hardpoints.lca_out == m.front.hardpoints.lca_out


def test_adaptive_roll_stiffness_target():
    n, _, _ = adapt.apply(ModelIn(), "front.roll_stiffness_Nm_deg", 450.0)
    assert susp(n).linear_roll_stiffness() * math.pi / 180 == pytest.approx(450.0, rel=1e-5)
    assert n.front.arb_share == pytest.approx(ModelIn().front.arb_share, abs=1e-5)


def test_absolute_mode_changes_only_the_edited_value():
    m = model(param_mode="absolute")
    n, ch, _ = adapt.apply(m, "front.h_rc_mm", 70.0)
    assert [c["path"] for c in ch] == ["front.h_rc_mm"]


def test_corner_mode_ay_from_speed_and_radius():
    m = model(maneuver__ay_source="corner", maneuver__speed_kmh=36.0, maneuver__radius_m=10.0)
    lt = load_transfer.compute(m)
    assert lt["ay"] == pytest.approx(10.0 ** 2 / 10.0)


def test_road_profile_steady_arc_and_timing():
    m = model(maneuver__transient__profile="road", maneuver__transient__road=[[5, 0, 36], [40, 10, 36]],
              maneuver__transient__road_transition_m=0.0, maneuver__transient__t_end_s=4.0)
    f, plan = transient.road_profile(m)
    assert f(3.0) == pytest.approx(10.0 ** 2 / 10.0 / m.maneuver.g)  # on the arc: v^2/R
    assert f(0.2) == pytest.approx(0.0)                                # on the straight
    assert plan["t"][-1] == pytest.approx(45 / 10.0, rel=1e-6)          # 45 m at 10 m/s


# ------------------------------------------------------------------ v3: force-based geometric LT
def test_ic_angle_model_equals_rc_model_at_zero_roll():
    from vd import geo_coupling as gc
    m = ModelIn()
    for ax in (m.front, m.rear):
        to, ti, _, _ = gc.ic_tangents(ax, 0.0)
        assert to == pytest.approx(ti, rel=1e-9)
        assert to * ax.track_mm / 2 == pytest.approx(gc.static_rc(ax) * 1000, rel=1e-6)


def test_ic_angle_transient_settles_to_coupled_steady_state():
    from vd import cornering
    m = model(maneuver__geo_model="ic_angles", maneuver__transient__t_end_s=4.0)
    tr = transient.simulate(m)
    c = Calc()
    cornering.compute(m, c)
    assert tr["phi_deg"][-1] == pytest.approx(c.get("phi_deg"), rel=1e-4)
    for t in "fr":
        assert tr["axles"][t]["total"][-1] == pytest.approx(c.get(f"{t}.dFz"), rel=1e-4)
        assert tr["axles"][t]["jack"][-1] == pytest.approx(c.get(f"geo.{t}.J"), rel=1e-3)


def test_jacking_sign_follows_rc_height():
    from vd import geo_coupling as gc
    up = gc.solve(model(maneuver__geo_model="ic_angles"), 1.2)
    assert up["axles"]["f"]["J"] > 0  # RC above ground -> body jacked up
    n, _, _ = adapt.apply(ModelIn(), "front.h_rc_mm", -40.0)
    dn = gc.solve(with_param(n, "maneuver.geo_model", "ic_angles"), 1.2)
    assert dn["axles"]["f"]["J"] < 0  # RC below ground -> pushed down


# ------------------------------------------------------------------ v3: bell-crank, maps, optimiser
def test_bellcrank_mr_is_derivative_of_spring_travel():
    from vd.bellcrank import BellCrank
    m = ModelIn()
    b = BellCrank(m.front.hardpoints, m.front.spring.bellcrank, m.front.track_mm)
    s = susp(m)
    e = 0.05
    for z in (-20.0, 0.0, 15.0):
        num = (b.state(z + e)["xs"] - b.state(z - e)["xs"]) / (2 * e)
        assert s.mr.mr(z) == pytest.approx(num, rel=2e-3)
    assert s.wheel_rate(5.0) == pytest.approx((s.spring_wheel(5.0001) - s.spring_wheel(4.9999)) / 0.0002, rel=1e-5)


def test_bellcrank_pushrod_length_is_constant():
    from vd.bellcrank import BellCrank
    import numpy as np
    m = ModelIn()
    b = BellCrank(m.front.hardpoints, m.front.spring.bellcrank, m.front.track_mm)
    for z in (-30.0, 0.0, 30.0):
        st = b.state(z)
        assert float(np.linalg.norm(st["A"] - st["P"])) == pytest.approx(b.Lpr, abs=1e-8)


def test_optimizer_improves_objective_and_respects_bounds():
    from vd import optimize
    o = optimize.run(ModelIn(), [{"path": "rear.roll_stiffness_Nm_deg", "min": 200.0, "max": 600.0}],
                     [{"key": "lltd_front", "op": "between", "a": 0.50, "b": 0.52}], max_evals=25)
    assert o["best"]["J"] <= o["initial"]["J"]
    assert 200.0 <= o["best"]["x"][0] <= 600.0
    assert 0.50 - 1e-3 <= o["best"]["values"]["lltd_front"] <= 0.52 + 1e-3


def test_sweep2d_shape():
    from vd.sweep import run_sweep2d
    r = run_sweep2d(ModelIn(), "front.h_rc_mm", [20, 50], "rear.h_rc_mm", [30, 60, 90])
    assert len(r["grids"]["lltd_front"]) == 3 and len(r["grids"]["lltd_front"][0]) == 2


# ------------------------------------------------------------------ corner replay
def _arc_model(R=10.0, kmh=36.0):
    return model(maneuver__transient__profile="road", maneuver__transient__road=[[5, 0, kmh], [120, R, kmh]],
                 maneuver__transient__road_transition_m=0.0, maneuver__transient__t_end_s=6.0)


def test_replay_steady_arc_matches_steady_state():
    from vd import replay, cornering
    m = _arc_model()
    r = replay.run(m, dt=0.01)
    ay_g = (10.0 ** 2 / 10.0) / m.maneuver.g
    c = Calc()
    cornering.compute(m, c, ay_g=ay_g)
    i = -1  # end of the arc: roll settled, constant yaw rate
    assert r["ay_g"][i] == pytest.approx(ay_g, rel=1e-9)
    assert r["alpha_f"][i] == pytest.approx(c.get("f.alpha"), rel=1e-4)
    assert r["alpha_r"][i] == pytest.approx(c.get("r.alpha"), rel=1e-4)
    assert r["Fz"]["FR"][i] == pytest.approx(c.get("f.Fz_out"), rel=1e-4)
    assert r["Fz"]["RL"][i] == pytest.approx(c.get("r.Fz_in"), rel=1e-4)


def test_replay_satisfies_lateral_and_yaw_equilibrium():
    from vd import replay
    import numpy as np
    m = model(maneuver__transient__profile="road", maneuver__transient__road=[[8, 0, 26], [14, 4.5, 30], [8, 0, 34]],
              maneuver__transient__t_end_s=3.9)
    r = replay.run(m, dt=0.01)
    V = m.vehicle
    l = V.wheelbase_mm / 1000; a = l * (1 - V.weight_front); b = l * V.weight_front
    Fyf, Fyr = np.array(r["Fyf"]), np.array(r["Fyr"])
    assert np.allclose(Fyf + Fyr, V.mass_kg * np.array(r["ay_g"]) * m.maneuver.g, atol=1e-6)
    assert np.allclose(a * Fyf - b * Fyr, np.array(r["Mz_yaw"]), atol=1e-6)
    # vertical equilibrium of the four wheel loads at every instant
    tot = sum(np.array(v) for v in r["Fz"].values())
    assert np.allclose(tot, V.mass_kg * m.maneuver.g, atol=1e-6)
