"""Input schemas (pydantic) with placeholder defaults.

UNITS OF INPUT (chosen for engineering convenience; converted to SI inside
the models, and every conversion is shown in the derivation log):
    lengths          mm
    masses           kg
    roll stiffness   N*m/deg   (per axle, springs + ARB at the wheel)
    lateral accel.   g
    tire forces      N
    tire slip angles deg       (lateral slip s_y = tan(alpha), Rill eq. 3.72)
    cornering stiff. N/deg

ALL DEFAULT NUMBERS ARE PLACEHOLDERS - replace them with your own data.

Geometry coordinate system (front view, 2D, ready for a later 3D 'x'):
    y  lateral, positive outward to the side being described, 0 = vehicle centreline
    z  vertical, positive up, 0 = ground plane at static ride height
Hardpoints are given for ONE side (the model mirrors them for the other side).
"""
from __future__ import annotations

from typing import Literal, Optional

from pydantic import BaseModel, Field, model_validator


class Point2(BaseModel):
    y: float
    z: float
    # x: float | None = None   # reserved for 3D (longitudinal)


class Hardpoints(BaseModel):
    """Double-wishbone front-view hardpoints for one side (mm)."""
    lca_in: Point2 = Field(description="Lower control arm inboard (chassis) pivot")
    lca_out: Point2 = Field(description="Lower ball joint (upright)")
    uca_in: Point2 = Field(description="Upper control arm inboard (chassis) pivot")
    uca_out: Point2 = Field(description="Upper ball joint (upright)")
    tire_radius_mm: float = Field(228.0, gt=0, description="Loaded tire radius")
    tire_width_mm: float = Field(190.0, gt=0, description="Tire section width (drawing only)")
    static_camber_deg: float = Field(-1.5, description="Static camber, + = top of wheel outward")


class BellCrankIn(BaseModel):
    """Front-view pushrod / rocker / spring layout (one side, mm, same axes as the hardpoints)."""
    pushrod_on: Literal["lca", "upright"] = Field("lca", description="Pushrod outboard end fixed to the LCA or the upright")
    pushrod_out: Point2 = Field(default_factory=lambda: Point2(y=535, z=140), description="Pushrod outboard end")
    rocker_pivot: Point2 = Field(default_factory=lambda: Point2(y=215, z=365), description="Rocker pivot (chassis)")
    rocker_pushrod: Point2 = Field(default_factory=lambda: Point2(y=221, z=431), description="Pushrod joint on rocker")
    rocker_spring: Point2 = Field(default_factory=lambda: Point2(y=175, z=417.5), description="Spring joint on rocker")
    spring_mount: Point2 = Field(default_factory=lambda: Point2(y=-9, z=352), description="Spring chassis mount")


class SpringIn(BaseModel):
    """Coil spring acting through a (progressive) motion ratio.

    MR(z) = d(spring travel)/d(wheel travel), z = wheel travel in mm (+ jounce).
    mode 'poly' : MR = c0 + c1 z + c2 z^2
    mode 'table': monotone cubic (PCHIP) through [[z_mm, MR], ...]
    The spring preload is solved so the corner is in equilibrium at z = 0.
    """
    rate_N_mm: float = Field(35.0, gt=0, description="Spring rate (at the spring)")
    mr_mode: Literal["poly", "table", "bellcrank"] = "bellcrank"
    mr_c0: float = Field(0.75, gt=0, description="MR at static ride height")
    mr_c1: float = Field(0.002, description="MR slope [1/mm] (+ = progressive)")
    mr_c2: float = Field(0.0, description="MR curvature [1/mm^2]")
    mr_table: Optional[list[list[float]]] = Field(None, description="[[wheel travel mm, MR], ...]")
    bellcrank: BellCrankIn = Field(default_factory=BellCrankIn, description="Rocker geometry (mr_mode = bellcrank)")
    bump_gap_mm: Optional[float] = Field(25.0, description="Wheel travel to bump-stop contact (None = off)")
    bump_rate_N_mm: float = Field(150.0, ge=0, description="Bump-stop rate at the wheel")


class DamperIn(BaseModel):
    """Damper force-velocity at the damper; + velocity = bump (compression)."""
    mr_mode: Literal["spring", "constant"] = Field("spring", description="Damper MR = spring MR curve, or constant")
    mr_const: float = Field(0.75, gt=0)
    c_ls_bump: float = Field(1800.0, ge=0, description="Low-speed bump [N·s/m]")
    c_ls_reb: float = Field(2400.0, ge=0, description="Low-speed rebound [N·s/m]")
    v_knee_bump_mm_s: float = Field(50.0, gt=0)
    v_knee_reb_mm_s: float = Field(50.0, gt=0)
    c_hs_bump: float = Field(700.0, ge=0, description="High-speed bump [N·s/m]")
    c_hs_reb: float = Field(1000.0, ge=0, description="High-speed rebound [N·s/m]")
    use_table: bool = Field(False, description="Use the dyno table instead of coefficients")
    table: Optional[list[list[float]]] = Field(
        None, description="[[v mm/s (signed, + bump), F N (signed)], ...] dyno curve")


class AxleIn(BaseModel):
    track_mm: float = Field(gt=0)
    h_rc_mm: float = Field(description="Roll-centre height (load-transfer input)")
    roll_stiffness_source: Literal["components", "direct"] = Field(
        "components", description="Roll stiffness from springs/ARB/bump stops, or the direct input")
    roll_stiffness_Nm_deg: float = Field(gt=0, description="Axle roll stiffness incl. ARB (direct mode)")
    unsprung_mass_kg: float = Field(12.0, ge=0, description="Unsprung mass PER CORNER")
    h_unsprung_mm: float = Field(228.0, ge=0, description="Unsprung CG height (~ wheel centre)")
    arb_rate_N_mm: float = Field(9.6, ge=0, description="ARB rate at the wheel (per wheel, pure roll)")
    arb_share: float = Field(0.3127, ge=0, lt=1, description="ARB share of the axle roll stiffness (adaptive mode)")
    spring: SpringIn = Field(default_factory=SpringIn)
    damper: DamperIn = Field(default_factory=DamperIn)
    hardpoints: Hardpoints


class VehicleIn(BaseModel):
    mass_kg: float = Field(280.0, gt=0, description="Total mass incl. driver")
    weight_front: float = Field(0.47, gt=0, lt=1, description="Static front weight fraction")
    wheelbase_mm: float = Field(1550.0, gt=0)
    h_cg_mm: float = Field(290.0, gt=0, description="Total CG height")
    roll_inertia_kgm2: float = Field(15.0, gt=0, description="Sprung-mass roll inertia about its own CG")
    yaw_inertia_kgm2: float = Field(100.0, gt=0, description="Total yaw inertia about the CG (incl. driver)")


class TireIn(BaseModel):
    """TMeasy lateral characteristic (Rill, Road Vehicle Dynamics, Sec. 3.6/3.7).

    Nominal load data is mandatory; double-load data is optional. If the
    double-load set is omitted, forces scale linearly with Fz and slip
    locations stay constant (the quadratic of eq. 3.121 degenerates to a line).
    """
    Fz_nom_N: float = Field(800.0, gt=0, description="Nominal (pay)load F_z^N")
    C_alpha_nom_N_deg: float = Field(700.0, gt=0, description="Cornering stiffness at F_z^N")
    alpha_M_nom_deg: float = Field(8.0, gt=0, description="Slip angle at peak force, F_z^N")
    F_M_nom_N: float = Field(1320.0, gt=0, description="Peak lateral force at F_z^N")
    alpha_S_nom_deg: float = Field(20.0, gt=0, description="Slip angle where sliding starts, F_z^N")
    F_S_nom_N: float = Field(1190.0, gt=0, description="Sliding lateral force at F_z^N")

    C_alpha_2nom_N_deg: Optional[float] = Field(1100.0, description="Cornering stiffness at 2 F_z^N")
    alpha_M_2nom_deg: Optional[float] = Field(9.0)
    F_M_2nom_N: Optional[float] = Field(2480.0)
    alpha_S_2nom_deg: Optional[float] = Field(22.0)
    F_S_2nom_N: Optional[float] = Field(2230.0)

    @property
    def has_double_load(self) -> bool:
        return all(v is not None for v in (
            self.C_alpha_2nom_N_deg, self.alpha_M_2nom_deg, self.F_M_2nom_N,
            self.alpha_S_2nom_deg, self.F_S_2nom_N))


class TransientIn(BaseModel):
    """Open-loop lateral-acceleration input a_y(t) (amplitude = maneuver.ay_g)."""
    profile: Literal["ramp", "step", "sine", "csv", "road"] = "ramp"
    t0_s: float = Field(0.5, ge=0, description="Start of the input")
    rise_s: float = Field(0.25, gt=0, description="Ramp rise time")
    freq_hz: float = Field(1.0, gt=0, description="Sine frequency")
    t_end_s: float = Field(2.0, gt=0, le=20)
    csv: Optional[list[list[float]]] = Field(None, description="[[t s, a_y g], ...]")
    t_probe_s: float = Field(0.55, ge=0, description="Time instant for the derivation breakdown")
    road: Optional[list[list[float]]] = Field(
        default_factory=lambda: [[8, 0, 30], [10, 5, 30], [8, 0, 30]],
        description="Road segments [[length m, radius m (0 = straight, - = other direction), speed km/h], ...]")
    road_transition_m: float = Field(3.0, ge=0, description="Clothoid length between segments (curvature ramps linearly)")


class ManeuverIn(BaseModel):
    """a_y for the steady-state calculation: see ay_source."""
    ay_source: Literal["input", "corner"] = Field(
        "input", description="input: a_y given directly; corner: a_y = v^2 / R from speed and path radius")
    ay_g: float = Field(1.2, ge=0, description="Steady-state lateral acceleration")
    radius_m: float = Field(9.125, gt=0, description="Path radius (default: FSAE skidpad centreline)")
    speed_kmh: float = Field(37.3, gt=0, description="Speed on the corner (ay_source = corner)")
    g: float = Field(9.81, gt=0)
    roll_gravity_term: bool = Field(True, description="Include m_s g h1 phi (lateral CG shift) in roll")
    geo_model: Literal["rc_height", "ic_angles"] = Field(
        "rc_height", description="Geometric LT: classic RC-height formula, or force-based IC angles at the rolled "
                                 "pose with tyre force split (RC migration + jacking)")
    transient: TransientIn = Field(default_factory=TransientIn)


def _default_front() -> AxleIn:
    return AxleIn(
        track_mm=1220, h_rc_mm=35.1416, roll_stiffness_Nm_deg=403.1185, arb_share=0.309318,
        unsprung_mass_kg=12, h_unsprung_mm=228,
        hardpoints=Hardpoints(
            lca_in=Point2(y=210, z=120.5), lca_out=Point2(y=580, z=125),
            uca_in=Point2(y=255, z=282.2), uca_out=Point2(y=560, z=320),
            tire_radius_mm=228, static_camber_deg=-1.5),
    )


def _default_rear() -> AxleIn:
    return AxleIn(
        track_mm=1180, h_rc_mm=54.9676, roll_stiffness_Nm_deg=377.3118,
        unsprung_mass_kg=13, h_unsprung_mm=228, arb_rate_N_mm=5.6,
        spring=SpringIn(rate_N_mm=40.0, mr_c0=0.72, mr_c1=0.002), arb_share=0.180343,
        damper=DamperIn(mr_const=0.72),
        hardpoints=Hardpoints(
            lca_in=Point2(y=220, z=130.2), lca_out=Point2(y=560, z=125),
            uca_in=Point2(y=260, z=285.0), uca_out=Point2(y=540, z=315),
            tire_radius_mm=228, static_camber_deg=-1.0),
    )


class ModelIn(BaseModel):
    param_mode: Literal["adaptive", "absolute"] = Field(
        "adaptive", description="adaptive: dependent parameters follow edits; absolute: every input independent")
    vehicle: VehicleIn = Field(default_factory=VehicleIn)
    front: AxleIn = Field(default_factory=_default_front)
    rear: AxleIn = Field(default_factory=_default_rear)
    tire: TireIn = Field(default_factory=TireIn)
    maneuver: ManeuverIn = Field(default_factory=ManeuverIn)

    @model_validator(mode="after")
    def _check_masses(self):
        mu = 2 * (self.front.unsprung_mass_kg + self.rear.unsprung_mass_kg)
        if mu >= self.vehicle.mass_kg:
            raise ValueError("Total unsprung mass must be smaller than total mass")
        if 2 * self.front.unsprung_mass_kg >= self.vehicle.mass_kg * self.vehicle.weight_front:
            raise ValueError("Front unsprung mass exceeds front axle mass")
        if 2 * self.rear.unsprung_mass_kg >= self.vehicle.mass_kg * (1 - self.vehicle.weight_front):
            raise ValueError("Rear unsprung mass exceeds rear axle mass")
        return self


def effective_ay_g(M: "ManeuverIn") -> float:
    """Steady-state lateral acceleration in g (corner mode: v^2 / (R g))."""
    if M.ay_source == "corner":
        v = M.speed_kmh / 3.6
        return v * v / (M.radius_m * M.g)
    return M.ay_g
