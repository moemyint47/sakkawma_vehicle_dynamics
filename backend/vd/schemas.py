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

from typing import Optional

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


class AxleIn(BaseModel):
    track_mm: float = Field(gt=0)
    h_rc_mm: float = Field(description="Roll-centre height (load-transfer input)")
    roll_stiffness_Nm_deg: float = Field(gt=0, description="Axle roll stiffness incl. ARB")
    unsprung_mass_kg: float = Field(12.0, ge=0, description="Unsprung mass PER CORNER")
    h_unsprung_mm: float = Field(228.0, ge=0, description="Unsprung CG height (~ wheel centre)")
    hardpoints: Hardpoints


class VehicleIn(BaseModel):
    mass_kg: float = Field(280.0, gt=0, description="Total mass incl. driver")
    weight_front: float = Field(0.47, gt=0, lt=1, description="Static front weight fraction")
    wheelbase_mm: float = Field(1550.0, gt=0)
    h_cg_mm: float = Field(290.0, gt=0, description="Total CG height")


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


class ManeuverIn(BaseModel):
    ay_g: float = Field(1.2, ge=0, description="Steady-state lateral acceleration")
    radius_m: float = Field(9.125, gt=0, description="Path radius (default: FSAE skidpad centreline)")
    g: float = Field(9.81, gt=0)
    roll_gravity_term: bool = Field(True, description="Include m_s g h1 phi (lateral CG shift) in roll")


def _default_front() -> AxleIn:
    return AxleIn(
        track_mm=1220, h_rc_mm=35, roll_stiffness_Nm_deg=380,
        unsprung_mass_kg=12, h_unsprung_mm=228,
        hardpoints=Hardpoints(
            lca_in=Point2(y=210, z=120.5), lca_out=Point2(y=580, z=125),
            uca_in=Point2(y=255, z=282.2), uca_out=Point2(y=560, z=320),
            tire_radius_mm=228, static_camber_deg=-1.5),
    )


def _default_rear() -> AxleIn:
    return AxleIn(
        track_mm=1180, h_rc_mm=55, roll_stiffness_Nm_deg=320,
        unsprung_mass_kg=13, h_unsprung_mm=228,
        hardpoints=Hardpoints(
            lca_in=Point2(y=220, z=130.2), lca_out=Point2(y=560, z=125),
            uca_in=Point2(y=260, z=285.0), uca_out=Point2(y=540, z=315),
            tire_radius_mm=228, static_camber_deg=-1.0),
    )


class ModelIn(BaseModel):
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
