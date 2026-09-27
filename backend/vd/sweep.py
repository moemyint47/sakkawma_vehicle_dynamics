"""Parameter sweeps and multi-configuration comparison."""
from __future__ import annotations

import copy
import math
from typing import Any

from . import cornering, kinematics2d
from .derivation import NullCalc
from .schemas import ModelIn

# output catalogue: key -> (label, unit, group)
OUTPUTS: dict[str, tuple[str, str, str]] = {
    "phi_deg": ("Body roll angle", "deg", "Roll"),
    "h1_mm": ("Roll moment arm h1", "mm", "Roll"),
    "f.dFz": ("Front total ΔFz (per wheel)", "N", "Front load transfer"),
    "f.dFz_g": ("Front geometric ΔFz", "N", "Front load transfer"),
    "f.dFz_e": ("Front elastic ΔFz", "N", "Front load transfer"),
    "f.dFz_u": ("Front unsprung ΔFz", "N", "Front load transfer"),
    "f.geo_share": ("Front geometric share", "-", "Front load transfer"),
    "f.el_share": ("Front elastic share", "-", "Front load transfer"),
    "r.dFz": ("Rear total ΔFz (per wheel)", "N", "Rear load transfer"),
    "r.dFz_g": ("Rear geometric ΔFz", "N", "Rear load transfer"),
    "r.dFz_e": ("Rear elastic ΔFz", "N", "Rear load transfer"),
    "r.dFz_u": ("Rear unsprung ΔFz", "N", "Rear load transfer"),
    "r.geo_share": ("Rear geometric share", "-", "Rear load transfer"),
    "r.el_share": ("Rear elastic share", "-", "Rear load transfer"),
    "lltd_front": ("LLTD front", "-", "Distribution"),
    "f.Fz_out": ("Front outer wheel load", "N", "Wheel loads"),
    "f.Fz_in": ("Front inner wheel load", "N", "Wheel loads"),
    "r.Fz_out": ("Rear outer wheel load", "N", "Wheel loads"),
    "r.Fz_in": ("Rear inner wheel load", "N", "Wheel loads"),
    "f.alpha_deg": ("Front slip angle", "deg", "Tires & balance"),
    "r.alpha_deg": ("Rear slip angle", "deg", "Tires & balance"),
    "dalpha_deg": ("α_f − α_r (understeer > 0)", "deg", "Tires & balance"),
    "delta_deg": ("Required steer angle", "deg", "Tires & balance"),
    "f.util": ("Front grip utilisation", "-", "Tires & balance"),
    "r.util": ("Rear grip utilisation", "-", "Tires & balance"),
    "f.Fy_max": ("Front axle capacity", "N", "Tires & balance"),
    "r.Fy_max": ("Rear axle capacity", "N", "Tires & balance"),
    "f.lt_loss": ("Front force lost to LT", "N", "Tires & balance"),
    "r.lt_loss": ("Rear force lost to LT", "N", "Tires & balance"),
    "f.h_rc_eff": ("Front effective RC height (migrated)", "mm", "Roll centre & jacking"),
    "r.h_rc_eff": ("Rear effective RC height (migrated)", "mm", "Roll centre & jacking"),
    "f.jack": ("Front jacking force (+ up)", "N", "Roll centre & jacking"),
    "r.jack": ("Rear jacking force (+ up)", "N", "Roll centre & jacking"),
    "f.dz_jack": ("Front ride-height change from jacking", "mm", "Roll centre & jacking"),
    "r.dz_jack": ("Rear ride-height change from jacking", "mm", "Roll centre & jacking"),
    "f.outer_share": ("Front outer-tyre share of side force", "-", "Roll centre & jacking"),
    "ay_max_g": ("Limit lateral acceleration", "g", "Limit"),
    "v_max_kmh": ("Limit speed on radius R", "km/h", "Limit"),
    "tr.phi_peak": ("Peak roll angle (transient)", "deg", "Transient"),
    "tr.overshoot": ("Roll overshoot", "%", "Transient"),
    "tr.t90": ("Time to 90 % roll", "s", "Transient"),
    "tr.f.pct_geo_50": ("Front geometric share @ t0+50 ms", "%", "Transient"),
    "tr.f.pct_damper_50": ("Front damper share @ t0+50 ms", "%", "Transient"),
    "tr.r.pct_geo_50": ("Rear geometric share @ t0+50 ms", "%", "Transient"),
    "tr.r.pct_damper_50": ("Rear damper share @ t0+50 ms", "%", "Transient"),
    "tr.f.vdmax": ("Front peak damper velocity", "mm/s", "Transient"),
    "tr.r.vdmax": ("Rear peak damper velocity", "mm/s", "Transient"),
}

KIN_OUTPUTS = {
    "camber_o_deg": ("Outer camber to ground", "deg"),
    "camber_i_deg": ("Inner camber to ground", "deg"),
    "camber_o_body_deg": ("Outer camber to body", "deg"),
    "rc_height_mm": ("Roll-centre height", "mm"),
    "rc_lateral_mm": ("Roll-centre lateral position (+ outer)", "mm"),
    "track_change_mm": ("Track change", "mm"),
    "travel_o_mm": ("Outer wheel travel (+ jounce)", "mm"),
    "travel_i_mm": ("Inner wheel travel (+ jounce)", "mm"),
}


def get_path(d: dict, path: str) -> Any:
    for p in path.split("."):
        d = d[int(p)] if isinstance(d, list) else d[p]
    return d


def set_path(d: dict, path: str, value) -> None:
    parts = path.split(".")
    for p in parts[:-1]:
        d = d[p]
    if parts[-1] not in d:
        raise KeyError(path)
    d[parts[-1]] = value


def with_param(base: ModelIn, path: str | None, value) -> ModelIn:
    """Return a copy with one parameter changed. In adaptive mode the dependent
    parameters follow (same rules as editing in the UI)."""
    if not path:
        return base
    if base.param_mode == "adaptive" and path != "maneuver.ay_g" and not path.startswith("maneuver."):
        from . import adapt
        return adapt.apply(base, path, value)[0]
    d = base.model_dump()
    set_path(d, path, value)
    if path == "maneuver.ay_g":  # sweeping a_y directly overrides corner mode
        d["maneuver"]["ay_source"] = "input"
    return ModelIn.model_validate(d)


def flat_outputs(m: ModelIn, include_limit: bool, include_transient: bool = False) -> dict:
    nan = float("nan")
    try:
        r = cornering.compute(m, NullCalc(), with_gradient=False)
    except Exception:  # invalid combination -> NaNs
        return {k: nan for k in OUTPUTS}
    lt = r["lt"]
    o = {"phi_deg": lt["phi_deg"], "h1_mm": lt["h1"] * 1000, "lltd_front": lt["lltd_front"],
         "dalpha_deg": r["dalpha_deg"], "delta_deg": r["delta_deg"]}
    geo = r.get("geo")
    for t, ax in (("f", m.front), ("r", m.rear)):
        for k in ("dFz", "dFz_g", "dFz_e", "dFz_u", "geo_share", "el_share", "Fz_out", "Fz_in"):
            o[f"{t}.{k}"] = lt["axles"][t][k]
        if geo:
            a = geo["axles"][t]
            o[f"{t}.h_rc_eff"] = geo["h"][t] * 1000
            o[f"{t}.jack"] = a["J"]
            o[f"{t}.outer_share"] = a["r"]
            from .suspension import AxleSuspension
            wf = m.vehicle.weight_front if t == "f" else 1 - m.vehicle.weight_front
            su = AxleSuspension(ax, m.vehicle.mass_kg * wf - 2 * ax.unsprung_mass_kg, m.maneuver.g)
            kw = su.K_direct / (1000 * su.t ** 2 / 2) if su.direct else su.wheel_rate(0.0)
            o[f"{t}.dz_jack"] = a["J"] / (2 * kw) if kw > 0 else nan
        else:
            o[f"{t}.h_rc_eff"] = ax.h_rc_mm
            o[f"{t}.jack"] = 0.0
            o[f"{t}.dz_jack"] = 0.0
            o[f"{t}.outer_share"] = nan
        for k in ("alpha_deg", "util", "Fy_max", "lt_loss"):
            o[f"{t}.{k}"] = r["axles"][t].get(k, nan)
    if include_limit:
        L = cornering.limit_ay(m)
        o["ay_max_g"] = L["ay_max_g"]
        o["v_max_kmh"] = 3.6 * math.sqrt(L["ay_max_g"] * m.maneuver.g * m.maneuver.radius_m)
    else:
        o["ay_max_g"] = o["v_max_kmh"] = nan
    if include_transient:
        from . import transient
        from .derivation import Calc
        c = Calc()
        try:
            transient.simulate(m, c)
            vals = {st.key: st.value for st in c.steps}
        except Exception:
            vals = {}
        for k in OUTPUTS:
            if k.startswith("tr."):
                o[k] = vals.get(k, nan)
    return o


def _clean(v):
    if v is None:
        return None
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return None
        if abs(v) < 1e-9:  # numerical noise of exactly-zero quantities (e.g. RC lateral in heave)
            return 0.0
    return v


def run_sweep(base: ModelIn, x_path: str, x_values: list[float], compare_path: str | None,
              compare_values: list[float] | None, include_limit: bool = False,
              include_transient: bool = False) -> dict:
    compare_values = compare_values if compare_path else [None]
    d = base.model_dump()
    for p in (x_path, compare_path):
        if p:
            get_path(d, p)  # raises KeyError for unknown parameter paths
    series = []
    for cv in compare_values:
        mc = with_param(base, compare_path, cv) if compare_path else base
        cols: dict[str, list] = {k: [] for k in OUTPUTS}
        for xv in x_values:
            try:
                mx = with_param(mc, x_path, xv)
                o = flat_outputs(mx, include_limit, include_transient)
            except Exception:
                o = {k: float("nan") for k in OUTPUTS}
            for k in OUTPUTS:
                cols[k].append(_clean(o.get(k)))
        series.append({"compare_value": cv, "outputs": cols})
    return {"x_path": x_path, "x": x_values, "compare_path": compare_path, "series": series}


def run_kin_sweep(base: ModelIn, axle: str, mode: str, values: list[float],
                  compare_path: str | None, compare_values: list[float] | None) -> dict:
    compare_values = compare_values if compare_path else [None]
    series = []
    for cv in compare_values:
        mc = with_param(base, compare_path, cv) if compare_path else base
        ax = getattr(mc, axle)
        res = kinematics2d.sweep(ax.hardpoints, ax.hardpoints, ax.track_mm, mode, values,
                                 h_cg_mm=mc.vehicle.h_cg_mm)
        series.append({"compare_value": cv,
                       "outputs": {k: [_clean(v) for v in res[k]] for k in KIN_OUTPUTS}})
    return {"mode": mode, "axle": axle, "x": values, "compare_path": compare_path, "series": series}


def param_catalog(m: ModelIn) -> list[dict]:
    """All numeric scalar inputs (dotted paths) for sweep/compare selectors."""
    out = []

    def walk(d, prefix):
        for k, v in d.items():
            p = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                walk(v, p)
            elif isinstance(v, (int, float)) and not isinstance(v, bool):
                out.append({"path": p, "value": v})
    walk(copy.deepcopy(m.model_dump()), "")
    return out


def run_sweep2d(base: ModelIn, x_path: str, x_values: list[float], y_path: str, y_values: list[float],
                include_limit: bool = False, include_transient: bool = False) -> dict:
    """Grid over two parameters; returns every output as grid[y_index][x_index]."""
    d = base.model_dump()
    get_path(d, x_path); get_path(d, y_path)
    grids = {k: [] for k in OUTPUTS}
    for yv in y_values:
        rows = {k: [] for k in OUTPUTS}
        try:
            my = with_param(base, y_path, yv)
        except Exception:
            my = None
        for xv in x_values:
            try:
                o = flat_outputs(with_param(my, x_path, xv), include_limit, include_transient) if my else {}
            except Exception:
                o = {}
            for k in OUTPUTS:
                rows[k].append(_clean(o.get(k)))
        for k in OUTPUTS:
            grids[k].append(rows[k])
    return {"x_path": x_path, "x": x_values, "y_path": y_path, "y": y_values, "grids": grids}
