"""FastAPI app: JSON API + static browser UI.

Run from the repository root:
    pip install -r backend/requirements.txt
    uvicorn backend.app:app --reload
then open http://127.0.0.1:8000
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vd import cornering, kin_summary, kinematics2d, report, sweep, tire  # noqa: E402
from vd.derivation import Calc  # noqa: E402
from vd.schemas import ModelIn  # noqa: E402

FRONTEND = Path(__file__).resolve().parents[1] / "frontend"

app = FastAPI(title="Sakkawma Vehicle Dynamics", version="0.1.0")


def _clean(o):
    """Recursively replace NaN/inf with None so the JSON is valid."""
    if isinstance(o, float):
        return None if (math.isnan(o) or math.isinf(o)) else o
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    return o


def full_calc(m: ModelIn) -> tuple[Calc, dict]:
    c = Calc()
    r = cornering.compute(m, c)
    lim = cornering.limit_ay(m, c)
    kin = kin_summary.compute(m, c)
    return c, {"cornering": r, "limit": lim, "kin": kin}


# ---------------------------------------------------------------- API models
class ComputeReq(BaseModel):
    model: ModelIn


class PoseReq(BaseModel):
    model: ModelIn
    axle: str = Field("front", pattern="^(front|rear)$")
    phi_deg: float = 0.0
    heave_mm: float = 0.0
    bump_o_mm: float = 0.0
    bump_i_mm: float = 0.0


class FramesReq(BaseModel):
    model: ModelIn
    axle: str = Field("front", pattern="^(front|rear)$")
    mode: str = Field("roll", pattern="^(roll|heave|bump)$")
    values: list[float]


class SweepReq(BaseModel):
    model: ModelIn
    x_path: str = "maneuver.ay_g"
    x_values: list[float]
    compare_path: Optional[str] = None
    compare_values: Optional[list[float]] = None
    include_limit: bool = False


class KinSweepReq(BaseModel):
    model: ModelIn
    axle: str = Field("front", pattern="^(front|rear)$")
    mode: str = Field("heave", pattern="^(roll|heave|bump)$")
    values: list[float]
    compare_path: Optional[str] = None
    compare_values: Optional[list[float]] = None


class TireReq(BaseModel):
    model: ModelIn
    loads: Optional[list[float]] = None


class ReportReq(BaseModel):
    model: ModelIn
    notes: str = ""


# ---------------------------------------------------------------- endpoints
@app.get("/api/defaults")
def defaults():
    m = ModelIn()
    return {
        "model": m.model_dump(),
        "params": sweep.param_catalog(m),
        "outputs": {k: {"label": v[0], "unit": v[1], "group": v[2]} for k, v in sweep.OUTPUTS.items()},
        "kin_outputs": {k: {"label": v[0], "unit": v[1]} for k, v in sweep.KIN_OUTPUTS.items()},
        "assumptions": report.ASSUMPTIONS,
    }


@app.post("/api/compute")
def compute(req: ComputeReq):
    c, res = full_calc(req.model)
    return JSONResponse(_clean({
        "steps": c.to_list(),
        "warnings": c.warnings,
        "limit": res["limit"],
        "kin": res["kin"],
    }))


@app.post("/api/pose")
def pose(req: PoseReq):
    ax = getattr(req.model, req.axle)
    s = kinematics2d.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm, phi_deg=req.phi_deg,
                                heave_mm=req.heave_mm, bump_o_mm=req.bump_o_mm, bump_i_mm=req.bump_i_mm,
                                h_cg_mm=req.model.vehicle.h_cg_mm)
    return JSONResponse(_clean(s))


@app.post("/api/pose_frames")
def pose_frames(req: FramesReq):
    if len(req.values) > 400:
        raise HTTPException(400, "too many frames (max 400)")
    ax = getattr(req.model, req.axle)
    frames = []
    for v in req.values:
        kw = {"roll": {"phi_deg": v}, "heave": {"heave_mm": -v}, "bump": {"bump_o_mm": v}}[req.mode]
        frames.append(kinematics2d.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm,
                                              h_cg_mm=req.model.vehicle.h_cg_mm, **kw))
    return JSONResponse(_clean({"mode": req.mode, "values": req.values, "frames": frames}))


@app.post("/api/sweep")
def run_sweep(req: SweepReq):
    n = len(req.x_values) * max(len(req.compare_values or [None]), 1)
    if n > 2000 or (req.include_limit and n > 400):
        raise HTTPException(400, "sweep too large (max 2000 points, 400 with limit a_y)")
    try:
        res = sweep.run_sweep(req.model, req.x_path, req.x_values, req.compare_path,
                              req.compare_values, req.include_limit)
    except KeyError as e:
        raise HTTPException(400, f"unknown parameter path {e}")
    return JSONResponse(_clean(res))


@app.post("/api/kin_sweep")
def run_kin_sweep(req: KinSweepReq):
    try:
        res = sweep.run_kin_sweep(req.model, req.axle, req.mode, req.values,
                                  req.compare_path, req.compare_values)
    except KeyError as e:
        raise HTTPException(400, f"unknown parameter path {e}")
    return JSONResponse(_clean(res))


@app.post("/api/tire_curves")
def tire_curves(req: TireReq):
    t = req.model.tire
    loads = req.loads or [0.5 * t.Fz_nom_N, t.Fz_nom_N, 1.5 * t.Fz_nom_N, 2 * t.Fz_nom_N]
    return JSONResponse(_clean({
        "fy_alpha": tire.curves(t, loads),
        "fy_fz": tire.load_curves(t, [1, 2, 4, 8], 2.5 * t.Fz_nom_N),
    }))


@app.post("/api/report", response_class=HTMLResponse)
def make_report(req: ReportReq):
    c, _ = full_calc(req.model)
    return HTMLResponse(report.build(req.model.model_dump(), c, notes=req.notes))


# ---------------------------------------------------------------- static UI
app.mount("/static", StaticFiles(directory=FRONTEND), name="static")


@app.get("/")
def index():
    return FileResponse(FRONTEND / "index.html")
