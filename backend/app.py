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

from vd import adapt, cornering, kin_summary, kinematics2d, report, sweep, tire, transient  # noqa: E402
from vd.suspension import AxleSuspension  # noqa: E402
from vd.derivation import Calc  # noqa: E402
from vd.schemas import ModelIn  # noqa: E402

FRONTEND = Path(__file__).resolve().parents[1] / "frontend"
WORKSPACES = Path(__file__).resolve().parents[1] / "workspaces"

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


def full_calc(m: ModelIn, with_transient: bool = True) -> tuple[Calc, dict]:
    c = Calc()
    r = cornering.compute(m, c)
    lim = cornering.limit_ay(m, c)
    kin = kin_summary.compute(m, c)
    tr = None
    if with_transient:
        try:
            tr = transient.simulate(m, c)
        except Exception as e:  # keep steady-state results usable
            c.warn(f"Transient: {e}")
    return c, {"cornering": r, "limit": lim, "kin": kin, "transient": tr}


def susp_curves(m: ModelIn) -> dict:
    out = {}
    g = m.maneuver.g
    for tag, ax, mf in (("f", m.front, m.vehicle.weight_front), ("r", m.rear, 1 - m.vehicle.weight_front)):
        ms_ax = m.vehicle.mass_kg * mf - 2 * ax.unsprung_mass_kg
        s = AxleSuspension(ax, ms_ax, g)
        out[tag] = {"wheel": s.curves(), "roll": s.roll_curves(), "damper": s.damper_curve(),
                    "direct": s.direct}
    return out


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
    include_transient: bool = False


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


def _bc(ax):
    return ax.spring.bellcrank if ax.spring.mr_mode == "bellcrank" else None


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
        "transient": res["transient"],
        "susp": susp_curves(req.model),
    }))


class AdaptReq(BaseModel):
    model: ModelIn
    path: str
    value: object = None


@app.post("/api/adapt")
def adapt_param(req: AdaptReq):
    try:
        new, changes, c = adapt.apply(req.model, req.path, req.value)
    except KeyError as e:
        raise HTTPException(400, f"unknown parameter path {e}")
    except (ValueError, TypeError) as e:
        raise HTTPException(422, str(e))
    return JSONResponse(_clean({"model": new.model_dump(), "changes": changes, "steps": c.to_list()}))


class Sweep2dReq(BaseModel):
    model: ModelIn
    x_path: str
    x_values: list[float]
    y_path: str
    y_values: list[float]
    include_limit: bool = False
    include_transient: bool = False


@app.post("/api/sweep2d")
def sweep2d(req: Sweep2dReq):
    n = len(req.x_values) * len(req.y_values)
    if n > 625 or (req.include_limit and n > 225) or (req.include_transient and n > 121):
        raise HTTPException(400, "map too large (max 25x25, 15x15 with limit a_y, 11x11 with transient)")
    try:
        res = sweep.run_sweep2d(req.model, req.x_path, req.x_values, req.y_path, req.y_values,
                                req.include_limit, req.include_transient)
    except KeyError as e:
        raise HTTPException(400, f"unknown parameter path {e}")
    return JSONResponse(_clean(res))


class OptReq(BaseModel):
    model: ModelIn
    variables: list[dict]
    targets: list[dict]
    max_evals: int = Field(60, ge=5, le=400)


@app.post("/api/optimize")
def optimize_ep(req: OptReq):
    from vd import optimize
    try:
        return JSONResponse(_clean(optimize.run(req.model, req.variables, req.targets, req.max_evals)))
    except (ValueError, KeyError) as e:
        raise HTTPException(422, str(e))


class ReplayReq(BaseModel):
    model: ModelIn
    dt: float = Field(0.01, ge=0.002, le=0.1)
    band_deg: float = Field(0.1, ge=0, le=5)


@app.post("/api/replay")
def replay_ep(req: ReplayReq):
    from vd import replay
    c = Calc()
    try:
        r = replay.run(req.model, dt=req.dt, band_deg=req.band_deg, calc=c)
    except (ValueError, RuntimeError) as e:
        raise HTTPException(422, str(e))
    r["steps"] = c.to_list()
    r["warnings"] = c.warnings
    return JSONResponse(_clean(r))


class TransientCmpReq(BaseModel):
    model: ModelIn
    compare_path: Optional[str] = None
    compare_values: Optional[list] = None


@app.post("/api/transient")
def transient_compare(req: TransientCmpReq):
    vals = req.compare_values if req.compare_path else [None]
    if len(vals) > 8:
        raise HTTPException(400, "max 8 compare values")
    series = []
    for v in vals:
        try:
            mm = sweep.with_param(req.model, req.compare_path, v) if req.compare_path else req.model
            series.append({"compare_value": v, "result": transient.simulate(mm), "susp": susp_curves(mm)})
        except KeyError as e:
            raise HTTPException(400, f"unknown parameter path {e}")
        except Exception as e:
            series.append({"compare_value": v, "error": str(e)})
    return JSONResponse(_clean({"compare_path": req.compare_path, "series": series}))


# ---------------------------------------------------------------- workspaces (files in /workspaces)
import json as _json  # noqa: E402
import re as _re  # noqa: E402


def _ws_path(name: str) -> Path:
    slug = _re.sub(r"[^a-z0-9_-]+", "-", name.strip().lower()).strip("-")
    if not slug:
        raise HTTPException(400, "invalid workspace name")
    return WORKSPACES / f"{slug}.json"


@app.get("/api/workspaces")
def ws_list():
    WORKSPACES.mkdir(exist_ok=True)
    out = []
    for f in sorted(WORKSPACES.glob("*.json")):
        try:
            d = _json.loads(f.read_text(encoding="utf-8"))
            out.append({"id": f.stem, "name": d.get("name", f.stem), "description": d.get("description", "")})
        except Exception:
            continue
    return out


@app.get("/api/workspaces/{wid}")
def ws_get(wid: str):
    f = _ws_path(wid)
    if not f.exists():
        raise HTTPException(404, "workspace not found")
    return _json.loads(f.read_text(encoding="utf-8"))


@app.put("/api/workspaces/{wid}")
def ws_put(wid: str, body: dict):
    WORKSPACES.mkdir(exist_ok=True)
    if "model" in body and body["model"] is not None:
        ModelIn.model_validate(body["model"])  # validate parameter set
    f = _ws_path(wid)
    f.write_text(_json.dumps(body, indent=2), encoding="utf-8")
    return {"id": f.stem, "saved": True}


@app.delete("/api/workspaces/{wid}")
def ws_delete(wid: str):
    f = _ws_path(wid)
    if f.exists():
        f.unlink()
    return {"deleted": True}


@app.post("/api/pose")
def pose(req: PoseReq):
    ax = getattr(req.model, req.axle)
    s = kinematics2d.solve_pose(ax.hardpoints, ax.hardpoints, ax.track_mm, phi_deg=req.phi_deg,
                                heave_mm=req.heave_mm, bump_o_mm=req.bump_o_mm, bump_i_mm=req.bump_i_mm,
                                h_cg_mm=req.model.vehicle.h_cg_mm, bc=_bc(ax))
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
                                              h_cg_mm=req.model.vehicle.h_cg_mm, bc=_bc(ax), **kw))
    return JSONResponse(_clean({"mode": req.mode, "values": req.values, "frames": frames}))


@app.post("/api/sweep")
def run_sweep(req: SweepReq):
    n = len(req.x_values) * max(len(req.compare_values or [None]), 1)
    if n > 2000 or (req.include_limit and n > 400) or (req.include_transient and n > 200):
        raise HTTPException(400, "sweep too large (max 2000 points, 400 with limit a_y, 200 with transient)")
    try:
        res = sweep.run_sweep(req.model, req.x_path, req.x_values, req.compare_path,
                              req.compare_values, req.include_limit, req.include_transient)
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
