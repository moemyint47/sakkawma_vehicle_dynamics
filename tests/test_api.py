import sys
from pathlib import Path

from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from backend.app import app  # noqa: E402

c = TestClient(app)
M = c.get("/api/defaults").json()["model"]


def test_compute_has_derivations_for_every_result():
    r = c.post("/api/compute", json={"model": M}).json()
    keys = {s["key"] for s in r["steps"]}
    for k in ("f.dFz_g", "f.dFz_e", "r.alpha", "ay_max", "kfront.h_rc", "krear.rc_lat_mig"):
        assert k in keys
    for s in r["steps"]:
        assert s["formula"] and s["subst"]


def test_endpoints_ok():
    assert c.post("/api/sweep", json={"model": M, "x_values": [0.5, 1.0], "compare_path": "front.h_rc_mm",
                                      "compare_values": [20, 60]}).status_code == 200
    assert c.post("/api/kin_sweep", json={"model": M, "mode": "roll", "values": [-1, 0, 1]}).status_code == 200
    assert c.post("/api/pose_frames", json={"model": M, "mode": "bump", "values": [0, 20]}).status_code == 200
    assert c.post("/api/tire_curves", json={"model": M}).status_code == 200
    assert "<html" in c.post("/api/report", json={"model": M}).text


def test_bad_input_rejected():
    bad = {**M, "vehicle": {**M["vehicle"], "mass_kg": -5}}
    assert c.post("/api/compute", json={"model": bad}).status_code == 422
    assert c.post("/api/sweep", json={"model": M, "x_path": "nope.x", "x_values": [1]}).status_code == 400


def test_workspace_roundtrip(tmp_path, monkeypatch):
    import backend.app as A
    monkeypatch.setattr(A, "WORKSPACES", tmp_path)
    body = {"name": "Test WS", "layout": [], "model": M}
    assert c.put("/api/workspaces/test-ws", json=body).status_code == 200
    assert any(w["id"] == "test-ws" for w in c.get("/api/workspaces").json())
    assert c.get("/api/workspaces/test-ws").json()["name"] == "Test WS"
    c.delete("/api/workspaces/test-ws")
    assert c.get("/api/workspaces/test-ws").status_code == 404


def test_adapt_and_transient_endpoints():
    r = c.post("/api/adapt", json={"model": M, "path": "rear.h_rc_mm", "value": 30}).json()
    assert any(ch["path"] == "rear.hardpoints.uca_in.z" for ch in r["changes"])
    r = c.post("/api/transient", json={"model": M, "compare_path": "front.spring.mr_c1",
                                       "compare_values": [0, 0.004]})
    assert r.status_code == 200 and len(r.json()["series"]) == 2
