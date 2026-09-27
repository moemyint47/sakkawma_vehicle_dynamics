"""Target-driven optimisation of selected parameters.

Objective (minimised), summed over the targets with weights w_k:
    maximise v   :  -w v / s            s = max(|v_initial|, 1e-9)
    minimise v   :  +w v / s
    v >= a       :  w * 100 * (max(0, a - v) / d)^2          d = max(0.05|a|, 1e-3)
    v <= a       :  w * 100 * (max(0, v - a) / d)^2
    a <= v <= b  :  w * 100 * (dist(v, [a, b]) / d)^2       d = max(0.05 (|a|+|b|)/2, 1e-3)
Variables are scaled to [0, 1] inside their bounds; derivative-free bounded
Powell search (scipy). Parameter changes go through the adaptive rules when the
model is in adaptive mode (e.g. RC height moves the inboard pivots).
"""
from __future__ import annotations

import math
import time

import numpy as np
from scipy.optimize import minimize

from . import cornering, kin_summary
from .derivation import Calc, NullCalc, fmt
from .schemas import ModelIn
from .sweep import with_param, get_path

class _Budget(Exception):
    pass


LIMIT_KEYS = {"ay_max", "v_max", "v_max_kmh", "lap_time"}


def evaluate(m: ModelIn, keys: set[str]) -> dict:
    c = Calc()
    cornering.compute(m, c, with_gradient=any(k == "K_us" for k in keys))
    if keys & LIMIT_KEYS:
        cornering.limit_ay(m, c)
    if any(k.startswith("k") and "." in k and k.split(".")[0] in ("kfront", "krear") for k in keys):
        kin_summary.compute(m, c)
    if any(k.startswith(("tr.", "p.")) for k in keys):
        from . import transient
        transient.simulate(m, c)
    return {s.key: s.value for s in c.steps}


def penalty(t: dict, v: float, scale0: float) -> float:
    w = float(t.get("weight", 1.0) or 1.0)
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return 1e4 * w
    op = t["op"]
    if op == "max":
        return -w * v / scale0
    if op == "min":
        return w * v / scale0
    a = float(t.get("a", 0.0) or 0.0)
    b = float(t.get("b", a) if t.get("b") is not None else a)
    if op == "ge":
        d = max(0.05 * abs(a), 1e-3); return w * 100 * (max(0.0, a - v) / d) ** 2
    if op == "le":
        d = max(0.05 * abs(a), 1e-3); return w * 100 * (max(0.0, v - a) / d) ** 2
    if op == "between":
        lo, hi = min(a, b), max(a, b)
        d = max(0.05 * (abs(lo) + abs(hi)) / 2, 1e-3)
        dist = lo - v if v < lo else (v - hi if v > hi else 0.0)
        return w * 100 * (dist / d) ** 2
    raise ValueError(f"unknown target op {op}")


def run(model: ModelIn, variables: list[dict], targets: list[dict], max_evals: int = 60) -> dict:
    if not variables:
        raise ValueError("choose at least one variable")
    if not targets:
        raise ValueError("define at least one target")
    keys = {t["key"] for t in targets}
    lo = np.array([float(v["min"]) for v in variables])
    hi = np.array([float(v["max"]) for v in variables])
    if np.any(hi <= lo):
        raise ValueError("each variable needs max > min")
    d0 = model.model_dump()
    x_init = np.array([float(get_path(d0, v["path"])) for v in variables])
    u0 = np.clip((x_init - lo) / (hi - lo), 0, 1)
    base_vals = evaluate(model, keys)
    scale = {t["key"]: max(abs(base_vals.get(t["key"]) or 0.0), 1e-9) for t in targets}
    hist, best = [], {"J": math.inf}
    t_start = time.time()

    def build(u):
        m = model
        for v, x in zip(variables, lo + np.clip(u, 0, 1) * (hi - lo)):
            m = with_param(m, v["path"], float(x))
        return m

    def J(u):
        try:
            m = build(u)
            vals = evaluate(m, keys)
            parts = [penalty(t, vals.get(t["key"]), scale[t["key"]]) for t in targets]
            j = float(sum(parts))
        except Exception as e:  # infeasible geometry etc.
            j, vals, m, parts = 1e6, {}, None, []
        x = (lo + np.clip(u, 0, 1) * (hi - lo)).tolist()
        hist.append({"J": j, "x": x})
        if j < best["J"]:
            best.update(J=j, x=x, vals=vals, model=m, parts=parts)
        if len(hist) >= max_evals:
            raise _Budget
        return j

    j0 = J(u0)
    initial = {"J": j0, "x": x_init.tolist()}
    try:
        # 1) global exploration: space-filling samples (1/3 of the budget), 2) local bounded Powell from the best
        n = len(variables)
        n_init = max(4, max_evals // 3)
        rng = np.random.default_rng(0)
        # Latin-hypercube style samples
        samples = (np.argsort(rng.random((n_init, n)), axis=0) + rng.random((n_init, n))) / n_init
        for u in samples:
            J(u)
        ub = (np.array(best["x"]) - lo) / (hi - lo) if best.get("x") is not None else u0
        minimize(J, np.clip(ub, 0, 1), method="Powell", bounds=[(0.0, 1.0)] * n,
                 options={"maxfev": max_evals, "xtol": 1e-3, "ftol": 1e-6})
    except _Budget:
        pass

    calc = Calc()
    S = "12. Optimisation"
    calc.add("opt.J0", r"J_0", "Objective at the start point", r"\sum_k w_k\,\pi_k(v_k)",
             fmt(initial["J"]), initial["J"], "-", S, note="pi_k: see objective definition in vd/optimize.py")
    calc.add("opt.J", r"J^*", "Objective at the best point", r"\sum_k w_k\,\pi_k(v_k)",
             "+".join(fmt(p) for p in best.get("parts", [])) or "-", best["J"], "-", S)
    for v, x0, x in zip(variables, x_init, best.get("x", x_init)):
        calc.add(f"opt.x.{v['path']}", v["path"].replace("_", r"\_"), f"Optimised {v['path']}",
                 rf"\in[{fmt(v['min'])},\ {fmt(v['max'])}]", rf"\text{{start }}{fmt(x0)}", x, "", S)
    for t in targets:
        v = best.get("vals", {}).get(t["key"])
        calc.add(f"opt.t.{t['key']}", t["key"].replace("_", r"\_"), f"Target {t['key']} ({t['op']})",
                 r"\text{value at optimum}", rf"\text{{start }}{fmt(base_vals.get(t['key']))}", v, "", S)
    return {
        "evals": len(hist), "seconds": time.time() - t_start,
        "history": hist, "initial": initial,
        "best": {"J": best["J"], "x": best.get("x"), "values": {t["key"]: best.get("vals", {}).get(t["key"]) for t in targets},
                 "start_values": {t["key"]: base_vals.get(t["key"]) for t in targets}},
        "model": best["model"].model_dump() if best.get("model") is not None else None,
        "steps": calc.to_list(),
    }
