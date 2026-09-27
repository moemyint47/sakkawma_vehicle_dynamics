// Shared application state, API helpers, adaptive parameter editing, baseline & compare runs.
import { getPath, setPath } from "./spec.js";

export const state = {
  defaults: null, model: null, res: null,
  base: null,          // {model, res} baseline snapshot
  cmp: null,           // {path, values, series:[{compare_value, result, susp}]}
  cmpSpec: { path: "", values: [] },
  lastAdapt: null,     // {changes, steps}
};

const listeners = {};
export const on = (ev, fn) => (listeners[ev] = listeners[ev] || []).push(fn);
export const emit = (ev, data) => (listeners[ev] || []).forEach((fn) => { try { fn(data); } catch (e) { console.error(e); } });

export async function post(url, body, asText = false) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  if (!r.ok) throw new Error(await errText(r));
  return asText ? r.text() : r.json();
}
export async function api(method, url, body) {
  const r = await fetch(url, { method, headers: { "Content-Type": "application/json" }, body: body ? JSON.stringify(body) : undefined });
  if (!r.ok) throw new Error(await errText(r));
  return r.json();
}
async function errText(r) {
  let msg = `${r.status}`;
  try {
    const j = await r.json();
    msg = Array.isArray(j.detail) ? j.detail.map((d) => `${(d.loc || []).slice(1).join(".")}: ${d.msg}`).join("; ") : j.detail || msg;
  } catch { /* ignore */ }
  return msg;
}

export const setStatus = (t, err = false) => {
  const s = document.getElementById("status");
  if (!s) return;
  s.textContent = t; s.classList.toggle("err", err);
};

const STORE = "sakkawma.vd.model.v2";
export const saveLocal = () => { try { localStorage.setItem(STORE, JSON.stringify(state.model)); } catch { /* unavailable */ } };
export const loadLocal = () => { try { const t = localStorage.getItem(STORE); return t ? JSON.parse(t) : null; } catch { return null; } };

// ------------------------------------------------------------------ recompute
let tmr = null, seq = 0;
export function scheduleRecompute(delay = 180) {
  saveLocal();
  emit("model", state.model);
  clearTimeout(tmr);
  tmr = setTimeout(recompute, delay);
}

export async function recompute() {
  const my = ++seq;
  const t0 = performance.now();
  setStatus("computing…");
  try {
    const res = await post("/api/compute", { model: state.model });
    if (my !== seq) return true; // a newer request is in flight
    res.radius = state.model.maneuver.radius_m;
    state.res = res;
    setStatus(`updated · ${Math.round(performance.now() - t0)} ms`);
    emit("results", res);
    if (state.cmpSpec.path && state.cmpSpec.values.length) runCompare();
    return true;
  } catch (e) {
    setStatus(`error: ${e.message}`, true);
    return false;
  }
}

// ------------------------------------------------------------------ parameter edits (absolute / adaptive)
let chain = Promise.resolve(), pending = null;
export function applyParam(path, value) {
  if (state.model.param_mode !== "adaptive" && path !== "param_mode") {
    setPath(state.model, path, value);
    state.lastAdapt = { changes: [{ path, new: value }], steps: [] };
    emit("adapted", state.lastAdapt);
    scheduleRecompute();
    return Promise.resolve();
  }
  // adaptive: serialise requests, keep only the latest pending edit
  pending = { path, value };
  chain = chain.then(async () => {
    if (!pending) return;
    const p = pending; pending = null;
    try {
      const r = await post("/api/adapt", { model: state.model, path: p.path, value: p.value });
      state.model = r.model;
      state.lastAdapt = r;
      emit("adapted", r);
      scheduleRecompute(60);
    } catch (e) {
      setStatus(`adapt: ${e.message}`, true);
      emit("adapted", { changes: [], steps: [], error: e.message });
    }
  });
  return chain;
}

export function replaceModel(m) {
  state.model = m;
  emit("adapted", { changes: [], steps: [] });
  scheduleRecompute(0);
}

// ------------------------------------------------------------------ baseline
export async function setBaseline() {
  state.base = { model: structuredClone(state.model), res: state.res };
  emit("baseline", state.base);
}
export function clearBaseline() { state.base = null; emit("baseline", null); }

// ------------------------------------------------------------------ compare runs (transient + suspension curves)
let cmpSeq = 0;
export async function runCompare() {
  const { path, values } = state.cmpSpec;
  if (!path || !values.length) { state.cmp = null; emit("compare", null); return; }
  const my = ++cmpSeq;
  try {
    const r = await post("/api/transient", { model: state.model, compare_path: path, compare_values: values });
    if (my !== cmpSeq) return;
    state.cmp = r;
    emit("compare", r);
  } catch (e) { setStatus(`compare: ${e.message}`, true); }
}
export function setCompare(path, values) {
  state.cmpSpec = { path: path || "", values: values || [] };
  runCompare();
}

export const val = (key, res = state.res) => {
  if (!res) return null;
  const s = res.steps.find((x) => x.key === key);
  return s ? s.value : null;
};
export { getPath, setPath };
