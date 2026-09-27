import { LABELS, getPath } from "./spec.js";
import { S, setSteps, stepHTML, gotoStep, renderAll } from "./derivations.js";
import { renderResults } from "./results.js";
import { FrontView, readouts } from "./view2d.js";
import { plotSweep, plotKin, plotTire } from "./plots.js";
import { state, post, setStatus, on, recompute, applyParam, replaceModel, loadLocal,
  setBaseline, clearBaseline, setCompare } from "./store.js";
import { buildForm, fillForm } from "./form.js";
import { mountFixed, renderAllIn, paramOptions as wParamOptions } from "./widgets.js";
import { Workspace } from "./workspace.js";
import { replayState as RS, loadReplay, setIdx, TopView, panelHTML, drawPlots, moveCursor, transportHTML, bindTransport } from "./replay.js";

const $ = (s) => document.querySelector(s);
const vs = { frames: null, framesKey: "", playing: false, viewDirty: true };
let ws = null;

on("model", () => { vs.viewDirty = true; });
on("results", (res) => {
  setSteps(res.steps);
  renderResults(res, state.defaults);
  refreshTire();
  if (activeTab() === "view") refreshView();
  renderTransientDeriv(res);
  const pr = $("#tr-probe");
  if (pr && document.activeElement !== pr) pr.value = state.model.maneuver.transient.t_probe_s;
});

function renderTransientDeriv(res) {
  const el = $("#tr-deriv");
  if (!el) return;
  const secs = res.steps.filter((s) => s.key.startsWith("p.") || s.key.startsWith("tr."));
  const keep = S.steps;
  S.steps = secs;
  renderAll(el);
  S.steps = keep;
  el.querySelectorAll("details").forEach((d) => (d.open = true));
}

// ------------------------------------------------------------------ derivation popovers
document.addEventListener("click", (ev) => {
  const g = ev.target.closest("[data-goto]");
  if (g) { ev.preventDefault(); gotoStep(g.dataset.goto); return; }
  const b = ev.target.closest(".fx");
  if (!b) return;
  const key = b.dataset.fx, st = S.byKey[key];
  const tr = b.closest("tr");
  if (b.closest("#kpis")) {
    const host = $("#kpi-deriv");
    const on = b.classList.contains("on");
    document.querySelectorAll("#kpis .fx.on").forEach((x) => x.classList.remove("on"));
    host.innerHTML = on ? "" : stepHTML(st);
    if (!on) b.classList.add("on");
    return;
  }
  if (tr) {
    const next = tr.nextElementSibling;
    const openKey = next && next.classList.contains("deriv-row") ? next.dataset.key : null;
    if (openKey) next.remove();
    if (openKey === key) { b.classList.remove("on"); return; }
    tr.querySelectorAll(".fx.on").forEach((x) => x.classList.remove("on"));
    b.classList.add("on");
    const d = document.createElement("tr");
    d.className = "deriv-row"; d.dataset.key = key;
    d.innerHTML = `<td colspan="${tr.children.length}">${stepHTML(st)}</td>`;
    tr.after(d);
  }
});

// ------------------------------------------------------------------ tabs
const activeTab = () => document.querySelector(".tab.active").dataset.tab;
document.querySelectorAll(".tab").forEach((b) => b.addEventListener("click", () => {
  document.querySelectorAll(".tab").forEach((x) => x.classList.toggle("active", x === b));
  document.querySelectorAll(".panel").forEach((p) => p.classList.toggle("active", p.id === `tab-${b.dataset.tab}`));
  if (b.dataset.tab === "view") refreshView();
  if (b.dataset.tab === "transient") renderAllIn($("#tr-grid"));
  if (b.dataset.tab === "susp") renderAllIn($("#su-grid"));
  if (b.dataset.tab === "ws" && ws) ws.show();
  if (b.dataset.tab === "replay") loadReplay().then(() => renderReplay(true));
  requestAnimationFrame(() => document.querySelectorAll(`#tab-${b.dataset.tab} .js-plotly-plot`).forEach((p) => Plotly.Plots.resize(p)));
}));

// ------------------------------------------------------------------ front view
const view = new FrontView($("#svg"));
const RANGES = { roll: [-4, 4, 0.1, "Roll φ", "°"], heave: [-40, 40, 1, "Wheel travel", "mm"], bump: [-40, 40, 1, "Bump under outer wheel", "mm"] };
let vmode = "roll";

async function refreshView() {
  const axle = $("#v-axle").value;
  const [a, b, st] = RANGES[vmode];
  const n = Math.round((b - a) / st);
  const values = Array.from({ length: n + 1 }, (_, k) => +(a + k * st).toFixed(4));
  const key = JSON.stringify([axle, vmode, state.model[axle], state.model.vehicle.h_cg_mm]);
  if (key !== vs.framesKey || vs.viewDirty) {
    try {
      const r = await post("/api/pose_frames", { model: state.model, axle, mode: vmode, values });
      vs.frames = r; vs.framesKey = key; vs.viewDirty = false;
      const i0 = values.indexOf(0);
      view.setStatic(r.frames[i0]);
    } catch (e) { setStatus(`view: ${e.message}`, true); return; }
  }
  drawCurrent();
}

function currentFrame() {
  if (!vs.frames) return null;
  const v = Number($("#v-slider").value);
  const vals = vs.frames.values;
  let best = 0;
  for (let i = 1; i < vals.length; i++) if (Math.abs(vals[i] - v) < Math.abs(vals[best] - v)) best = i;
  return vs.frames.frames[best];
}

function loadsData() {
  if (!S.byKey["f.Fz_out"]) return null;
  const t = $("#v-axle").value === "front" ? "f" : "r";
  const g = (k) => (S.byKey[k] ? S.byKey[k].value : null);
  return { Fz_out: g(`${t}.Fz_out`), Fz_in: g(`${t}.Fz_in`), may: g(`${t}.Fy_req`) };
}

function drawCurrent() {
  const f = currentFrame();
  const [, , , , u] = RANGES[vmode];
  $("#v-out").textContent = `${Number($("#v-slider").value).toFixed(vmode === "roll" ? 1 : 0)} ${u}`;
  view.draw(f, {
    constr: $("#v-constr").checked, dims: $("#v-dims").checked, ghost: $("#v-ghost").checked,
    loads: $("#v-loads").checked, loadsData: loadsData(),
  });
  readouts($("#v-read"), f);
}

function setMode(m) {
  vmode = m;
  document.querySelectorAll("#v-mode button").forEach((b) => b.classList.toggle("on", b.dataset.mode === m));
  const [a, b, st, lab] = RANGES[m];
  const sl = $("#v-slider");
  sl.min = a; sl.max = b; sl.step = st; sl.value = 0;
  $("#v-label").textContent = lab;
  refreshView();
}
document.querySelectorAll("#v-mode button").forEach((b) => b.addEventListener("click", () => setMode(b.dataset.mode)));
$("#v-axle").addEventListener("change", refreshView);
$("#v-slider").addEventListener("input", drawCurrent);
["#v-constr", "#v-dims", "#v-loads", "#v-ghost"].forEach((s) => $(s).addEventListener("change", drawCurrent));
$("#v-fromay").addEventListener("click", () => {
  if (!S.byKey.phi_deg) return;
  if (vmode !== "roll") setMode("roll");
  const phi = S.byKey.phi_deg.value;
  $("#v-slider").value = Math.max(-4, Math.min(4, phi));
  if (Math.abs(phi) > 4) setStatus("roll beyond ±4° – clamped in view", true);
  drawCurrent();
});
$("#v-play").addEventListener("click", () => {
  vs.playing = !vs.playing;
  $("#v-play").textContent = vs.playing ? "❚❚ Pause" : "▶ Animate";
  if (!vs.playing) return;
  const sl = $("#v-slider");
  let t0 = performance.now();
  const step = (now) => {
    if (!vs.playing) return;
    const a = Number(sl.min), b = Number(sl.max);
    const ph = ((now - t0) / 3000) * 2 * Math.PI;
    sl.value = (0.5 * (a + b) + 0.5 * (b - a) * Math.sin(ph)).toFixed(3);
    drawCurrent();
    requestAnimationFrame(step);
  };
  requestAnimationFrame(step);
});

// ------------------------------------------------------------------ tire tab
async function refreshTire() {
  try {
    const c = await post("/api/tire_curves", { model: state.model });
    const g = (k) => (S.byKey[k] ? S.byKey[k].value : null);
    const ops = [];
    for (const [t, n] of [["f", "F"], ["r", "R"]]) {
      if (g(`${t}.alpha`) == null) continue;
      ops.push({ name: `${n}-out`, alpha: g(`${t}.alpha`), Fy: g(`${t}.tire_o.Fy`), Fz: g(`${t}.Fz_out`) });
      ops.push({ name: `${n}-in`, alpha: g(`${t}.alpha`), Fy: g(`${t}.tire_i.Fy`), Fz: g(`${t}.Fz_in`) });
    }
    plotTire(c, ops);
  } catch (e) { setStatus(`tire: ${e.message}`, true); }
}

// ------------------------------------------------------------------ sweeps
function paramOptions(sel, filter) {
  const ps = state.defaults.params.filter((p) => typeof p.value === "number" && filter(p.path));
  sel.innerHTML += ps.map((p) => `<option value="${p.path}">${LABELS[p.path] || p.path}</option>`).join("");
}

function linspace(a, b, n) { return Array.from({ length: n }, (_, k) => +(a + (b - a) * k / (n - 1)).toPrecision(8)); }
const parseList = (s) => s.split(/[,;\s]+/).filter(Boolean).map(Number).filter(Number.isFinite);

function initSweeps() {
  const sx = $("#s-x"), sc = $("#s-c");
  paramOptions(sx, () => true);
  paramOptions(sc, () => true);
  sx.value = "maneuver.ay_g"; $("#s-x0").value = 0.05; $("#s-x1").value = 2.0;
  sc.value = "front.h_rc_mm"; $("#s-cv").value = "0, 35, 70, 120";
  sx.addEventListener("change", () => {
    const v = getPath(state.model, sx.value);
    if (sx.value === "maneuver.ay_g") { $("#s-x0").value = 0.05; $("#s-x1").value = 2.0; return; }
    const span = Math.abs(v) > 1e-9 ? Math.abs(v) * 0.4 : 50;
    $("#s-x0").value = +(v - span).toPrecision(4); $("#s-x1").value = +(v + span).toPrecision(4);
  });
  const outs = state.defaults.outputs;
  const groups = {};
  Object.entries(outs).forEach(([k, m]) => (groups[m.group] = groups[m.group] || []).push([k, m]));
  const def = new Set(["f.dFz_g", "f.dFz_e", "r.dFz_g", "r.dFz_e", "f.geo_share", "r.geo_share", "phi_deg", "dalpha_deg", "f.util", "r.util"]);
  $("#s-outs").innerHTML = Object.entries(groups).map(([g, arr]) => `<div class="og"><b>${g}</b>${arr.map(([k, m]) =>
    `<label><input type="checkbox" value="${k}" ${def.has(k) ? "checked" : ""}> ${m.label}</label>`).join("")}</div>`).join("");
  $("#s-run").addEventListener("click", runSweep);
}

async function runSweep() {
  const xp = $("#s-x").value, cp = $("#s-c").value || null;
  const xs = linspace(Number($("#s-x0").value), Number($("#s-x1").value), Math.max(2, Number($("#s-xn").value)));
  const cv = cp ? parseList($("#s-cv").value) : null;
  if (cp && !cv.length) { setStatus("enter compare values", true); return; }
  let sel = [...document.querySelectorAll("#s-outs input:checked")].map((i) => i.value);
  const lim = $("#s-lim").checked;
  if (lim && !sel.includes("ay_max_g")) sel = ["ay_max_g", ...sel];
  const trn = sel.some((k) => k.startsWith("tr."));
  setStatus("sweeping…");
  const t0 = performance.now();
  try {
    const r = await post("/api/sweep", { model: state.model, x_path: xp, x_values: xs, compare_path: cp, compare_values: cv, include_limit: lim, include_transient: trn });
    plotSweep($("#s-plots"), r, state.defaults.outputs, sel);
    setStatus(`sweep · ${Math.round(performance.now() - t0)} ms`);
  } catch (e) { setStatus(`sweep: ${e.message}`, true); }
}

function initKin() {
  paramOptions($("#k-c"), (p) => p.includes("hardpoints") || p.endsWith("track_mm"));
  $("#k-c").value = "front.hardpoints.uca_in.z"; $("#k-cv").value = "265, 282.2, 300";
  $("#k-mode").addEventListener("change", () => {
    const m = $("#k-mode").value;
    const [a, b] = m === "roll" ? [-3, 3] : m === "heave" ? [-30, 30] : [-30, 30];
    $("#k-x0").value = a; $("#k-x1").value = b;
  });
  $("#k-axle").addEventListener("change", () => {
    const c = $("#k-c");
    if (c.value.startsWith("front.") || c.value.startsWith("rear.")) c.value = c.value.replace(/^(front|rear)\./, `${$("#k-axle").value}.`);
  });
  $("#k-run").addEventListener("click", runKin);
}

async function runKin() {
  const cp = $("#k-c").value || null;
  const cv = cp ? parseList($("#k-cv").value) : null;
  const xs = linspace(Number($("#k-x0").value), Number($("#k-x1").value), Math.max(2, Number($("#k-xn").value)));
  if (!xs.includes(0)) xs.push(0), xs.sort((a, b) => a - b);
  try {
    const r = await post("/api/kin_sweep", { model: state.model, axle: $("#k-axle").value, mode: $("#k-mode").value,
      values: xs, compare_path: cp, compare_values: cv });
    plotKin($("#k-plots"), r, state.defaults.kin_outputs);
  } catch (e) { setStatus(`kinematics: ${e.message}`, true); }
}

// ------------------------------------------------------------------ top bar actions
$("#btn-save").addEventListener("click", () => {
  const blob = new Blob([JSON.stringify(state.model, null, 2)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = "sakkawma_params.json"; a.click();
  URL.revokeObjectURL(a.href);
});
$("#btn-load").addEventListener("click", () => $("#file-in").click());
$("#file-in").addEventListener("change", async (e) => {
  const f = e.target.files[0]; if (!f) return;
  try {
    const m = JSON.parse(await f.text());
    await post("/api/adapt", { model: m, path: "param_mode", value: m.param_mode || "absolute" }); // validates
    replaceModel(m); setStatus(`loaded ${f.name}`);
  } catch (err) { setStatus(`load failed: ${err.message}`, true); }
  e.target.value = "";
});
$("#btn-reset").addEventListener("click", () => {
  if (!confirm("Replace all parameters with the placeholder defaults?")) return;
  replaceModel(structuredClone(state.defaults.model));
});
$("#btn-report").addEventListener("click", async () => {
  setStatus("building report…");
  try {
    const html = await post("/api/report", { model: state.model }, true);
    const blob = new Blob([html], { type: "text/html" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url; a.download = `sakkawma_report_${new Date().toISOString().slice(0, 16).replace(/[:T]/g, "-")}.html`; a.click();
    window.open(url, "_blank");
    setStatus("report downloaded");
  } catch (e) { setStatus(`report: ${e.message}`, true); }
});
$("#btn-copy-rc").addEventListener("click", async () => {
  const k = state.res && state.res.kin;
  if (!k || !k.front || !k.rear) return;
  await applyParam("front.h_rc_mm", +k.front.h_rc_mm.toFixed(3));
  await applyParam("rear.h_rc_mm", +k.rear.h_rc_mm.toFixed(3));
});
$("#btn-base").addEventListener("click", () => {
  setBaseline();
  $("#btn-base-clr").hidden = false;
  $("#base-info").textContent = `baseline @ ${new Date().toLocaleTimeString()}`;
});
$("#btn-base-clr").addEventListener("click", () => { clearBaseline(); $("#btn-base-clr").hidden = true; $("#base-info").textContent = ""; });
$("#tr-probe").addEventListener("change", (e) => applyParam("maneuver.transient.t_probe_s", Number(e.target.value)));

// ------------------------------------------------------------------ compare bars (shared by Transient, Springs & dampers)
function mountCompareBars() {
  document.querySelectorAll(".cmpbar").forEach((el) => {
    el.innerHTML = `<label>Compare <select class="cb-p"><option value="">— none —</option>${wParamOptions("")}</select></label>
      <label>values <input type="text" class="cb-v" placeholder="e.g. 900, 1800, 3600" style="width:170px"></label>
      <button class="small primary cb-run">Run</button> <button class="small ghost cb-clr">Clear</button>`;
    el.querySelector(".cb-run").addEventListener("click", () => {
      const p = el.querySelector(".cb-p").value;
      const v = el.querySelector(".cb-v").value.split(/[,;\s]+/).filter(Boolean).map(Number).filter(Number.isFinite);
      syncBars(p, el.querySelector(".cb-v").value);
      setCompare(p, v);
    });
    el.querySelector(".cb-clr").addEventListener("click", () => { syncBars("", ""); setCompare("", []); });
  });
}
function syncBars(p, v) {
  document.querySelectorAll(".cmpbar").forEach((el) => { el.querySelector(".cb-p").value = p; el.querySelector(".cb-v").value = v; });
}

// ------------------------------------------------------------------ corner replay tab
const rpTop = new TopView($("#rp-svg"));
const rpFront = new FrontView($("#rp-fv"));
let rpFrames = null, rpFramesKey = "";
async function rpFrontFrames() {
  const key = JSON.stringify([state.model.front, state.model.vehicle.h_cg_mm]);
  if (key === rpFramesKey) return rpFrames;
  const vals = Array.from({ length: 161 }, (_, k) => +(-4 + k * 0.05).toFixed(3));
  rpFrames = await post("/api/pose_frames", { model: state.model, axle: "front", mode: "roll", values: vals });
  rpFramesKey = key;
  rpFront.setStatic(rpFrames.frames[80]);
  return rpFrames;
}
function renderReplayTime() {
  const d = RS.data; if (!d || activeTab() !== "replay") return;
  const i = RS.idx;
  rpTop.draw(d, i);
  $("#rp-panel").innerHTML = panelHTML(d, i);
  moveCursor($("#rp-plots"), d);
  if (rpFrames) {
    const phi = d.phi_deg[i], k = Math.max(0, Math.min(160, Math.round((phi + 4) / 0.05)));
    const f = rpFrames.frames[k];
    const g = (key) => (S.byKey[key] ? S.byKey[key].value : null);
    rpFront.draw(f, { constr: true, dims: true, ghost: true, loads: true,
      loadsData: { Fz_out: d.Fz.FR[i], Fz_in: d.Fz.FL[i], may: d.Fyf[i] } });
  }
}
async function renderReplay(full) {
  const d = RS.data; if (!d) return;
  if (full) {
    drawPlots($("#rp-plots"), d);
    const S_ = d.summary;
    $("#rp-sum").innerHTML = `<table class="vals small-text"><tbody>
      ${Object.entries(S_.time_in_state_s).map(([k, v]) => `<tr><td>Time in “${k}”</td><td class="n">${v.toFixed(2)} s</td></tr>`).join("")}
      <tr><td>Peak grip used front / rear</td><td class="n">${(100 * S_.max_util_f).toFixed(0)} % / ${(100 * S_.max_util_r).toFixed(0)} %</td></tr>
      <tr><td>Most understeer / oversteer Δα</td><td class="n">${S_.max_dalpha.toFixed(2)}° / ${S_.min_dalpha.toFixed(2)}°</td></tr>
      <tr><td>Lowest wheel load</td><td class="n">${Math.round(S_.min_wheel_load)} N</td></tr></tbody></table>
      ${(d.warnings || []).length ? `<div class="warn">${d.warnings.join("<br>")}</div>` : ""}`;
    $("#rp-deriv").innerHTML = d.steps.filter((s) => s.key.startsWith("rp.")).map((s) => stepHTML(s)).join("") || "–";
    $("#rp-dt").textContent = state.model.maneuver.transient.t_probe_s;
    try { await rpFrontFrames(); } catch (e) { /* view optional */ }
  }
  renderReplayTime();
}
$("#rp-bar").innerHTML = transportHTML();
bindTransport($("#rp-bar"));
$("#rp-follow").addEventListener("change", (e) => { rpTop.follow = e.target.checked; renderReplayTime(); });
$("#rp-derive").addEventListener("click", () => {
  if (!RS.data) return;
  applyParam("maneuver.transient.t_probe_s", +RS.data.t[RS.idx].toFixed(3));
});
on("replay", () => renderReplay(true));
on("replay-time", () => renderReplayTime());
on("results", () => { if (activeTab() === "replay") loadReplay(); });

// ------------------------------------------------------------------ boot
(async function boot() {
  try {
    const r = await fetch("/api/defaults");
    state.defaults = await r.json();
  } catch (e) { setStatus("backend not reachable", true); return; }
  state.model = loadLocal() || structuredClone(state.defaults.model);
  buildForm();
  initSweeps();
  initKin();
  mountCompareBars();
  if (!(await recompute())) {
    // saved set from an older version is invalid -> fall back to defaults
    state.model = structuredClone(state.defaults.model); fillForm(); await recompute();
  }
  mountFixed($("#tr-grid"), [
    ["tr-series", { key: "ay_g" }, 4], ["tr-series", { key: "phi_deg" }, 4], ["road", {}, 4],
    ["tr-components", { axle: "f" }, 6], ["tr-components", { axle: "r" }, 6],
    ["tr-components", { axle: "f", percent: "p" }, 6], ["tr-components", { axle: "r", percent: "p" }, 6],
    ["tr-series", { key: "f.damper" }, 4], ["tr-series", { key: "f.v_d_out" }, 4], ["tr-series", { key: "f.Fz_in" }, 4],
  ]);
  mountFixed($("#su-grid"), [
    ["susp-curve", { kind: "mr" }, 4], ["susp-curve", { kind: "kw" }, 4], ["susp-curve", { kind: "fw" }, 4],
    ["susp-curve", { kind: "roll_k" }, 4], ["susp-curve", { kind: "roll_m" }, 4], ["susp-curve", { kind: "damper" }, 4],
    ["view2d", { axle: "front", mode: "heave", value: 20 }, 6], ["view2d", { axle: "rear", mode: "heave", value: 20 }, 6],
  ]);
  ws = new Workspace($("#ws-root"));
})();
