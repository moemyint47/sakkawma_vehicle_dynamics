// Workspace: snapping 12-column grid of widgets, pinned parameters, targets, named layouts saved to /workspaces.
import { state, api, applyParam, setStatus, replaceModel, getPath, setCompare } from "./store.js";
import { WIDGETS, WidgetHost, paramOptions, stepOptions, broadcast } from "./widgets.js";
import { LABELS, FIELD } from "./spec.js";
import { fmtNum } from "./derivations.js";

const $ = (s) => document.querySelector(s);
let uid = 0;
const nid = () => `w${Date.now().toString(36)}${(uid++).toString(36)}`;

const PRESETS = {
  "2 × 2": (n) => Array.from({ length: n }, (_, i) => ({ x: (i % 2) * 6, y: Math.floor(i / 2) * 5, w: 6, h: 5 })),
  "1 large + 2 small": (n) => Array.from({ length: n }, (_, i) => (i === 0 ? { x: 0, y: 0, w: 8, h: 8 }
    : i <= 2 ? { x: 8, y: (i - 1) * 4, w: 4, h: 4 } : { x: ((i - 3) % 3) * 4, y: 8 + Math.floor((i - 3) / 3) * 4, w: 4, h: 4 })),
  "3 columns": (n) => Array.from({ length: n }, (_, i) => ({ x: (i % 3) * 4, y: Math.floor(i / 3) * 5, w: 4, h: 5 })),
  "Full-width stack": (n) => Array.from({ length: n }, (_, i) => ({ x: 0, y: i * 5, w: 12, h: 5 })),
};
const SIZES = { "¼": 3, "⅓": 4, "½": 6, "Full": 12 };
const HEIGHTS = { "Small": 3, "Medium": 5, "Tall": 8 };

const TEMPLATE = () => ({
  name: "My workspace", description: "",
  layout: [
    { type: "params", x: 0, y: 0, w: 3, h: 8, cfg: {} },
    { type: "value", x: 3, y: 0, w: 3, h: 2, cfg: { key: "ay_max" } },
    { type: "value", x: 6, y: 0, w: 3, h: 2, cfg: { key: "lltd_front" } },
    { type: "value", x: 9, y: 0, w: 3, h: 2, cfg: { key: "tr.f.pct_damper_50" } },
    { type: "tr-components", x: 3, y: 2, w: 5, h: 6, cfg: { axle: "f" } },
    { type: "targets", x: 8, y: 2, w: 4, h: 6, cfg: {} },
  ],
  pinned: [
    { path: "front.h_rc_mm", min: -30, max: 120, step: 1 },
    { path: "front.damper.c_ls_bump", min: 500, max: 5000, step: 50 },
    { path: "front.spring.mr_c1", min: -0.004, max: 0.008, step: 0.0005 },
  ],
  targets: [
    { key: "ay_max", op: "max" },
    { key: "lltd_front", op: "between", a: 0.49, b: 0.53 },
    { key: "roll_gradient", op: "le", a: 1.0 },
  ],
  compare: { path: "", values: [] },
});

export class Workspace {
  constructor(root) {
    this.root = root;
    this.hosts = new Map();
    this.doc = TEMPLATE();
    this.id = null;
    this.locked = false;
    root.innerHTML = `
      <div class="toolbar ws-bar">
        <label>Workspace <select id="ws-sel"></select></label>
        <button class="small" id="ws-new">New</button>
        <button class="small" id="ws-save">Save</button>
        <button class="small" id="ws-saveas">Save as…</button>
        <button class="small ghost" id="ws-del">Delete</button>
        <label title="Also store the parameter set in the workspace file"><input type="checkbox" id="ws-withmodel" checked> include parameters</label>
        <span class="sep"></span>
        <label>Add <select id="ws-add"><option value="">— widget —</option>${Object.entries(WIDGETS).map(([k, d]) =>
          `<option value="${k}">${WLABEL[k] || k}</option>`).join("")}</select></label>
        <label>Layout <select id="ws-preset"><option value="">— preset —</option>${Object.keys(PRESETS).map((k) => `<option>${k}</option>`).join("")}</select></label>
        <button class="small" id="ws-compact" title="Close gaps">Tidy</button>
        <button class="small" id="ws-lock" title="Freeze the layout">🔓 Unlocked</button>
        <button class="small" id="ws-focus" title="Hide the full parameter sidebar">Focus mode</button>
      </div>
      <div class="ws-steps" id="ws-steps"></div>
      <div class="grid-stack" id="ws-grid"></div>`;
    this.grid = GridStack.init({ column: 12, cellHeight: 64, margin: 6, float: false, handle: ".w-head",
      resizable: { handles: "se, e, s, w" }, animate: true }, "#ws-grid");
    this.grid.on("resizestop", (e, el) => { const h = this.hosts.get(el.getAttribute("gs-id")); if (h) h.resize(); });
    this.grid.on("change", () => this.syncLayout());
    $("#ws-add").addEventListener("change", (e) => { if (e.target.value) this.addWidget(e.target.value); e.target.value = ""; });
    $("#ws-preset").addEventListener("change", (e) => { if (e.target.value) this.applyPreset(e.target.value); e.target.value = ""; });
    $("#ws-compact").addEventListener("click", () => this.grid.compact());
    $("#ws-lock").addEventListener("click", () => this.setLock(!this.locked));
    $("#ws-focus").addEventListener("click", () => {
      const f = document.body.classList.toggle("focus");
      $("#ws-focus").classList.toggle("on", f);
      requestAnimationFrame(() => this.resizeAll());
    });
    $("#ws-new").addEventListener("click", () => { this.id = null; this.load({ ...TEMPLATE(), pages: undefined }); $("#ws-sel").value = ""; });
    $("#ws-save").addEventListener("click", () => this.save(false));
    $("#ws-saveas").addEventListener("click", () => this.save(true));
    $("#ws-del").addEventListener("click", () => this.remove());
    $("#ws-sel").addEventListener("change", (e) => this.open(e.target.value));
    this.refreshList();
    this.load(this.doc);
  }

  // ---------------- widgets
  addWidget(type, pos = null, cfg = null, id = null) {
    const def = WIDGETS[type];
    const [w, h] = def.size || [4, 4];
    const opts = { id: id || nid(), w: pos ? pos.w : w, h: pos ? pos.h : h, minW: 2, minH: 2 };
    if (pos) { opts.x = pos.x; opts.y = pos.y; }
    const el = this.grid.addWidget(opts);
    const content = el.querySelector(".grid-stack-item-content");
    const host = new WidgetHost(content, type, cfg, {
      removable: true, ws: this,
      onRemove: () => { this.grid.removeWidget(el); this.hosts.delete(opts.id); this.syncLayout(); },
      onChange: () => this.syncLayout(),
    });
    this.addSizeMenu(content, el);
    this.hosts.set(opts.id, host);
    this.syncLayout();
    return host;
  }
  addSizeMenu(content, el) {
    const tools = content.querySelector(".w-tools");
    const b = document.createElement("select");
    b.className = "w-size"; b.title = "Snap size";
    b.innerHTML = `<option value="">▦</option><optgroup label="Width">${Object.entries(SIZES).map(([k, v]) => `<option value="w${v}">${k} width</option>`).join("")}</optgroup>
      <optgroup label="Height">${Object.entries(HEIGHTS).map(([k, v]) => `<option value="h${v}">${k}</option>`).join("")}</optgroup>`;
    b.addEventListener("change", () => {
      const v = b.value; b.value = ""; if (!v) return;
      this.grid.update(el, v[0] === "w" ? { w: Number(v.slice(1)) } : { h: Number(v.slice(1)) });
      requestAnimationFrame(() => this.resizeAll());
    });
    tools.prepend(b);
  }
  applyPreset(name) {
    const items = this.grid.getGridItems().sort((a, b) => (a.gridstackNode.y - b.gridstackNode.y) || (a.gridstackNode.x - b.gridstackNode.x));
    const pos = PRESETS[name](items.length);
    this.grid.batchUpdate();
    items.forEach((el, i) => this.grid.update(el, pos[i]));
    this.grid.batchUpdate(false);
    requestAnimationFrame(() => this.resizeAll());
  }
  setLock(v) {
    this.locked = v;
    this.grid.setStatic(v);
    $("#ws-lock").textContent = v ? "🔒 Locked" : "🔓 Unlocked";
    $("#ws-lock").classList.toggle("on", v);
  }
  resizeAll() { for (const h of this.hosts.values()) h.resize(); }
  syncLayout() {
    if (this.loading) return;
    const lay = this.grid.getGridItems().map((el) => {
      const n = el.gridstackNode, h = this.hosts.get(n.id);
      return h ? { id: n.id, type: h.type, x: n.x, y: n.y, w: n.w, h: n.h, cfg: h.cfg } : null;
    }).filter(Boolean);
    if (this.doc.pages && this.doc.pages.length) this.doc.pages[this.page].layout = lay;
    else this.doc.layout = lay;
  }

  // ---------------- steps (multi-page workflow)
  renderSteps() {
    const el = document.getElementById("ws-steps");
    const P = this.doc.pages;
    if (!P || !P.length) {
      el.innerHTML = `<button class="small ghost" id="st-conv" title="Turn this workspace into a step-by-step workflow">+ make it a step-by-step workflow</button>`;
      el.querySelector("#st-conv").addEventListener("click", () => {
        this.syncLayout();
        this.doc.pages = [{ title: "Step 1", layout: this.doc.layout, done: false }];
        this.page = 0; this.renderSteps();
      });
      return;
    }
    const cur = P[this.page];
    el.innerHTML = `<div class="steps">${P.map((p, i) => `<button class="step ${i === this.page ? "on" : ""} ${p.done ? "done" : ""}" data-i="${i}">
        <span class="sn">${p.done ? "✓" : i + 1}</span>${p.title}</button>`).join("")}</div>
      <div class="step-tools">
        <button class="small" id="st-prev" ${this.page === 0 ? "disabled" : ""}>◀ Prev</button>
        <label><input type="checkbox" id="st-done" ${cur.done ? "checked" : ""}> step done</label>
        <button class="small primary" id="st-next" ${this.page === P.length - 1 ? "disabled" : ""}>Next ▶</button>
        <span class="sep"></span>
        <button class="small ghost" id="st-add">+ step</button><button class="small ghost" id="st-ren">Rename</button>
        <button class="small ghost" id="st-del">Delete step</button></div>`;
    el.querySelectorAll(".step").forEach((b) => b.addEventListener("click", () => this.goPage(Number(b.dataset.i))));
    el.querySelector("#st-prev").addEventListener("click", () => this.goPage(this.page - 1));
    el.querySelector("#st-next").addEventListener("click", () => this.goPage(this.page + 1));
    el.querySelector("#st-done").addEventListener("change", (e) => { cur.done = e.target.checked; this.renderSteps(); });
    el.querySelector("#st-add").addEventListener("click", () => {
      const t = prompt("Step title", `Step ${P.length + 1}`); if (!t) return;
      this.syncLayout(); P.splice(this.page + 1, 0, { title: t, layout: [], done: false }); this.goPage(this.page + 1);
    });
    el.querySelector("#st-ren").addEventListener("click", () => { const t = prompt("Step title", cur.title); if (t) { cur.title = t; this.renderSteps(); } });
    el.querySelector("#st-del").addEventListener("click", () => {
      if (P.length <= 1 || !confirm(`Delete step “${cur.title}” and its widgets?`)) return;
      P.splice(this.page, 1); this.page = Math.max(0, this.page - 1); this.showLayout(P[this.page].layout); this.renderSteps();
    });
  }
  goPage(i) {
    const P = this.doc.pages;
    if (!P || i < 0 || i >= P.length) return;
    this.syncLayout();
    this.page = i; this.doc.page = i;
    this.showLayout(P[i].layout);
    this.renderSteps();
  }
  showLayout(layout) {
    this.loading = true;
    this.grid.removeAll();
    this.hosts.clear();
    this.grid.batchUpdate();
    for (const it of layout || []) if (WIDGETS[it.type]) this.addWidget(it.type, it, it.cfg, it.id);
    this.grid.batchUpdate(false);
    this.loading = false;
    this.syncLayout();
    requestAnimationFrame(() => this.resizeAll());
  }
  show() { requestAnimationFrame(() => { for (const h of this.hosts.values()) if (h.dirty || !h.body.firstChild) h.render(); this.resizeAll(); }); }

  // ---------------- documents
  load(doc) {
    this.doc = { ...TEMPLATE(), ...doc };
    if (doc.pages && !doc.layout) this.doc.layout = [];
    this.page = this.doc.pages && this.doc.pages.length ? Math.min(this.doc.page || 0, this.doc.pages.length - 1) : 0;
    this.showLayout(this.doc.pages && this.doc.pages.length ? this.doc.pages[this.page].layout : this.doc.layout);
    this.renderSteps();
    if (this.doc.model) replaceModel(this.doc.model);
    if (this.doc.compare && this.doc.compare.path) setCompare(this.doc.compare.path, this.doc.compare.values);
    this.setLock(false);
    requestAnimationFrame(() => this.resizeAll());
  }
  async refreshList() {
    try {
      const list = await api("GET", "/api/workspaces");
      $("#ws-sel").innerHTML = `<option value="">(unsaved)</option>` + list.map((w) => `<option value="${w.id}" ${w.id === this.id ? "selected" : ""}>${w.name}</option>`).join("");
    } catch (e) { setStatus(`workspaces: ${e.message}`, true); }
  }
  async open(id) {
    if (!id) return;
    try { this.id = id; this.load(await api("GET", `/api/workspaces/${id}`)); setStatus(`workspace “${this.doc.name}” loaded`); }
    catch (e) { setStatus(`open: ${e.message}`, true); }
  }
  async save(asNew) {
    this.syncLayout();
    let id = this.id;
    if (asNew || !id) {
      const name = prompt("Workspace name", asNew ? `${this.doc.name} copy` : this.doc.name);
      if (!name) return;
      this.doc.name = name;
      id = name;
    }
    const body = { ...this.doc, compare: state.cmpSpec, model: $("#ws-withmodel").checked ? state.model : null, saved: new Date().toISOString() };
    try {
      const r = await api("PUT", `/api/workspaces/${encodeURIComponent(id)}`, body);
      this.id = r.id; await this.refreshList();
      setStatus(`saved workspaces/${r.id}.json – commit it to keep it in git`);
    } catch (e) { setStatus(`save: ${e.message}`, true); }
  }
  async remove() {
    if (!this.id || !confirm(`Delete workspace “${this.doc.name}”?`)) return;
    await api("DELETE", `/api/workspaces/${this.id}`);
    this.id = null; await this.refreshList(); setStatus("workspace deleted");
  }

  // ---------------- pinned parameters
  renderPinned(el) {
    const pins = this.doc.pinned;
    const rows = pins.map((p, i) => {
      const v = getPath(state.model, p.path);
      const f = FIELD[p.path] || {};
      return `<div class="pin" data-i="${i}">
        <div class="pin-h"><span title="${p.path}">${LABELS[p.path] || p.path}</span><button class="w-btn pin-x" title="Unpin">✕</button></div>
        <div class="pin-c"><input type="range" min="${p.min}" max="${p.max}" step="${p.step}" value="${v}">
          <input type="number" step="${p.step}" value="${v}"><span class="u">${f.unit && !String(f.unit).startsWith("select") ? f.unit : ""}</span></div>
        <div class="pin-r muted"><input class="pr" data-r="min" value="${p.min}" title="slider min"> … <input class="pr" data-r="max" value="${p.max}" title="slider max"> step <input class="pr" data-r="step" value="${p.step}"></div>
      </div>`;
    }).join("");
    el.innerHTML = `<div class="pins">${rows || `<div class="wmsg">Pin the parameters you are working on.</div>`}</div>
      <div class="pin-add"><select><option value="">+ pin a parameter…</option>${paramOptions("")}</select></div>`;
    el.querySelectorAll(".pin").forEach((row) => {
      const i = Number(row.dataset.i), p = pins[i];
      const [rng, num] = row.querySelectorAll(".pin-c input");
      rng.addEventListener("input", () => { num.value = rng.value; applyParam(p.path, Number(rng.value)); });
      num.addEventListener("change", () => { rng.value = num.value; applyParam(p.path, Number(num.value)); });
      row.querySelector(".pin-x").addEventListener("click", () => { pins.splice(i, 1); this.renderPinned(el); });
      row.querySelectorAll(".pr").forEach((inp) => inp.addEventListener("change", () => { p[inp.dataset.r] = Number(inp.value); this.renderPinned(el); }));
    });
    el.querySelector(".pin-add select").addEventListener("change", (e) => {
      const path = e.target.value; if (!path) return;
      const v = Number(getPath(state.model, path)) || 0;
      const span = Math.abs(v) > 1e-9 ? Math.abs(v) * 0.5 : 10;
      const step = (FIELD[path] && FIELD[path].step) || +(span / 50).toPrecision(2);
      pins.push({ path, min: +(v - span).toPrecision(4), max: +(v + span).toPrecision(4), step });
      this.renderPinned(el);
    });
  }
  refreshPinnedValues() {
    document.querySelectorAll(".pin").forEach((row) => {
      const p = this.doc.pinned[Number(row.dataset.i)]; if (!p) return;
      const v = getPath(state.model, p.path);
      const [rng, num] = row.querySelectorAll(".pin-c input");
      if (document.activeElement !== num) num.value = v;
      if (document.activeElement !== rng) rng.value = v;
    });
  }

  // ---------------- targets
  renderTargets(el) {
    const T = this.doc.targets;
    const OPS = { max: "maximise", min: "minimise", ge: "≥", le: "≤", between: "between" };
    const res = state.res, base = state.base && state.base.res;
    const rows = T.map((t, i) => {
      const st = res && res.steps.find((s) => s.key === t.key);
      const v = st ? st.value : null;
      const b = base ? (base.steps.find((s) => s.key === t.key) || {}).value : null;
      let status = "", cls = "";
      if (v !== null && v !== undefined) {
        if (t.op === "ge") { const ok = v >= t.a; status = ok ? "✓ met" : `✗ short by ${fmtNum(t.a - v, 3)}`; cls = ok ? "ok" : "bad"; }
        else if (t.op === "le") { const ok = v <= t.a; status = ok ? "✓ met" : `✗ over by ${fmtNum(v - t.a, 3)}`; cls = ok ? "ok" : "bad"; }
        else if (t.op === "between") {
          const ok = v >= t.a && v <= t.b; cls = ok ? "ok" : "bad";
          status = ok ? "✓ in range" : `✗ ${v < t.a ? "below by " + fmtNum(t.a - v, 3) : "above by " + fmtNum(v - t.b, 3)}`;
        } else if (b !== null && b !== undefined) {
          const d = v - b, better = t.op === "max" ? d > 0 : d < 0;
          status = Math.abs(d) < 1e-12 ? "= baseline" : `${better ? "▲ better" : "▼ worse"} by ${fmtNum(Math.abs(d), 3)}`;
          cls = Math.abs(d) < 1e-12 ? "" : better ? "ok" : "bad";
        } else status = "set a baseline to compare";
      }
      return `<tr data-i="${i}"><td>${st ? st.label : t.key}</td>
        <td><select class="t-op">${Object.entries(OPS).map(([k, l]) => `<option value="${k}" ${k === t.op ? "selected" : ""}>${l}</option>`).join("")}</select>
          ${["ge", "le", "between"].includes(t.op) ? `<input class="t-a" type="number" step="any" value="${t.a ?? ""}">` : ""}
          ${t.op === "between" ? `<input class="t-b" type="number" step="any" value="${t.b ?? ""}">` : ""}</td>
        <td class="n">${fmtNum(v, 3)} <span class="muted">${st && st.unit !== "-" ? st.unit : ""}</span></td>
        <td class="t-st ${cls}">${status}</td><td><button class="w-btn t-x" title="Remove">✕</button></td></tr>`;
    }).join("");
    el.innerHTML = `<table class="vals targets"><thead><tr><td>Target</td><td>Goal</td><td class="n">Now</td><td>Status</td><td></td></tr></thead><tbody>${rows}</tbody></table>
      <div class="pin-add"><select><option value="">+ add target…</option>${stepOptions("")}</select></div>`;
    el.querySelectorAll("tr[data-i]").forEach((tr) => {
      const t = T[Number(tr.dataset.i)];
      tr.querySelector(".t-op").addEventListener("change", (e) => { t.op = e.target.value; if (t.a === undefined) t.a = 0; if (t.b === undefined) t.b = 1; this.renderTargets(el); });
      const a = tr.querySelector(".t-a"), b = tr.querySelector(".t-b");
      if (a) a.addEventListener("change", () => { t.a = Number(a.value); this.renderTargets(el); });
      if (b) b.addEventListener("change", () => { t.b = Number(b.value); this.renderTargets(el); });
      tr.querySelector(".t-x").addEventListener("click", () => { T.splice(Number(tr.dataset.i), 1); this.renderTargets(el); });
    });
    el.querySelector(".pin-add select").addEventListener("change", (e) => {
      if (!e.target.value) return;
      T.push({ key: e.target.value, op: "max" }); this.renderTargets(el);
    });
  }
}

const WLABEL = {
  "tr-components": "Transient LT components (OptimumG style)", "tr-series": "Transient signal vs time",
  "susp-curve": "Spring / damper / roll-stiffness curve", "road": "Road plan view", "value": "Value tile",
  "lt-bar": "Steady-state LT split", "view2d": "Front view (animated)", "sweep": "Parameter sweep",
  "kin": "Kinematic curve", "tire": "Tire curve", "params": "Pinned parameters", "targets": "Targets",
  "map2d": "2D map (heatmap + contours)", "optimizer": "Optimiser", "notes": "Notes / instructions",
};
