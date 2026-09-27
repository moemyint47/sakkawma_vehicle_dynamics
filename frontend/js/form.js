// Sidebar parameter form (absolute / adaptive editing).
import { GROUPS, ADAPTIVE_TARGETS, ROAD_PRESETS, getPath, tableToText, textToTable } from "./spec.js";
import { state, applyParam, on, setStatus } from "./store.js";
import { stepHTML } from "./derivations.js";

const $ = (s) => document.querySelector(s);

export function buildForm() {
  const host = $("#form");
  host.innerHTML = `
    <div class="modebar">
      <div class="seg" id="pmode">
        <button data-m="adaptive" title="Edits keep the design intent: dependent parameters (coordinates, spring rate…) follow">Adaptive</button>
        <button data-m="absolute" title="Every input is independent">Absolute</button>
      </div>
      <span class="muted small-text" id="pmode-hint"></span>
    </div>
    <div class="placeholder-banner">Default numbers are <b>placeholders</b> – replace them with your car and tire data.</div>
    <div id="adapt-log" class="adapt-log"></div>`;
  host.querySelectorAll("#pmode button").forEach((b) => b.addEventListener("click", () => applyParam("param_mode", b.dataset.m)));
  for (const g of GROUPS) {
    const d = document.createElement("details");
    d.open = g.open;
    d.innerHTML = `<summary>${g.title}</summary>${g.note ? `<div class="gnote">${g.note}</div>` : ""}`;
    for (const [path, label, unit, step, nullable] of g.fields) d.appendChild(fieldRow(path, label, unit, step, nullable));
    if (g.title.startsWith("Transient")) {
      const r = document.createElement("div");
      r.className = "frow wide";
      r.innerHTML = `<label>Road presets</label><select id="road-preset"><option value="">— choose —</option>${Object.keys(ROAD_PRESETS).map((k) => `<option>${k}</option>`).join("")}</select>`;
      r.querySelector("select").addEventListener("change", async (e) => {
        const k = e.target.value; if (!k) return;
        await applyParam("maneuver.transient.road", ROAD_PRESETS[k]);
        await applyParam("maneuver.transient.profile", "road");
        const tt = ROAD_PRESETS[k].reduce((a, r2) => a + r2[0] / (r2[2] / 3.6), 0);
        await applyParam("maneuver.transient.t_end_s", Math.min(20, +(tt + 0.3).toFixed(2)));
        e.target.value = "";
      });
      d.appendChild(r);
    }
    host.insertBefore(d, $("#adapt-log"));
  }
  fillForm();
  on("adapted", (r) => { fillForm(); flash(r.changes || []); renderLog(r); });
}

function fieldRow(path, label, unit, step, nullable) {
  const row = document.createElement("div");
  row.className = "frow";
  row.dataset.path = path;
  const id = `in-${path.replace(/\./g, "_")}`;
  const tgt = ADAPTIVE_TARGETS.has(path) ? `<span class="tgt" title="Adaptive mode: editing this moves dependent parameters">↻</span>` : "";
  if (unit === "bool") {
    row.innerHTML = `<label for="${id}">${label}${tgt}</label><input type="checkbox" id="${id}"><span></span>`;
  } else if (String(unit).startsWith("select:")) {
    const opts = unit.slice(7).split("|");
    row.innerHTML = `<label for="${id}">${label}${tgt}</label><select id="${id}">${opts.map((o) => `<option>${o}</option>`).join("")}</select><span></span>`;
  } else if (String(unit).startsWith("table")) {
    row.classList.add("wide");
    row.innerHTML = `<label for="${id}">${label}</label><textarea id="${id}" rows="4" placeholder="(none)"></textarea>`;
  } else {
    row.innerHTML = `<label for="${id}" title="${path}">${label}${tgt}</label>
      <input type="number" id="${id}" step="${step}" ${nullable ? 'placeholder="(none)"' : ""}><span class="u">${unit}</span>`;
  }
  const inp = row.querySelector("input, select, textarea");
  const commit = () => {
    let v;
    if (unit === "bool") v = inp.checked;
    else if (String(unit).startsWith("select:")) v = inp.value;
    else if (String(unit).startsWith("table")) {
      try { v = textToTable(inp.value, unit === "table3" ? 3 : 2); } catch (e) { setStatus(e.message, true); return; }
    } else if (inp.value === "") { if (!nullable) return; v = null; }
    else { v = Number(inp.value); if (!Number.isFinite(v)) return; }
    applyParam(path, v);
  };
  // absolute mode: live on every keystroke; adaptive: on commit (Enter / blur / spinner)
  inp.addEventListener("change", commit);
  if (!String(unit).startsWith("table") && unit !== "bool" && !String(unit).startsWith("select:")) {
    inp.addEventListener("input", () => { if (state.model.param_mode !== "adaptive") commit(); });
  }
  return row;
}

export function fillForm() {
  const m = state.model;
  document.querySelectorAll("#pmode button").forEach((b) => b.classList.toggle("on", b.dataset.m === m.param_mode));
  const hint = $("#pmode-hint");
  if (hint) hint.textContent = m.param_mode === "adaptive" ? "↻ fields are design targets" : "all inputs independent";
  document.querySelectorAll(".frow[data-path]").forEach((row) => {
    const p = row.dataset.path, inp = row.querySelector("input, select, textarea"), v = getPath(m, p);
    if (document.activeElement === inp) return;
    if (inp.type === "checkbox") inp.checked = !!v;
    else if (inp.tagName === "TEXTAREA") inp.value = tableToText(v);
    else inp.value = v == null ? "" : v;
    row.classList.toggle("changed", JSON.stringify(v) !== JSON.stringify(getPath(state.defaults.model, p)));
  });
  document.body.classList.toggle("adaptive", m.param_mode === "adaptive");
}

function flash(changes) {
  for (const c of changes) {
    const row = document.querySelector(`.frow[data-path="${c.path}"]`);
    if (!row) continue;
    row.classList.remove("flash"); void row.offsetWidth; row.classList.add("flash");
  }
}

function renderLog(r) {
  const el = $("#adapt-log");
  if (!el) return;
  if (r.error) { el.innerHTML = `<div class="warn">Adaptation refused: ${r.error}</div>`; return; }
  const ch = (r.changes || []).filter((c) => c.old !== undefined);
  if (!ch.length) { el.innerHTML = ""; return; }
  el.innerHTML = `<details open><summary>Last edit changed ${ch.length} parameter${ch.length > 1 ? "s" : ""}</summary>
    <table class="vals small-text"><tbody>${ch.map((c) => `<tr><td>${c.path}</td><td class="n">${fmt(c.old)}</td><td>→</td><td class="n"><b>${fmt(c.new)}</b></td></tr>`).join("")}</tbody></table>
    ${(r.steps || []).length ? `<details><summary>Adaptation maths (${r.steps.length})</summary>${r.steps.map((s) => stepHTML(s)).join("")}</details>` : ""}
  </details>`;
}
const fmt = (v) => (typeof v === "number" ? +v.toPrecision(6) : JSON.stringify(v));
