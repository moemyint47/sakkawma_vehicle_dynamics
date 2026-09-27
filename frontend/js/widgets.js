// Widget library: every graph / view / tile that can live in a fixed tab or in the workspace grid.
import { state, post, on, val, applyParam, setStatus, getPath } from "./store.js";
import { plotTheme } from "./plots.js";
import { stepHTML, fmtNum } from "./derivations.js";
import { LABELS, FIELD } from "./spec.js";
import { FrontView, readouts } from "./view2d.js";

// ------------------------------------------------------------------ catalogues
export const TR_SERIES = {
  ay_g: ["Input a_y(t)", "g"], phi_deg: ["Roll angle φ", "°"], phidot_deg_s: ["Roll rate φ̇", "°/s"],
};
for (const [t, n] of [["f", "Front"], ["r", "Rear"]]) {
  Object.assign(TR_SERIES, {
    [`${t}.total`]: [`${n} total ΔF_z`, "N"], [`${t}.geo`]: [`${n} geometric ΔF_z`, "N"],
    [`${t}.spring`]: [`${n} spring ΔF_z`, "N"], [`${t}.arb`]: [`${n} ARB ΔF_z`, "N"],
    [`${t}.damper`]: [`${n} damper ΔF_z`, "N"], [`${t}.bump`]: [`${n} bump-stop ΔF_z`, "N"],
    [`${t}.unsprung`]: [`${n} unsprung ΔF_z`, "N"], [`${t}.elastic`]: [`${n} elastic ΔF_z`, "N"],
    [`${t}.Fz_out`]: [`${n} outer wheel load`, "N"], [`${t}.Fz_in`]: [`${n} inner wheel load`, "N"],
    [`${t}.v_d_out`]: [`${n} outer damper velocity`, "mm/s"], [`${t}.z_out`]: [`${n} outer wheel travel`, "mm"],
    [`${t}.pct_geo`]: [`${n} geometric share`, "%"], [`${t}.pct_damper`]: [`${n} damper share`, "%"],
    [`${t}.pct_spring`]: [`${n} spring share`, "%"], [`${t}.pct_arb`]: [`${n} ARB share`, "%"],
  });
}
const COMP = [["geo", "Geometric", 0], ["spring", "Springs", 2], ["arb", "ARB", 3], ["damper", "Dampers", 6],
  ["bump", "Bump stops", 4], ["unsprung", "Unsprung", 1]];
const SUSP_KINDS = { mr: "Motion ratio vs wheel travel", kw: "Wheel rate vs wheel travel", fw: "Wheel force vs wheel travel",
  roll_k: "Roll stiffness vs roll angle", roll_m: "Roll moment vs roll angle", damper: "Damper force vs velocity" };
const AX = { f: "Front", r: "Rear" };

const trGet = (res, key) => {
  if (!res) return null;
  const [a, b] = key.split(".");
  return b ? res.axles[a][b] : res[a];
};

// runs to draw: compare runs if active, else current (+ baseline)
function runs() {
  if (state.cmp && state.cmp.series && state.cmp.series.length) {
    const nm = state.cmp.compare_path.split(".").slice(-2).join(".");
    return state.cmp.series.filter((s) => s.result).map((s, i) => ({ name: `${nm} = ${s.compare_value}`, tr: s.result, susp: s.susp, ci: i, dash: "solid" }));
  }
  const out = [];
  if (state.res) out.push({ name: "current", tr: state.res.transient, susp: state.res.susp, ci: 0, dash: "solid" });
  if (state.base && state.base.res) out.push({ name: "baseline", tr: state.base.res.transient, susp: state.base.res.susp, ci: -1, dash: "dash" });
  return out;
}

function layoutFor(extra = {}) {
  const th = plotTheme();
  return { th, layout: { ...th.layout, margin: { l: 56, r: 12, t: 8, b: 40 }, autosize: true,
    legend: { orientation: "h", x: 0, y: 1.0, yanchor: "bottom", font: { color: th.layout.font.color, size: 10.5 } }, ...extra } };
}
const ax = (th, title, more = {}) => ({ ...th.layout.xaxis, title: { text: title, font: { size: 11 } }, ...more });
const colorOf = (th, ci) => (ci < 0 ? th.layout.xaxis.tickfont.color : th.c[ci % 8]);

function plotInto(el, traces, layout, config) {
  let d = el.querySelector(".plot-fill");
  if (!d) { el.innerHTML = `<div class="plot-fill"></div>`; d = el.querySelector(".plot-fill"); }
  Plotly.react(d, traces, layout, config);
}
const msg = (el, t) => { el.innerHTML = `<div class="wmsg">${t}</div>`; };

// ------------------------------------------------------------------ widget definitions
export const WIDGETS = {
  "tr-components": {
    title: (c) => `${AX[c.axle]} load transfer components vs time`, size: [6, 5],
    fields: [{ k: "axle", label: "Axle", type: "select", options: { f: "Front", r: "Rear" } },
             { k: "percent", label: "Show", type: "select", options: { n: "Newtons", p: "% of suspended LT" } }],
    defaults: { axle: "f", percent: "n" }, on: ["results", "baseline"],
    render(el, c) {
      const tr = state.res && state.res.transient;
      if (!tr) return msg(el, "No transient result yet.");
      const { th, layout } = layoutFor();
      const A = tr.axles[c.axle], pct = c.percent === "p";
      const tr_ = [];
      for (const [k, lab, ci] of COMP) {
        if (pct && k === "unsprung") continue;
        const y = pct ? A[`pct_${k}`] : A[k];
        if (!y || y.every((v) => !v)) continue;
        tr_.push({ x: tr.t, y, name: lab, mode: "lines", line: { color: th.c[ci], width: 2 }, hovertemplate: `%{y:.1f} ${pct ? "%" : "N"}` });
      }
      if (!pct) tr_.push({ x: tr.t, y: A.total, name: "Total", mode: "lines", line: { color: th.layout.font.color, width: 2.5 }, hovertemplate: "%{y:.1f} N" });
      if (!pct && state.base && state.base.res && state.base.res.transient) {
        tr_.push({ x: state.base.res.transient.t, y: state.base.res.transient.axles[c.axle].total, name: "Total (baseline)", mode: "lines",
          line: { color: th.layout.font.color, width: 1.5, dash: "dash" }, hovertemplate: "%{y:.1f} N" });
      }
      const probe = state.model.maneuver.transient.t_probe_s;
      plotInto(el, tr_, { ...layout, xaxis: ax(th, "t [s]"), yaxis: ax(th, pct ? "share of suspended ΔF_z [%]" : "ΔF_z per wheel [N]"),
        shapes: [{ type: "line", x0: probe, x1: probe, yref: "paper", y0: 0, y1: 1, line: { color: th.layout.xaxis.gridcolor, width: 1, dash: "dot" } }] }, th.config);
    },
  },

  "tr-series": {
    title: (c) => `${(TR_SERIES[c.key] || [c.key])[0]} vs time`, size: [6, 4],
    fields: [{ k: "key", label: "Signal", type: "select", options: Object.fromEntries(Object.entries(TR_SERIES).map(([k, v]) => [k, `${v[0]} [${v[1]}]`])) }],
    defaults: { key: "phi_deg" }, on: ["results", "baseline", "compare"],
    render(el, c) {
      const R = runs().filter((r) => r.tr);
      if (!R.length) return msg(el, "No transient result yet.");
      const { th, layout } = layoutFor();
      const [lab, unit] = TR_SERIES[c.key] || [c.key, ""];
      const traces = R.map((r) => ({ x: r.tr.t, y: trGet(r.tr, c.key), name: r.name, mode: "lines",
        line: { color: colorOf(th, r.ci), width: 2, dash: r.dash }, hovertemplate: `%{y:.3g} ${unit}` }));
      plotInto(el, traces, { ...layout, showlegend: traces.length > 1, xaxis: ax(th, "t [s]"), yaxis: ax(th, `${lab} [${unit}]`) }, th.config);
    },
  },

  "susp-curve": {
    title: (c) => `${c.axle === "both" ? "" : AX[c.axle] + " "}${SUSP_KINDS[c.kind]}`, size: [4, 4],
    fields: [{ k: "axle", label: "Axle", type: "select", options: { both: "Front + rear", f: "Front", r: "Rear" } },
             { k: "kind", label: "Curve", type: "select", options: SUSP_KINDS }],
    defaults: { axle: "both", kind: "kw" }, on: ["results", "baseline", "compare"],
    render(el, c) {
      const R = runs().filter((r) => r.susp);
      if (!R.length) return msg(el, "No data yet.");
      const { th, layout } = layoutFor();
      const axes = c.axle === "both" ? ["f", "r"] : [c.axle];
      const traces = [];
      const cmp = !!(state.cmp && state.cmp.series);
      R.forEach((r) => axes.forEach((a, ai) => {
        const S = r.susp[a];
        let x, y, xt, yt;
        if (c.kind === "mr") { x = S.wheel.z; y = S.wheel.mr; xt = "wheel travel z [mm] (+ bump)"; yt = "MR [–]"; }
        else if (c.kind === "kw") { x = S.wheel.z; y = S.wheel.kw; xt = "wheel travel z [mm]"; yt = "wheel rate [N/mm]"; }
        else if (c.kind === "fw") { x = S.wheel.z; y = S.wheel.Fw; xt = "wheel travel z [mm]"; yt = "wheel force [N]"; }
        else if (c.kind === "roll_k") { x = S.roll.phi_deg; y = S.roll.K_Nm_deg; xt = "roll φ [°]"; yt = "roll stiffness [N·m/°]"; }
        else if (c.kind === "roll_m") { x = S.roll.phi_deg; y = S.roll.M; xt = "roll φ [°]"; yt = "roll moment [N·m]"; }
        else { x = S.damper.v; y = S.damper.F; xt = "damper velocity [mm/s] (+ bump)"; yt = "damper force [N]"; }
        const ci = cmp ? r.ci : (r.ci < 0 ? -1 : (axes.length > 1 ? ai : 0));
        const compact = cmp && axes.length > 1;
        traces.push({ x, y, mode: "lines", legendgroup: r.name, showlegend: !compact || ai === 0,
          name: compact ? r.name : (`${axes.length > 1 ? AX[a] + " " : ""}${R.length > 1 ? r.name : ""}`.trim() || AX[a]),
          line: { color: colorOf(th, ci), width: 2, dash: cmp && axes.length > 1 ? (ai ? "dash" : "solid") : r.dash },
          hovertemplate: "%{y:.4g}", _xt: xt, _yt: yt });
      }));
      const lay = { ...layout, showlegend: traces.length > 1, hovermode: "closest",
        xaxis: ax(th, traces[0]._xt), yaxis: ax(th, traces[0]._yt) };
      if (cmp && axes.length > 1) lay.legend = { ...lay.legend, title: { text: "solid = front, dashed = rear:", font: { size: 10.5 } } };
      plotInto(el, traces, lay, th.config);
    },
  },

  "road": {
    title: () => "Road plan view", size: [4, 5], fields: [], defaults: {}, on: ["results"],
    render(el) {
      const tr = state.res && state.res.transient;
      if (!tr || !tr.road) return msg(el, "Choose the <b>road</b> profile (Transient input a_y(t) → Profile, or a road preset) to see the path.");
      const { th, layout } = layoutFor();
      const R = tr.road, tp = state.model.maneuver.transient.t_probe_s;
      let i = R.t.findIndex((t) => t >= tp); if (i < 0) i = R.t.length - 1;
      const traces = [
        { x: R.x, y: R.y, mode: "lines", name: "path", line: { color: th.c[0], width: 3 },
          customdata: R.ay_g.map((a, k) => [a, R.v[k] * 3.6, R.s[k]]),
          hovertemplate: "s=%{customdata[2]:.1f} m<br>a_y=%{customdata[0]:.2f} g<br>v=%{customdata[1]:.0f} km/h<extra></extra>" },
        { x: [R.x[i]], y: [R.y[i]], mode: "markers", name: `car @ t=${tp}s`, marker: { size: 12, color: th.c[1], line: { width: 2, color: th.layout.paper_bgcolor } } },
      ];
      plotInto(el, traces, { ...layout, hovermode: "closest", xaxis: ax(th, "x [m]"), yaxis: ax(th, "y [m]", { scaleanchor: "x", scaleratio: 1 }) }, th.config);
    },
  },

  "value": {
    title: (c) => (state.res && state.res.steps.find((s) => s.key === c.key) || { label: c.key }).label, size: [3, 2],
    fields: [{ k: "key", label: "Quantity", type: "stepkey" }], defaults: { key: "ay_max" }, on: ["results", "baseline"],
    render(el, c) {
      const st = state.res && state.res.steps.find((s) => s.key === c.key);
      if (!st) return msg(el, `“${c.key}” not available.`);
      const b = state.base && state.base.res ? val(c.key, state.base.res) : null;
      const d = typeof st.value === "number" && typeof b === "number" ? st.value - b : null;
      el.innerHTML = `<div class="vtile"><div class="vv">${fmtNum(st.value, 3)}<small>${st.unit === "-" ? "" : st.unit}</small>
        <button class="fx" data-fxw>ƒ</button></div>
        ${d !== null ? `<div class="vd">${d > 0 ? "▲" : d < 0 ? "▼" : "="} ${fmtNum(d, 3)} vs baseline (${fmtNum(b, 3)})</div>` : ""}
        ${st.note ? `<div class="vn">${st.note}</div>` : ""}<div class="vder" hidden></div></div>`;
      el.querySelector("[data-fxw]").addEventListener("click", (e) => {
        const box = el.querySelector(".vder"); box.hidden = !box.hidden; e.target.classList.toggle("on", !box.hidden);
        if (!box.hidden) box.innerHTML = stepHTML(st);
      });
    },
  },

  "lt-bar": {
    title: () => "Steady-state load-transfer split (per wheel)", size: [4, 4], fields: [], defaults: {}, on: ["results", "baseline"],
    render(el) {
      if (!state.res) return;
      const { th, layout } = layoutFor();
      const cats = ["Front", "Rear"];
      const t = (name, key, ci) => ({ type: "bar", name, x: cats, y: ["f", "r"].map((a) => val(`${a}.${key}`)), marker: { color: th.c[ci] }, hovertemplate: `%{y:.1f} N<extra>${name}</extra>` });
      const els = ["f", "r"].some((a) => val(`${a}.dFz_e_spring`) !== null)
        ? [t("Springs", "dFz_e_spring", 2), t("ARB", "dFz_e_arb", 3), t("Bump stops", "dFz_e_bump", 4)] : [t("Elastic", "dFz_e", 2)];
      plotInto(el, [t("Unsprung", "dFz_u", 1), t("Geometric", "dFz_g", 0), ...els],
        { ...layout, barmode: "relative", hovermode: "closest", yaxis: ax(th, "ΔF_z [N]") }, th.config);
    },
  },

  "view2d": {
    title: (c) => `${c.axle === "front" ? "Front" : "Rear"} suspension – front view`, size: [6, 7],
    fields: [{ k: "axle", label: "Axle", type: "select", options: { front: "Front", rear: "Rear" } },
             { k: "mode", label: "Motion", type: "select", options: { roll: "Roll [°]", heave: "Heave travel [mm]", bump: "1-wheel bump [mm]" } }],
    defaults: { axle: "front", mode: "roll", value: 0 }, on: ["results"],
    render(el, c, host) {
      if (!el.querySelector("svg")) {
        el.innerHTML = `<div class="wbar"><input type="range" class="w-sl"><output class="w-out"></output>
          <button class="small w-ay" title="Roll angle at the steady-state a_y">φ(a_y)</button></div>
          <div class="wsvg"><svg xmlns="http://www.w3.org/2000/svg"></svg></div><div class="readouts mini"></div>`;
        host.view = new FrontView(el.querySelector("svg"));
        const sl = el.querySelector(".w-sl");
        sl.addEventListener("input", () => { c.value = Number(sl.value); host.changed(); this.draw(el, c, host); });
        el.querySelector(".w-ay").addEventListener("click", () => { c.mode = "roll"; c.value = +(val("phi_deg") || 0).toFixed(2); host.changed(); this.render(el, c, host); });
      }
      const sl = el.querySelector(".w-sl");
      const [a, b, s] = c.mode === "roll" ? [-4, 4, 0.1] : [-40, 40, 1];
      sl.min = a; sl.max = b; sl.step = s; sl.value = c.value || 0;
      host.static = null;
      this.draw(el, c, host);
    },
    async draw(el, c, host) {
      const v = Number(c.value || 0);
      el.querySelector(".w-out").textContent = `${v} ${c.mode === "roll" ? "°" : "mm"}`;
      const kw = { roll: { phi_deg: v }, heave: { heave_mm: -v }, bump: { bump_o_mm: v } }[c.mode];
      const my = (host.seq = (host.seq || 0) + 1);
      try {
        if (!host.static) {
          host.static = await post("/api/pose", { model: state.model, axle: c.axle });
          host.view.setStatic(host.static);
        }
        const p = await post("/api/pose", { model: state.model, axle: c.axle, ...kw });
        if (my !== host.seq) return;
        const t = c.axle === "front" ? "f" : "r";
        host.view.draw(p, { constr: true, dims: true, ghost: true, loads: c.mode === "roll",
          loadsData: { Fz_out: val(`${t}.Fz_out`), Fz_in: val(`${t}.Fz_in`), may: val(`${t}.Fy_req`) } });
        readouts(el.querySelector(".readouts"), p);
      } catch (e) { setStatus(`view: ${e.message}`, true); }
    },
  },

  "sweep": {
    title: (c) => `${(state.defaults.outputs[c.output] || { label: c.output }).label} vs ${LABELS[c.x_path] || c.x_path}`, size: [6, 5],
    fields: [{ k: "x_path", label: "X parameter", type: "param" }, { k: "x0", label: "from", type: "number" },
             { k: "x1", label: "to", type: "number" }, { k: "n", label: "points", type: "number" },
             { k: "compare_path", label: "Compare parameter", type: "param", optional: true },
             { k: "compare_values", label: "compare values", type: "text" },
             { k: "output", label: "Output", type: "output" },
             { k: "limit", label: "compute limit a_y", type: "bool" }, { k: "transient", label: "compute transient metrics", type: "bool" }],
    defaults: { x_path: "maneuver.ay_g", x0: 0.05, x1: 2.0, n: 21, compare_path: "front.h_rc_mm", compare_values: "20, 40, 80", output: "f.geo_share", limit: false, transient: false },
    on: ["results"],
    async render(el, c, host) {
      const cv = c.compare_path ? String(c.compare_values).split(/[,;\s]+/).filter(Boolean).map(Number).filter(Number.isFinite) : null;
      const n = Math.max(2, Math.min(200, Number(c.n) || 21));
      const xs = Array.from({ length: n }, (_, k) => +(Number(c.x0) + (Number(c.x1) - Number(c.x0)) * k / (n - 1)).toPrecision(8));
      const lim = c.limit || c.output === "ay_max_g" || c.output === "v_max_kmh";
      const trn = c.transient || String(c.output).startsWith("tr.");
      const key = JSON.stringify([state.model, c, lim, trn]);
      if (host.cacheKey === key && host.cache) return this.plot(el, c, host.cache);
      msg(el, "running sweep…");
      try {
        const r = await post("/api/sweep", { model: state.model, x_path: c.x_path, x_values: xs, compare_path: c.compare_path || null,
          compare_values: cv && cv.length ? cv : null, include_limit: lim, include_transient: trn });
        host.cache = r; host.cacheKey = key; this.plot(el, c, r);
      } catch (e) { msg(el, `sweep failed: ${e.message}`); }
    },
    plot(el, c, r) {
      const { th, layout } = layoutFor();
      const meta = state.defaults.outputs[c.output] || { label: c.output, unit: "" };
      const traces = r.series.map((s, j) => ({ x: r.x, y: s.outputs[c.output], mode: "lines+markers", marker: { size: 4 },
        name: s.compare_value === null ? meta.label : `${r.compare_path.split(".").pop()} = ${s.compare_value}`,
        line: { color: th.c[j % 8], width: 2 }, hovertemplate: `%{y:.4g} ${meta.unit}` }));
      const cur = getPath(state.model, c.x_path);
      plotInto(el, traces, { ...layout, showlegend: traces.length > 1, xaxis: ax(th, LABELS[c.x_path] || c.x_path),
        yaxis: ax(th, `${meta.label} [${meta.unit}]`),
        shapes: typeof cur === "number" ? [{ type: "line", x0: cur, x1: cur, yref: "paper", y0: 0, y1: 1, line: { color: th.layout.xaxis.tickfont.color, width: 1, dash: "dot" } }] : [] }, th.config);
    },
  },

  "kin": {
    title: (c) => `${c.axle === "front" ? "Front" : "Rear"} ${(state.defaults.kin_outputs[c.output] || { label: c.output }).label} (${c.mode})`, size: [4, 4],
    fields: [{ k: "axle", label: "Axle", type: "select", options: { front: "Front", rear: "Rear" } },
             { k: "mode", label: "Motion", type: "select", options: { heave: "Heave [mm]", roll: "Roll [°]", bump: "1-wheel bump [mm]" } },
             { k: "x0", label: "from", type: "number" }, { k: "x1", label: "to", type: "number" },
             { k: "output", label: "Output", type: "kinout" }],
    defaults: { axle: "front", mode: "roll", x0: -3, x1: 3, output: "rc_lateral_mm" }, on: ["results", "baseline"],
    async render(el, c, host) {
      const n = 31;
      const xs = Array.from({ length: n }, (_, k) => +(Number(c.x0) + (Number(c.x1) - Number(c.x0)) * k / (n - 1)).toFixed(6));
      if (!xs.includes(0)) { xs.push(0); xs.sort((a, b) => a - b); }
      const models = [["current", state.model, 0, "solid"]];
      if (state.base) models.push(["baseline", state.base.model, -1, "dash"]);
      const key = JSON.stringify([models.map((m) => m[1][c.axle]), c]);
      if (host.cacheKey !== key) {
        try {
          host.cache = await Promise.all(models.map((m) => post("/api/kin_sweep", { model: m[1], axle: c.axle, mode: c.mode, values: xs })));
          host.cacheKey = key;
        } catch (e) { return msg(el, `kinematics: ${e.message}`); }
      }
      const { th, layout } = layoutFor();
      const meta = state.defaults.kin_outputs[c.output] || { label: c.output, unit: "" };
      const traces = host.cache.map((r, i) => ({ x: r.x, y: r.series[0].outputs[c.output], mode: "lines", name: models[i][0],
        line: { color: colorOf(th, models[i][2]), width: 2, dash: models[i][3] }, hovertemplate: `%{y:.4g} ${meta.unit}` }));
      const xt = { heave: "wheel travel [mm]", roll: "roll φ [°]", bump: "bump [mm]" }[c.mode];
      plotInto(el, traces, { ...layout, showlegend: traces.length > 1, xaxis: ax(th, xt), yaxis: ax(th, `${meta.label} [${meta.unit}]`) }, th.config);
    },
  },

  "tire": {
    title: (c) => (c.kind === "fa" ? "Tire F_y(α)" : "Tire F_y(F_z)"), size: [4, 4],
    fields: [{ k: "kind", label: "Curve", type: "select", options: { fa: "F_y vs slip angle", fz: "F_y vs wheel load" } }],
    defaults: { kind: "fz" }, on: ["results"],
    async render(el, c, host) {
      const key = JSON.stringify(state.model.tire);
      if (host.cacheKey !== key) { host.cache = await post("/api/tire_curves", { model: state.model }); host.cacheKey = key; }
      const { th, layout } = layoutFor();
      const src = c.kind === "fa" ? host.cache.fy_alpha : host.cache.fy_fz;
      const traces = src.map((s, j) => ({ x: c.kind === "fa" ? s.alpha_deg : s.Fz, y: s.Fy, mode: "lines",
        name: c.kind === "fa" ? `F_z ${Math.round(s.Fz)} N` : `α ${s.alpha_deg}°`, line: { color: th.c[j % 8], width: 2 } }));
      const pts = [];
      for (const [t, n] of [["f", "F"], ["r", "R"]]) for (const [sd, lab] of [["o", "out"], ["i", "in"]]) {
        const a = val(`${t}.alpha`), fy = val(`${t}.tire_${sd}.Fy`), fz = val(`${t}.Fz_${sd === "o" ? "out" : "in"}`);
        if (a !== null && fy !== null) pts.push([c.kind === "fa" ? a : fz, fy, `${n}-${lab}`]);
      }
      traces.push({ x: pts.map((p) => p[0]), y: pts.map((p) => p[1]), text: pts.map((p) => p[2]), mode: "markers+text", textposition: "top left",
        name: "operating points", marker: { size: 9, color: th.layout.paper_bgcolor, line: { width: 2, color: th.layout.font.color } } });
      plotInto(el, traces, { ...layout, hovermode: "closest", xaxis: ax(th, c.kind === "fa" ? "α [°]" : "F_z [N]"), yaxis: ax(th, "F_y [N]") }, th.config);
    },
  },

  "params": {
    title: () => "Pinned parameters", size: [3, 6], fields: [], defaults: {}, on: ["model", "adapted"], workspaceOnly: true,
    render(el, c, host) {
      if (!host.ws) return;
      if (host._built && el.querySelector(".pins")) return host.ws.refreshPinnedValues();
      host._built = true; host.ws.renderPinned(el);
    },
  },
  "targets": {
    title: () => "Targets", size: [4, 4], fields: [], defaults: {}, on: ["results", "baseline"], workspaceOnly: true,
    render(el, c, host) { host.ws && host.ws.renderTargets(el); },
  },
};

// ------------------------------------------------------------------ host (header, settings, lifecycle)
const live = new Set();
export function broadcast(ev) {
  for (const h of live) {
    if (!h.el.isConnected) { live.delete(h); continue; }
    if ((WIDGETS[h.type].on || []).includes(ev) && h.visible()) h.render();
    else if ((WIDGETS[h.type].on || []).includes(ev)) h.dirty = true;
  }
}
["results", "baseline", "compare", "model", "adapted"].forEach((ev) => on(ev, () => broadcast(ev)));

export class WidgetHost {
  constructor(el, type, cfg, opts = {}) {
    this.el = el; this.type = type; this.def = WIDGETS[type];
    this.cfg = { ...this.def.defaults, ...(cfg || {}) };
    this.opts = opts; this.ws = opts.ws || null;
    el.classList.add("widget");
    el.innerHTML = `<div class="w-head"><span class="w-title"></span><span class="w-tools">
        ${this.def.fields.length ? `<button class="w-btn w-set" title="Settings">⚙</button>` : ""}
        ${opts.removable ? `<button class="w-btn w-rm" title="Remove">✕</button>` : ""}</span></div>
      <div class="w-settings" hidden></div><div class="w-body"></div>`;
    this.body = el.querySelector(".w-body");
    const set = el.querySelector(".w-set");
    if (set) set.addEventListener("click", () => this.toggleSettings());
    const rm = el.querySelector(".w-rm");
    if (rm) rm.addEventListener("click", () => opts.onRemove && opts.onRemove(this));
    live.add(this);
    this.render();
  }
  visible() { return this.el.offsetParent !== null; }
  changed() { this.opts.onChange && this.opts.onChange(this); }
  async render() {
    this.dirty = false;
    try { this.el.querySelector(".w-title").textContent = this.def.title(this.cfg); } catch { /* defaults not ready */ }
    try { await this.def.render.call(this.def, this.body, this.cfg, this); } catch (e) { msg(this.body, `error: ${e.message}`); }
  }
  resize() { const p = this.body.querySelector(".plot-fill"); if (p && p.data) Plotly.Plots.resize(p); }
  toggleSettings() {
    const box = this.el.querySelector(".w-settings");
    box.hidden = !box.hidden;
    if (box.hidden) return;
    box.innerHTML = this.def.fields.map((f) => `<label>${f.label}${fieldInput(f, this.cfg[f.k])}</label>`).join("") +
      `<button class="small primary w-apply">Apply</button>`;
    box.querySelector(".w-apply").addEventListener("click", () => {
      for (const f of this.def.fields) {
        const inp = box.querySelector(`[data-k="${f.k}"]`);
        this.cfg[f.k] = f.type === "bool" ? inp.checked : f.type === "number" ? Number(inp.value) : inp.value;
      }
      box.hidden = true; this.cache = null; this.cacheKey = null; this.static = null;
      if (this.type === "view2d") this.body.innerHTML = "";
      this.changed(); this.render();
    });
  }
}

function fieldInput(f, v) {
  const opt = (o, sel) => Object.entries(o).map(([k, l]) => `<option value="${k}" ${String(k) === String(sel) ? "selected" : ""}>${l}</option>`).join("");
  if (f.type === "select") return `<select data-k="${f.k}">${opt(f.options, v)}</select>`;
  if (f.type === "number") return `<input type="number" step="any" data-k="${f.k}" value="${v ?? ""}">`;
  if (f.type === "bool") return `<input type="checkbox" data-k="${f.k}" ${v ? "checked" : ""}>`;
  if (f.type === "text") return `<input type="text" data-k="${f.k}" value="${v ?? ""}">`;
  if (f.type === "param") return `<select data-k="${f.k}">${f.optional ? `<option value="">— none —</option>` : ""}${paramOptions(v)}</select>`;
  if (f.type === "output") return `<select data-k="${f.k}">${groupedOptions(state.defaults.outputs, v)}</select>`;
  if (f.type === "kinout") return `<select data-k="${f.k}">${opt(Object.fromEntries(Object.entries(state.defaults.kin_outputs).map(([k, m]) => [k, m.label])), v)}</select>`;
  if (f.type === "stepkey") return `<select data-k="${f.k}">${stepOptions(v)}</select>`;
  return "";
}
export function paramOptions(v) {
  return state.defaults.params.filter((p) => typeof p.value === "number")
    .map((p) => `<option value="${p.path}" ${p.path === v ? "selected" : ""}>${LABELS[p.path] || p.path}</option>`).join("");
}
function groupedOptions(outs, v) {
  const g = {};
  Object.entries(outs).forEach(([k, m]) => (g[m.group] = g[m.group] || []).push([k, m]));
  return Object.entries(g).map(([name, arr]) => `<optgroup label="${name}">${arr.map(([k, m]) => `<option value="${k}" ${k === v ? "selected" : ""}>${m.label}</option>`).join("")}</optgroup>`).join("");
}
export function stepOptions(v) {
  if (!state.res) return "";
  const g = {};
  state.res.steps.filter((s) => typeof s.value === "number").forEach((s) => (g[s.section] = g[s.section] || []).push(s));
  return Object.entries(g).map(([sec, arr]) => `<optgroup label="${sec}">${arr.map((s) => `<option value="${s.key}" ${s.key === v ? "selected" : ""}>${s.label} [${s.unit}]</option>`).join("")}</optgroup>`).join("");
}

// ------------------------------------------------------------------ fixed layouts (non-editable tabs)
export function mountFixed(container, list) {
  container.innerHTML = "";
  return list.map(([type, cfg, span]) => {
    const el = document.createElement("div");
    el.className = "fixed-w";
    el.style.gridColumn = `span ${span || 6}`;
    container.appendChild(el);
    return new WidgetHost(el, type, cfg);
  });
}
export const renderAllIn = (container) => {
  for (const h of live) if (container.contains(h.el) && (h.dirty || !h.body.firstChild)) h.render();
  requestAnimationFrame(() => { for (const h of live) if (container.contains(h.el)) h.resize(); });
};
export { applyParam, FIELD };
