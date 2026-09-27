// Plotly helpers: theme, parameter sweeps, kinematic curves, tire curves.
import { LABELS } from "./spec.js";

// Categorical palette (fixed order, validated light/dark steps).
const CAT_LIGHT = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948"];
const CAT_DARK = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"];
const DASH = ["solid", "dash", "dot", "dashdot", "longdash", "longdashdot", "solid", "dash"];

export function isDark() {
  const t = document.documentElement.dataset.theme;
  if (t) return t === "dark";
  return window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
}

export function plotTheme() {
  const cs = getComputedStyle(document.documentElement);
  const g = (n) => cs.getPropertyValue(n).trim();
  const text = g("--text"), muted = g("--muted"), border = g("--border"), surf = g("--surface");
  const ax = { gridcolor: border, zerolinecolor: muted, linecolor: border, tickfont: { color: muted },
    titlefont: { color: muted, size: 12 }, automargin: true };
  return {
    c: isDark() ? CAT_DARK : CAT_LIGHT,
    layout: {
      paper_bgcolor: surf, plot_bgcolor: surf, font: { color: text, size: 12, family: "system-ui, sans-serif" },
      xaxis: { ...ax }, yaxis: { ...ax }, margin: { l: 64, r: 16, t: 36, b: 48 },
      hovermode: "x unified", hoverlabel: { bgcolor: surf, bordercolor: border, font: { color: text } },
      legend: { orientation: "h", y: -0.22, font: { color: text } },
    },
    config: { responsive: true, displaylogo: false, modeBarButtonsToRemove: ["lasso2d", "select2d"] },
  };
}

const labelOf = (path) => LABELS[path] || path;

function card(container, id, title) {
  const d = document.createElement("div");
  d.className = "card";
  d.innerHTML = `<h3>${title}</h3><div id="${id}" class="plot"></div>`;
  container.appendChild(d);
  return id;
}

export function plotSweep(container, res, outputs, selected) {
  container.innerHTML = "";
  const th = plotTheme();
  const xs = res.x;
  const cmpLabel = res.compare_path ? labelOf(res.compare_path) : null;
  selected.forEach((key, i) => {
    const meta = outputs[key];
    const id = card(container, `sp-${i}`, `${meta.label} <span class="muted">[${meta.unit}]</span>`);
    const traces = res.series.map((s, j) => ({
      type: "scatter", mode: "lines+markers", x: xs, y: s.outputs[key],
      name: s.compare_value === null ? meta.label : `${res.compare_path.split(".").pop()} = ${s.compare_value}`,
      line: { color: th.c[j % 8], width: 2, dash: DASH[j % 8] }, marker: { size: 4 },
      hovertemplate: `%{y:.4g} ${meta.unit}`,
    }));
    Plotly.newPlot(id, traces, {
      ...th.layout,
      xaxis: { ...th.layout.xaxis, title: labelOf(res.x_path) },
      yaxis: { ...th.layout.yaxis, title: `${meta.label} [${meta.unit}]` },
      showlegend: res.series.length > 1,
      title: cmpLabel ? { text: `compare: ${cmpLabel}`, font: { size: 11, color: th.layout.xaxis.tickfont.color }, x: 0.01 } : undefined,
    }, th.config);
  });
  resizeAll(container);
}

const resizeAll = (c) => requestAnimationFrame(() => c.querySelectorAll(".js-plotly-plot").forEach((p) => Plotly.Plots.resize(p)));

export function plotKin(container, res, kinOutputs) {
  container.innerHTML = "";
  const th = plotTheme();
  const xt = { heave: "Wheel travel, + jounce [mm]", roll: "Body roll φ, + outer down [°]", bump: "Ground bump under outer wheel [mm]" }[res.mode];
  const groups = [
    ["Camber to ground", ["camber_o_deg", "camber_i_deg"], "°"],
    ["Roll-centre height", ["rc_height_mm"], "mm"],
    ["Roll-centre lateral position (+ toward outer wheel)", ["rc_lateral_mm"], "mm"],
    ["Track change", ["track_change_mm"], "mm"],
    ["Wheel travel (+ jounce)", ["travel_o_mm", "travel_i_mm"], "mm"],
    ["Outer camber relative to body", ["camber_o_body_deg"], "°"],
  ];
  groups.forEach(([title, keys, unit], i) => {
    const id = card(container, `kp-${i}`, `${title} <span class="muted">[${unit}]</span>`);
    const traces = [];
    res.series.forEach((s, j) => keys.forEach((k, q) => {
      const side = keys.length > 1 ? (k.includes("_i_") ? " inner" : " outer") : "";
      traces.push({
        type: "scatter", mode: "lines", x: res.x, y: s.outputs[k],
        name: (s.compare_value === null ? kinOutputs[k].label : `${res.compare_path.split(".").slice(-2).join(".")} = ${s.compare_value}${side}`),
        line: { color: th.c[j % 8], width: 2, dash: q === 1 ? "dash" : "solid" },
        hovertemplate: `%{y:.3f} ${unit}`,
      });
    }));
    Plotly.newPlot(id, traces, {
      ...th.layout, xaxis: { ...th.layout.xaxis, title: xt },
      yaxis: { ...th.layout.yaxis, title: `${title} [${unit}]` }, showlegend: traces.length > 1,
    }, th.config);
  });
  resizeAll(container);
}

export function plotTire(curves, ops) {
  const th = plotTheme();
  const t1 = curves.fy_alpha.map((c, j) => ({
    type: "scatter", mode: "lines", x: c.alpha_deg, y: c.Fy, name: `F_z = ${Math.round(c.Fz)} N`,
    line: { color: th.c[j % 8], width: 2 }, hovertemplate: "%{y:.0f} N",
  }));
  const t2 = curves.fy_fz.map((c, j) => ({
    type: "scatter", mode: "lines", x: c.Fz, y: c.Fy, name: `α = ${c.alpha_deg}°`,
    line: { color: th.c[j % 8], width: 2 }, hovertemplate: "%{y:.0f} N",
  }));
  if (ops && ops.length) {
    const mk = { size: 10, color: th.layout.paper_bgcolor, line: { width: 2, color: th.layout.font.color } };
    t1.push({ type: "scatter", mode: "markers+text", x: ops.map((o) => o.alpha), y: ops.map((o) => o.Fy),
      text: ops.map((o) => o.name), textposition: ops.map((o) => (o.name.endsWith("out") ? "top left" : "bottom right")), marker: mk, name: "operating points",
      hovertemplate: "%{text}: α=%{x:.2f}°, F_y=%{y:.0f} N<extra></extra>" });
    t2.push({ type: "scatter", mode: "markers+text", x: ops.map((o) => o.Fz), y: ops.map((o) => o.Fy),
      text: ops.map((o) => o.name), textposition: ops.map((o) => (o.name.startsWith("F") ? "top left" : "bottom right")), marker: mk, name: "operating points",
      hovertemplate: "%{text}: F_z=%{x:.0f} N, F_y=%{y:.0f} N<extra></extra>" });
  }
  Plotly.react("t-fa", t1, { ...th.layout, hovermode: "closest", xaxis: { ...th.layout.xaxis, title: "Slip angle α [°]" },
    yaxis: { ...th.layout.yaxis, title: "Lateral force F_y [N]" } }, th.config);
  Plotly.react("t-fz", t2, { ...th.layout, hovermode: "closest", xaxis: { ...th.layout.xaxis, title: "Wheel load F_z [N]" },
    yaxis: { ...th.layout.yaxis, title: "Lateral force F_y [N]" } }, th.config);
}
