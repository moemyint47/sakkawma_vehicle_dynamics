// Corner replay: top-down animation of the car on the road, synced timeline, plots and derivations.
import { state, post, on, emit, setStatus, applyParam } from "./store.js";
import { plotTheme, isDark } from "./plots.js";
import { stepHTML } from "./derivations.js";

const R = { data: null, key: "", idx: 0, loading: null };
export const replayState = R;

const STATUS = {
  "straight": { icon: "—", ci: -1, label: "Straight" },
  "neutral": { icon: "●", ci: 2, label: "Neutral" },
  "understeer": { icon: "▲", ci: 0, label: "Understeer" },
  "oversteer": { icon: "▼", ci: 1, label: "Oversteer" },
  "front sliding (understeer)": { icon: "⚠", ci: 7, label: "Front sliding – limit understeer" },
  "rear sliding (oversteer)": { icon: "⚠", ci: 7, label: "Rear sliding – limit oversteer" },
  "both sliding": { icon: "⚠", ci: 7, label: "Both axles sliding" },
  "wheel lift": { icon: "⚠", ci: 4, label: "Inner wheel lift" },
};
const WHEELS = ["FL", "FR", "RL", "RR"];
const colorOf = (th, ci) => (ci < 0 ? th.layout.xaxis.tickfont.color : th.c[ci]);

export async function loadReplay(force = false) {
  const key = JSON.stringify(state.model);
  if (!force && key === R.key && R.data) return R.data;
  if (R.loading && R.loadingKey === key) return R.loading;
  R.loadingKey = key;
  R.loading = (async () => {
    try {
      const d = await post("/api/replay", { model: state.model, dt: 0.01 });
      R.data = d; R.key = key;
      R.idx = Math.min(R.idx, d.t.length - 1);
      emit("replay", d);
      return d;
    } catch (e) { setStatus(`replay: ${e.message}`, true); return null; }
    finally { R.loading = null; }
  })();
  return R.loading;
}
export function setIdx(i) {
  if (!R.data) return;
  R.idx = Math.max(0, Math.min(R.data.t.length - 1, i));
  emit("replay-time", R.idx);
}

// ------------------------------------------------------------------ top view (SVG)
export class TopView {
  constructor(svg, opts = {}) { this.svg = svg; this.follow = opts.follow ?? true; this.compact = !!opts.compact; }

  draw(d, i) {
    if (!d) return;
    const th = plotTheme();
    const W = this.follow ? 16 : null;
    const x = d.x[i], y = d.y[i], psi = d.psi_deg[i];
    let vb;
    if (this.follow) vb = [x - W / 2, -y - W * 0.36, W, W * 0.72];
    else {
      const xs = d.road.x, ys = d.road.y;
      const x0 = Math.min(...xs) - 3, x1 = Math.max(...xs) + 3, y0 = Math.min(...ys) - 3, y1 = Math.max(...ys) + 3;
      vb = [x0, -y1, x1 - x0, y1 - y0];
    }
    this.svg.setAttribute("viewBox", vb.join(" "));
    const P = (px, py) => `${px.toFixed(3)},${(-py).toFixed(3)}`;
    const o = [];
    const u = vb[2] / 60; // text / stroke unit
    // road band (3.5 m) and centreline
    const rx = d.road.x, ry = d.road.y, n = rx.length;
    const left = [], right = [];
    for (let k = 0; k < n; k++) {
      const k0 = Math.max(0, k - 1), k1 = Math.min(n - 1, k + 1);
      const tx = rx[k1] - rx[k0], ty = ry[k1] - ry[k0], L = Math.hypot(tx, ty) || 1;
      const nx = -ty / L, ny = tx / L;
      left.push(P(rx[k] + nx * 1.75, ry[k] + ny * 1.75)); right.push(P(rx[k] - nx * 1.75, ry[k] - ny * 1.75));
    }
    o.push(`<polygon points="${left.join(" ")} ${right.reverse().join(" ")}" fill="var(--surface-2)" stroke="var(--border)" stroke-width="${u * 0.15}"/>`);
    o.push(`<polyline points="${rx.map((v, k) => P(v, ry[k])).join(" ")}" fill="none" stroke="var(--muted)" stroke-width="${u * 0.12}" stroke-dasharray="${u} ${u}"/>`);
    // trail coloured by state
    let seg = [P(d.x[0], d.y[0])], cur = d.status[0];
    const flush = (st) => {
      const s = STATUS[st] || STATUS.neutral;
      if (seg.length > 1) o.push(`<polyline points="${seg.join(" ")}" fill="none" stroke="${colorOf(th, s.ci)}" stroke-width="${u * 0.55}" stroke-linecap="round" opacity=".85"/>`);
    };
    for (let k = 1; k <= i; k++) {
      seg.push(P(d.x[k], d.y[k]));
      if (d.status[k] !== cur || k === i) { flush(cur); seg = [P(d.x[k], d.y[k])]; cur = d.status[k]; }
    }
    // car (vehicle frame: x forward, y left) -> rotate by psi
    const G = d.geom, tf = G.tf / 2, tr = G.tr / 2, a = G.a, b = G.b;
    const Fzref = (G.Fz0f + G.Fz0r) / 2;
    const st = STATUS[d.status[i]] || STATUS.neutral;
    const sc = colorOf(th, st.ci);
    const car = [];
    car.push(`<rect x="${-b - 0.35}" y="${-Math.max(tf, tr) * 0.55}" width="${a + b + 0.8}" height="${Math.max(tf, tr) * 1.1}" rx="0.25"
      fill="var(--surface)" stroke="${sc}" stroke-width="${u * 0.25}" opacity=".95"/>`);
    const delta = d.delta[i];
    const wheels = { FL: [a, tf, delta], FR: [a, -tf, delta], RL: [-b, tr, 0], RR: [-b, -tr, 0] };
    for (const w of WHEELS) {
      const [wx, wy, dl] = wheels[w];
      const Fz = Math.max(d.Fz[w][i], 0), rad = 0.32 * Math.sqrt(Fz / Fzref);
      const share = Fz / (4 * Fzref);
      car.push(`<circle cx="${wx}" cy="${-wy}" r="${rad.toFixed(3)}" fill="${th.c[0]}" fill-opacity="${(0.15 + 1.2 * share).toFixed(2)}" stroke="${th.c[0]}" stroke-width="${u * 0.08}"/>`);
      car.push(`<g transform="translate(${wx},${-wy}) rotate(${-dl})"><rect x="-0.23" y="-0.1" width="0.46" height="0.2" rx="0.04" fill="var(--tire)"/>`);
      const Fy = d[`Fy${w}`][i], len = Fy / 1500;
      if (Math.abs(len) > 0.02) car.push(`<line x1="0" y1="0" x2="0" y2="${(-len).toFixed(3)}" stroke="var(--load)" stroke-width="${u * 0.18}" marker-end="url(#rpA)"/>`);
      car.push(`</g>`);
    }
    // velocity (heading + beta) and heading
    const be = d.beta[i] * Math.PI / 180, vl = 1.4 + d.v_kmh[i] / 40;
    car.push(`<line x1="0" y1="0" x2="${(a + 0.9).toFixed(2)}" y2="0" stroke="var(--muted)" stroke-width="${u * 0.1}" stroke-dasharray="${u * 0.4} ${u * 0.3}"/>`);
    car.push(`<line x1="0" y1="0" x2="${(vl * Math.cos(be)).toFixed(3)}" y2="${(-vl * Math.sin(be)).toFixed(3)}" stroke="${th.c[6]}" stroke-width="${u * 0.2}" marker-end="url(#rpV)"/>`);
    car.push(`<circle r="${u * 0.35}" fill="var(--text)"/>`);
    o.push(`<defs><marker id="rpA" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="var(--load)"/></marker>
      <marker id="rpV" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="4" markerHeight="4" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="${th.c[6]}"/></marker></defs>`);
    o.push(`<g transform="translate(${x},${-y}) rotate(${-psi})">${car.join("")}</g>`);
    // badge (screen-aligned text in world units)
    const bx = vb[0] + u, by = vb[1] + u * 2.2;
    o.push(`<text x="${bx}" y="${by}" font-size="${u * 1.6}" fill="${sc}" font-weight="700" paint-order="stroke" stroke="var(--surface)" stroke-width="${u * 0.4}">${st.icon} ${st.label}</text>`);
    o.push(`<text x="${bx}" y="${by + u * 2}" font-size="${u * 1.15}" fill="var(--muted)" paint-order="stroke" stroke="var(--surface)" stroke-width="${u * 0.3}">t = ${d.t[i].toFixed(2)} s · ${d.v_kmh[i].toFixed(1)} km/h · a_y ${d.ay_g[i].toFixed(2)} g · a_x ${d.ax_g[i].toFixed(2)} g · δ ${d.delta[i].toFixed(1)}° · β ${d.beta[i].toFixed(1)}°</text>`);
    this.svg.innerHTML = o.join("");
  }
}

// ------------------------------------------------------------------ side panel
export function panelHTML(d, i) {
  const G = d.geom;
  const box = (w) => {
    const Fz = d.Fz[w][i], base = w[0] === "F" ? G.Fz0f : G.Fz0r, dlt = Fz - base, tot = WHEELS.reduce((s, k) => s + d.Fz[k][i], 0);
    return `<div class="wl"><div class="wl-h">${w}</div><div class="wl-v">${Math.round(Fz)} <small>N</small></div>
      <div class="wl-bar"><span style="width:${Math.max(0, Math.min(100, 100 * Fz / (2.2 * base)))}%"></span></div>
      <div class="wl-s">${(100 * Fz / tot).toFixed(1)} % · ${dlt >= 0 ? "+" : ""}${Math.round(dlt)} N</div>
      <div class="wl-s">F<sub>y</sub> ${Math.round(d[`Fy${w}`][i])} N</div></div>`;
  };
  const f = (v, dgt = 2) => (v == null || Number.isNaN(v) ? "—" : Number(v).toFixed(dgt));
  const fr = d.Fz.FL[i] + d.Fz.FR[i], tot = fr + d.Fz.RL[i] + d.Fz.RR[i];
  return `<div class="wl-grid">${box("FL")}${box("FR")}${box("RL")}${box("RR")}</div>
    <table class="vals small-text"><tbody>
      <tr><td>Front axle load share</td><td class="n">${f(100 * fr / tot, 1)} %</td></tr>
      <tr><td>α_f / α_r</td><td class="n">${f(d.alpha_f[i])}° / ${f(d.alpha_r[i])}°</td></tr>
      <tr><td>Balance Δα (+ understeer)</td><td class="n"><b>${f(d.dalpha[i], 3)}°</b></td></tr>
      <tr><td>Grip used front / rear</td><td class="n">${f(100 * d.util_f[i], 0)} % / ${f(100 * d.util_r[i], 0)} %</td></tr>
      <tr><td>Yaw rate r / ṙ</td><td class="n">${f(d.r_deg_s[i], 1)} °/s / ${f(d.rdot_deg_s2[i], 0)} °/s²</td></tr>
      <tr><td>Yaw moment I_z·ṙ</td><td class="n">${f(d.Mz_yaw[i], 0)} N·m</td></tr>
      <tr><td>Body roll φ</td><td class="n">${f(d.phi_deg[i])}°</td></tr>
    </tbody></table>`;
}

// ------------------------------------------------------------------ synced plots
const PLOTS = [
  { id: "loads", title: "Wheel loads", unit: "N", tr: (d) => WHEELS.map((w, k) => ({ y: d.Fz[w], name: w, ci: k })) },
  { id: "bal", title: "Balance Δα = sgn(a_y)(α_f − α_r)  (+ understeer)", unit: "°", band: true, tr: (d) => [{ y: d.dalpha, name: "Δα", ci: 0 }] },
  { id: "util", title: "Grip utilisation", unit: "–", tr: (d) => [{ y: d.util_f, name: "front", ci: 0 }, { y: d.util_r, name: "rear", ci: 1 }], one: true },
  { id: "acc", title: "Accelerations", unit: "g", tr: (d) => [{ y: d.ay_g, name: "a_y", ci: 0 }, { y: d.ax_g, name: "a_x", ci: 1 }] },
  { id: "slip", title: "Slip angles, sideslip, steer", unit: "°", tr: (d) => [{ y: d.alpha_f, name: "α_f", ci: 0 }, { y: d.alpha_r, name: "α_r", ci: 1 }, { y: d.beta, name: "β", ci: 6 }, { y: d.delta, name: "δ", ci: 3 }] },
  { id: "yaw", title: "Yaw moment I_z·ṙ", unit: "N·m", tr: (d) => [{ y: d.Mz_yaw, name: "I_z ṙ", ci: 6 }] },
];

export function drawPlots(host, d, ids = null) {
  const th = plotTheme();
  host.innerHTML = "";
  for (const p of PLOTS.filter((q) => !ids || ids.includes(q.id))) {
    const card = document.createElement("div");
    card.className = "card rp-plot";
    card.innerHTML = `<h3>${p.title} <span class="muted">[${p.unit}]</span></h3><div class="plot-fill-rel"></div>`;
    host.appendChild(card);
    const div = card.querySelector(".plot-fill-rel");
    const traces = p.tr(d).map((t) => ({ x: d.t, y: t.y, name: t.name, mode: "lines", line: { color: th.c[t.ci], width: 2 }, hovertemplate: `%{y:.3g} ${p.unit}` }));
    const shapes = [{ type: "line", x0: d.t[R.idx], x1: d.t[R.idx], yref: "paper", y0: 0, y1: 1, line: { color: th.layout.font.color, width: 1.5 } }];
    // state strip along the bottom (categorical colour + legend in the badge)
    let s0 = 0;
    for (let k = 1; k <= d.t.length; k++) {
      if (k === d.t.length || d.status[k] !== d.status[s0]) {
        const S = STATUS[d.status[s0]] || STATUS.neutral;
        shapes.push({ type: "rect", xref: "x", yref: "paper", x0: d.t[s0], x1: d.t[Math.min(k, d.t.length - 1)], y0: 0, y1: 0.04,
          fillcolor: colorOf(th, S.ci), line: { width: 0 }, opacity: 0.8 });
        s0 = k;
      }
    }
    if (p.band) {
      const bd = d.geom.band_deg;
      shapes.push({ type: "line", xref: "paper", x0: 0, x1: 1, y0: bd, y1: bd, line: { color: th.layout.xaxis.gridcolor, dash: "dot", width: 1 } });
      shapes.push({ type: "line", xref: "paper", x0: 0, x1: 1, y0: -bd, y1: -bd, line: { color: th.layout.xaxis.gridcolor, dash: "dot", width: 1 } });
    }
    if (p.one) shapes.push({ type: "line", xref: "paper", x0: 0, x1: 1, y0: 1, y1: 1, line: { color: th.c[7], dash: "dash", width: 1 } });
    Plotly.newPlot(div, traces, { ...th.layout, margin: { l: 52, r: 10, t: 6, b: 34 }, shapes, showlegend: traces.length > 1,
      legend: { orientation: "h", x: 0, y: 1, yanchor: "bottom", font: { size: 10.5, color: th.layout.font.color } },
      xaxis: { ...th.layout.xaxis, title: { text: "t [s]", font: { size: 11 } } }, yaxis: { ...th.layout.yaxis } }, th.config);
    div.on("plotly_click", (ev) => { if (ev.points && ev.points.length) setIdx(ev.points[0].pointIndex); });
  }
}
let lastCursor = 0;
export function moveCursor(host, d) {
  const now = performance.now();
  if (now - lastCursor < 60) return; // throttle relayouts while playing
  lastCursor = now;
  host.querySelectorAll(".plot-fill-rel").forEach((div) => {
    if (div.layout) Plotly.relayout(div, { "shapes[0].x0": d.t[R.idx], "shapes[0].x1": d.t[R.idx] });
  });
}

// ------------------------------------------------------------------ transport controls
export function transportHTML() {
  return `<button class="small rp-play">▶ Play</button>
    <button class="small rp-back" title="Step back 0.01 s">◀</button><button class="small rp-fwd" title="Step forward 0.01 s">▶</button>
    <select class="rp-speed" title="Playback speed"><option value="0.25">0.25×</option><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option></select>
    <input type="range" class="rp-slider" min="0" max="1" value="0" step="1">
    <output class="rp-t">t = 0.00 s</output>`;
}
let playing = false, raf = 0;
export function bindTransport(el) {
  const sl = el.querySelector(".rp-slider"), play = el.querySelector(".rp-play");
  sl.addEventListener("input", () => setIdx(Number(sl.value)));
  el.querySelector(".rp-back").addEventListener("click", () => setIdx(R.idx - 1));
  el.querySelector(".rp-fwd").addEventListener("click", () => setIdx(R.idx + 1));
  play.addEventListener("click", () => {
    playing = !playing;
    document.querySelectorAll(".rp-play").forEach((b) => (b.textContent = playing ? "❚❚ Pause" : "▶ Play"));
    if (!playing || !R.data) return;
    if (R.idx >= R.data.t.length - 1) setIdx(0);
    let last = performance.now(), t = R.data.t[R.idx];
    const step = (now) => {
      if (!playing || !R.data) return;
      const sp = Number(el.querySelector(".rp-speed").value);
      t += ((now - last) / 1000) * sp; last = now;
      const dt = R.data.t[1] - R.data.t[0];
      const i = Math.round(t / dt);
      if (i >= R.data.t.length - 1) { setIdx(R.data.t.length - 1); playing = false; document.querySelectorAll(".rp-play").forEach((b) => (b.textContent = "▶ Play")); return; }
      setIdx(i);
      raf = requestAnimationFrame(step);
    };
    raf = requestAnimationFrame(step);
  });
  const sync = () => {
    if (!R.data) return;
    sl.max = R.data.t.length - 1;
    if (document.activeElement !== sl) sl.value = R.idx;
    el.querySelector(".rp-t").textContent = `t = ${R.data.t[R.idx].toFixed(2)} s`;
  };
  on("replay", sync); on("replay-time", sync);
  sync();
}
export { STATUS, stepHTML, applyParam };
