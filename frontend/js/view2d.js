// Front-view SVG renderer with SolidWorks-style dimensions.
// Pure renderer: takes a solved pose (ground frame, mm) from the backend and draws it.
// A future 3D view (three.js) can consume the same pose/scene data.

const NS = "http://www.w3.org/2000/svg";
const f1 = (v) => (v == null || Number.isNaN(v) ? "—" : v.toFixed(1));
const f2 = (v) => (v == null || Number.isNaN(v) ? "—" : v.toFixed(2));

export class FrontView {
  constructor(svg) {
    this.svg = svg;
    this.static = null;
  }

  setStatic(pose) {
    this.static = pose;
    const o = pose.sides.o, i = pose.sides.i;
    const half = Math.max(Math.abs(o.points.cp[0]), Math.abs(i.points.cp[0]));
    const tw = Math.max(...o.tire.map((p) => Math.abs(p[0])));
    const W = Math.max(half, tw) + 330;
    const top = Math.max(...pose.body.map((p) => p[1]), pose.cg ? pose.cg[1] : 0, o.points.wc[1] * 2) + 140;
    this.vb = { x: -W, y: -top, w: 2 * W, h: top + 250 };
    this.svg.setAttribute("viewBox", `${this.vb.x} ${this.vb.y} ${this.vb.w} ${this.vb.h}`);
    this.fs = this.vb.w / 70; // font size in mm-units
  }

  draw(pose, opt) {
    if (!pose || !this.static) return;
    if (!pose.ok) {
      this.svg.innerHTML = `<text x="0" y="0" text-anchor="middle" fill="var(--bad)" font-size="${this.fs * 1.2}">${pose.error || "pose not reachable"}</text>`;
      return;
    }
    const P = (p) => `${p[0].toFixed(2)},${(-p[1]).toFixed(2)}`;
    const X = (p) => p[0], Y = (p) => -p[1];
    const fs = this.fs;
    const out = [];
    out.push(`<defs>
      <marker id="arD" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="${fs * 0.7}" markerHeight="${fs * 0.7}" orient="auto-start-reverse" markerUnits="userSpaceOnUse">
        <path d="M0,1.5 L10,5 L0,8.5 z" fill="var(--dim)"/></marker>
      <marker id="arL" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="${fs * 0.9}" markerHeight="${fs * 0.9}" orient="auto" markerUnits="userSpaceOnUse">
        <path d="M0,1 L10,5 L0,9 z" fill="var(--load)"/></marker>
      <pattern id="hatch" width="18" height="18" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
        <line x1="0" y1="0" x2="0" y2="18" stroke="var(--ground)" stroke-width="1.2" vector-effect="non-scaling-stroke"/></pattern>
    </defs>`);
    const L = (a, b, stroke, w, extra = "") =>
      `<line x1="${X(a)}" y1="${Y(a)}" x2="${X(b)}" y2="${Y(b)}" stroke="${stroke}" stroke-width="${w}" vector-effect="non-scaling-stroke" stroke-linecap="round" ${extra}/>`;
    const T = (p, s, anchor = "middle", color = "var(--text)", size = fs, extra = "") =>
      `<text x="${X(p)}" y="${Y(p)}" text-anchor="${anchor}" font-size="${size}" fill="${color}" ${extra}
        paint-order="stroke" stroke="var(--surface)" stroke-width="${size * 0.35}" stroke-linejoin="round">${s}</text>`;
    const C = (p, r, fill, stroke = "none", w = 1.5) =>
      `<circle cx="${X(p)}" cy="${Y(p)}" r="${r}" fill="${fill}" stroke="${stroke}" stroke-width="${w}" vector-effect="non-scaling-stroke"/>`;

    // ---------- ground (with optional one-wheel bump)
    const vb = this.vb;
    const gx0 = vb.x, gx1 = vb.x + vb.w;
    const cpo = pose.sides.o.points.cp, cpi = pose.sides.i.points.cp;
    const bo = pose.bump_o_mm || 0;
    let gpath;
    if (Math.abs(bo) > 0.01) {
      const a = cpo[0] - 220, b = cpo[0] + 220;
      gpath = `M${gx0},0 L${a - 60},0 L${a},${-bo} L${b},${-bo} L${b + 60},0 L${gx1},0`;
    } else gpath = `M${gx0},0 L${gx1},0`;
    out.push(`<path d="${gpath} L${gx1},60 L${gx0},60 z" fill="url(#hatch)" stroke="none" opacity=".55"/>`);
    out.push(`<path d="${gpath}" fill="none" stroke="var(--ground)" stroke-width="2" vector-effect="non-scaling-stroke"/>`);

    // ---------- static ghost
    if (opt.ghost && this.static && (Math.abs(pose.phi_deg) > 1e-6 || Math.abs(pose.heave_mm) > 1e-6 || Math.abs(bo) > 1e-6)) {
      for (const s of Object.values(this.static.sides)) {
        const q = s.points;
        out.push(`<polygon points="${s.tire.map(P).join(" ")}" fill="none" stroke="var(--ghost)" stroke-width="1.5" vector-effect="non-scaling-stroke"/>`);
        out.push(L(q.lca_in, q.lbj, "var(--ghost)", 2), L(q.uca_in, q.ubj, "var(--ghost)", 2), L(q.lbj, q.ubj, "var(--ghost)", 2));
      }
      out.push(`<polygon points="${this.static.body.map(P).join(" ")}" fill="none" stroke="var(--ghost)" stroke-width="1.5" vector-effect="non-scaling-stroke"/>`);
    }

    // ---------- body + centreline
    out.push(`<polygon points="${pose.body.map(P).join(" ")}" fill="var(--surface-2)" stroke="var(--link)" stroke-width="1.5" vector-effect="non-scaling-stroke"/>`);
    out.push(L(pose.centreline[0], pose.centreline[1], "var(--muted)", 1, `stroke-dasharray="14 5 3 5"`));

    // ---------- construction lines (IC / RC)
    const rc = pose.rc;
    if (opt.constr) {
      for (const s of Object.values(pose.sides)) {
        const q = s.points;
        if (s.ic) {
          out.push(L(q.lbj, s.ic, "var(--constr)", 1, `stroke-dasharray="8 6" opacity=".9"`));
          out.push(L(q.ubj, s.ic, "var(--constr)", 1, `stroke-dasharray="8 6" opacity=".9"`));
          out.push(L(q.cp, s.ic, "var(--rc)", 1, `stroke-dasharray="3 5" opacity=".9"`));
          const inView = s.ic[0] > vb.x && s.ic[0] < vb.x + vb.w && -s.ic[1] > vb.y && -s.ic[1] < vb.y + vb.h;
          if (inView) {
            out.push(C(s.ic, fs * 0.35, "none", "var(--constr)", 2));
            out.push(T([s.ic[0], s.ic[1] + fs * 0.7], "IC", "middle", "var(--constr)", fs * 0.8));
          }
        }
      }
      if (rc) {
        out.push(C(rc, fs * 0.45, "var(--surface)", "var(--rc)", 2.5));
        out.push(L([rc[0] - fs * 0.45, rc[1]], [rc[0] + fs * 0.45, rc[1]], "var(--rc)", 1.5));
        out.push(L([rc[0], rc[1] - fs * 0.45], [rc[0], rc[1] + fs * 0.45], "var(--rc)", 1.5));
        out.push(T([rc[0] + fs * 0.7, rc[1] + fs * 0.35], "RC", "start", "var(--rc)", fs * 0.9, `font-weight="700"`));
      }
    }

    // ---------- suspension, uprights, tires
    for (const [tag, s] of Object.entries(pose.sides)) {
      const q = s.points;
      out.push(`<polygon points="${s.tire.map(P).join(" ")}" fill="var(--tire)" fill-opacity=".12" stroke="var(--tire)" stroke-width="2" vector-effect="non-scaling-stroke" stroke-linejoin="round"/>`);
      out.push(`<polygon points="${s.rim.map(P).join(" ")}" fill="none" stroke="var(--tire)" stroke-width="1" vector-effect="non-scaling-stroke" opacity=".6"/>`);
      out.push(L(q.lca_in, q.lbj, "var(--link)", 4), L(q.uca_in, q.ubj, "var(--link)", 4));
      out.push(L(q.lbj, q.ubj, "var(--upright)", 6), L([(q.lbj[0] + q.ubj[0]) / 2, (q.lbj[1] + q.ubj[1]) / 2], q.wc, "var(--upright)", 4));
      for (const k of ["lca_in", "uca_in"]) out.push(C(q[k], fs * 0.28, "var(--surface)", "var(--link)", 2));
      for (const k of ["lbj", "ubj"]) out.push(C(q[k], fs * 0.28, "var(--upright)", "var(--surface)", 1.5));
      out.push(C(q.wc, fs * 0.18, "var(--tire)"));
      out.push(`<path d="M${X(q.cp)},${Y(q.cp)} l${-fs * 0.3},${fs * 0.5} h${fs * 0.6} z" fill="var(--tire)"/>`);
      out.push(T([q.cp[0], -222], tag === "o" ? "OUTER" : "INNER", "middle", "var(--muted)", fs * 0.75, `font-weight="600" letter-spacing="1"`));
    }

    // ---------- bell-crank: pushrod, rocker, spring
    for (const s of Object.values(pose.sides)) {
      const r = s.rocker;
      if (!r) continue;
      out.push(L(r.P, r.A, "var(--link)", 2.5));
      out.push(`<polygon points="${[r.O, r.A, r.B].map(P).join(" ")}" fill="var(--accent-2)" stroke="var(--upright)" stroke-width="2" vector-effect="non-scaling-stroke" stroke-linejoin="round"/>`);
      // spring as a coil: zig-zag between B and C
      const dx = r.C[0] - r.B[0], dz = r.C[1] - r.B[1], len = Math.hypot(dx, dz), ux = dx / len, uz = dz / len;
      const nx = -uz, nz = ux, n = 10, amp = fs * 0.35, pts = [r.B];
      for (let k = 1; k < n; k++) {
        const f = 0.15 + 0.7 * k / n, side = k % 2 ? 1 : -1;
        pts.push([r.B[0] + ux * len * f + nx * amp * side, r.B[1] + uz * len * f + nz * amp * side]);
      }
      pts.push(r.C);
      out.push(`<polyline points="${pts.map(P).join(" ")}" fill="none" stroke="var(--load)" stroke-width="1.8" vector-effect="non-scaling-stroke" stroke-linejoin="round"/>`);
      out.push(C(r.O, fs * 0.25, "var(--surface)", "var(--upright)", 2), C(r.C, fs * 0.2, "var(--surface)", "var(--link)", 1.5));
      out.push(C(r.P, fs * 0.18, "var(--link)"), C(r.A, fs * 0.15, "var(--upright)"), C(r.B, fs * 0.15, "var(--upright)"));
      out.push(T([(r.B[0] + r.C[0]) / 2, (r.B[1] + r.C[1]) / 2 + fs * 0.9], `x<tspan font-size="${fs * 0.6}" dy="${fs * 0.2}">s</tspan><tspan dy="${-fs * 0.2}"> ${r.xs.toFixed(1)}</tspan>`, "middle", "var(--load)", fs * 0.75));
    }

    // ---------- CG
    if (pose.cg) {
      const g = pose.cg, r = fs * 0.55;
      out.push(`<g transform="translate(${X(g)},${Y(g)})"><circle r="${r}" fill="var(--surface)" stroke="var(--text)" stroke-width="1.5" vector-effect="non-scaling-stroke"/>
        <path d="M0,0 L${r},0 A${r},${r} 0 0,1 0,${r} z M0,0 L${-r},0 A${r},${r} 0 0,1 0,${-r} z" fill="var(--text)"/></g>`);
      out.push(T([g[0] - fs * 0.9, g[1] + fs * 0.2], "CG", "end", "var(--text)", fs * 0.85, `font-weight="700"`));
    }

    // ---------- loads & inertial force
    if (opt.loads && opt.loadsData) {
      const d = opt.loadsData, sc = 0.16; // mm per N
      for (const [tag, F] of [["o", d.Fz_out], ["i", d.Fz_in]]) {
        const cp = pose.sides[tag].points.cp;
        if (F == null) continue;
        const len = Math.max(F, 0) * sc;
        const base = [cp[0], cp[1] - len - 10];
        if (len > 1) out.push(L(base, [cp[0], cp[1] - 8], "var(--load)", 3, `marker-end="url(#arL)"`));
        out.push(T([cp[0] + (tag === "o" ? 1 : -1) * fs * 0.6, cp[1] - Math.min(len, 150) * 0.5 - 20], `${Math.round(F)} N`,
          tag === "o" ? "start" : "end", "var(--load)", fs * 0.85, `font-weight="600"`));
      }
      if (pose.cg && d.may) {
        const g = pose.cg, len = Math.min(d.may * 0.12, 300);
        out.push(L([g[0] + fs * 0.6, g[1]], [g[0] + fs * 0.6 + len, g[1]], "var(--load)", 3, `marker-end="url(#arL)"`));
        out.push(T([g[0] + fs * 0.6 + len / 2, g[1] + fs * 0.6], `m·a_y ${Math.round(d.may)} N`, "middle", "var(--load)", fs * 0.8));
      }
    }

    // ---------- dimensions
    if (opt.dims) out.push(this.dims(pose, T, L, fs));

    this.svg.innerHTML = out.join("");
  }

  dims(pose, T, L, fs) {
    const o = [];
    const col = "var(--dim)";
    const ext = (a, b) => L(a, b, col, 0.8);
    const dl = (a, b) => L(a, b, col, 1, `marker-start="url(#arD)" marker-end="url(#arD)"`);
    const cpo = pose.sides.o.points.cp, cpi = pose.sides.i.points.cp;
    // track (horizontal, below ground)
    const zt = -150;
    o.push(ext([cpo[0], cpo[1] - 10], [cpo[0], zt - 15]), ext([cpi[0], cpi[1] - 10], [cpi[0], zt - 15]));
    o.push(dl([cpi[0], zt], [cpo[0], zt]));
    o.push(T([(cpo[0] + cpi[0]) / 2, zt + fs * 0.35], `${f1(pose.track_mm)}`, "middle", col, fs, `font-weight="600"`));
    o.push(T([(cpo[0] + cpi[0]) / 2, zt - fs * 1.1], "track", "middle", "var(--muted)", fs * 0.7));
    // RC height (vertical) + lateral offset
    if (pose.rc) {
      const rc = pose.rc;
      const yd = rc[0] - fs * 3.2;
      o.push(ext([rc[0] - fs * 0.6, rc[1]], [yd - 12, rc[1]]), ext([rc[0] - 30, 0], [yd - 12, 0]));
      if (Math.abs(rc[1]) > 2) o.push(dl([yd, 0], [yd, rc[1]]));
      o.push(T([yd - fs * 0.4, rc[1] / 2 + fs * 0.3], `h<tspan font-size="${fs * 0.7}" dy="${fs * 0.25}">RC</tspan><tspan dy="${-fs * 0.25}"> ${f1(pose.rc_height_mm)}</tspan>`, "end", col, fs, `font-weight="600"`));
      if (Math.abs(rc[0]) > 1) {
        const zl = Math.min(rc[1], 0) - 40;
        o.push(ext([0, -5], [0, zl - 12]), ext([rc[0], rc[1] - fs * 0.5], [rc[0], zl - 12]));
        o.push(dl([0, zl], [rc[0], zl]));
        o.push(T([rc[0] / 2, zl - fs * 1.05], `y<tspan font-size="${fs * 0.7}" dy="${fs * 0.25}">RC</tspan><tspan dy="${-fs * 0.25}"> ${f1(pose.rc_lateral_mm)}</tspan>`, "middle", col, fs * 0.9, `font-weight="600"`));
      }
    }
    // CG height
    if (pose.cg) {
      const g = pose.cg, yd = g[0] + fs * 4.2;
      o.push(ext([g[0] + fs * 0.7, g[1]], [yd + 12, g[1]]), ext([g[0] + 30, 0], [yd + 12, 0]));
      o.push(dl([yd, 0], [yd, g[1]]));
      o.push(T([yd + fs * 0.4, g[1] * 0.62], `h<tspan font-size="${fs * 0.7}" dy="${fs * 0.25}">CG</tspan><tspan dy="${-fs * 0.25}"> ${f1(g[1])}</tspan>`, "start", col, fs, `font-weight="600"`));
    }
    // camber labels (angle to ground normal)
    for (const [tag, s] of Object.entries(pose.sides)) {
      const cp = s.points.cp, wc = s.points.wc;
      const top = [2 * wc[0] - cp[0], 2 * wc[1] - cp[1]];
      o.push(L(cp, [cp[0], top[1] + 40], col, 0.8, `stroke-dasharray="5 4"`));
      const sgn = tag === "o" ? 1 : -1;
      o.push(T([cp[0] + sgn * (fs * 0.2), top[1] + 55 + fs * 0.4], `γ ${f2(s.camber_ground_deg)}°`, tag === "o" ? "start" : "end", col, fs * 0.95, `font-weight="600"`));
    }
    // roll angle
    o.push(T([0, (pose.body[2][1] + pose.body[3][1]) / 2 + fs * 1.6], `φ = ${f2(pose.phi_deg)}°`, "middle", col, fs, `font-weight="600"`));
    return o.join("");
  }
}

export function readouts(el, pose) {
  if (!pose || !pose.ok) { el.innerHTML = ""; return; }
  const o = pose.sides.o, i = pose.sides.i;
  const r = (k, v, u) => `<div class="r"><div class="k">${k}</div><div class="v">${v} <span class="muted">${u}</span></div></div>`;
  el.innerHTML = [
    r("Roll φ", f2(pose.phi_deg), "°"), r("RC height", f1(pose.rc_height_mm), "mm"),
    r("RC lateral (+ outer)", f1(pose.rc_lateral_mm), "mm"), r("Track", f1(pose.track_mm), "mm"),
    r("Camber outer (ground)", f2(o.camber_ground_deg), "°"), r("Camber inner (ground)", f2(i.camber_ground_deg), "°"),
    r("Travel outer", f1(o.travel_mm), "mm"), r("Travel inner", f1(i.travel_mm), "mm"),
  ].join("");
}
