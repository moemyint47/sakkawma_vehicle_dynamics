// Rendering of derivation steps (formula -> substitution -> result) with KaTeX.
export const S = { byKey: {}, steps: [] };

export function setSteps(steps) {
  S.steps = steps;
  S.byKey = Object.fromEntries(steps.map((s) => [s.key, s]));
}

const tex = (s) => {
  try { return katex.renderToString("\\displaystyle " + s, { throwOnError: false, displayMode: false }); }
  catch { return `<code>${s}</code>`; }
};

export function fmtNum(v, d = 3) {
  if (v === null || v === undefined || Number.isNaN(v)) return "—";
  if (typeof v !== "number") return String(v);
  const a = Math.abs(v);
  if (a !== 0 && (a >= 1e5 || a < 1e-3)) return v.toExponential(2);
  return v.toFixed(d);
}

export function stepHTML(st, { compact = false } = {}) {
  if (!st) return `<div class="deriv muted">No derivation recorded.</div>`;
  const val = typeof st.value === "number" ? fmtNum(st.value, 4) : st.value;
  const unit = st.unit && st.unit !== "-" ? ` ${st.unit}` : "";
  return `<div class="deriv">
    ${compact ? "" : `<div class="dl">${st.label} <span class="key">${st.key}</span></div>`}
    <div class="eq">${tex(`${st.symbol} = ${st.formula}`)}</div>
    <div class="eq">${tex(`${st.symbol} = ${st.subst}`)}</div>
    <div class="eq">${tex(`${st.symbol} = \\mathbf{${val}}`)}&nbsp;<b>${unit}</b></div>
    ${st.ref ? `<div class="meta">Ref: ${st.ref}</div>` : ""}
    ${st.note ? `<div class="meta">${st.note}</div>` : ""}
    <div class="meta"><a href="#" data-goto="${st.key}">show in full derivation list ↓</a></div>
  </div>`;
}

export const fx = (key) => (S.byKey[key] ? `<button class="fx" data-fx="${key}" title="Show derivation">ƒ</button>` : "");

export function renderAll(container) {
  const secs = {};
  S.steps.forEach((s) => (secs[s.section] = secs[s.section] || []).push(s));
  container.innerHTML = Object.entries(secs).map(([sec, arr]) => `
    <details class="dsec" ${sec.startsWith("1.") ? "" : ""}><summary>${sec} <span class="muted">(${arr.length})</span></summary>
      <div class="dlist">${arr.map((s) => `
        <div class="ditem" id="d-${cssId(s.key)}">
          <div><b>${s.label}</b><div class="key" style="font-family:var(--mono);font-size:11px;color:var(--muted)">${s.key}</div>
            ${s.ref ? `<div class="muted small-text">Ref: ${s.ref}</div>` : ""}
            ${s.note ? `<div class="muted small-text">${s.note}</div>` : ""}</div>
          <div>${tex(`${s.symbol} = ${s.formula}`)}<br>${tex(`${s.symbol} = ${s.subst}`)}</div>
          <div class="res">${typeof s.value === "number" ? fmtNum(s.value, 4) : s.value} <span class="muted">${s.unit === "-" ? "" : s.unit}</span></div>
        </div>`).join("")}</div>
    </details>`).join("");
}

export const cssId = (k) => k.replace(/[^a-zA-Z0-9_-]/g, "_");

export function gotoStep(key) {
  const el = document.getElementById(`d-${cssId(key)}`);
  if (!el) return;
  el.closest("details").open = true;
  el.scrollIntoView({ behavior: "smooth", block: "center" });
  el.classList.add("flash");
  setTimeout(() => el.classList.remove("flash"), 1600);
}
