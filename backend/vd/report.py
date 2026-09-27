"""Self-contained HTML calculation report (KaTeX inlined, works offline)."""
from __future__ import annotations

import base64
import datetime as dt
import html
import json
import re
from pathlib import Path

from .derivation import Calc

VENDOR = Path(__file__).resolve().parents[2] / "frontend" / "vendor" / "katex"

ASSUMPTIONS = [
    "Steady-state cornering, lateral dynamics only (no drive/brake forces, no aerodynamic loads).",
    "Rigid chassis: front and rear share one body roll angle; small roll angles.",
    "Unsprung masses sit on the axle lines; sprung-mass CG position derived from total and unsprung data.",
    "Load transfer split per Milliken RCVD ch. 18: unsprung (direct) + geometric (roll centre) + elastic (roll stiffness).",
    "Roll-centre heights and roll stiffnesses in the load-transfer calculation are direct inputs "
    "(kinematic RC is computed and shown separately; not yet coupled).",
    "Tire: TMeasy lateral characteristic (Rill), pure lateral slip, no camber, no relaxation; s_y = tan(alpha).",
    "Both wheels of an axle operate at the same slip angle (no toe, Ackermann or compliance steer).",
    "Front-view kinematics: planar four-bar per side, rigid links, radially rigid tire (contact point on "
    "the wheel plane at loaded radius); roll about the body point on the centreline at ground level.",
    "Springs act through a motion-ratio curve MR(z); wheel force F_w = F_s MR, wheel rate k_s MR^2 + F_s dMR/dz "
    "(virtual work); preload solved for static equilibrium at z = 0; bump stops and ARB linear at the wheel.",
    "Transient: roll-only body (no heave/pitch), open-loop a_y(t) input, I_ra = I_xx,cg + m_s h1^2; "
    "wheel travel +-(t/2) phi; geometric and unsprung load transfer instantaneous; net vertical force from "
    "asymmetric elements (progressive springs, bump/rebound damping) is reacted (no jacking yet).",
    "Road profile: a_y = v^2 kappa(s) with linear curvature (clothoid) transitions between segments.",
    "Geometric LT model 'ic_angles': link forces along contact patch -> instant centre lines at the rolled pose, "
    "tyre force split from TMeasy, fixed-point iteration with roll; jacking reported, ride height as a linear "
    "estimate J/(2 k_w) (no heave DOF). In the transient, the outer-force share r(a_y) is taken from the steady "
    "coupled solution.",
    "Bell-crank: planar pushrod/rocker/spring linkage in the front view; MR(z) = dx_s/dz from a C2 spline.",
    "Corner replay: path replay (the car follows the road exactly); lateral + yaw equilibrium give the axle forces; "
    "longitudinal load transfer without pitch; drive/brake forces not in the lateral/yaw balance; small angles.",
    "Adaptive mode: edits keep the design intent (RC height moves inboard pivots at constant ball joints and "
    "swing-arm length; roll-stiffness target solves the spring rate at constant ARB share; tire radius / camber "
    "move the upright rigidly); the load-transfer RC is locked to the kinematic RC.",
]


def _katex_inline() -> str:
    css = (VENDOR / "katex.min.css").read_text(encoding="utf-8")

    def repl(mo):
        name = mo.group(1)
        f = VENDOR / "fonts" / name
        if f.exists():
            b64 = base64.b64encode(f.read_bytes()).decode()
            return f"url(data:font/woff2;base64,{b64})"
        return "url()"
    css = re.sub(r"url\(fonts/([^)]+\.woff2)\)", repl, css)
    css = re.sub(r",url\(fonts/[^)]+\.(woff|ttf)\) format\(\"(woff|truetype)\"\)", "", css)
    js = (VENDOR / "katex.min.js").read_text(encoding="utf-8")
    ar = (VENDOR / "auto-render.min.js").read_text(encoding="utf-8")
    return f"<style>{css}</style><script>{js}</script><script>{ar}</script>"


def _fmt_val(v):
    if isinstance(v, float):
        if v != v:
            return "NaN"
        return f"{v:.6g}"
    return html.escape(str(v))


def build(model_dump: dict, calc: Calc, title: str = "Lateral load transfer – calculation report",
          notes: str = "") -> str:
    sections: dict[str, list] = {}
    for s in calc.steps:
        sections.setdefault(s.section, []).append(s)
    rows = []
    for sec in sections:
        rows.append(f"<h2>{html.escape(sec)}</h2><table class='d'><thead><tr><th>Quantity</th>"
                    f"<th>Formula</th><th>Substitution</th><th>Result</th></tr></thead><tbody>")
        for s in sections[sec]:
            extra = ""
            if s.ref:
                extra += f"<div class='ref'>Ref: {html.escape(s.ref)}</div>"
            if s.note:
                extra += f"<div class='ref'>{html.escape(s.note)}</div>"
            rows.append(
                f"<tr><td><b>{html.escape(s.label)}</b><div class='key'>{html.escape(s.key)}</div></td>"
                f"<td>\\(\\displaystyle {s.symbol} = {s.formula}\\){extra}</td>"
                f"<td>\\(\\displaystyle {s.symbol} = {s.subst}\\)</td>"
                f"<td class='num'>{_fmt_val(s.value)} {html.escape(s.unit)}</td></tr>")
        rows.append("</tbody></table>")
    warn = "".join(f"<li>{html.escape(w)}</li>" for w in calc.warnings) or "<li>None</li>"
    assum = "".join(f"<li>{html.escape(a)}</li>" for a in ASSUMPTIONS)
    inputs = html.escape(json.dumps(model_dump, indent=2))
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(title)}</title>{_katex_inline()}
<style>
body{{font:14px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;color:#1b1f24;background:#fff;margin:24px auto;max-width:1200px;padding:0 16px}}
h1{{font-size:22px;margin:0 0 4px}} h2{{font-size:16px;margin:28px 0 8px;border-bottom:2px solid #1b1f24;padding-bottom:4px}}
.meta{{color:#555;margin-bottom:16px}} table.d{{border-collapse:collapse;width:100%}}
table.d th,table.d td{{border:1px solid #d0d4d9;padding:6px 8px;vertical-align:top;text-align:left}}
table.d th{{background:#f1f3f5;font-weight:600}} td.num{{white-space:nowrap;font-variant-numeric:tabular-nums;font-weight:600}}
.key{{color:#888;font-size:11px;font-family:ui-monospace,monospace}} .ref{{color:#666;font-size:12px;margin-top:4px}}
pre{{background:#f6f8fa;padding:12px;overflow:auto;font-size:12px;border:1px solid #d0d4d9}}
.katex{{font-size:1.02em}} @media print{{body{{margin:0}} h2{{break-after:avoid}} tr{{break-inside:avoid}}}}
</style></head><body>
<h1>{html.escape(title)}</h1>
<div class="meta">Sakkawma Vehicle Dynamics · generated {now}</div>
{f'<p>{html.escape(notes)}</p>' if notes else ''}
<h2>Warnings</h2><ul>{warn}</ul>
<h2>Model assumptions</h2><ol>{assum}</ol>
{''.join(rows)}
<h2>Input parameter set (JSON)</h2><pre>{inputs}</pre>
<script>document.addEventListener("DOMContentLoaded",function(){{renderMathInElement(document.body,{{delimiters:[{{left:"\\\\(",right:"\\\\)",display:false}}],throwOnError:false}});}});</script>
</body></html>"""
