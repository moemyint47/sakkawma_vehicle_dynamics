// Results tab: KPI tiles, per-axle tables, LT split chart, kinematics table.
import { S, fx, fmtNum, renderAll } from "./derivations.js";
import { plotTheme } from "./plots.js";

const v = (k) => (S.byKey[k] ? S.byKey[k].value : null);

function kpi(label, key, digits, unit, sub = "", hl = false) {
  return `<div class="kpi ${hl ? "hl" : ""}"><div class="k">${label}${fx(key)}</div>
    <div class="v">${fmtNum(v(key), digits)}<small>${unit}</small></div>${sub ? `<div class="s">${sub}</div>` : ""}</div>`;
}

function row(label, key, digits, unit, cls = "") {
  return `<tr class="${cls}" data-row="${key}"><td>${label}${fx(key)}</td><td class="n">${fmtNum(v(key), digits)}</td><td class="u">${unit}</td></tr>`;
}

export function renderResults(res, defaults) {
  const w = res.warnings || [];
  document.getElementById("warnings").innerHTML = w.length
    ? `<div class="warn"><b>Warnings</b><ul>${w.map((x) => `<li>${x}</li>`).join("")}</ul></div>` : "";

  const lim = res.limit || {};
  const us = v("K_us");
  const usTxt = us == null ? "" : us > 0.05 ? "understeer" : us < -0.05 ? "oversteer" : "≈ neutral";
  document.getElementById("kpis").innerHTML = [
    kpi("Limit lateral acceleration", "ay_max", 3, "g", lim.reason || "", true),
    kpi("Limit speed on R", "v_max_kmh", 1, "km/h", `on R = ${res.radius} m`),
    kpi("Lap time, one circle of R", "lap_time", 3, "s"),
    kpi("Body roll at a_y", "phi_deg", 2, "°"),
    kpi("Roll gradient", "roll_gradient", 2, "°/g"),
    kpi("LLTD front", "lltd_front", 3, "", "front share of ΔF_z"),
    kpi("α_f − α_r at a_y", "dalpha", 3, "°", "> 0 understeer"),
    kpi("Understeer gradient", "K_us", 3, "°/g", usTxt),
  ].join("");

  for (const t of ["f", "r"]) {
    document.getElementById(`axle-${t}`).innerHTML = `<table class="vals"><tbody>
      ${row("Static wheel load", `${t}.Fz_static`, 1, "N")}
      ${row("Total load transfer ΔF_z (per wheel)", `${t}.dFz`, 1, "N")}
      ${row("unsprung (direct)", `${t}.dFz_u`, 1, "N", "sub")}
      ${row("geometric (via RC / links)", `${t}.dFz_g`, 1, "N", "sub")}
      ${row("elastic (via springs / ARB)", `${t}.dFz_e`, 1, "N", "sub")}
      ${row("Geometric share", `${t}.geo_share`, 3, "–")}
      ${row("Elastic share", `${t}.el_share`, 3, "–")}
      ${row("Outer wheel load", `${t}.Fz_out`, 1, "N")}
      ${row("Inner wheel load", `${t}.Fz_in`, 1, "N")}
      ${row("Required axle lateral force", `${t}.Fy_req`, 1, "N")}
      ${row("Axle lateral capacity", `${t}.Fy_max`, 1, "N")}
      ${row("Grip utilisation", `${t}.util`, 3, "–")}
      ${row("Slip angle", `${t}.alpha`, 3, "°")}
      ${row("Outer tire F_y", `${t}.tire_o.Fy`, 1, "N", "sub")}
      ${row("Inner tire F_y", `${t}.tire_i.Fy`, 1, "N", "sub")}
      ${row("Force lost to load transfer", `${t}.lt_loss`, 1, "N")}
    </tbody></table>`;
  }

  document.getElementById("kin-table").innerHTML = `<table class="vals"><tbody>
    <tr><td></td><td class="n"><b>Front</b></td><td class="n"><b>Rear</b></td><td class="u"></td></tr>
    ${[["Static RC height (kinematic)", "h_rc", 1, "mm"], ["RC height check (virtual velocity)", "h_rc_check", 1, "mm"],
       ["RC height input (load transfer)", "__in", 1, "mm"],
       ["Instant centre y", "ic_y", 0, "mm"], ["Instant centre z", "ic_z", 1, "mm"],
       ["Front-view swing-arm length", "fvsa", 0, "mm"], ["Camber gain in heave", "camber_gain_heave", 4, "°/mm"],
       ["Outer camber per ° roll", "roll_camber_o", 3, "°/°"], ["RC lateral migration", "rc_lat_mig", 1, "mm/°"],
       ["RC height at 1° roll", "rc_roll_h", 1, "mm"], ["RC height change in heave", "rc_heave_gain", 3, "mm/mm"]]
      .map(([lab, k, d, u]) => {
        const cell = (ax) => {
          if (k === "__in") return fmtNum(v(ax === "front" ? "h_rf" : "h_rr") * 1000, d);
          return `${fmtNum(v(`k${ax}.${k}`), d)}${fx(`k${ax}.${k}`)}`;
        };
        return `<tr><td>${lab}</td><td class="n">${cell("front")}</td><td class="n">${cell("rear")}</td><td class="u">${u}</td></tr>`;
      }).join("")}
  </tbody></table>`;

  // stacked bar of LT split
  const th = plotTheme();
  const cats = ["Front", "Rear"];
  const tr = (name, key, color) => ({ type: "bar", name, x: cats, y: ["f", "r"].map((t) => v(`${t}.${key}`)),
    marker: { color }, hovertemplate: "%{y:.1f} N<extra>" + name + "</extra>" });
  Plotly.react("lt-bar", [tr("Unsprung", "dFz_u", th.c[2]), tr("Geometric", "dFz_g", th.c[0]), tr("Elastic", "dFz_e", th.c[1])],
    { ...th.layout, barmode: "relative", yaxis: { ...th.layout.yaxis, title: "ΔF_z per wheel [N]" },
      legend: { orientation: "h", y: -0.18 }, margin: { l: 60, r: 10, t: 10, b: 50 } }, th.config);

  renderAll(document.getElementById("all-deriv"));
  document.getElementById("assumptions").innerHTML = (defaults.assumptions || []).map((a) => `<li>${a}</li>`).join("");
}
