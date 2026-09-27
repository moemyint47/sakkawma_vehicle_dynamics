// Parameter form specification: path in the model JSON -> label, unit/type, step, nullable.
// unit "bool" -> checkbox; "select:a|b" -> dropdown; "table" -> textarea "x, y" per line.
const axleFields = (ax) => [
  [`${ax}.track_mm`, "Track", "mm", 5],
  [`${ax}.h_rc_mm`, "Roll-centre height", "mm", 1],
  [`${ax}.roll_stiffness_source`, "Roll stiffness from", "select:components|direct"],
  [`${ax}.roll_stiffness_Nm_deg`, "Roll stiffness (linear)", "N·m/°", 5],
  [`${ax}.arb_rate_N_mm`, "ARB rate at wheel", "N/mm", 0.5],
  [`${ax}.arb_share`, "ARB share of roll stiffness", "–", 0.01],
  [`${ax}.unsprung_mass_kg`, "Unsprung mass per corner", "kg", 0.5],
  [`${ax}.h_unsprung_mm`, "Unsprung CG height", "mm", 1],
];
const springFields = (ax) => [
  [`${ax}.spring.rate_N_mm`, "Spring rate", "N/mm", 1],
  [`${ax}.spring.mr_mode`, "Motion-ratio curve", "select:poly|table"],
  [`${ax}.spring.mr_c0`, "MR c0 (at ride height)", "–", 0.01],
  [`${ax}.spring.mr_c1`, "MR c1 (slope, + progressive)", "1/mm", 0.0005],
  [`${ax}.spring.mr_c2`, "MR c2 (curvature)", "1/mm²", 0.00001],
  [`${ax}.spring.mr_table`, "MR table: travel mm, MR", "table", 0, true],
  [`${ax}.spring.bump_gap_mm`, "Bump-stop gap (wheel)", "mm", 1, true],
  [`${ax}.spring.bump_rate_N_mm`, "Bump-stop rate (wheel)", "N/mm", 10],
];
const damperFields = (ax) => [
  [`${ax}.damper.mr_mode`, "Damper motion ratio", "select:spring|constant"],
  [`${ax}.damper.mr_const`, "Damper MR (if constant)", "–", 0.01],
  [`${ax}.damper.c_ls_bump`, "Low-speed bump", "N·s/m", 50],
  [`${ax}.damper.c_ls_reb`, "Low-speed rebound", "N·s/m", 50],
  [`${ax}.damper.v_knee_bump_mm_s`, "Knee velocity bump", "mm/s", 5],
  [`${ax}.damper.v_knee_reb_mm_s`, "Knee velocity rebound", "mm/s", 5],
  [`${ax}.damper.c_hs_bump`, "High-speed bump", "N·s/m", 50],
  [`${ax}.damper.c_hs_reb`, "High-speed rebound", "N·s/m", 50],
  [`${ax}.damper.use_table`, "Use dyno table instead", "bool"],
  [`${ax}.damper.table`, "Dyno table: v mm/s (+bump), F N", "table", 0, true],
];
const hpFields = (ax) => [
  [`${ax}.hardpoints.lca_in.y`, "LCA inboard y", "mm", 1],
  [`${ax}.hardpoints.lca_in.z`, "LCA inboard z", "mm", 1],
  [`${ax}.hardpoints.lca_out.y`, "Lower ball joint y", "mm", 1],
  [`${ax}.hardpoints.lca_out.z`, "Lower ball joint z", "mm", 1],
  [`${ax}.hardpoints.uca_in.y`, "UCA inboard y", "mm", 1],
  [`${ax}.hardpoints.uca_in.z`, "UCA inboard z", "mm", 1],
  [`${ax}.hardpoints.uca_out.y`, "Upper ball joint y", "mm", 1],
  [`${ax}.hardpoints.uca_out.z`, "Upper ball joint z", "mm", 1],
  [`${ax}.hardpoints.tire_radius_mm`, "Loaded tire radius", "mm", 1],
  [`${ax}.hardpoints.tire_width_mm`, "Tire width (drawing)", "mm", 5],
  [`${ax}.hardpoints.static_camber_deg`, "Static camber (+ top out)", "°", 0.1],
];

export const GROUPS = [
  { title: "Vehicle", open: true, fields: [
    ["vehicle.mass_kg", "Total mass (incl. driver)", "kg", 1],
    ["vehicle.weight_front", "Static front weight fraction", "–", 0.005],
    ["vehicle.wheelbase_mm", "Wheelbase", "mm", 5],
    ["vehicle.h_cg_mm", "CG height (total)", "mm", 1],
    ["vehicle.roll_inertia_kgm2", "Sprung roll inertia (about CG)", "kg·m²", 0.5],
  ]},
  { title: "Manoeuvre", open: true, fields: [
    ["maneuver.ay_source", "a_y from", "select:input|corner"],
    ["maneuver.ay_g", "Lateral acceleration a_y (input)", "g", 0.05],
    ["maneuver.radius_m", "Path radius R", "m", 0.25],
    ["maneuver.speed_kmh", "Speed v (corner: a_y = v²/R)", "km/h", 1],
    ["maneuver.roll_gravity_term", "Include m·g·h₁·φ roll term", "bool"],
  ]},
  { title: "Transient input a_y(t)", open: false, note: "Open-loop input; amplitude = a_y above.", fields: [
    ["maneuver.transient.profile", "Profile", "select:ramp|step|sine|csv|road"],
    ["maneuver.transient.t0_s", "Start time t₀", "s", 0.05],
    ["maneuver.transient.rise_s", "Ramp rise time", "s", 0.05],
    ["maneuver.transient.freq_hz", "Sine frequency", "Hz", 0.1],
    ["maneuver.transient.t_end_s", "End time", "s", 0.1],
    ["maneuver.transient.t_probe_s", "Derivation time instant", "s", 0.01],
    ["maneuver.transient.csv", "CSV: t s, a_y g", "table", 0, true],
    ["maneuver.transient.road", "Road: length m, radius m (0 straight, − other way), km/h", "table3", 0, true],
    ["maneuver.transient.road_transition_m", "Clothoid transition length", "m", 0.5],
  ]},
  { title: "Front axle", open: true, fields: axleFields("front") },
  { title: "Rear axle", open: true, fields: axleFields("rear") },
  { title: "Front spring & bump stop", open: false, note: "MR = spring travel / wheel travel; MR(z) = c0 + c1·z + c2·z² (z = wheel travel, mm, + bump).", fields: springFields("front") },
  { title: "Rear spring & bump stop", open: false, fields: springFields("rear") },
  { title: "Front damper", open: false, note: "Force–velocity at the damper; + = bump.", fields: damperFields("front") },
  { title: "Rear damper", open: false, fields: damperFields("rear") },
  { title: "Tire (TMeasy lateral)", open: false, note:
    "Nominal load set is required. Clear any double-load field to fall back to linear load scaling.", fields: [
    ["tire.Fz_nom_N", "Nominal load F_z^N", "N", 10],
    ["tire.C_alpha_nom_N_deg", "Cornering stiffness @F_z^N", "N/°", 10],
    ["tire.alpha_M_nom_deg", "Slip angle at peak @F_z^N", "°", 0.25],
    ["tire.F_M_nom_N", "Peak force @F_z^N", "N", 10],
    ["tire.alpha_S_nom_deg", "Slip angle full sliding @F_z^N", "°", 0.5],
    ["tire.F_S_nom_N", "Sliding force @F_z^N", "N", 10],
    ["tire.C_alpha_2nom_N_deg", "Cornering stiffness @2F_z^N", "N/°", 10, true],
    ["tire.alpha_M_2nom_deg", "Slip angle at peak @2F_z^N", "°", 0.25, true],
    ["tire.F_M_2nom_N", "Peak force @2F_z^N", "N", 10, true],
    ["tire.alpha_S_2nom_deg", "Slip angle full sliding @2F_z^N", "°", 0.5, true],
    ["tire.F_S_2nom_N", "Sliding force @2F_z^N", "N", 10, true],
  ]},
  { title: "Front hardpoints (front view)", open: false,
    note: "y = lateral from centreline (outward +), z = height above ground. One side; mirrored.", fields: hpFields("front") },
  { title: "Rear hardpoints (front view)", open: false, fields: hpFields("rear") },
];

// Fields that adaptive mode treats as design targets (editing them moves other parameters)
export const ADAPTIVE_TARGETS = new Set(["front.h_rc_mm", "rear.h_rc_mm", "front.roll_stiffness_Nm_deg",
  "rear.roll_stiffness_Nm_deg", "front.arb_share", "rear.arb_share", "front.hardpoints.tire_radius_mm",
  "rear.hardpoints.tire_radius_mm", "front.hardpoints.static_camber_deg", "rear.hardpoints.static_camber_deg"]);

export const FIELD = Object.fromEntries(GROUPS.flatMap((g) => g.fields.map((f) => [f[0], { group: g.title, label: f[1], unit: f[2], step: f[3], nullable: !!f[4] }])));

export const LABELS = Object.fromEntries(
  GROUPS.flatMap((g) => g.fields.map((f) => [f[0], `${g.title.split(" (")[0]} · ${f[1]}${f[2] && !String(f[2]).startsWith("select") && f[2] !== "bool" && !String(f[2]).startsWith("table") ? ` [${f[2]}]` : ""}`]))
);

export const getPath = (o, p) => p.split(".").reduce((a, k) => (a == null ? a : a[k]), o);
export const setPath = (o, p, v) => {
  const ks = p.split("."); let a = o;
  for (let i = 0; i < ks.length - 1; i++) a = a[ks[i]];
  a[ks[ks.length - 1]] = v;
};

export const tableToText = (t) => (Array.isArray(t) ? t.map((r) => r.join(", ")).join("\n") : "");
export const textToTable = (s, ncol = 2) => {
  const rows = s.split(/\n+/).map((l) => l.split(/[,;\s\t]+/).filter(Boolean).map(Number)).filter((r) => r.length >= ncol);
  if (!rows.length) return null;
  if (rows.some((r) => r.slice(0, ncol).some((x) => !Number.isFinite(x)))) throw new Error("table: numbers only");
  return rows.map((r) => r.slice(0, ncol));
};

export const ROAD_PRESETS = {
  "Slow hairpin (R 4.5 m, 26 km/h)": [[8, 0, 26], [14.1, 4.5, 26], [8, 0, 26]],
  "Skidpad (R 9.125 m, 40 km/h)": [[8, 0, 40], [30, 9.125, 40]],
  "Fast sweeper (R 20 m, 55 km/h)": [[10, 0, 55], [25, 20, 55], [10, 0, 55]],
  "Slalom (±R 8 m, 40 km/h)": [[6, 0, 40], [8, 8, 40], [8, -8, 40], [8, 8, 40], [8, -8, 40], [6, 0, 40]],
};
