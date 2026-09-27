// Parameter form specification: path in the model JSON -> label, unit, step.
// Groups render as collapsible sections in the sidebar.
export const GROUPS = [
  { title: "Vehicle", open: true, fields: [
    ["vehicle.mass_kg", "Total mass (incl. driver)", "kg", 1],
    ["vehicle.weight_front", "Static front weight fraction", "–", 0.005],
    ["vehicle.wheelbase_mm", "Wheelbase", "mm", 5],
    ["vehicle.h_cg_mm", "CG height (total)", "mm", 1],
  ]},
  { title: "Manoeuvre", open: true, fields: [
    ["maneuver.ay_g", "Lateral acceleration a_y", "g", 0.05],
    ["maneuver.radius_m", "Path radius R", "m", 0.25],
    ["maneuver.roll_gravity_term", "Include m·g·h₁·φ roll term", "bool"],
  ]},
  { title: "Front axle", open: true, fields: [
    ["front.track_mm", "Track", "mm", 5],
    ["front.h_rc_mm", "Roll-centre height", "mm", 1],
    ["front.roll_stiffness_Nm_deg", "Roll stiffness (springs + ARB)", "N·m/°", 5],
    ["front.unsprung_mass_kg", "Unsprung mass per corner", "kg", 0.5],
    ["front.h_unsprung_mm", "Unsprung CG height", "mm", 1],
  ]},
  { title: "Rear axle", open: true, fields: [
    ["rear.track_mm", "Track", "mm", 5],
    ["rear.h_rc_mm", "Roll-centre height", "mm", 1],
    ["rear.roll_stiffness_Nm_deg", "Roll stiffness (springs + ARB)", "N·m/°", 5],
    ["rear.unsprung_mass_kg", "Unsprung mass per corner", "kg", 0.5],
    ["rear.h_unsprung_mm", "Unsprung CG height", "mm", 1],
  ]},
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
  ...["front", "rear"].map((ax) => ({
    title: `${ax === "front" ? "Front" : "Rear"} hardpoints (front view)`, open: false,
    note: "y = lateral from centreline (outward +), z = height above ground. One side; mirrored.",
    fields: [
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
    ],
  })),
];

export const LABELS = Object.fromEntries(
  GROUPS.flatMap((g) => g.fields.map((f) => [f[0], `${g.title.split(" (")[0]} · ${f[1]} [${f[2]}]`]))
);

export const getPath = (o, p) => p.split(".").reduce((a, k) => (a == null ? a : a[k]), o);
export const setPath = (o, p, v) => {
  const ks = p.split("."); let a = o;
  for (let i = 0; i < ks.length - 1; i++) a = a[ks[i]];
  a[ks[ks.length - 1]] = v;
};
