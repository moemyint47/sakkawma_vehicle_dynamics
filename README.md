# sakkawma_vehicle_dynamics

Personal tools for learning and developing an FSAE suspension. The backend is Python (FastAPI) and the UI runs in the browser.

**v1 scope:** steady-state **lateral load transfer** in low-speed corners. It covers the geometric, elastic and unsprung load-transfer paths, a TMeasy tire model, slip angles and balance, the limit lateral acceleration, and an animated 2D front view of the double-wishbone kinematics.

> All default numbers are **placeholders**. Replace them with your own car and tire data.

## Run

```bash
pip install -r backend/requirements.txt
uvicorn backend.app:app --reload        # from the repository root
# open http://127.0.0.1:8000
```

Tests: `python -m pytest -q tests`

The UI works offline. Plotly and KaTeX are included under `frontend/vendor/`, and the exported report is a single self-contained HTML file.

## What it computes

Every derived number is produced through a derivation log (`backend/vd/derivation.py`). It stores the **formula → substituted values → result** together with the source reference. In the UI, click **ƒ** next to any value to see its derivation. **Export report** writes all of them to a printable HTML file.

| Stage | Model | Source |
|---|---|---|
| Mass split | sprung / unsprung, sprung CG position and height | — |
| Body roll | φ = m_s a_y h₁ / (K_φf + K_φr − m_s g h₁) (the gravity term can be switched off) | Milliken RCVD ch. 18 |
| Load transfer per axle | ΔF_z = 2m_u a_y h_u/t (unsprung) + m_s,axle a_y h_rc/t (geometric) + K_φ φ/t (elastic) | Milliken RCVD ch. 18; Rill §9.2 |
| Checks | ΣΔF_z·t = m a_y h (+ m_s g h₁ φ); sum of wheel loads = m g | — |
| Tire | TMeasy lateral characteristic, with load-dependent dF₀, F_M, F_S (quadratic) and s_M, s_S (linear) | Rill Listing 3.3, eq. 3.121/3.122 |
| Slip angles | per axle: F_y(F_z,o, α) + F_y(F_z,i, α) = m_axle a_y (Brent's method) | steady state |
| Balance | Δα = α_f − α_r, understeer gradient, steer angle δ = l/R + Δα | Rill §9.3 |
| **Limit a_y** | bisection: the largest a_y at which neither axle saturates and neither inner wheel lifts. Also reports which one limits it | — |
| Kinematics (2D) | planar four-bar per side: instant centres, roll centre (height and **lateral migration**), camber to ground and to body, track change under heave, roll and one-wheel bump | Chepkasov et al., Procedia Eng. 150 (2016) |
| Kinematic checks | RC from the graphical construction = RC from virtual velocities, (t/2)·dy/dz | Rill §9.2.3 |

The tire model needs only the nominal-load data. If you add the double-load data set (2F_zᴺ), it produces the degressive load sensitivity that makes load transfer cost grip.

## UI

- **Results**: limit a_y and the mechanism that limits it, limit speed on radius R (default: FSAE skidpad centreline, 9.125 m), roll, LLTD, balance, per-axle load-transfer split, wheel loads, slip angles and kinematic summary.
- **Front view**: SVG drawing with SolidWorks-style dimensions. It animates roll, heave and one-wheel bump, and shows IC/RC construction lines, a static ghost, wheel-load arrows and camber labels.
- **Sweeps & compare**: sweep any numeric input and overlay several values of a second input, e.g. front RC height 0/35/70/120 mm against a_y. Optionally computes the limit a_y at each point.
- **Kinematic curves**: camber, RC height, RC lateral position, track change and travel against heave, roll or bump, compared across hardpoint variants.
- **Tire**: F_y(α) and F_y(F_z) curves, with the four wheels' operating points marked.
- Parameter sets can be saved and loaded as JSON. **Copy kinematic RC → load-transfer inputs** links the two models manually.

## Layout

```
backend/
  app.py                 FastAPI routes + static UI
  vd/derivation.py       formula/substitution/result log
  vd/schemas.py          inputs (units, placeholders)
  vd/tire.py             TMeasy lateral
  vd/load_transfer.py    geometric / elastic / unsprung split, roll
  vd/cornering.py        slip angles, balance, limit a_y
  vd/kinematics2d.py     front-view double wishbone solver
  vd/kin_summary.py      derived kinematic numbers + derivations
  vd/sweep.py            parameter sweeps / comparisons
  vd/report.py           self-contained HTML report
frontend/
  index.html, css/, js/  (spec, derivations, results, view2d, plots, main)
tests/                   physics identities, limiting cases, API
```

**3D-ready:** the kinematics solver returns a renderer-agnostic pose (named points, tire outline, IC/RC), and `view2d.js` is only a renderer. A 3D solver can return the same structure with an added `x` coordinate (reserved in `schemas.Point2`), and a three.js view can sit alongside the 2D one.

## Model assumptions (v1)

Steady state; lateral dynamics only (no aero, drive or brake forces); rigid chassis. Both wheels on an axle run at the same slip angle (no toe or Ackermann). Camber is not yet fed into the tire force. The load-transfer RC heights are inputs, not coupled to the kinematics yet. The tire is radially rigid in the kinematics. Roll happens about the body point on the centreline at ground level.

## Roadmap

Couple the kinematic RC and camber into load transfer and the tire model (camber thrust). Add RC migration against a_y, the bell-crank / progressive-rate motion ratio feeding roll stiffness, transient roll (sprung-mass inertia, damper lag), then 3D kinematics.
