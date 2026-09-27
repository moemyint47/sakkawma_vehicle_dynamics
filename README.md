# sakkawma_vehicle_dynamics

Personal tools for learning and developing an FSAE suspension. The backend is Python (FastAPI) and the UI runs in the browser.

**Scope:** **lateral load transfer** in low-speed corners.

- **v1:** steady state. It covers the geometric, elastic and unsprung load-transfer paths, a TMeasy tire model, slip angles and balance, the limit lateral acceleration, and an animated 2D front view of the double-wishbone kinematics.
- **v2:** adds springs with progressive motion-ratio curves, bump stops, ARB and dampers, a **transient roll** model driven by a_y(t) or by a **road** of radii and speeds, **adaptive/absolute** parameter editing, and a **workspace** of snapping widgets.

> All default numbers are **placeholders**. Replace them with your own car and tire data.

## Run

```bash
pip install -r backend/requirements.txt
uvicorn backend.app:app --reload        # from the repository root
# open http://127.0.0.1:8000
```

Tests: `python -m pytest -q tests`

The UI works offline. Plotly, KaTeX and gridstack are included under `frontend/vendor/`, and the exported report is a single self-contained HTML file.

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

| Springs | spring rate k_s through a motion-ratio curve MR(z), either a polynomial c0 + c1 z + c2 z² or a PCHIP table. Spring travel x_s = ∫MR dz, wheel force F_w = F_s·MR, **wheel rate k_w = k_s MR² + F_s dMR/dz**. Preload is solved for equilibrium at ride height. Bump stops, ARB rate at the wheel | virtual work |
| Steady roll with components | nonlinear equilibrium m_s a_y h₁ (+ m_s g h₁ φ) = M_f(φ) + M_r(φ), with elastic LT split into springs, bump stops and ARB | — |
| Dampers | bump/rebound, low/high speed with knee velocities, or a dyno table (linear extrapolation). Damper MR = spring MR, or a constant | — |
| **Transient roll** | I_ra φ̈ = m_s a_y(t) h₁ (+ m_s g h₁ φ) − Σ t·(ΔF_spring + ΔF_bump + ΔF_ARB + ΔF_damper), with I_ra = I_xx,cg + m_s h₁². Solved with RK45 | OptimumG “Entry requirements” |
| a_y(t) input | step, ramp, sine, CSV, or **road**: segments (length, radius, speed) joined by clothoids, a_y = v²κ(s(t)) | — |
| Corner mode | a_y = v²/R for the steady-state calculation | — |
| Adaptive edits | RC height → inboard pivots move (ball joints and swing-arm length kept). Roll-stiffness target → spring rate (ARB share kept). Tire radius / camber → upright moves rigidly. The load-transfer RC is locked to the kinematic RC. Each adaptation is logged with its maths | — |

Consistency checks: the transient result settles exactly to the steady-state solution, the wheel rate is the derivative of the wheel force, and adaptive RC edits hit the target while keeping the swing-arm length. See `tests/`.

The tire model needs only the nominal-load data. If you add the double-load data set (2F_zᴺ), it produces the degressive load sensitivity that makes load transfer cost grip.

## UI

- **Results**: limit a_y and the mechanism that limits it, limit speed on radius R (default: FSAE skidpad centreline, 9.125 m), roll, LLTD, balance, per-axle load-transfer split, wheel loads, slip angles and kinematic summary.
- **Front view**: SVG drawing with SolidWorks-style dimensions. It animates roll, heave and one-wheel bump, and shows IC/RC construction lines, a static ghost, wheel-load arrows and camber labels.
- **Sweeps & compare**: sweep any numeric input and overlay several values of a second input, e.g. front RC height 0/35/70/120 mm against a_y. Optionally computes the limit a_y at each point.
- **Kinematic curves**: camber, RC height, RC lateral position, track change and travel against heave, roll or bump, compared across hardpoint variants.
- **Tire**: F_y(α) and F_y(F_z) curves, with the four wheels' operating points marked.
- **Transient**: OptimumG-style plots of the load-transfer components (N and % of suspended LT) against time, input a_y, roll, damper velocity, wheel loads and the road plan view. Includes a full derivation breakdown at any time instant, plus run metrics (e.g. damper share at t₀ + 50 ms).
- **Springs & dampers**: MR, wheel rate, wheel force, roll stiffness and roll moment against roll, and damper force against velocity. One compare bar overlays several values of any parameter (e.g. MR slope 0 / 0.002 / 0.005).
- **Workspace**: a 12-column snapping grid. You can add any widget (graphs, front view, value tiles with ƒ, sweeps, kinematic and tire curves, **pinned-parameter sliders**, **targets**), and use preset layouts, size presets, lock, tidy and focus mode (hides the sidebar). Workspaces are saved as JSON in `workspaces/`, so they're tracked in git. There are three examples: corner-entry dampers, RC-height study, and slow vs fast corner.
- **Adaptive / Absolute** switch at the top of the sidebar. ↻ marks fields that act as design targets in adaptive mode.
- **Baseline**: freeze the current setup, then graphs and tiles show baseline against current.
- Road presets: slow hairpin, skidpad, fast sweeper, slalom.
- Parameter sets can be saved and loaded as JSON.

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
  vd/suspension.py       spring + MR curve, bump stop, ARB, damper
  vd/transient.py        roll EOM, a_y(t) profiles, road model, metrics, probe derivations
  vd/adapt.py            adaptive parameter editing
  vd/sweep.py            parameter sweeps / comparisons
  vd/report.py           self-contained HTML report
frontend/
  index.html, css/, js/  (store, form, spec, derivations, results, view2d, plots, widgets, workspace, main)
workspaces/              saved workspace layouts (JSON)
tests/                   physics identities, limiting cases, API
```

**3D-ready:** the kinematics solver returns a renderer-agnostic pose (named points, tire outline, IC/RC), and `view2d.js` is only a renderer. A 3D solver can return the same structure with an added `x` coordinate (reserved in `schemas.Point2`), and a three.js view can sit alongside the 2D one.

## Model assumptions

- **Scope:** lateral dynamics only (no aero, drive or brake forces), rigid chassis.
- **Slip angles:** both wheels on an axle run at the same slip angle (no toe or Ackermann).
- **Tire:** camber is not yet fed into the tire force, and the tire is radially rigid in the kinematics.
- **Transient model:** body roll only (no heave, pitch or jacking yet). The net vertical force from asymmetric elements is reacted. The a_y input is open loop, and the geometric and unsprung load transfer act instantly.
- **Roll centre:** in absolute mode, the load-transfer RC heights are independent inputs. In adaptive mode, they follow the kinematics.

## Roadmap

1. Heave and pitch DOF with jacking forces (OptimumG "Rolling about": θ_IC angles and inner/outer side-force split), and ride height against a_y.
2. Kinematic RC migration and camber fed into the load transfer and the tire (camber thrust).
3. Bell-crank geometry → MR(z), replacing the curve input.
4. Closed-loop steering input with tire relaxation.
5. 3D kinematics.
