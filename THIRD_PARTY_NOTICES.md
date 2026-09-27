# Third-party software

The browser UI bundles these libraries under `frontend/vendor/`. Each keeps its own license, and the full texts are in that folder:

| Library | Version | License | Copyright |
|---|---|---|---|
| [Plotly.js](https://github.com/plotly/plotly.js) (`plotly.js-dist-min`) | 2.35.3 | MIT | Plotly, Inc. |
| [KaTeX](https://github.com/KaTeX/KaTeX) | 0.16.47 | MIT | Khan Academy and other contributors |
| [gridstack.js](https://github.com/gridstack/gridstack.js) | 10.3.1 | MIT | Alain Dumesny, Pavel Reznikov, Dylan Weiss |

The Python dependencies in `backend/requirements.txt` are installed separately and aren't redistributed here:

| Package | License |
|---|---|
| FastAPI, Pydantic, Uvicorn | MIT / BSD |
| NumPy, SciPy | BSD-3-Clause |
| pytest, httpx | MIT / BSD |

The **models** follow published literature, which is cited in the code and in the calculation report (e.g. G. Rill, *Road Vehicle Dynamics*; W. & D. Milliken, *Race Car Vehicle Dynamics*; C. Rouelle / OptimumG articles; Chepkasov et al. 2016). This repository contains no text or figures from those works.
