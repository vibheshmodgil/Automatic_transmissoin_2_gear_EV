# Original notebooks — historical, superseded

These are the analysis this project started from. They are kept for provenance, not
for use: **`shift_core.py` supersedes them entirely** and fixes a number of defects
they contain. Every conclusion in them has been re-derived and, in several cases,
reversed.

| File | What it is |
|---|---|
| `City_drive_cycle_data_analysis_no_regen.ipynb` | The original analysis, 45 cells. **Does not run top to bottom as saved.** |
| `City_drive_cycle_data_analysis_no_regen.RUN.ipynb` | An executable copy with four documented shims, 48 cells |

Absolute paths pointing at the original author's machine have been redacted.

## Why they do not run

- `SIGNED_RPM_mesh` / `SIGNED_TORQUE_mesh` / `signed_eff_map` are used but never
  defined — the cell that mirrored the map about zero torque was deleted
- a function is called one cell before it is defined
- `np.trapezoid` needs NumPy ≥ 2.0; the pinned environment has 1.26
- hard-coded absolute paths into another machine's Downloads folder

The `.RUN` copy carries shims for all four, each labelled `RUN SHIM` in the cell
source. **No physics or optimisation was changed by the shims.**

## What was wrong with the conclusions

Summarised here; the full account is in [`../docs/ENGINEERING_NOTES.md`](../docs/ENGINEERING_NOTES.md).

- **Every reported optimum sat on the edge of its own search range.** A search that
  returns a corner has not found an optimum.
- **The headline answer had the upshift below the downshift** — an inverted
  hysteresis band, which is not a schedule at all. The validation assert that was
  supposed to catch this covered only the 2-D grid, not the 1-D sweeps that produced
  the headline.
- **The 11 kW constant-power envelope was drawn in two figures and never enforced.**
  Feasibility tested a flat ±45 Nm, so operating points demanding 14.5 kW from an
  11 kW motor were reported as valid.
- **Shifting was free** — no energy, no torque interruption, no rate limit — which is
  why the optimiser bought efficiency with 2,768 gear changes.
- **Reflected rotor inertia was omitted.** It scales with the square of the ratio, so
  it biases precisely the comparison being made.
- The claimed saving was smaller than the model's own error bar.

The re-derived answer, its verification (37 independent checks) and the reasoning are
in the repository root.
