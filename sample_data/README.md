# Sample data — SYNTHETIC, not measurements

**Neither file in this folder is real vehicle data.** They exist so the tool runs
out of the box and so you can see exactly what format your own files need.

| File | What it is |
|---|---|
| `N603_AMT_onroadrange_SpeedBased_.csv` | Synthetic drive cycle — 28,368 samples, 7.88 h, 110.9 km, 1 Hz |
| `N603_42UH_230_Fineint_EfficiencyMap.xlsx` | Synthetic motor efficiency map — 42 torque rows × 901 speed columns |

## Where they came from

The real N603 drive-cycle log and dyno efficiency map were not available when this
tool was written. These stand-ins were reconstructed by fitting to every diagnostic
the original analysis notebook had printed — sample counts, duration, peak speed,
map dimensions, invalid-cell count, peak efficiency and its location, and the shift
counts at a known threshold pair. The map itself is a copper + iron + windage loss
model fitted to the handful of cell values visible in the notebook's output.

They reproduce the original's *diagnostics* closely. They are not a sample-for-sample
reconstruction of the vehicle, and **no number produced from them is an N603 result.**

## Replacing them with real data

Use `Load drive cycle` and `Load efficiency map` in the app, or point
`verify_model.py` at your files. Nothing else needs to change — the loaders accept
several common column spellings.

### Drive cycle (CSV)

Two columns. The loader accepts any of these headers:

- time: `Time`, `Time[s]`, `Time [s]`, `t` (also tolerates the typo `Tiime[s]`)
- speed: `Speed`, `Speed[kmph]`, `Speed [km/h]`, `Speed[km/h]`, `v`

Units are seconds and km/h. Rows are sorted by time, duplicate timestamps are
averaged, and blank rows are dropped. Negative speed is rejected. A non-uniform
time base is fine — every integration uses the actual timestamps.

### Efficiency map (XLSX)

A single grid, no header row of its own:

- row 0, columns 1..N — motor speed in rpm
- column 0, rows 1..M — motor torque in Nm
- the rest — efficiency, either as a fraction (0–1) or a percentage (auto-detected
  by the median; anything above 1.5 is treated as a percentage and divided by 100)

Blank cells and non-positive values are treated as unmeasured. Both axes must be
unique and strictly increasing after sorting; they do not need to be evenly spaced.

**One thing worth checking in your real map:** if the zero-torque row is blank or
zero, every query below the lowest measured torque row falls outside the linear
interpolator. This tool extrapolates those points physically (holding the
speed-dependent loss constant so efficiency goes to zero with torque) rather than
substituting a neighbouring value — see `EfficiencyMap._below_grid_eff`.
