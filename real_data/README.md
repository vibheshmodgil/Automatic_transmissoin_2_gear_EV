# Real N603 data

| file | source | what it is |
|---|---|---|
| `N603_real_drive_cycle.xlsx` | `Book3.xlsx` (kept beside it for provenance) | **the drive cycle to use** — 1,801 samples, 1,800 s, **11.81 km**, max 44.71 km/h |
| `N603_real_efficiency_map.xlsx` | `Book2.xlsx` Sheet2 | 42 torque rows × 901 rpm columns, peak **94.13 %** at 4260 rpm / 13.0 Nm |

`Book2.xlsx` Sheet1 held a longer 111 km log. That is **not** the cycle this study
is about — the city cycle above is, and it is the one the reported screenshots were
produced from. The longer log is kept only in `sample_data/` as a second dataset to
check that nothing in the tool depends on one cycle's shape.

**Load them straight into the app** — `Load drive cycle` accepts `.xlsx` as well as
`.csv`; the map loader already did.

`verify_model.py` runs against these by default and passes on both datasets.
