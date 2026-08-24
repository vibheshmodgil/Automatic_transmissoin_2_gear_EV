# Real N603 data

Extracted from `Book2.xlsx` (two sheets: drive cycle, efficiency map) into the two
files the tool loads directly.

| file | what it is |
|---|---|
| `N603_real_drive_cycle.xlsx` | 28,368 samples, 28,367 s, 111.35 km, max 42.26 km/h |
| `N603_real_efficiency_map.xlsx` | 42 torque rows x 901 rpm columns, peak **94.13 %** at 4260 rpm / 13.0 Nm, 14,091 blank cells |

These match every diagnostic the original notebook printed, so the synthetic
stand-in in `sample_data/` can now be retired for anything quantitative.

**Load them straight into the app** — `Load drive cycle` accepts `.xlsx` as well as
`.csv`; the map loader already did.

Note the earlier screenshots in this project were produced with a different,
much shorter city cycle (12 km, 0.5 h). Which cycle is loaded changes the shape of
the efficiency-vs-downshift curve — see ENGINEERING_NOTES section 7s.
