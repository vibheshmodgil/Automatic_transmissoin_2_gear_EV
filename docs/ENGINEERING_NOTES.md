# Engineering notes

> Working notes kept during development: the audit of the original notebook, every
> defect found, the derivations behind each analysis, and the reasoning for each
> decision. Written for whoever picks this up next, not as polished documentation.
> Start with `README.md`; come here when you need to know *why*.

---

# CLAUDE.md — N603 Two-Speed Shift-Speed Energy Optimization

Working notes for this folder. Read this before touching the notebook.

---

## 1. What this project is

A study of how the **1→2 upshift and 2→1 downshift speeds** of a two-speed gearbox
change motor operating points, efficiency usage, and consumed battery energy over the
N603 on-road drive cycle. Vehicle is a light EV (995 kg, 52 V pack, 11 kW / 45 Nm motor).

The deliverable the work is aiming at: a recommended shift schedule, and evidence for
whether the second ratio earns its place at all.

**Status: the notebook's current conclusion (upshift at 20 km/h) is wrong.** See §5.

---

## 2. Files

| File | What it is |
|---|---|
| `City_drive_cycle_data_analysis_no_regen.ipynb` | The analysis. 45 cells. **Does not run top-to-bottom as saved.** |
| `City_drive_cycle_data_analysis_no_regen.RUN.ipynb` | Executable copy with 4 documented shims (§7). Generated, safe to delete/regenerate. |
| `N603_AMT_onroadrange_SpeedBased_.csv` | **Synthetic stand-in** drive cycle. Not the real N603 log. See §6. |
| `N603_42UH_230_Fineint_EfficiencyMap.xlsx` | **Synthetic stand-in** efficiency map. Not the real dyno map. See §6. |
| `Base - Part 1..4.step` | CAD geometry. Unrelated to the notebook. |

The real source data (on the original author's machine) was not available.
**Any number quoted from a run of this notebook is a number from synthetic data**
until the real CSV/XLSX are dropped in.

---

## 3. Physics model and conventions

Get these right or results silently drift.

### Road load (cell 10)
```
F_total = m·a + m·g·Crr·cos(θ)·[v>0] + ½·ρ·Cd·A·v² + m·g·sin(θ)·[v>0]
T_wheel = F_total · r
P_wheel = T_wheel · ω_wheel
```
`a = np.gradient(v, t, edge_order=2)` — correct for a non-uniform time base.

### Gear projection (cell 12) — sign convention matters
```
n_motor = n_wheel · G
motoring (P_wheel ≥ 0):  T_motor = T_wheel / (G · η_gb)      # motor supplies MORE
regen    (P_wheel <  0):  T_motor = T_wheel · η_gb / G       # motor receives LESS
```

### Battery power (cell 30) — the trap
```
motoring: P_batt = P_wheel / (η_motor · η_gb)
regen   : P_batt = P_wheel ·  η_motor · η_gb
```
**Build from `wheel_power`, never from `motor_power`.** `motor_power` already contains
the gearbox loss (`P_mech / P_wheel = 1/η_gb = 1.030928`), so using it double-counts η_gb.
The current code does this correctly — do not "simplify" it.

### Energy
```
consumed  = ∫ max(P_batt, 0) dt / 3.6e6     # the no-regen objective
recovered = −∫ min(P_batt, 0) dt / 3.6e6
net       = consumed − recovered
```

### Shift controller (cell 12)
Hysteresis: upshift when `v ≥ upshift`, downshift when `v ≤ downshift`.
**Requires `downshift < upshift`.** Violating it inverts the band and the controller
chatters every sample. This is not currently enforced — see §5.

---

## 4. Parameters (cell 3)

| Symbol | Value | Note |
|---|---|---|
| `vehicle_mass` | 995 kg | |
| `wheel_radius` | 0.247 m | |
| `Cd` × `frontal_area` | 1.104 × 1.0 | **This is a lumped CdA.** A 995 kg vehicle has ~1.6–2.2 m² frontal area. Do not "fix" the area to a realistic value — it would inflate drag ~80%. |
| `Crr` | 0.02 | |
| `road_grade_deg` | **0.0** | Gradeability is never tested. See §5. |
| `G1` / `G2` | 19 / 11 | G1 is the LOW gear (higher reduction) |
| `eta_gear_1/2` | 0.97 | |
| `motor_max_rpm` | 10 000 | but the map only goes to 9 000 |
| `motor_max_torque` | ±45 Nm | but the map only goes to 41 Nm |
| `battery_voltage` | 52 V | **declared and never used anywhere** |
| `battery_max_power` | 15 000 W | exceeds the 11 kW motor, so this check never binds |
| `power_epsilon` | 1.0 W | below this, efficiency is treated as irrelevant |
| `minimum_efficiency` | 0.01 | map cells below this become NaN |

Motor envelope used in plots (hardcoded in cells 16 and 19, **duplicating** `motor_max_torque`):
peak 45 Nm / 11 kW, base speed `9550 · 11 / 45 = 2334 rpm`.

---

## 5. Known problems — read before trusting any output

Full writeup: **https://claude.ai/code/artifact/e7165726-8bde-4094-9a14-493ed1910b14**

### Blocking (notebook won't run as saved)
- **B1** `SIGNED_RPM_mesh` / `SIGNED_TORQUE_mesh` / `signed_eff_map` used by cells 26 and 28, defined nowhere. The cell that mirrored the map about zero torque was deleted.
- **B2** Cell 28 calls `calculate_cycle_energy_for_shift()`; cell 30 defines it.
- **B3** `np.trapezoid` (cells 30, 34) requires **NumPy ≥ 2.0**. This machine has 1.26.4 → `AttributeError`. Use `np.trapz` or pin numpy.
- **B4** Hard-coded absolute paths into another user's Downloads folder.

### The conclusion is wrong
- **C1** Every reported optimum sits on the **edge of its own search range**: upshift-only → 20 (floor of `arange(20,43)`), downshift-only → 5 (floor of `arange(5,32)`), combined → (20, 8). Energy is monotone across the whole sweep. An optimizer returning a corner has not found an optimum.
- **C2** The winner **20/22 has upshift below downshift** — an inverted hysteresis band, 1384 upshifts + 1384 downshifts. Cell 39 prints "Optimization validation passed…" but that assert guards only the *combined* grid; cell 32 and case 1 include 20/21/22 against a fixed 22 and the headline answer is one of them.
- **C3** The claimed 85.6 Wh (0.98%) saving is **smaller than the model's error bar**: 74% of gear-2 points use nearest-neighbour fallback vs 12% for gear 1; shifting costs nothing; unfiltered `np.gradient` noise gets rectified by `max(P,0)`; no aux load (150 W over 7.88 h = 14% of the total).

### Physics gaps that move the answer
- **P1** The 11 kW constant-power envelope is **drawn in two figures and never enforced**. `torque_ok` tests only flat ±45 Nm. Demonstrated: a 1.06 m/s² accel in gear 2 demands 30.7 Nm where the envelope allows 23.3, and 14.5 kW from an 11 kW motor — reported as `Valid=True, Motor Limit Points = 0`.
- **P2** `road_grade_deg = 0.0`. Gradeability is the reason a low first gear exists. On flat ground the optimizer correctly finds gear 1 useless — and the notebook reports that as an optimum instead of a missing constraint.
- **P3** No reflected rotor inertia. Scales with G², so gear 1 carries ~3× gear 2's penalty (~30 kg vs ~10 kg equivalent mass). Gear-dependent → biases exactly what is being optimized.
- **P4** Map handling: `eff_map[eff_map < 0.01] = nan` deletes real measurements; the 14 672 NaNs are the region above the ~10.84 kW envelope plus the zero rows, so the fallback fills **infeasible** points with a feasible neighbour's efficiency; `RegularGridInterpolator` returns NaN if *any* of 4 corners is NaN, and the whole `T=0` row is NaN, so **every query below 1 Nm falls back**; `NearestNDInterpolator` uses **unscaled** (Nm, rpm) coordinates.
- **P5** `battery_voltage` unused. At 52 V / 181 A, 20 mΩ = 655 W = 7% of peak, current-squared and therefore gear-dependent. No aux load. Undocumented whether the map is motor-only or motor+inverter.
- **P6** Speed is differentiated unfiltered, then `max(P,0)` rectifies the noise.
- **P7** Shifting is free — no torque interruption, sync loss, or inertia. This is *why* the optimizer buys efficiency with 2768 shifts.
- **P8** A file named `no_regen` prints "Recovered energy: 0.78 kWh" next to the consumed figure.

### The finding the notebook should be making
Cruise sits at **3–7 Nm on a 45 Nm motor** (6–15% load) in *both* gears. Peak map
efficiency is 13.0 Nm × 4260 rpm = **5.80 kW**; cycle mean battery power is
8.64 kWh / 7.88 h = **1.10 kW** — 19% of that.

A gear ratio moves the operating point *along* an iso-power hyperbola. It cannot move it
*between* hyperbolas. **No shift schedule or ratio choice reaches the high-efficiency
island** — the motor is oversized for this duty by roughly 5×. That is the real result.

---

## 6. The stand-in data (how it was built, and what it is not)

Generated by fitting to every diagnostic the notebook printed, because the real files
were unavailable.

**Exact matches:** 28 368 samples, 28 367 s, max 42.26 km/h, map shape (42, 901),
14 672 invalid cells, peak efficiency 94.13% at 4260 rpm / 13.0 Nm, 44 upshifts at 32/22.

**Within a few percent:** consumed 8.84 vs 8.73 kWh, recovered 0.82 vs 0.77,
fallbacks 2446/1332 vs 2385/1358, peak battery ±9.8/−8.4 vs ±9.47/−9.12 kW.

Method: with a valid hysteresis band, `N(v)` (upshifts at threshold `v` > downshift) is
exactly the count of speed excursions peaking above `v`. Differencing that series over
thresholds 22–42 inverts it into a peak-speed histogram — 355 excursions, recovered
exactly. Gear-2 sample counts then pin each excursion's duration via a triangular system
solved top-down. The map is a copper + iron + windage loss fit to the values visible in
the notebook's own `df_eff_raw.head()` output.

Two side findings corroborate §5: the NaN count matches the area above a **10.84 kW**
envelope almost exactly (→ P4), and the real `fb2/fb1 = 0.57` equals `1/1.727`, the
inverse gear-ratio ratio, meaning torque is spread through zero by **measurement noise**,
not pinned at zero by clean coasting.

**Caveat:** the notebook's real sweep is strictly monotone with minimum at 20; the
stand-in bottoms out at 22. Same pathology (optimum on the inverted-band boundary),
not a sample-for-sample match. **Do not quote stand-in numbers as N603 results.**

---

## 7. Running it

```bash
python -m pip install nbclient nbformat ipykernel     # one-time
```

The `.RUN.ipynb` copy carries four shims, each labelled `RUN SHIM` in the cell source:
1. `matplotlib.use("Agg")` + `np.trapezoid = np.trapz` (B3)
2. Paths repointed to this folder (B4)
3. Signed-map cell rebuilt (B1)
4. Function definition moved above its first use (B2)

**Nothing in the physics or the optimization was changed by the shims.**

The combined grid in cell 39 is ~500 candidate pairs × a 28 368-sample Python loop
(~15 M iterations). Expect minutes. The state machine vectorizes cleanly, or memoize by
gear-sequence hash — cell 36 already shows it collapses to only 23 distinct sequences.

---

## 7b. Latest run (stand-in data, 23 s, zero errors)

All 48 cells executed, 12 figures produced. Outputs in `.RUN.ipynb`, figures in `figures/`.

| Result | Value |
|---|---|
| Manager baseline 32/22 | 8.840483 kWh, 44 upshifts |
| **Upshift-only "optimum"** | **22.0 / 22.0 km/h** — 8.727364 kWh, **366 upshifts**, −113 Wh (1.28%) |
| Downshift-only optimum | 32.0 / **5.0** km/h — 8.732647 kWh, −108 Wh |
| Combined optimum | **22.0** / 11.0 km/h — 8.642366 kWh, −198 Wh (2.24%) |
| Peak efficiency | 94.13% at 4260 rpm / 13.0 Nm |
| Fallback points | gear 1 = 2438, gear 2 = 1333 |

**The upshift-only winner is 22/22 — upshift equal to downshift, a zero-width band.**
Cell 42 prints *"Optimization validation passed: every combined candidate has downshift
speed below upshift speed"* immediately below it. The assert covers only the combined
grid; the upshift-only case is never checked. This is C2, reproduced live.

Cell 37's manual verification also **passes on 20/22** — it asserts only internal energy
consistency (`consumed − recovered = net`), never whether the strategy is physical.

New plot defect found in the run: in cell 42 panels 1 and 2 the red "Best" and black
"Manager 32/22" markers are drawn *under* the series markers with default size and no
`zorder`, so they are invisible — only a thin red rim shows at x=22. Cell 39's standalone
sweep plot gets this right (`s=100, zorder=3`). Fix cell 42 to match.

Minor stand-in artifact: the generated cycle starts moving at t=1 instead of idling, so
`np.gradient(edge_order=2)` yields a non-zero acceleration at t=0 and cell 13 shows
12.69 Nm at 0 rpm. Mechanical power is 0 there so energy is unaffected, but the real log
began with a long idle and did not show this.

---

## 7c. Report UI and the VMI integration path

`make_shift_report.py` builds a self-contained HTML report with all 12 figures embedded.

```bash
python make_shift_report.py               # -> shift_study_report.html  (standalone, 3.1 MB)
python make_shift_report.py --body-only   # -> shift_study_report.body.html (fragment)
```

**Written deliberately in the VMI report idiom** so it can drop into that tool later:

| Aspect | Matches VMI |
|---|---|
| Target | `C:\Users\vibhe\OneDrive\Desktop\PowertrainTool\vehicle_motor_suite\vehicle_motor_suite` |
| Reference | `vmi/enhancements.py` → `_MULTI_REPORT_TEMPLATE`, `_REPORT_SECTION_TEMPLATE` |
| Palette | primary `#4f46e5`, header `#1e1b4b`, bg `#eef1f7`, border `#e2e8f0` (from `vmi/theme.py` `COLORS`) |
| Structure | header → inputs card → assumptions card → TOC nav → figure cards → footer |
| CSS classes | `card`, `toc`, `obs`, `interp`, `summary`, `assume` — same names |
| Figures | `<img src="data:image/png;base64,…">`, fully self-contained |
| Font | `'Segoe UI', Arial` |

Additions beyond the VMI template: a `.kpi` grid, a `.warn` variant of `.interp`, and a
two-column TOC. All additive — nothing conflicts with VMI's existing CSS.

### To integrate as a VMI analysis type
VMI already has `vmi/drive_cycle.py`, `vmi/efficiency.py`, `vmi/range_analysis.py`,
`vmi/physics.py`. The shift study belongs alongside them as `vmi/shift_optimizer.py`,
registered through `vmi/dispatch.py` like the other analyses, with its figures returned via
`_render_report_views` so the existing report machinery embeds them automatically.

> **`np.trapezoid` blocks this today.** `requirements.txt` pins `numpy==1.26.4`, where that
> attribute does not exist. Port the notebook to `np.trapz` before moving any of this into
> VMI — it is not an optional cleanup, it is a hard incompatibility with the suite's pinned
> environment.

---

## 7d. FIXED IMPLEMENTATION — `shift_core.py` + `shift_app.py`

The notebook is now superseded for analysis purposes. Two new files:

| File | Role |
|---|---|
| `shift_core.py` | Physics engine. Pure functions + dataclasses, no UI, no globals. **This is what ports into VMI as `vmi/shift_optimizer.py`.** |
| `shift_app.py` | CustomTkinter front end. Presentation only — every number comes from `shift_core`. |

Run: `python shift_app.py`

### What was fixed, and how it is verified

| ID | Fix | Verification |
|---|---|---|
| B3 | `np.trapz` via `_trapz` shim | runs on numpy 1.26.4 (VMI's pin) |
| C1 | Boundary detection — `SweepResult.converged` is False when the optimum touches a range edge, with `boundary_note` explaining | 23/22 correctly flagged NOT converged |
| C2 | `downshift < upshift` enforced in `simulate()` **and** when candidates are generated | 22/22 and 20/22 both refused: `invalid band` |
| P1 | `Motor.envelope()` = `min(T_peak, 9550·P_peak/n)`, enforced in feasibility | 3° grade → 158 envelope violations → infeasible |
| P2 | `Vehicle.grade_deg` real; plus `gradeability_table()` for the meaningful per-gear check | see result below |
| P3 | Reflected rotor inertia `J·G²/r²`, two-pass because gear choice depends on load | gear-dependent, so the ratios compare fairly |
| P4 | Only blank cells missing (genuine low-η kept); nearest-neighbour on **normalised** axes; above-envelope flagged not filled | fallback 2439 at baseline |
| P5 | Aux load (the pack I²R term has since been removed — see §7k) | consumed 8.84 → 10.31 kWh at baseline |
| P6 | Optional Savitzky-Golay before `np.gradient` | `Numerics.smooth_window` |
| P7 | Energy per shift, torque-interruption window, shifts/hour cap | see result below |
| — | Wh/km, distance, time-in-gear, energy-weighted mean efficiency | all in `ShiftResult` |

`gear_sequence()` was rewritten to evaluate only at threshold crossings —
**verified identical to the notebook's per-sample loop** on four threshold pairs, and
~250× faster (200 sequences in 0.05 s), which is what makes the GUI responsive.

### THE HEADLINE RESULT — SUPERSEDED, see §7e

> The table and conclusion below were produced by the **old torque-interruption model**,
> which charged every shift `interrupt_s x cycle-mean positive battery power`. That model
> over-charged downshifts (93 % of which happen under braking, where there is no traction
> to interrupt) and under-charged upshifts (which happen at ~4x the cycle-mean power).
> Because it scaled with *shift count* rather than with *shift power*, it pushed the
> optimum toward high upshift speeds. With the corrected model (§7e) the upshift optimum
> is flat from 6 to 22 km/h and the 30-32 km/h result does not reproduce.

### THE OLD HEADLINE RESULT — the manager's baseline was approximately right

Upshift sweep, downshift 22, range 20–42:

| Shift cost | Best upshift | Consumed | Shifts/h | Converged |
|---|---|---|---|---|
| free (the notebook's assumption) | 23 | 10.1923 kWh | 77.7 | **No — on the edge** |
| 200 J + 0.2 s | 23 | 10.2703 kWh | 77.7 | **No — on the edge** |
| 500 J + 0.4 s | **30** | 10.3260 kWh | 19.0 | **Yes** |
| 1500 J + 0.6 s | **32** | 10.3641 kWh | 11.2 | **Yes** |

**Charging anything realistic for a gear change moves the optimum interior and lands it at
30–32 km/h — essentially the manager's existing 32 km/h baseline.** The notebook's
"improvement" was an artifact of shifting being free. That is the defensible conclusion.

With a proper band (downshift 5–10) the flat-ground optimum is a genuine interior optimum
at **upshift ≈ 22 km/h**, stable across search ranges [8,42] through [20,42]. Combined grid
optimum 22/10, converged, 10.08 kWh / 90.9 Wh/km.

### Gradeability — what the low gear is actually for

`gradeability_table()`, max sustainable grade at steady speed:

| Speed | Gear 1 | Gear 2 |
|---|---|---|
| 5–10 km/h | 18.9° | 10.3° |
| 15 km/h | 14.0° | 10.3° |
| **20 km/h** | **10.1°** | **10.1°** |
| 25 km/h + | identical | identical |

**Gear 1 gives ~2× the climbing ability below 20 km/h and nothing above it** — both ratios
are power-limited there. So the energy optimum (≈22 km/h) and the gradeability requirement
agree: upshift around 20–22 keeps the low gear exactly through its useful range.

Note `Vehicle.grade_deg` applies the grade to the **whole** cycle, which is a deliberately
harsh test (every acceleration is uphill for 110 km). Use `gradeability_table()` for the
real question; use `grade_deg` for a sustained-climb scenario.

---

## 7e. Audit: why downshifting to a higher-rpm point does not save energy

Prompted by "the downshift logic and convergence is flawed — we shift a low-rpm low-efficiency
point up into a higher-rpm higher-efficiency zone and that is not reflected in the energy".
Everything below is measured on the stand-in data (§6), engine `shift_core.py`.

### The premise is right, but only below ~7 km/h

Steady cruise, both ratios, straight out of `EfficiencyMap.query`:

| v [km/h] | gear 2 rpm / eff | gear 1 rpm / eff | downshift gains |
|---|---|---|---|
| 4 | 473 / 89.91 % | 816 / 93.40 % | **+3.48 pts** |
| 5 | 591 / 91.30 % | 1020 / 93.35 % | **+2.05 pts** |
| 6 | 709 / 92.19 % | 1224 / 93.13 % | **+0.94 pts** |
| 7 | 827 / 92.78 % | 1428 / 92.81 % | **+0.03 pts**  <- crossover |
| 10 | 1181 / 93.63 % | 2040 / 91.60 % | -2.03 pts |
| 14 | 1654 / 93.87 % | 2857 / 89.86 % | -4.01 pts |
| 20 | 2363 / 93.64 % | 4081 / 87.47 % | -6.17 pts |

The map's efficiency ridge for a 4-5 Nm load sits at **1400-2400 rpm**, and ratio 11 puts
the 10-18 km/h cruise band *exactly on it*. Ratio 19 does not move the point up the hill —
it moves it *over the crest* to the high-rpm, low-torque side. Gear 1 only wins where ratio
11 has fallen off the low-rpm end, which is below ~7 km/h.

Then look at how little of the cycle is there: **0.5 % of samples sit between 2 and 8 km/h**,
against 45 % between 10 and 18 km/h and 23 % standing still. So the regime where the
intuition holds is real, and is worth almost nothing over a 110 km cycle.

### Why the plots suggest otherwise

`Shift movement` draws downshift arrows sweeping right into a brighter part of the map.
Measured at 22/14: **93 % of downshifts happen at `P_wheel <= 0`** — under braking. With
`regen_enabled=False` the motor supplies nothing there (battery power is aux only, 150 W),
so the efficiency those arrows travel through **has no effect on energy at all**. The
negative half of that plot is also a *mirror assumption*, not measured data. Of the
downshifts that do happen under traction, the map efficiency falls 1.94 points.

That is the whole disconnect: the picture is drawn in a region the energy model deliberately
ignores. Not a calculation error — but the plot should say so.

### Four real defects found and fixed in `shift_core.py`

| # | Defect | Direction of the error | Effect |
|---|---|---|---|
| 1 | **Tie plateau read as a boundary optimum.** Downshift 4-9 km/h produce a bit-identical energy (the threshold never changes the gear sequence there); `min()` returned the first, i.e. the bottom of the range, which then tripped the edge test | reported a converged answer as NOT converged, and quoted an arbitrary member of a flat plateau | `_tie_plateau()` reports the plateau and its midpoint. Downshift sweep now: **best 7 km/h, converged, "6 candidates tie exactly over [4, 9]"** instead of "4, NOT CONVERGED" |
| 2 | **Torque-interruption charged on braking downshifts.** `n_shifts x interrupt_s x cycle-mean positive power` | invented a penalty proportional to shift count — it suppressed exactly the low-speed downshift benefit being looked for, and under-charged upshifts | now `interrupt_s x local traction power at each shift`. Baseline 22/10: 40.3 Wh over **78 of 136 shifts under traction**. Moves the upshift answer materially — see below |
| 3 | **The all-blank `T = 0` row poisoned the interpolator.** Every query below 1 Nm had a NaN grid corner, fell back to nearest-neighbour and was handed ~81 % — the efficiency of a properly loaded point | flattered near-zero-torque running, i.e. flattered gear 1 | `_below_grid_eff()`: hold the rpm-dependent loss at its lowest measured value and let output power fall, so efficiency goes to 0 with torque (0.5 Nm @ 2000 rpm: 81 % -> 71 %) |
| 4 | **`mean_efficiency` was an input-power-weighted arithmetic mean of the map value** — not the efficiency of anything | read ~2.8 points high | now total shaft output / total electrical input: 92.66 % -> **89.9 %** on the same run (the 92.59 % now reported is the motor-only chain, gearbox excluded) |

Side effect of #3 caught in testing: near-zero-torque points then fell under
`Numerics.min_efficiency` and `eff_ok` declared the **whole strategy infeasible** over a
coasting sample carrying a fraction of a watt — silently deleting candidates from sweeps.
`min_efficiency` now screens map cells only; a genuinely low efficiency is counted, reported
in `reasons`, and never fatal.

Energy accounting closes exactly after all four:
`trapz(max(P_batt,0)) + actuation + interruption = consumed` to 0.000 uWh.

### The corrected answer

Upshift sweep, downshift 5, range 6-42, after the fixes:

| Shift cost | Best upshift | Consumed | Shifts | Converged |
|---|---|---|---|---|
| free | 22 | 10.0922 kWh | 62 | Yes |
| 200 J + 0.2 s | 17 | 10.1035 kWh | 62 | Yes |
| 500 J + 0.4 s | 7 | 10.1135 kWh | 62 | Yes |
| 1500 J + 0.6 s | 6 | 10.1329 kWh | 62 | No — on the edge |

But read the spread, not the argmin: **upshift 6 through 22 covers 10.1135 to 10.1188 kWh —
5 Wh, 0.05 %.** The curve only starts to matter above 24 km/h, reaching +3.5 % at 42.

And the question behind the question:

| Strategy | Consumed | Wh/km | mean eff |
|---|---|---|---|
| best two-speed 12/5 | 10.1140 kWh | 91.2 | 92.46 % |
| **ratio 11 alone** | **10.1137 kWh** | **91.2** | 92.36 % |
| ratio 19 alone | 10.4849 kWh | 94.6 | 89.02 % |

**The second ratio is worth 0.2 Wh over 110 km — nothing.** The energy case for the
two-speed box on this duty does not exist; the case for gear 1 is gradeability and launch
(§7d table: ~2x the climbing ability below 20 km/h), and the upshift threshold should be
set from that, not from a 0.05 % energy difference. This is the same conclusion §5 reached
("no shift schedule reaches the high-efficiency island — the motor is oversized by ~5x"),
now with the downshift side closed off too.

**Caveat that outranks all of the above:** this is the stand-in map (§6). The crossover
speed is set entirely by where the map's efficiency ridge sits. Drop the real
`N603_42UH_230` dyno map in and the 7 km/h crossover moves; every number in this section
must be re-run before it is quoted as an N603 result.

---

## 7f. Acceleration run — wide-open-throttle 0 -> V time

`Acceleration run` in the analysis menu. **Needs the efficiency map only — no drive cycle.**

> Supersedes the first version of this tab, which prescribed a constant acceleration and
> asked what it cost. That was the wrong question: it made acceleration an *input*, so the
> ratio could never change the time, only the energy. The motor now sits on its **peak
> curve** and the acceleration is an **output**.

`shift_core.wot_run()` integrates the launch forward from rest:

```
        F_traction(v, gear) - F_road(v)
a(v) = ----------------------------------      m_eff = m + J*ratio^2/r^2
                    m_eff
```

with, at every step: the constant-power envelope `min(T_peak, 9550*P/n)` scaled by the
throttle, the pack power ceiling referred to the shaft, the ratio's rpm limit (torque goes
to zero above it — that is what caps top speed in gear 1), reflected rotor inertia, and
the efficiency map for the energy. A gear change costs `ShiftCost.interrupt_s` of **zero
traction**, during which the vehicle decelerates against road load, plus
`energy_per_shift`. `wot_sweep()` runs that for every 1-2 upshift speed, plus one
"hold gear 1" sentinel.

| Parameter | Default |
|---|---|
| Target speed [km/h] | 30 |
| Throttle [0-1] | 1.0 |

Shift speeds swept come from the **Sweep range** upshift bounds and step.

### Panels

1. speed against time for the fastest, slowest and latest-shifting schedules, shift
   instants dotted — the interruption dip is visible;
2. **0-V time against shift speed** (left axis) with energy per run on the right;
3. **tractive force per ratio against speed**, with road load and the crossover marked —
   the diagram that explains panel 2;
4. the full-throttle operating line on the efficiency map.

### Accuracy — quadrature, not time stepping

The first version integrated forward in time with Euler at `dt = 0.01 s`. The differences
between adjacent shift speeds are 1-8 ms, i.e. *smaller than the time step*, so the sweep
could not resolve what it was being asked to resolve.

Inside a phase the acceleration depends only on speed, so time, distance and energy are
integrals rather than a simulation:

```
t = INT dv/a(v)        x = INT v dv/a(v)        E = INT P_batt(v) dv/a(v)
```

`_wot_phase()` evaluates those by quadrature on a 1501-point speed grid; the torque
interruption between phases is RK4 over its (short) fixed duration. Verification:

| check | result |
|---|---|
| grid convergence 101 -> 24001 points | settles by 1501; last change **0.00004 ms** |
| against an independent fixed-step RK4 at `dt = 2e-5 s` | agrees to **±0.02 ms**, ~100x faster |

`dt` is still accepted by `wot_run()` for call compatibility and is ignored.

### Results on the stand-in map

| Run | Fastest | Slowest | Force crossover |
|---|---|---|---|
| 0-30 km/h, 100 % | **4.308 s, no shift** | 5.445 s | 20.0 km/h |
| 0-50 km/h, 100 % | **13.16 s, shift at 19 km/h** | 13.69 s | 20.1 km/h |
| 0-30 km/h, 50 % | **9.96 s, no shift** | 11.65 s | 20.0 km/h |

**Does the upshift speed affect the 0-30 time? Yes — 1.137 s across the searched range,
26 %.** The curve is a clean interior minimum, resolved to milliseconds:

| shift at | 0-30 time | vs best |
|---|---|---|
| 5 km/h | 5.445 s | +1.137 |
| 10 | 5.097 s | +0.789 |
| 15 | 4.802 s | +0.493 |
| 18 | 4.723 s | +0.415 |
| **19** | **4.715 s** | **+0.407** |
| 20 | 4.717 s | +0.408 |
| 25 | 4.754 s | +0.445 |
| 29 | 4.791 s | +0.482 |
| **no shift** | **4.308 s** | **0** |

Among schedules that *do* shift the spread is 0.730 s (15.5 %) and the optimum is 19 km/h,
one grid step off the 20.0 km/h tractive-force crossover.

**The model checks itself here.** Not shifting beats the best shifting schedule by
0.407 s, against a torque interruption of 0.400 s. So at the optimum the ratio change
itself costs +0.007 s and the interruption is the entire penalty — exactly what the force
diagram predicts, since at the crossover the two ratios pull the same and nothing but the
cut should remain. Any other split would mean the integrator or the force model is wrong.

Two things fall straight out and both are textbook:

* **The minimum-time shift speed is the tractive-force crossover** (19 km/h found against
  a 20.1 km/h crossover, one grid step). Below it gear 1 pulls harder — 3357 N against
  1943 N off the line; above it both ratios are on constant power and pull identically, so
  shifting early throws force away and shifting late gains nothing.
* **Over a short run, not shifting at all wins**, because the 0.4 s torque interruption
  costs more than the crossover saves. It stops winning as soon as the target passes what
  gear 1 can reach: ratio 19 hits 10 000 rpm at ~49 km/h, so the 0-50 run *must* shift and
  the crossover answer takes over.

The energy optimum and the time optimum are **not the same point** (0-50: fastest at
19 km/h, least energy at 30 km/h). That is the honest tension between this tab and the
drive-cycle analyses, and it is why both exist.

### Plot height control

Under the toolbar, on every analysis: a **Plot height** slider (360-2600 px) and an
**Auto** button. Auto restores the fitted height (155 px per stacked signal panel in
Single strategy, 560 px elsewhere); a manual height survives re-renders and option
changes. Taller than the window simply scrolls, via the same `CTkScrollableFrame` the
signal stack uses.

## 7g. One set of per-gear numbers, and the braking half of the map

### The mismatch that prompted this

Single strategy and Points on map quoted different figures for the same run (22/10):

| | Single strategy said | Points on map said |
|---|---|---|
| gear 1 share | 30.3 % | 9.4 % |
| gear 2 share | 69.7 % | 90.6 % |
| efficiency | 92.59 % | 90.11 % (g1) / 90.30 % (g2) |

Neither was wrong; they were answering different questions with the same words.

* **Share.** `time_gear1_pct` counts *every* sample. 6,546 of 28,368 samples (23 %) are
  stationary and all of them are "in gear 1" — the gearbox has to be somewhere while the
  vehicle stands still. Points on map counted only samples carrying torque.
* **Efficiency.** The summary reported the energy-true output/input; the map panel
  reported an unweighted arithmetic mean of the map values it had plotted. An arithmetic
  mean of an efficiency is not the efficiency of anything.

### The fix — `shift_core.gear_breakdown()`

One function, one set of definitions, called by both panels (`_gear_table()` in the app
renders it), so they cannot drift apart again:

| Row | Definition |
|---|---|
| time share | share of ALL samples, idle included |
| traction share | share of the MOTORING samples only |
| output energy | share of the motor's mechanical output energy |
| efficiency | output energy / input energy for that gear |
| motor rpm from/to | range over that gear's motoring samples |

The `all` column reproduces `ShiftResult.mean_efficiency` exactly (0.925892 both ways) and
the time share reproduces `time_gear1_pct` exactly, so the block is consistent with the
headline numbers as well as with itself. Baseline 22/10 now reads the same in both views:

```
  PER GEAR            gear 1     gear 2        all
  time share          30.3 %     69.7 %     100.0 %
  traction share       9.7 %     90.3 %     100.0 %
  output energy        8.4 %     91.6 %     100.0 %
  efficiency          92.90%     92.56%      92.59%
```

Read together those four rows are the real story of the baseline: gear 1 is engaged
30 % of the time but does 8 % of the work, because most of its time share is standing
still.

### Negative-torque (braking) half — `Negative-torque (braking) half` checkbox

On by default, in EFFICIENCY-MAP DISPLAY. Points on map now draws the mirrored map and
both point clouds — filled markers motoring, open markers braking (gear 1: 1,909 / 146
points, gear 2: 17,421 / 2,346). Shift movement honours the same switch; turning it off
clamps both views to the motoring half.

Two things the panel says out loud when the half is shown, because neither is obvious:

* the negative half is the measured map **mirrored** about zero torque
  (`eta(-T, n)` assumed equal to `eta(|T|, n)`) — an assumption, not data;
* with `regen_enabled = False` those points carry **no energy at all**, so they change
  nothing in the totals above them. This is the same trap §7e diagnosed: the braking
  cloud is visually prominent and energetically inert.

### Display controls (`shift_app.py`, left panel below the parameters)

All of these replot the **cached** result — nothing is re-simulated, so toggling is instant.
`_render()` stores `(kind, payload, params)` in `self._last_render`; `redraw()` re-renders it
with a 180 ms debounce so dragging a slider does not queue a contour redraw per pixel.

**SIGNALS (single strategy)** — one stacked, x-shared panel per ticked signal, from
`SIGNALS` in `shift_app.py`: road speed, gear, motor rpm, motor torque, motor efficiency,
battery power, shaft power, acceleration, cumulative consumed energy. Ticking more panels
grows the figure (`_fit_canvas`: 155 px per panel) inside a `CTkScrollableFrame`, so the
plot column scrolls instead of squeezing 9 panels into 500 px.
Motor efficiency is blanked wherever the motor is idle — `EfficiencyMap.query()` fills
`eff = 1.0` on inactive points and plotting that raw draws a false 100% line.

**TIME WINDOW** — window length menu (full / 3600 / 1800 / 600 / 300 / 120 / 60 s) plus a
start-position slider. This is the fix for §10's "30 samples per pixel" problem: the full
cycle stays available, and the slider pans a readable slice across 28 367 s. The title
reports the visible span and the gear-change count *inside it*.

**EFFICIENCY-MAP DISPLAY** — applies to every map view (Efficiency map, Points on map,
Shift movement) and, for the colour bar and colormap, to the Combined grid too:

| Control | Effect |
|---|---|
| Filled colour shading | the colours **on the map**; off leaves a white plane |
| Contour lines | line contours; coloured by the colormap when the fill is off, black over it |
| Colour bar (legend strip) | only the strip beside the plot, nothing on the map itself |
| Label contour lines | inline `clabel` percentages |
| Envelope + peak marker | hides the red constant-power envelope and the peak star |
| Shading levels / Number of contour lines | 2–40 and 2–25, gated by their checkboxes above |
| Colormap | viridis / plasma / magma / cividis / turbo / coolwarm / Greys |

Shading and colour bar are **separate switches on purpose** — the first thing a user tries
when they want the colours gone is the one labelled "colour bar", so that label now says
what it actually does and the map colours have their own tick box. `_colorbar()` also
returns None when neither shading nor lines are drawn, so the strip can never outlive the
thing it describes. `_contours()` returns the filled set if there is one, else the line
set, so the colour bar still works in lines-only mode. Both off draws a bare plane with
just the envelope and the operating points — intended, for overlay-only views.

Layout note: `_render()` now sets `fig.set_layout_engine("tight")` instead of calling
`fig.tight_layout()` once, because the scrollable host resizes the figure *after* the draw
call and a one-shot layout would be stale. It falls back to `tight_layout()` where
`set_layout_engine` is missing (matplotlib < 3.6).

**Panel sizing.** Both boundaries are `tk.PanedWindow` sashes - parameters | plot, and
plot | results. CTk widgets that build their own container (`CTkScrollableFrame`,
`CTkTextbox`) cannot be added to a `PanedWindow` directly, because the object you hold is
a grandchild of the master rather than a child; each pane therefore holds a plain
`CTkFrame` with the real widget packed inside it. `_on_sash_moved` refits the figure on
release. The canvas height itself tracks the live viewport (`_viewport_h`) and only
exceeds it when a stacked signal view needs the room - forcing a fixed minimum taller than
the viewport is what made the plot area read as blank on smaller screens.

## 7h. Why the downshift intuition never resolves — the ceiling

The recurring question: *the shift points look like they move into a higher-efficiency
zone, so choosing the downshift speed properly should save energy — why doesn't it?*

The intuition is **correct about the direction and wrong about the magnitude**, and the
old plot was actively feeding it. Three measurements settle it.

### 1. The arrow in "Shift movement" was two things added together

It ran from sample *i* to sample *i+1*, so it contained the ratio change **and** whatever
the driver did in between. `shift_decomposition()` splits it exactly (the terms sum to the
arrow), at 22/10:

| | before -> next sample | of which the gearbox | of which the driver |
|---|---|---|---|
| upshift (68) | −0.94 pts | **−1.97** | +1.03 |
| downshift (68) | −0.78 pts | **+1.61** | −2.39 |

So a downshift really does move the point to a **+1.61 point better** place — the intuition
is right — but the arrow you were looking at showed −0.78, because the driver's demand moved
further than the gearbox did. **17 of the 68 downshift arrows cross zero torque**: braking
at one end, pulling at the other. Those two ends are not even the same kind of operating
point.

The view now draws them separately: **solid arrow = the gearbox** (same instant, ratio
swapped, via `counterfactual_point()`), **dotted arrow = the driver** (demand moving on to
the next sample). Panel titles quote the gearbox delta, not the temporal one.

### 2. The gain is concentrated exactly where no energy flows

Split the same 68 downshifts by what the wheel is doing:

| | events | gearbox worth |
|---|---|---|
| under traction (`P_wheel > 0`) | 10 | **−10.24 pts** |
| braking / coasting (`P_wheel <= 0`) | 58 | **+3.65 pts** |
| **weighted by the power actually flowing** | | **−7.12 pts** |

With `regen_enabled = False` the motor supplies nothing while braking, so a +3.65 point
improvement there multiplies a zero. On the few downshifts that happen under load, the low
ratio is 10 points *worse*. That last row is the number that decides energy, and it is
negative.

### 3. The ceiling: 47 Wh, and no schedule can reach it

`oracle_bound()` gives the better ratio for **every sample independently**, with
clairvoyance, free shifting and no rate limit — nothing causal can beat it:

| | |
|---|---|
| always ratio 19 | 10.4849 kWh |
| always ratio 11 | 10.1051 kWh |
| **oracle (best ratio per sample)** | **10.0580 kWh** |
| **the entire prize** | **47.1 Wh = 0.47 %** |

An exhaustive search of all 640 valid threshold pairs with free shifting finds 22/10 at
10.0898 kWh — it captures 32 of the 47 Wh and leaves 15 Wh that **no speed threshold can
reach**, because the low ratio wins on **10.5 % of moving samples selected by LOAD, not by
speed**: gear 1 is better on 31 % of samples in the 18-22 km/h band and 8 % in the
10-14 km/h band. A vertical line in the speed axis takes all of a band or none of it.
That is the same finding the `Optimal gear map` analysis draws as a curve.

This ceiling is now printed at the bottom of every sweep summary, so the question cannot be
asked again without the answer being on screen next to it. When the sweep's best is worse
than the best single ratio, the line says so explicitly — the schedule is paying more in
shift cost than the ratio choice returns.

### Why the downshift curve is flat to 9 km/h and then rises

Two separate mechanisms, neither of them an optimisation artifact:

* **below ~9 km/h** the threshold never changes the gear sequence — the cycle almost never
  dwells between 4 and 9 km/h without going to a full stop, and a stop returns to gear 1
  anyway. Six candidates tie *bit-identically*; `_tie_plateau()` reports the plateau and
  its midpoint instead of the range edge (§7e defect 1);
* **above ~10 km/h** each step up puts more of the 10-18 km/h cruise band (45 % of the
  cycle) into ratio 19, which is the wrong side of the map's efficiency ridge at cruise
  (§7e) — so it costs energy monotonically, and it costs shifts on top.

**Conclusion.** The downshift speed is not an unexploited lever. The prize for the whole
gear-selection question is 0.47 % under perfect play; a speed threshold can reach about
two-thirds of it; and the remainder needs a load-aware 2-D shift map, not a better
threshold. If a bigger prize is expected, it has to come from the real dyno map (§6)
having a sharper low-rpm falloff than the stand-in — not from the schedule.

---

## 7i. Realistic regen — tested against the downshift question

Hypothesis: the downshift looks bad only because regen is off, so the braking-side
efficiency gain (§7h: +3.65 points, on 58 of 68 downshifts) multiplies a zero. Turning
regen on should let that gain count.

**Tested. The direction is right; it does not change the answer.**

### The model that was there before

`p_shaft[reg] = p_wheel * eff * eta` — the motor absorbs *all* of the braking power, at the
efficiency read at the *full* brake torque, with no charge limit and no low-speed blend-out.
That is not a regen model, it is an upper bound.

### The model now

| `Electrical` field | Default | What it does |
|---|---|---|
| `regen_fraction` | 0.70 | share of brake torque sent to the motor; friction brakes take the rest |
| `regen_max_power` | 0 (= `max_power`) | what the pack will accept on charge |
| `regen_min_speed_kmh` | 5.0 | regen blended out below this road speed |

Applied in that order, then capped by the motor envelope, and **the map is queried at the
torque the motor actually takes** — a motor recovering 4 Nm is not at the operating point
of one holding 12. Exposed in the app under a new REGEN section.

Baseline 22/10, 500 J + 0.4 s:

| | consumed | recovered | net | Wh/km |
|---|---|---|---|---|
| regen off | 10.1490 | 0.0000 | 10.1490 | 91.5 |
| regen on, 70 % blend | 10.0638 | 0.4957 | **9.5681** | **86.3** |
| regen on, 100 % blend | 10.0601 | 0.7361 | 9.3239 | 84.1 |

2,413 samples regenerate, at a mean 4.1 Nm to the motor.

### What it did to the downshift

| | best downshift | net | penalty for D = 18 |
|---|---|---|---|
| regen off | 10 km/h | 10.0898 kWh | 98.5 Wh |
| regen on, 70 % | 10 km/h | 9.5089 kWh | 94.5 Wh |
| regen on, 100 % | **11 km/h** | 9.2643 kWh | **85.6 Wh** |

The optimum moves by **one grid step** at full blend. Splitting D = 10 -> D = 18 shows why:

```
regen ON 100%   consumed 10.0008 -> 10.0995  (+98.7 Wh)
                recovered 0.7361 ->  0.7496  (+13.5 Wh)
```

Holding gear 1 longer **does** recover more — the intuition is real and now measurable at
+13.5 Wh — but the same change spends +98.7 Wh more while pulling, because it puts the
10-18 km/h cruise band (45 % of the cycle) into ratio 19. The braking gain is a seventh of
the traction loss.

### The ceiling does grow

`oracle_bound()` now models regen too (it previously ignored the braking side entirely, so
its numbers did not move at all when regen was switched on — a real blind spot):

| | always 19 | always 11 | oracle | prize | gear 1 wins |
|---|---|---|---|---|---|
| regen off | 10.4849 | 10.1051 | 10.0580 | 47.1 Wh | 10.5 % |
| regen on, 70 % | 9.9144 | 9.5252 | 9.4689 | 56.3 Wh | 13.8 % |
| regen on, 100 % | 9.6524 | 9.2819 | 9.2092 | **72.7 Wh** | 15.0 % |

So regen raises the prize by 54 % and makes the low ratio the better one on 15 % of moving
samples instead of 10.5 %. It is still 0.78 % of the cycle, and a speed threshold captures
about a quarter of it. **Regen makes the gear choice matter more; it does not make a
speed-threshold downshift schedule worth tuning.**

### Two bugs found while doing this

* **The objective was `consumed`, not `net`.** Harmless with regen off (recovered is zero),
  wrong the moment it is on: ranking by consumed prefers whichever schedule recovers least.
  `_objective()` now returns `net_kwh` for every sweep, and `wh_per_km` is built from net.
* **The motor envelope was enforced on the braking side.** Exceeding it while braking is
  not infeasible — the friction brakes take the excess, which is what a blended-braking
  controller does. It used to be counted as a violation and could void a whole strategy.
  (On this cycle **zero** braking samples exceed it, so nothing in the numbers above moved;
  the fix matters for harder decelerations or a smaller motor.)

## 7j. The 11 km/h upshift was a missing constraint, not a result

The upshift sweep returned **11 km/h** with the downshift at 10 - a **1 km/h hysteresis
band**. That is not a shift schedule, it is chatter, and it should never have been a
candidate. Two independent gaps let it through.

### Gap 1 - `min_band` was only enforced on the 2-D grid

`sweep_grid()` took a `min_band` argument. `sweep_upshift()` and `sweep_downshift()`
enforced only `downshift < upshift`, so the 1-D sweeps could return the tightest band the
step size allowed - and energy always prefers it, because a tighter band means more time
in the efficient ratio. The `max_shifts_per_hour` cap did not catch it either: at 11/10 the
cycle produces 136 shifts = 17.3/h, far under the 120/h limit, because a 1 Hz cycle rarely
crosses a 1 km/h window twice in quick succession. The rate cap is the wrong instrument
for band width.

Both constraints now live on `ShiftCost` and are enforced inside `simulate()`, so every
search inherits them and none can return a candidate another would reject.

### Gap 2 - nothing represented what an upshift costs in capability

An upshift hands the vehicle to the high ratio. `accel_capability()` measures what that
ratio can still deliver:

| upshift speed | gear 1 | gear 2 | given up |
|---|---|---|---|
| 11 km/h | 3.08 m/s2 | 1.73 m/s2 | **44 %** |
| 14 | 2.48 | 1.73 | 30 % |
| 16 | 2.14 | 1.73 | 19 % |
| 18 | 1.88 | 1.72 | 8 % |
| **20** | 1.66 | 1.70 | **0 %** |
| 26 | 1.22 | 1.24 | 0 % |

Upshifting at 11 km/h throws away **44 % of the vehicle's acceleration**, in the band where
the cycle does most of its accelerating. The energy objective cannot see that: it scores a
schedule on one fixed speed trace that never happens to demand what the schedule gave up.
(Note the absolute reserve is *highest* at 11 km/h - the high ratio is on flat torque
there - so an absolute floor does not catch this. The constraint has to be relative.)

`ShiftCost.max_accel_loss` caps the fraction of available acceleration an upshift may give
up. `min_accel_reserve` still guards the other end, where the high ratio runs out of
constant power.

### The result, and why it is trustworthy

Upshift sweep, downshift 10, 500 J + 0.4 s:

| constraints | best upshift | net |
|---|---|---|
| none - what produced the original answer | 11 km/h | 10.1334 kWh |
| band >= 3 km/h | 13 | 10.1383 |
| band >= 3, give up <= 20 % | 17 | 10.1404 |
| band >= 3, give up <= 10 % | 18 | 10.1425 |
| band >= 3, give up <= 5 % | 19 | 10.1435 |
| **band >= 3, give up <= 0 %** | **20** | 10.1463 |

**A schedule forbidden from giving up any acceleration lands on 20 km/h - and the
tractive-force crossover is 19.76 km/h.** Those are two completely independent routes to
the same threshold: one from the efficiency map and the drive cycle, the other from
envelope geometry and vehicle dynamics with no reference to the cycle at all. That
agreement is the strongest single piece of evidence in the study.

**And the whole range costs 8 Wh - 0.08 %.** The energy objective has essentially no
opinion between 13 and 20 km/h, which is exactly why it drifted to the edge of the search
range. The upshift threshold is a driveability decision, not an energy one, and it should
be argued on tractive-force continuity: **upshift at 20 km/h**, where the handover is free.

Verified by C8 in `verify_model.py` (4 checks). Defaults now shipped in the app:
minimum band 3 km/h, maximum acceleration given up 10 %, minimum reserve 0.5 m/s2.

## 7k. The shift actuator is the shift cost — and the pack is ideal again

Two changes, both asked for directly: give every gear change the energy the hardware
actually costs, and delete the I²R term that was never measured.

### `energy_per_shift` is no longer a tuning knob

The vehicle team supplied the actuator: **12 V, 20 A, 0.5 s per change**. So

```
energy_per_shift = V x I x t = 12 x 20 x 0.5 = 120 J
interrupt_s      = t         = 0.5 s
```

`ShiftCost` now carries `actuator_voltage` / `actuator_current` / `actuator_time_s` and
derives both costs in `__post_init__`. `energy_per_shift` and `interrupt_s` default to
`None` meaning *derive*; passing numbers still overrides (`ShiftCost(0, 0, np.inf)` is
still the free-shifting baseline every comparison is drawn against). Every sweep goes
through `simulate()`, so **the upshift sweep, the downshift sweep and the combined grid
all inherit the penalty** — that was the whole point of putting the costs on `ShiftCost`
in §7j, and nothing new had to be wired into the sweeps.

The traction cut is *not* a second free parameter any more either: the actuator is moving
for 0.5 s, so traction is gone for 0.5 s. One number, one physical event.

### What it costs, and what it moves

Baseline 22/10 on the stand-in data, 136 shifts:

| term | Wh |
|---|---|
| actuator, 136 x 120 J | **4.5** |
| traction cut, 0.5 s x local traction power on the 78 shifts under load | **48.3** |
| total | 52.8 Wh of a 9.91 kWh cycle (0.53 %) |

The actuator itself is small; the cut it forces is 10x bigger. Both scale with shift
count, which is exactly the penalty that was wanted. Effect on the sweeps (min band 3,
max accel given up 10 %):

| sweep | free shifting | 12 V x 20 A x 0.5 s |
|---|---|---|
| upshift (D = 10) | 22 km/h, 9.8620 kWh, 136 shifts | **18 km/h**, 9.9068 kWh, 136 shifts |
| downshift (U = 22) | 10 km/h, 9.8620 kWh, 136 shifts | **7 km/h**, 9.8877 kWh, **62 shifts** |

The downshift answer is the one to read: charging for shifts more than halves the shift
count the optimiser is willing to buy. It does **not** overturn §7h — the whole
gear-selection prize is still under 0.5 % — but the schedule is now paying a real price
for chatter instead of a made-up one.

### I²R removed

`Electrical.pack_resistance` is gone and `_terminal_power()` returns its argument. The
reason is that it decided nothing: a whole-pack loss scales with total current, so it
shifts every candidate schedule by nearly the same amount, and it required an internal
resistance nobody has measured on this pack. `Electrical.voltage` stays as an
informational field. `energy_breakdown()`'s `pack` term is replaced by a **`shift`** term
(actuator + cut), which is the term that actually separates candidates.

Verified by A3 in `verify_model.py` (4 checks): 120 J derived, charged exactly
`n_shifts x V x I x t`, linear in current, and the pack returns demand unchanged.
40/40 checks still pass.


## 8. Fix order (first four change the answer)

1. **Enforce `downshift < upshift` in every sweep**, not just the combined grid. This alone deletes the headline result.
2. **Widen the search until the optimum is interior** — or state plainly that the model prefers a single ratio of 11 and the two-speed box earns nothing.
3. **Enforce the constant-power envelope**: replace flat ±45 with `min(45, 9550·11/n)`.
4. **Add grade cases (0/5/10%) and a shift cost** so gear 1 has a job and shifting isn't free.
5. Repair execution order and define the signed map (fold the shims into the real notebook).
6. Fix the efficiency fallback: normalize both axes, keep genuine low-efficiency cells, flag rather than fill above-envelope points.
7. Filter speed before differentiating; report the sensitivity — that number is the error bar.
8. Report **Wh/km**, distance, time-in-gear, energy-weighted mean efficiency, and a load-factor histogram (that last chart makes the oversizing self-evident).
9. State the error budget; stop quoting 6 decimals (8.64 kWh, not 8.642576).

---

## 9. Code hygiene notes

- Section numbering is broken: two `## 13`, two `## 16`, no `## 7`. Cell 40 is a heading with no code.
- Cell 32 and cell 39 case 1 **run the identical sweep twice** — two sources of truth for one number.
- Four names for one mask: `active_points`, `active`, `comparison_active`, `active_local`.
- `best_shift_speed` assigned in cells 37 and 42; `valid_results` in 36 and 37.
- `peak_torque`/`peak_power_kw` hardcoded in cells 16 and 19, duplicating `motor_max_torque`.
- `efficiency[~active] = 1.0` is harmless for energy but `selected_motor_efficiency` is returned with those 1.0s embedded — anyone averaging it gets a wildly optimistic number.
- `high_eff_threshold = max(0.85, nanpercentile(eff_map, 75))` is a percentile over **grid cells** on a 39%-NaN array. Change the map resolution and the "high-efficiency zone" moves. Use a fixed threshold.
- `plt.tight_layout()` and `constrained_layout=True` are mixed across figures; they conflict.

## 10. Plot notes

- The sweep plot puts its red "minimum" marker on the leftmost point and nothing signals that it's a boundary. Draw the search bounds; warn when the optimum is within one grid step of an edge.
- Cells 14, 16, 42 plot all 28 368 samples across the figure width — ~30 samples/pixel, solid ink. Keep the overview, add a zoomed inset on a 300–600 s window.
- Cell 28 draws 4 labelled shift arrows per panel; the 20 km/h panel actually has 1384 upshifts. Put the true count in the title.
- Cell 26 uses `plasma` points on `viridis` background (reads well); cell 16 uses `viridis` on `viridis` (does not).
