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


## 7l. Why three analyses named three different winners

Reported from the app: *Efficiency-only* said **22/16** best on efficiency and **20/14**
best on energy, while the *Downshift sweep* said **4 km/h**. Three answers, one cycle.
Two causes, one of them a real defect.

### Cause 1 - the efficiency panel was running a different physics (fixed)

`_work()` built a private `ShiftCost(0.0, 0.0, inf)` for *Efficiency-only* and passed that
instead of the user's. The reasoning was that shift costs should not pollute an efficiency
ranking - but `mean_efficiency` is shaft output over electrical input across the motoring
samples, and **neither the actuator energy nor the traction cut appears in either
integral**, so they cancel out of it already. Forcing them to zero changed nothing about
the efficiency ranking and put the **net-energy figures the same panel prints** on a
different footing from every sweep. That is the inconsistency, and it was real:

| | best efficiency | best energy |
|---|---|---|
| free shifting (what it used to do) | 22/11, 9.8627 kWh | 22/10, **9.8620 kWh** |
| the real 120 J + 0.5 s cost | 22/11, 9.9465 kWh | 18/7, **9.8841 kWh** |

The panel now uses the same `cost` object as everything else. Same efficiency answer,
comparable energy numbers.

### Cause 2 - the argmins were never distinguishable in the first place

They are three different searches, and the objective is flat across all of them:

| analysis | argmin | net kWh | indifference band |
|---|---|---|---|
| Upshift sweep (D held at 10) | 18/10 | 9.9068 | U 18-22 |
| Downshift sweep (U held at 22) | 22/7 | 9.8877 | **D 4-9, spread 0.0 Wh** |
| Combined grid | 18/7 | 9.8841 | U 18-23, D 4-9 |
| Efficiency-only (same grid) | 22/11 by efficiency | 9.9465 | U 18-23, D 4-9 - identical |

**Downshift 4 through 9 are bit-identical at every upshift** - the threshold never changes
the gear sequence there (section 7h), so "4" and "7" are the same answer, not two answers.
Upshift 18-23 spans 6 Wh of 9,884 Wh: **0.06 %**. An argmin quoted off that is the last
digit talking.

### The fix - quote the band, not the point

`SweepResult.indifference()` returns every candidate within 0.1 % of the best (well under
the +-4 % differentiation error bar C7b measures). Every summary in the app now prints it
under **WHAT THE OBJECTIVE CAN ACTUALLY RESOLVE**, so a reader sees the flat region rather
than inferring a contradiction from three argmins. The 1-D sweeps additionally state which
threshold they are holding fixed and that their answer is conditional on it.

`best_by_energy()` is now the single tie-break for "the energy optimum" of a 2-D candidate
set - among exact ties, widest hysteresis band, then middle upshift. `DataFrame.idxmin()`
returned whichever tie came first in row order, which is why the panel said 18/4 where the
grid said 18/7 for one identical number.

Verified by C9 in `verify_model.py` (4 checks): the efficiency panel's energy optimum IS
the combined grid's, both report the same band, a 1-D sweep held inside the band answers
inside the band, and the whole disagreement spans 0.036 % of the cycle. 44/44 pass.

**What to actually do with this:** stop reading the argmin. The energy objective has no
opinion between upshift 18 and 23, and none at all about the downshift below 9. Section 7j
already settled the upshift on tractive-force continuity (**20 km/h**); the downshift
should be set on shift count and driveability, and 7-9 km/h keeps the band wide while
sitting in the flat region.


## 7m. The shift cost has to reach every view - including the ceiling

Asked directly: *is the shift loss in the efficiency-map analyses too? it will definitely
be there.* It was not, in the one place it mattered most, and the omission was inverting
the study's headline claim.

### The audit

| view | uses the cycle? | carries the shift cost |
|---|---|---|
| Single strategy, all three sweeps, Efficiency-only, Points on map, Shift movement | yes | yes |
| Acceleration run | yes | yes (`interrupt_s` of zero traction + `energy_per_shift`) |
| Energy bins | yes | **in the totals, not in the cells** - see below |
| **`oracle_bound()` - the CEILING in every sweep summary** | yes | **no. Fixed.** |
| Efficiency map, Gear comparison, Gradeability, Optimal gear map | no cycle, no schedule | nothing to charge |

### The defect: a free-shifting ceiling quoted against cost-paying schedules

Every sweep summary prints the oracle - the better ratio chosen at every sample with
perfect foresight - as the bound on the whole optimisation. It was computed with shifting
free while the schedules beside it paid 120 J and 0.5 s per change. Not comparable, and it
flattered the oracle enormously, because **reaching that number takes 1,610 gear changes**:

| strategy | net kWh | gear changes |
|---|---|---|
| always the low ratio | 10.2443 | 0 |
| always the high ratio | **9.8765** | 0 |
| best real schedule 18/7 | 9.8841 | 62 |
| perfect per-sample choice, free shifting | 9.8321 | 1,610 |
| **the same, charged its own actuator energy** | **9.8858** | 1,610 |

1,610 x 120 J = **53.7 Wh of actuator against a 44.4 Wh prize.** The prize goes from
+44.4 Wh to **-9.3 Wh**: clairvoyant per-sample gear selection loses to simply staying in
ratio 11. And that is the generous reading - only the actuator is charged, never the
1,610 x 0.5 s of lost traction, which keeps `oracle_charged` a true lower bound on what
per-sample selection costs.

`oracle_bound()` now returns `oracle_shifts`, `oracle_shift_wh`, `oracle_charged` and
`prize_charged_wh`, and the CEILING block prints a gear-change count on every row - which
is what makes the rows comparable at all. The single-ratio baselines make zero changes and
so pay nothing; that is the correct comparison, not an omission.

### The conclusion this changes

Section 7e found the second ratio worth 0.2 Wh over 110 km - indistinguishable from zero,
with free shifting. Charge the hardware and it goes **negative**: the best schedule
(18/7, 62 changes, 19.2 Wh of shift cost) is **-7.6 Wh against just staying in ratio 11**.

> **The two-speed box does not pay for itself on this duty at any shift schedule, and now
> the ceiling says why: the ratio choice cannot pay for the act of changing ratio.**
> The case for gear 1 remains gradeability and launch (7d, 7f), not energy.

### Where the cost cannot be drawn, and why

The **Energy bins** view maps (rpm, torque) cells of the motor. The actuator runs off its
own 12 V supply and the traction cut is the *absence* of an operating point, so neither has
a cell to live in. Rather than let the totals silently disagree with every other view, the
panel now reconciles them:

```
  binned into cells        8.6800 kWh   (traction only)
  auxiliary load           1.1820 kWh   (no rpm, no torque)
  shift actuator + cut     0.0528 kWh   (136 changes)
                           ---------
  consumed (headline)      9.9148 kWh
```

C10d verifies that identity closes to **-0.0 Wh**.

**Optimal gear map** is drawn from the efficiency map alone: it says which ratio is better
*at* a point and is silent on what changing to it costs. A controller tracking that
boundary exactly *is* the oracle - ~1,600 changes. The panel now says so, so the map is
read as where a 2-D schedule could help, never as a controller.

Verified by C10 in `verify_model.py` (5 checks): the oracle is priced for its own changes,
charging can never flatter it, the single-ratio baselines pay nothing, consumed reconciles
against binned + aux + shift, and the penalty is exactly proportional to the change count.
49/49 pass.


## 7n. The 1-D sweeps were anchored on a typed number

From a real-data run: the downshift sweep (upshift held at 22) said **22/4**, the upshift
sweep (downshift held at 10) said **27/10**, and the combined grid said **21/4**. Three
answers. The reason is in the two parentheses: **each sweep held the other threshold at
whatever was typed in the Thresholds box**, and neither typed value was where the other
sweep said it should be. So the two sweeps were not two views of one question - they were
answers to two different questions, neither of which anyone had asked.

### The fix - anchor on the fixed point, not on a box

`fixed_point_thresholds()` alternates the two sweeps - optimise the upshift at the current
downshift, then the downshift at that upshift - until the pair stops moving. Each threshold
is then optimal **given** the other, which is precisely the property a typed anchor lacks.
Two 1-D sweeps per round, so it costs a fraction of the grid.

On the stand-in data (grid optimum 18/7):

| typed in the box | old: upshift sweep | old: downshift sweep | new anchor | new: both sweeps |
|---|---|---|---|---|
| 22 / 10 | 18/10 (9.9068) | 22/7 (9.8877) | 18/7 | **18/7 (9.8841)** |
| 32 / 22 | 31/22 (10.1537) | 32/7 (9.9858) | 18/7 | **18/7 (9.8841)** |
| 15 / 5 | 18/5 (9.8841) | **no feasible candidate** | 18/7 | **18/7 (9.8841)** |

Six answers become one, and it is the grid's. The third row is the sharpest illustration:
a typed upshift of 15 km/h is itself **rejected by the acceleration gate**, so the whole
downshift sweep anchored on it had nothing feasible to report. A fixed-point anchor cannot
do that - it is by construction an optimum of a feasible set.

The control is **SWEEP ANCHOR -> Self-consistent anchor (fixed point)**, on by default.
Untick it to hold the Thresholds value, which is still the right thing when the question
really is "what is the best downshift *for this specific upshift*". The summary now says
which anchor was used, where it came from, and how many rounds it took.

Where coordinate descent and the grid disagree, `matches_grid` is the honest signal that
the objective has structure a coordinate search cannot see - use the grid then.

### Three reporting defects the same run exposed

| what it printed | why it was wrong |
|---|---|
| `Sweep over upshift` above a 645-candidate two-threshold search | the **combined grid** was calling `_sweep_summary(sw, "upshift")`. Now titled `Sweep over both thresholds`. |
| `net 1.1188 kWh` then `spread across feasible set: 1.1691 - 1.1803` | the best was quoted on **net**, the spread on **consumed**. With regen on those differ by 50 Wh. The spread is now net - the quantity `_objective()` actually ranks on. |
| `-> -12 % of the gain is lower motor loss, i.e. the operating points genuinely moved into a better part of the map` | the motor-loss column showed the optimum losing **more** in the motor (-1.4 Wh) while the shift term carried the whole result (+11.4 Wh). A negative percentage printed next to a sentence contradicting it. |

That last one was hiding a real finding. On that cycle the downshift optimum **did not win
on the efficiency map at all** - it won by shifting less, and its mean efficiency was
*lower* than the schedule it beat (92.56 % against 92.68 %). The block now finds the
dominant term and names it:

> `-> 102 % of the +11.2 Wh comes from fewer/cheaper gear changes.`
> `The optimum did NOT win on the efficiency map - it won by shifting less`
> `(18 changes against 30). The motor term went -1.4 Wh, i.e. the map is against it.`

Verified by C11 in `verify_model.py` (4 checks): the fixed point is reached from every
starting anchor, both 1-D sweeps then return the same pair, that pair is inside the grid's
indifference band, and a typed anchor can be infeasible where the fixed point cannot.
53/53 pass.


## 7o. "Efficiency peaks at 16 but energy is minimum at 4" - and where the penalty went

Two questions from the same run: *why does the downshift with the best efficiency not have
the least energy*, and *it is not clear the penalty is active with more or fewer shifts*.
They have one answer, and the tool was not showing it.

### The mechanism

Efficiency and shift count rise **together**. Raising the downshift threshold keeps gear 1
engaged over more of the cycle, which the map likes - and makes the controller cross the
threshold far more often. On the stand-in data, downshift sweep at upshift 18:

| downshift | changes | shift cost | motor loss | mean eff | net |
|---|---|---|---|---|---|
| 4-9 | 62 | 19.2 Wh | 650.5 Wh | 92.509 % | **9884.1 Wh  <- least energy** |
| 10 | 136 | 42.9 | 647.6 | 92.541 % | 9906.8 |
| **11** | **220** | **69.3** | **647.6** | **92.542 %  <- best efficiency** | 9935.1 |
| 12 | 300 | 94.4 | 651.9 | 92.498 % | 9966.0 |
| 15 | 542 | 171.2 | 689.7 | 92.101 % | 10083.9 |

Moving from the efficiency optimum (11) to the energy optimum (4) **costs 7.8 Wh in the
motor and saves 60.2 Wh in shift cost** - 220 gear changes down to 62. Net **-52.3 Wh**.

**The map prefers the busier schedule; the battery refuses to pay for it.** Nothing is
inconsistent - the two optima answer different questions, and the shift term is the whole
difference between them. Same story on the upshift side, smaller: best efficiency 22,
least energy 18, worth -2.5 Wh.

### Why it was invisible

The sweep summary quotes the **winner's** shift count and nothing else. How the count moves
across the sweep - the one number that decides this - never appeared anywhere. So the sweep
read as if it were ranking on the efficiency map, and the penalty looked inert.

### New analysis: `Loss breakdown`

Both thresholds, each anchored at the self-consistent pair (7n), so the two columns are
slices through **one** operating point. Three rows per column:

| row | what it shows |
|---|---|
| 1 | the four **losses** stacked - gearbox, motor, auxiliary, shift. Road work is excluded and its value put in the title: at ~7,800 Wh it is 8x every loss combined and barely moves, so stacking it flattens everything that does. The green shift band visibly grows with the threshold. |
| 2 | **the answer.** Shift cost against motor loss above its own minimum, on one axis, with the gear-change count as bars behind. Whichever curve is higher decides. The dashed line is the energy optimum, the dotted line the efficiency optimum. |
| 3 | net energy (black, left) and mean efficiency (red, right), each optimum marked. When the dot and the cross are not at the same x, row 2 says why. |

The text summary prints the full per-candidate table - threshold, changes, shift Wh, motor
Wh, mean efficiency, net Wh - and then states the trade in Wh, naming the mechanism that
actually moved:

* fewer changes -> "the cheaper threshold makes N fewer gear changes";
* **same** count, different cost -> "the cut is charged at the power actually flowing when
  each change happens, so this is a shift-TIMING effect, not a shift-count one";
* identical shift cost -> "read this one as a genuine efficiency-map result".

`sweep_energy_terms()` in `shift_core.py` does the work: it re-runs each feasible candidate
with `keep_arrays=True` and returns the five terms plus the change count. 0.3 s for a 1-D
sweep; capped by `max_candidates` for anything larger.

Verified by C12 in `verify_model.py` (3 checks): the five terms sum to the battery total to
0.000 uWh, the shift cost tracks the change count (rank correlation 0.997) and never dips
below the 0.033 Wh actuator floor, and the efficiency optimum is provably not the energy
optimum with the shift term accounting for the gap. 56/56 pass.

> Note the shift cost is **not exactly** proportional to the change count: the actuator
> part is, but the traction cut is charged at the power flowing when each change happens.
> Three candidates with 710 changes each cost 271.8, 270.1 and 270.7 Wh. That is physics,
> not noise, and C12b asserts the correlation rather than a false proportionality.


## 7p. "Never converged", and a build stamp so stale output announces itself

### The output was three builds old

The same summary was reported three times as evidence of new problems. It could not have
come from the current code: it says `Sweep over upshift` above the 645-candidate
two-threshold grid (fixed in 6fbd403), prints `-12 % of the gain is lower motor loss`
(fixed in 6fbd403), lacks the `ANCHOR:` block (added in 6fbd403), and contains no
`Loss breakdown` at all (added in 6b0b837) - which is also why that analysis appeared "not
working": it was not in the build being run.

**Fix, so this cannot recur:** `shift_core.build_stamp()` reads `.git/HEAD` directly (no
subprocess, so a ZIP download works too) and every summary now carries it as its second
line, as does the window title:

```
Sweep over downshift
==========================================================================
  shift_core 2026-08-24 (build 6b0b837)
```

Any output pasted anywhere now names the code that produced it.

### The one real defect in that output: CONVERGED was lying

`CONVERGED : False` with `optimum 4 km/h sits on the lower edge - widen the range`, printed
directly above `5 of 16 feasible candidates are within 1.1 Wh (0.1 %) of the best, spanning
downshift 4-8`. Both cannot be true. If five candidates including the edge are
indistinguishable, the search did not fail - the objective is flat.

The cause: `_finish()` tested `len(tied) == 1`, where `tied` means **bit-identical** energy
(`_TIE_REL = 1e-9`). On the synthetic data whole blocks tie exactly, so the test worked. On
real data nothing ever ties exactly - every candidate differs in the last digit - so
`len(tied) == 1` is always true and **every** edge optimum was declared a failed search.
That is the common case on real data and it is not a failure.

An edge optimum is a boundary problem only if **the objective is still falling towards that
edge**. Both `_finish()` and `sweep_grid()` now test that instead:

| case | verdict |
|---|---|
| upshift [18, 22], 5 candidates within 9.9 Wh spanning the range | **converged** - "FLAT to that edge, not too narrow a search" |
| upshift [24, 42], only 2 candidates close, curve rising away | **NOT converged** - "still falling towards it - widen the range" |
| upshift [8, 42], interior optimum | converged, no note |

A plateau has to be real to count: at least three candidates spanning at least two steps,
or the whole range. Two adjacent points being close is not a flat curve - every smooth
curve is flat over one step near its minimum, and accepting that would have turned the
warning off everywhere.

### On "different analysis, different results"

Already answered by 7l (one cost model, indifference band) and 7n (fixed-point anchor); the
output above predates both. With the current build, on the same data, the two 1-D sweeps
and the grid return the same pair from any starting threshold - C11 asserts it.

Verified by C13 in `verify_model.py` (5 checks): flat-to-edge converges, still-falling does
not, an interior optimum is clean, a two-point run does not count as a plateau, and every
summary can name its build. 61/61 pass.


## 7q. A dead menu entry, and a share that read 116 %

Two defects, both mine, both from the run on build `f7ac04e`.

### 1. `Loss breakdown` crashed the moment it was selected

```
File "shift_app.py", line 609, in _refresh_checklist
    need = NEEDS[self.analysis.get()]
KeyError: 'Loss breakdown'
```

An analysis name keys **four** things - `ANALYSES` (the menu), `NEEDS` (what data it
requires), `DRAW` (how to plot it) and a branch in `_work()`. The new analysis was added to
three of them. The miss surfaced as a traceback in the console and a dead button in the UI,
which is why it looked like the feature "still is not working" after pulling: it genuinely
was not.

Registered, and made structurally impossible to repeat:

* `shift_app` now raises at **import** if any analysis is missing a `NEEDS` or `DRAW` entry,
  or if `DRAW` names a method that does not exist - so the failure happens on `import`,
  loudly, instead of inside a Tk click handler;
* both lookups fall back to `("cycle", "map")` rather than raising, so a future slip
  degrades instead of killing the handler;
* `WORK_EXEMPT` declares the one analysis that legitimately has no `_work` branch
  (`Efficiency map` needs no computation), so the exemption lives in the code rather than
  in a test's assumptions;
* **C14** in `verify_model.py` imports the app and checks all four registries for every
  analysis. No physics check could have caught this; the suite had no UI-wiring check at
  all.

### 2. "116 % of the +10.8 Wh comes from fewer/cheaper gear changes"

A share cannot exceed 100 %. The five terms from `energy_breakdown()` are **consumed**-side,
but the denominator was the **net** difference. With regen off those are identical, which
is why it never showed on the synthetic runs; with regen on the two schedules also recover
different amounts, so the table stopped summing to its own headline.

Now attributed against the consumed difference the terms actually sum to, and reconciled to
net explicitly:

```
    sum = consumed                9.8016   10.1394    +337.8 Wh
    recovered (regen)             0.5022    0.5068      -4.6 Wh
    NET                           9.2994    9.6327    +333.3 Wh
```

The verdict sentence was also contradictory when the motor term happened to agree with the
winner - it said "did NOT win on the efficiency map" while showing a motor term that
favoured it. It now separates the two cases: the map against the winner (a pure shift-count
result) or the map with it (both effects the same way, shift count the larger).

### On "efficiency rises with the downshift speed but so does energy"

That is the correct answer, not a bug, and it is the same finding as 7o. Raising the
downshift keeps ratio 19 engaged over more of the cycle, which the map likes by a fraction
of a point, and it multiplies the number of gear changes. Measured on the reported run,
upshift 21:

| | D = 4 | D = 18 |
|---|---|---|
| mean efficiency | 92.54 % | **92.70 %** (better) |
| gear changes | **18** | 50 |
| motor loss | 82.1 Wh | **80.3 Wh** (better) |
| shift cost | **6.6 Wh** | 19.2 Wh |
| net | **1.0663 kWh** | 1.0771 kWh |

The map gives back **1.8 Wh** and charges **12.5 Wh** to collect it. `mean_efficiency` is
the motor's share alone - the actuator and the traction cut are not inside that ratio - so
the two curves *must* be allowed to move in opposite directions. The sum is verified: on
the stand-in data the five terms reconcile against the measured total to **0.29 Wh**.

Verified by C14 (registry wiring). 62/62 pass.


## 7r. Efficiency rising with the downshift speed is the MAP talking

Reported as a fault: in the downshift sweep the threshold speed and the mean efficiency
rise together, when they were expected to move opposite ways.

### Two things were checked before answering

**Is the ratio being inflated by dropping samples?** `mean_efficiency` is
`sum(shaft out) / sum(electrical in)` over samples where the map returns a value. A
schedule that pushed samples into an unmappable region would score higher simply by
excluding them - the ratio would rise for no physical reason. Measured across four
schedules: **0.0000 uWh of motoring energy is uncounted**. The ratio covers everything.
(C15b.)

**Where do the two ratios actually swap places?** That single number decides the direction
of the curve, and `ratio_crossover()` now reports it:

| load | ratio 19 better below | above that |
|---|---|---|
| 0.0 m/s2 (cruise) | **7.0 km/h** | ratio 11 |
| 0.3 m/s2 | **19.2 km/h** | ratio 11 |
| 0.6 m/s2 | **31.0 km/h** | ratio 11 |

**The crossover walks up with load - 7 to 31 km/h on this map.** That is the whole answer.

### Why the curve rises

Raising the downshift threshold hands more of the low-speed cycle to ratio 19. If the
threshold is still *below* the crossover, those samples move into the genuinely better
ratio and mean efficiency **rises**. Past the crossover the same move costs efficiency and
the curve turns over. On the stand-in map that turn happens around 11-12 km/h; a cycle with
more low-speed acceleration sits nearer the 19-31 km/h crossovers and will keep climbing
much further - which is what the reported run shows, and it is correct.

It is a property of **the map and the cycle's load mix**, not of the code. A dyno map whose
ridge sits low will show the opposite slope, and that will also be correct.

Both sweep summaries and the efficiency-only panel now print the table above under
**WHY THE EFFICIENCY CURVE SLOPES THIS WAY**, so the slope is explained next to the curve
instead of looking like a defect.

### And it still does not decide the energy answer

This is the point 7o and 7q keep making, now with the mechanism attached: efficiency is the
**motor's share alone**. The actuator energy and the traction cut are outside that ratio -
neither appears in the numerator or the denominator. So the efficiency curve and the energy
curve are free to move in opposite directions, and on this duty they do: the map gives back
about 1.8 Wh and charges about 12.5 Wh in gear changes to collect it.

> **Efficiency rising with the downshift speed and energy rising with it are both true at
> once.** The first is the map preferring the low ratio at low speed under load; the second
> is what it costs to keep changing into it.

Verified by C15 in `verify_model.py` (2 checks). 64/64 pass.


## 7s. The real N603 data, and why two cycles over one map slope opposite ways

`Book2.xlsx` arrived with the cycle on Sheet1 and the dyno map on Sheet2. Split into
`real_data/` and loaded directly. It matches every diagnostic the original notebook
printed, so the synthetic stand-in of section 6 is retired for anything quantitative:

| | real data |
|---|---|
| cycle | 28,368 samples, 28,367 s, **111.35 km**, max 42.26 km/h |
| map | 42 x 901, peak **94.13 %** at 4260 rpm / 13.0 Nm, 14,091 blank cells |

`load_cycle()` now accepts `.xlsx` as well as `.csv` - that is how the logs actually
arrive, and the CSV conversion step is where a column gets renamed or a decimal comma
gets eaten. `verify_model.py` prefers `real_data/` and falls back to the stand-in.
**All 67 checks pass on the real data.**

### The screenshots were a different cycle

The two photographed panels carry the caption `cycle City cycle.csv (12 km, 0.5 h)`.
That is not the 111 km N603 log - it is a short dense city cycle. Which cycle is loaded
is the whole reason the efficiency-vs-downshift curve looked wrong:

| | City cycle (12 km) | real N603 (111 km) |
|---|---|---|
| efficiency peaks at downshift | ~15-16 km/h | **7 km/h** |
| efficiency at D = 4 -> 18 | rises | **falls** (92.01 % -> 91.43 %) |

Same map, opposite slopes, both correct. The reason is that the ratio crossover walks up
with load - on this map 6.9 km/h at cruise, 21.1 at 0.3 m/s2, 28.3 at 0.6 - and a short
city cycle spends far more of its low-speed energy accelerating than a long mixed one.

### `effective_crossover()` - ask the cycle, not a nominal load

`ratio_crossover()` gives the map's opinion at a steady load. `effective_crossover()`
asks which ratio the map prefers **at every motoring sample of the loaded cycle**,
aggregated per speed band and weighted by the shaft energy in that band - because a band
the vehicle passes through in two seconds cannot decide anything. On the real N603 cycle:

| band | share of energy | low ratio wins | low ratio gain |
|---|---|---|---|
| 4-6 km/h | 0.5 % | 96.9 % | **+3.87 pts** |
| 6-8 | 0.8 % | 95.2 % | +2.62 |
| 8-10 | 1.5 % | 89.6 % | +1.20 |
| **10-12** | 3.1 % | 70.8 % | -1.88 |
| 12-14 | 7.0 % | 46.6 % | -3.37 |
| 18-20 | 12.3 % | 31.0 % | -4.54 |
| 30-32 | 3.4 % | 0.0 % | -6.32 |

**The low ratio stops winning above about 12 km/h, and everything below that carries
6.2 % of the motoring energy.** That is the downshift threshold's entire playing field.
Both sweep summaries and the efficiency-only panel now print this table, so the slope is
explained next to the curve with the loaded cycle's own numbers.

### The answer on the real data

Grid over upshift 8-42, downshift 2-30, minimum band 3 km/h, 120 J + 0.5 s per change:

| | regen off | regen on |
|---|---|---|
| best schedule | **18 / 2** | **18 / 2** |
| net | 9.8503 kWh, 88.5 Wh/km | 9.3062 kWh, 83.6 Wh/km |
| gear changes | 72 | 72 |
| always ratio 11 | 9.8437 kWh | 9.2998 kWh |
| **two-speed box gains** | **-6.6 Wh** | **-6.4 Wh** |
| oracle prize, free shifting | +48.4 Wh | +56.9 Wh |
| **oracle prize, charged its own changes** | **-1.3 Wh** | **-22.0 Wh** |

All three searches - upshift sweep, downshift sweep, combined grid - return 18/2 from any
starting anchor, and the efficiency-only panel's energy optimum is the same point. The
efficiency optimum is 18/10 at 92.12 %, one grid block away and worth 51.6 Wh more.

> **The conclusion the stand-in reached now holds on the real N603 data: the two-speed box
> does not pay for itself on energy.** The best schedule is 6.5 Wh *worse* than staying in
> ratio 11 over 111 km, and even a clairvoyant per-sample controller goes negative once
> charged for the 1,490-2,366 gear changes it needs. The case for the low ratio remains
> gradeability and launch (7d, 7f).

Verified by C16 (3 checks): the bands account for 100.000000 % of the motoring energy, the
low ratio wins at the bottom and loses at the top, and the band where it wins carries under
a quarter of the energy. 67/67 pass **on the real data**.


## 7t. Why the downshift answer is 4 (or 1) when efficiency peaks at 10

The question, asked once more and this time worth a measurement rather than another
explanation: *the downshift sweep returns the bottom of the range even though efficiency
peaks later.* It does, and here is exactly what decides it.

Real N603 data, upshift 18, regen on, downshift 1-15:

| D | changes | under traction | actuator | traction cut | mean eff | net |
|---|---|---|---|---|---|---|
| **1** | 64 | 33 | 2.13 Wh | 13.5 Wh | 91.941 % | **9305.7 Wh** |
| 4 | 116 | 60 | 3.87 | 26.4 | 92.033 % | 9313.7 |
| 7 | 184 | 93 | 6.13 | 42.0 | 92.101 % | 9327.4 |
| **10** | 308 | 169 | 10.27 | 68.2 | **92.125 %** | 9357.8 |
| 15 | 604 | 338 | 20.13 | 131.0 | 91.892 % | 9456.3 |

From D = 1 to D = 10 the map gives back **0.18 points** and the traction cut takes
**54.7 Wh**. The actuator - the measured 120 J - contributes 8 Wh of that and decides
nothing.

### The measurement that settles it

Re-run the same sweep under different shift-cost models and watch where each optimum lands:

| shift-cost model | best downshift | net |
|---|---|---|
| free shifting | 8 | 9278.3 Wh |
| actuator only, 120 J, no cut | 7 | 9285.4 |
| actuator + 0.10 s cut | 5 | 9292.5 |
| actuator + 0.25 s cut | 2 | 9298.6 |
| **actuator + 0.50 s cut (shipped)** | **1** | 9305.7 |
| *efficiency peak* | *10* | *unmoved by any of them* |

**The optimum walks 7 km/h across these models while the efficiency peak does not move at
all.** So the gap between the two is not the map contradicting itself - it is the price of
changing gear, and specifically the **traction cut**.

That matters because the cut is **the least certain input in the whole study**. The
actuator is measured hardware from the vehicle team; the 0.5 s of lost traction is an
assumption about how the controller behaves. Anyone quoting a downshift speed should pin
that number down first. `shift_cost_sensitivity()` computes the table and both sweep
summaries print it under **IS THIS A MAP RESULT OR A SHIFT-COST RESULT?**

### What it does not change

The whole span, free shifting to a full 0.5 s cut, is **27.4 Wh over 111 km - 0.30 %**. It
moves the recommended threshold; it does not move the conclusion that the second ratio does
not pay for itself (7s).

And an optimum at the bottom of the range is not a failed search here - it is a statement:
**downshift only just before stopping.** Which is what a real AMT does anyway. It drops to
the low ratio to be ready to launch, not to save energy on the way down - and the
gradeability and launch case for the low ratio (7d, 7f) is unaffected by any of this.

Verified by C17 in `verify_model.py` (3 checks): the optimum walks with the cut while the
efficiency peak stays fixed, charging more per shift never lowers the energy, and the whole
span is under 0.30 % of the cycle. 70/70 pass on the real data.


## 7u. Reading the Loss breakdown, and where every input lives

### The five terms

Every watt-hour drawn from the pack lands in exactly one of these. They are computed by
`energy_breakdown()` and they sum to `consumed` by construction (C10d verifies it to
-0.0 Wh). On the real N603 cycle at 18/2 with regen on:

| term | kWh | share | what it is | what sets it |
|---|---|---|---|---|
| **road work** | 7.7166 | 79.3 % | the work the road actually demands - inertia, rolling, aero, grade | **Vehicle**: Mass, Cd x A, Crr, Road grade, Wheel radius (plus **Motor** > Rotor inertia, which is gear-dependent) |
| **gearbox loss** | 0.2387 | 2.5 % | mesh loss between motor and wheel | **Gearbox**: Efficiency gear 1 / gear 2 |
| **MOTOR loss** | 0.6956 | 7.1 % | what the efficiency map costs at the operating points the schedule creates | the map itself, plus **Gearbox** ratios and the **Thresholds** - this is the only term a shift schedule can move |
| **auxiliary** | 1.1820 | 12.1 % | constant hotel load x cycle duration | **Electrical** > **Auxiliary load [W]** |
| **shift actuator + cut** | 0.0176 | 0.2 % | 120 J per change, plus traction lost while the actuator moves | **Shift cost** > Actuator voltage, Actuator current, Shift duration |
| = consumed | 9.7367 | 100 % | | |
| recovered (regen) | -0.4305 | | what the motor puts back while braking | **Regen** section |
| **= NET** | **9.3062** | | **83.6 Wh/km over 111 km - the objective** | |

**Two things fall straight out of that table and neither is about shifting.**

The auxiliary load is **1.70x the entire motor loss**. 150 W for 7.88 h costs 1.18 kWh
against 0.70 kWh for every inefficiency in the motor at every operating point of the cycle.
Halving the hotel load is worth about ten times more than any shift schedule.

The shift cost is **0.18 % of consumed**. It decides the threshold (7t) and nothing else.

### The panels

Two columns - upshift sweep, downshift sweep - both anchored at the self-consistent pair
(7n), so they are slices through one operating point rather than two unrelated searches.

**Row 1 - share of the losses.** The four loss terms stacked. Road work is deliberately
excluded and its value put in the title: at ~7,800 Wh it is 8x every loss combined and
barely moves with the threshold, so stacking it flattens everything that does. Watch the
green band - that is the shift cost, and it grows with the threshold.

**Row 2 - the trade, and the panel that answers the recurring question.** Shift cost
against motor loss *above its own minimum*, on one axis, with the gear-change count as
bars behind. Whichever curve is higher decides the answer. Dashed vertical line = the
energy optimum, dotted = the efficiency optimum. When they are apart, this panel is why.

**Row 3 - the two optima side by side.** Net energy (black, left axis) and mean motor
efficiency (red, right axis), each with its own optimum marked. The dot and the cross
being at different x is not a fault - see 7o and 7t.

### Where every input lives

Left panel of the app, top to bottom. `Run analysis` re-simulates; the display controls
below only redraw the cached result.

| section | fields |
|---|---|
| **Vehicle** | Mass [kg], Wheel radius [m], Cd x A [m^2], Rolling resistance Crr, Road grade [deg] |
| **Motor** | Peak torque [Nm], Peak power [W], Max speed [rpm], Rotor inertia [kg.m^2] |
| **Gearbox** | Ratio 1 (low), Ratio 2 (high), Efficiency gear 1, Efficiency gear 2 |
| **Electrical** | Pack voltage [V], **Auxiliary load [W]**, Battery power limit [W] |
| **Regen** | Regen enabled (0/1), Brake torque to motor [0-1], Charge power limit [W], Blend out below [km/h] |
| **Shift cost** | Actuator voltage [V], Actuator current [A], Shift duration [s], Max shifts per hour, Min hysteresis band [km/h], Min accel reserve [m/s2], Max accel given up [0-1] |
| **Thresholds** | Upshift [km/h], Downshift [km/h] - the single-strategy pair, and the starting anchor for the sweeps |
| **Sweep range** | Upshift from/to, Downshift from/to, Step, Minimum band |
| **Energy bins** | Bin size [rpm], Bin size [Nm], Reference (0=g1, 1=g2, 2=custom), Compare upshift/downshift |
| **Acceleration run** | Target speed [km/h], Throttle [0-1] |
| **Numerics** | Smoothing window (0=off) - the error-bar control of C7 |

Below the parameters, the display controls: **SWEEP ANCHOR** (self-consistent anchor, 7n),
**SIGNALS**, **TIME WINDOW**, **EFFICIENCY-MAP DISPLAY**, and the **Plot height** slider.
None of those change a number - they only change what is drawn.

**So: auxiliary load is Electrical > Auxiliary load [W], default 150.** It is the second
largest term in the whole budget and the one nobody has questioned yet.

### Percentage shares, and three settings that were not settable

Two additions so the proportions are on screen rather than in a note, and so the inputs
that move them can actually be moved.

**Shares everywhere.** Single strategy now prints an **ENERGY BUDGET** block - each term in
kWh, as a **share of consumed**, in **Wh/km**, and the field that sets it - then says what
the shares mean:

```
  ENERGY BUDGET - every Wh, and what sets it
    term                        kWh   share    Wh/km   set by
    road work                7.7166   79.3%     69.3   Vehicle: mass, CdA, Crr, grade
    gearbox loss             0.2387    2.5%      2.1   Gearbox: efficiency gear 1 / 2
    MOTOR loss               0.6956    7.1%      6.2   the map + where the schedule puts it
    auxiliary load           1.1820   12.1%     10.6   Electrical: Auxiliary load [W]
    shift actuator + cut     0.0176    0.2%      0.2   Shift cost: V x A x s, + the cut
    = consumed               9.7367  100.0%     87.4
    recovered (regen)       -0.4305   -4.4%     -3.9   Regen section
    = NET (the objective)    9.3062   95.6%     83.6
```

Loss breakdown gained a share column on both the shift and motor terms in its table, a
**SHARE OF CONSUMED** block at the optimum, and the share printed **in the stack legend**
so the proportion is readable off the plot without measuring it.

**Three settings promoted to inputs:**

| new field | why it matters |
|---|---|
| **Shift cost > Traction cut [s]** (`-1` = same as shift duration) | 7t showed this is the single most influential *uncertain* input - it walks the downshift optimum 7 km/h while the efficiency peak does not move. It was previously reachable only by changing the shift duration, which also changes the actuation energy, so the two could not be separated. Now they can, including `0` for no cut at all. |
| **Numerics > Power epsilon [W]** | decides which samples count as active at all |
| **Numerics > Min map efficiency** | screens map cells; 7e defect 3 turned on this number |

Verified by C18 (3 checks): auxiliary load moves consumed by exactly `W x time`
(150 -> 300 W adds 1182.0 Wh against a predicted 1182.0), the traction cut is settable
independently of the actuator (0 s -> 0.0 Wh of cut with actuation unchanged at 3.87 Wh),
and the printed shares sum to 100.000000 %. 73/73 pass.


### Switching the two curves apart, and the total on the same picture

Net energy and mean efficiency answer different questions and, on this duty, move in
opposite directions (7o, 7t). Drawn on one pair of axes they read as a contradiction. New
section **LOSS BREAKDOWN** in the display panel, three switches, all display-only so
toggling is instant on the cached result:

| switch | default | what it does |
|---|---|---|
| **Net energy curve** | on | the black net-energy trace in row 3 |
| **Motor efficiency curve** | on | the red efficiency trace in row 3 |
| **Total consumed energy** | on | a dashed total-consumed line on row 1 |

Whichever of the two is shown **alone takes the left axis**, rather than sitting on a
twinned right axis - a lone curve on a right axis looks like half of a comparison that is
not there. The row title follows: "least energy 2 vs best efficiency 10 - 8 km/h apart"
with both on, "mean motor efficiency - best at 10 km/h" with one. Turn both off and the
row says so and points at the switches.

**Total consumed** is drawn on its own axis rather than added to the stack. Road work is
~8x every loss combined, so stacking it flattens the four terms that actually move with the
threshold - the same reason it is excluded from row 1 in the first place. On its own axis
the total is visible without hiding anything.

Verified by C19: all **8** switch combinations draw and summarise without raising. A
combination that throws inside a draw call reaches the user as a dead panel, which is
exactly how the `Loss breakdown` KeyError of 7q surfaced. 74/74 pass.

### The same two switches in every view that draws them

The switches introduced for Loss breakdown now drive **four** views, because a switch that
works in one panel and not the next is worse than no switch. Section renamed
**ENERGY / EFFICIENCY CURVES**:

| view | Net energy on | Motor efficiency on | one of them off |
|---|---|---|---|
| **Upshift / Downshift sweep** | energy on the left axis | efficiency on the twinned right | the survivor moves to the LEFT axis; the title changes to match ("best MOTOR EFFICIENCY at 10 km/h  (energy curve hidden)") |
| **Combined grid** | paints the net-energy surface, marks the energy optimum | - | with energy off it paints the **efficiency surface** and marks the **efficiency** optimum, because marking the energy optimum on an efficiency surface is exactly the mismatch these switches exist to stop |
| **Efficiency-only optimum** | energy on the twin | efficiency keeps the left axis (it is the efficiency run) | the survivor takes the left axis; the energy minimum now gets its own marker, which it never had |
| **Loss breakdown** | row 3 black trace | row 3 red trace | survivor to the left axis, row title follows |

Turning both off never yields an empty panel: the sweeps and the efficiency run fall back
to one curve, and Loss breakdown's row 3 says which switches to use.

Two things the combined grid does differently on the efficiency surface, both deliberate.
It marks the efficiency optimum rather than the energy one. And it drops the
`NOT CONVERGED` stamp - convergence is a property of the **energy** search, so on an
efficiency surface it describes nothing on screen.

**Hardening that came with it.** `_map_style()` and eleven other display reads indexed
`self.disp[...]` directly. An unknown key inside a draw path is a dead panel, which is
exactly how the `Loss breakdown` registry miss surfaced (7q). All of them now go through
`_disp()` / `_num_disp()`, which return a documented default rather than raising - and that
is also what lets the verification suite drive the real draw methods headlessly.

Verified by C19 (2 checks): all 8 switch combinations draw Loss breakdown, and all 4
combinations draw the sweeps and the combined grid. 75/75 pass.

## 7v. The real cycle is Book3, an Energy breakdown tab, and four checks that were wrong

### The cycle

`Book3.xlsx` is the drive cycle this study is about: **1,801 samples, 1,800 s, 11.81 km,
max 44.71 km/h**. It is the cycle the reported screenshots came from - they are captioned
`City cycle.csv (12 km, 0.5 h)`. `real_data/` now holds Book3 as the cycle and Book2's map.
The 111 km log on Book2 Sheet1 stays in `sample_data/` as a **second dataset**, which turned
out to be worth more than a spare: running the suite against both is what exposed the
checks below.

### The efficiency-peak "discrepancy" was a marker in the wrong place

Reported: the downshift analysis and the loss map disagree about the best speed on
efficiency. Measured first, before changing anything - **they do not**. Both give the
efficiency peak at **D = 16 km/h, 92.7206 %**, from the same anchor and the same candidate
set, with `max |difference| = 0.000e+00` across every shared row.

The defect was in the drawing. `_sweep_plot` marked the efficiency curve at
`sw.best` - the **energy** optimum - so a highlighted dot sat on the efficiency curve at a
speed that is not where efficiency peaks, while Loss breakdown marked the real peak. Two
markers, two speeds, one number. And the sweep never named the efficiency peak anywhere at
all, so there was nothing to reconcile against.

Fixed: the sweep marks the efficiency curve at **its own maximum**, names both optima in
the title (`least ENERGY at 4 km/h ... best EFFICIENCY at 16 km/h`), and the summary states
plainly that best efficiency is a different question with its own answer and its own price:

```
  BEST MOTOR EFFICIENCY is a DIFFERENT question and lands at 16 km/h
        92.721% there, against 92.541% at the energy optimum
        net energy there 1.0762 kWh (+10.0 Wh)
```

### New analysis: `Energy breakdown`

The sweeps ask which threshold is best. This asks the question underneath - what is the
energy actually spent on - for **one** schedule. Three panels: the budget as a sorted bar
with shares, a pie of everything that is **not** road work, and the cumulative build-up
over the cycle with the auxiliary-only line beneath it. Plus the per-gear table and a list
of which input moves which term. On Book3 at 21/4:

| term | Wh | share of consumed | share of the non-road-work part |
|---|---|---|---|
| road work | 988 | **84.5 %** | - |
| MOTOR loss | 82 | 7.0 % | **42.3 %** |
| auxiliary | 75 | 6.4 % | 38.6 % |
| gearbox loss | 31 | 2.6 % | 15.7 % |
| shift actuator + cut | 7 | 0.6 % | 3.4 % |

### Four checks asserted one dataset's behaviour as an invariant

Swapping the cycle broke them, which is exactly what a second dataset is for. Each is now
written around its mechanism rather than around a number.

| check | what it claimed | why that is not an invariant | what it asserts now |
|---|---|---|---|
| **C7** | the **ordering** of schedules survives smoothing | it does not, on either cycle: the four candidates span 0.6 Wh while smoothing moves the level 117 Wh, so the argmin flips between windows. The original claim was an artifact of the synthetic data | the differentiation error bar **dwarfs** the gap between schedules - a stronger statement, and true |
| **C8b** | with no acceleration given up, the optimum **is** the crossover | where energy lands above the floor is cycle-dependent: 20 km/h on the stand-in, 27 km/h here | every upshift below the crossover is **refused**, and the first feasible one is the crossover |
| **C13** | named ranges are "flat to the edge" / "still falling" | a range that is falling on one cycle is a plateau on another | the **definition**: converged means interior, or on an edge the objective is flat to - over whatever ranges the loaded cycle provides |
| **C16c** | energy below the crossover is under a quarter of the cycle | 1.9 % on the long log, 27.4 % here, because the low ratio also wins on scattered high-load samples **above** the crossover - a speed band is the wrong container for a load-selected set | the energy-weighted invariant: the **high** ratio is the better one for most of the cycle's energy (24.3 % / 35.1 % low-ratio share) |

C7 is the one that matters beyond the suite. It had been certifying that the schedule
ranking was robust to the model's own numerical error bar. It is not - **the error bar is
larger than the entire spread between schedules** - which sharpens 7h and 7s rather than
softening them: not only is the prize small, it is smaller than the noise on the
measurement of it.

**74/74 pass on Book3 and on the 111 km log.**


## 7w. Shift movement on a short cycle: 18 arrows drawn on 1,764 points

Reported: in Shift movement every gear-2 point looks as though it came from a shift, but
they are not all drawn that way.

### What is actually on the plot

Book3 at 21/4: **18 gear changes over 1,801 samples.** Gear 1 carries 298 loaded samples,
gear 2 carries 1,466. Only **36 of those 1,466 - 2.5 %** are within one sample of a change.

Nothing was being hidden or thinned: the cloud cap is 6,000 points and both clouds are far
under it, and every one of the 18 shifts already had its arrow. The problem is that **9
arrows drawn over 1,466 markers cannot be seen**, so the eye reads the dense population as
the thing the arrows describe. The plot was telling the truth and reading as its opposite.

### Three changes

**Cloud: only near a shift** (new, off by default) with a **neighbourhood** slider,
2-60 s, default 10 s. The background cloud keeps only samples within that window of a gear
change: 1,764 loaded samples become 333 - 19 %. The window is in **seconds, not samples**,
so the control means the same thing on a 1 Hz log and on a finer one, and the legend
changes to `gear 2 near a shift (184)` so the count on screen is never mistaken for the
whole cycle.

**Number the shifts** (new, on by default) - each arrow is numbered when there are 30 or
fewer, so an arrow on the map can be found in the Single-strategy traces. Above 30 the
labels are more ink than the arrows they annotate, so they stop.

**The cloud fades when the arrows are few** - alpha 0.55 above 60 shifts, 0.35 above 20,
0.22 below. Ink follows importance rather than population.

The summary now opens with the arithmetic, because that is the whole answer:

```
  HOW MUCH OF THE CLOUD IS A SHIFT
    18 gear changes over 1,801 samples (30 min)
    gear 1 carries 298 loaded samples, gear 2 carries 1,466
    within +-10 s of a change: 333 samples

    A dense cloud with a few arrows on it reads as though every point
    came from a shift. It does not: the arrows are the events, the cloud
    is where the vehicle spent its time between them.
```

Verified by C20: the filter keeps 19 % of the loaded samples, **retains every one of the
18 shifts** (a filter that dropped its own subject would be worse than none), and all four
switch combinations report **identical** decomposition numbers - turning a layer off must
never change a number. 75/75 pass.


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
