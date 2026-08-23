# Two-Speed Shift Optimiser

Shift-schedule analysis for a two-speed gearbox in a light EV: where to put the 1→2
upshift and 2→1 downshift speeds, what each choice costs in energy and in
acceleration, and whether the second ratio earns its place at all.

Built around the N603 (995 kg, 52 V pack, 11 kW / 45 Nm motor, ratios 19 and 11), but
every vehicle parameter is an input — point it at your own cycle, map and ratios.

```bash
python -m pip install -r requirements.txt
python shift_app.py            # the GUI
python verify_model.py         # 37 physics checks, exit code 0 if all pass
```

Windows users can double-click `Run Shift Optimiser.bat`.

> ### Read this before quoting any number
> The files in `sample_data/` are **synthetic stand-ins**, not measurements. They exist
> so the tool runs out of the box and to document the expected format. Load your real
> drive cycle and dyno map before treating any output as a result. See
> [`sample_data/README.md`](sample_data/README.md).

---

## What's here

| File | Role |
|---|---|
| `shift_core.py` | Physics engine. Pure functions and dataclasses — no UI, no globals. |
| `shift_app.py` | CustomTkinter front end. Presentation only; every number comes from the core. |
| `verify_model.py` | Independent verification suite. 37 checks against closed forms and analytic identities. |
| `make_shift_report.py` | Builds a self-contained HTML report with all figures embedded. |
| `sample_data/` | Synthetic cycle and map, plus the format specification. |
| `docs/ENGINEERING_NOTES.md` | Full working notes: every defect found, why each decision was made. |

`shift_core.py` has no dependency on the GUI and is designed to be imported directly:

```python
import shift_core as sc

cycle = sc.load_cycle("my_cycle.csv")
emap  = sc.load_efficiency_map("my_map.xlsx")

result = sc.simulate(cycle, emap, upshift=20, downshift=10,
                     veh=sc.Vehicle(mass=995, wheel_radius=0.247, cda=1.104, crr=0.02),
                     motor=sc.Motor(peak_torque=45, peak_power=11_000, max_rpm=10_000),
                     gb=sc.Gearbox(ratio_1=19, ratio_2=11),
                     elec=sc.Electrical(voltage=52, regen_enabled=True),
                     cost=sc.ShiftCost(energy_per_shift=500, interrupt_s=0.4))

print(result.net_kwh, result.wh_per_km, result.reserve_at_upshift)
```

---

## The analyses

Thirteen views, selected from the dropdown. Those marked ▸ need only the efficiency map,
not a drive cycle.

**Single strategy** — one schedule over the full cycle. Stacked traces for road speed,
gear, motor rpm, motor torque, motor efficiency, battery power, shaft power,
acceleration and cumulative energy — each one toggleable, with a time-window slider so
you can read a 300 s slice instead of 28,000 samples squeezed into 900 pixels.

**Points on map** — where the cycle actually sits on the efficiency map, per gear.
**Split the operating cloud by** offers four ways to read the same points, each answering
a different question: *motoring / braking*; *right or wrong ratio*; *accelerating /
cruising / braking* (the gear preference **inverts** between those regimes, which is the
study's central finding); and *just shifted / settled*, separating the transient after a
change from the steady operation the schedule actually buys. With *right or wrong ratio*
every point is scored
against the per-sample right answer: green where the engaged ratio really was the more
efficient one, red where it was not. The summary reports the share of energy delivered
through the better ratio — 71.8 % at the efficiency optimum against 36.0 % at a bad
schedule on the sample data. **This is the view that justifies a shift schedule.**

A note on why it is this view and not *Shift movement*: a shift happens under
acceleration, where the LOW ratio is the efficient one, so the arrow at the instant
legitimately points the wrong way (−1.83 points at the upshift). The cycle then settles to
cruise, where the HIGH ratio is worth +6.87 points over the 18,091 samples spent there.
The shift view shows the first of those and none of the second, so it can never justify an
optimum on its own.

**Shift movement** — what each gear change does to the operating point. Draws the
gearbox effect (same instant, ratio swapped) and the driver's demand change as
*separate* arrows, because combined into one they are indistinguishable and badly
misleading. Every shift is drawn, over the full operating cloud of both gears, and each
layer — gear 1 points, gear 2 points, shift arrows, driver arrows, before/after markers —
has its own switch under SHIFT-MOVEMENT LAYERS.

▸ **Gear comparison** — efficiency against road speed in both ratios, at several loads,
with the crossover marked.

▸ **Optimal gear map** — which ratio wins at every (speed, acceleration) point. The
boundary is a curve; your shift thresholds are vertical lines drawn on top of it.

**Upshift / Downshift / Combined sweeps** — energy against threshold, with mean motor
efficiency on a second axis, feasibility, convergence, a breakdown of what the optimum
actually won on (wheel / gearbox / motor / aux / pack), and the theoretical ceiling.

**Efficiency-only optimum** — every schedule ranked on motor efficiency *alone*. The
auxiliary load, pack resistance, shift energy and torque interruption all cancel out of
`mean_efficiency`, so nothing competes with the map. Shows the efficiency surface over
both thresholds, slices through the optimum with net energy overlaid, and where that
schedule puts the operating cloud. Use it when you want the answer to "which schedule
keeps the motor in the best part of its map" without any other cost in the way.
Candidates refused by the schedule constraints are drawn as red bars rather than dropped,
and the summary lists how many went and why — a curve that starts above the speed you
asked for is always explained.

▸ **Gradeability** — maximum sustainable grade at steady speed, per ratio.

▸ **Acceleration run** — wide-open-throttle 0→V. The motor sits on its peak curve and
the acceleration is an *output*, so the answer is a time. Swept over the upshift speed,
with the tractive-force diagram that explains the result.

**Energy bins** — the plane divided into (rpm × Nm) cells, each carrying the **energy**
drawn, delivered and lost there. A scatter weights an idling sample the same as one
pulling 25 Nm; this is the energy budget instead of a population count. Four panels:
energy drawn per cell, where it is *wasted*, the **transition** between two schedules
(which cells lost energy, which gained), and the efficiency achieved per cell.

The transition summary is the direct answer to "did this schedule move the operating
points somewhere better?" — on the sample data, going 22/10 → 32/22 moves 5.36 kWh
(61.8 % of the total) **out of cells averaging 92.73 % into cells averaging 89.20 %**, and
motor loss rises 643 → 857 Wh.

▸ **Efficiency map** — the map itself with the motor envelope and peak marked.

Display controls apply to any view: filled shading, contour lines, colour bar, level
counts, colormap, the negative-torque half, iso-power lines, and the **colour scale
range**.

**Efficiency ridge** draws the rpm that maximises efficiency *at each torque*. This is
the curve a shift schedule should be judged against, and it is not the map's peak cell:
that cell is only the optimum at its own torque. On the sample map the ridge runs from
670 rpm at 2 Nm to 4260 rpm at 13 Nm. A cycle working the motor at 5 Nm wants 1710 rpm,
not 4260 — sitting at 4260 rpm with 5 Nm costs 2.44 points against the ridge, and at
2 Nm the same mistake costs 10.5 points.

That last one matters more than it sounds. A motor map spans 0-95 % efficiency, but the
whole result of this study lives between 89 % and 93 %. Spread the colormap over 0-100 %
and everything above 80 % is the same yellow — a 3-point difference, which is the entire
finding, is invisible. The scale defaults to 78-95 % so the working band is resolved; open
it to 0-100 % when you want to see the stall region instead.

**Resizing.** Both panel boundaries are draggable — grab the edge between the parameter
column and the plot, or between the plot and the results panel, and the figure refits
when you let go. The *Plot height* slider does a different job: it makes the figure
**taller than the panel** so it scrolls, which is what you want with eight or nine signal
traces stacked up. `Auto` returns it to fitting the panel exactly.

---

## What the model includes

- **Road load** — inertia, rolling, aerodynamic, grade. Acceleration from
  `np.gradient` on the true (possibly non-uniform) time base.
- **Reflected rotor inertia** — `J·G²/r²`, so the two ratios are compared on equal
  terms. It scales with the square of the ratio, so the low gear carries ~3× the
  penalty; leaving it out biases exactly the thing being optimised.
- **Constant-power envelope** — `min(T_peak, 9550·P_peak/n)`, enforced as a feasibility
  constraint on the traction side. Not enforced on the braking side, where the friction
  brakes take whatever the motor cannot.
- **Efficiency map** — genuine low-efficiency cells kept; nearest-neighbour fallback on
  normalised axes; physical extrapolation below the lowest measured torque row rather
  than substituting a loaded point's efficiency.
- **Electrical** — constant auxiliary load and pack I²R solved exactly (quadratic root,
  not linearised).
- **Regen** — blend fraction, charge-power limit, low-speed blend-out, capped by the
  envelope, with the map queried at the torque the motor *actually* takes.
- **Shift cost** — actuation energy per shift, plus a torque interruption charged at the
  local traction power (zero while braking, where there is no traction to interrupt).
- **Schedule constraints** — minimum hysteresis band, minimum acceleration reserve, and
  a cap on the fraction of available acceleration an upshift may give up.

### Objective

Every sweep minimises **net** battery energy (consumed − recovered). With regen off
that is identical to consumed; with regen on, ranking by consumed alone would prefer
whichever schedule recovers least.

### Convergence reporting

An optimum sitting on the edge of its own search range is reported as **not converged**,
because a search that returns a corner has not found an optimum. Exact ties are detected
and reported as a plateau with its midpoint, rather than silently returning whichever
candidate happened to be evaluated first.

Every sweep also prints a **ceiling**: the energy a controller would use if it could pick
the better ratio at every single sample, with perfect foresight and free shifting. No
causal controller can beat that, so if the gap between it and a fixed ratio is small,
the shift schedule cannot matter however it is chosen.

---

## Verification

`verify_model.py` runs 37 checks. Each states a prediction derived independently — a
closed form, an analytic identity, or a bound physics requires — and compares. Nothing
calls the code under test to produce its own expected value.

```
A. Physics kernel      road load against closed forms; gearbox identities;
                       exact pack I²R; map queries at grid nodes; the vectorised
                       shift controller against a naive per-sample loop; the energy
                       books; reported statistics against their own definitions
B. Each analysis       envelope geometry; the iso-power identity; the optimal-gear
                       boundary; gradeability against a force balance
C. Sweeps and bounds   sweep rows against standalone runs; the oracle as a lower
                       bound; WOT against an analytic inertia identity; the
                       counterfactual; regen ceilings; monotonicity; numerical
                       sensitivity; schedule constraints
```

Two of these are worth knowing about because they are strong tests:

**The acceleration identity.** Above the tractive-force crossover both ratios pull
identical force, so the time spent in each must scale *exactly* with effective mass
`m + J·G²/r²`. The engine satisfies this to nine significant figures
(`t₁/t₂ = 1.019572997` against `m₁/m₂ = 1.019572997`), which pins the envelope, the
inertia model and the integrator simultaneously.

**Two routes to the same threshold.** A schedule forbidden from giving up any
acceleration lands on a 20 km/h upshift. The tractive-force crossover — computed from
envelope geometry and vehicle dynamics, with no reference to the drive cycle or the
efficiency map — is 19.76 km/h. Those are independent derivations agreeing to within one
grid step.

Accuracy notes: acceleration times come from quadrature over speed rather than
time-stepping, converged to well under a millisecond and checked against an independent
fine-step RK4 integrator (±0.02 ms). Cycle energy carries a one-sided error bar of about
4.6 % from differentiating an unfiltered 1 Hz speed trace — unfiltered reads *high*.
Set `Numerics.smooth_window` to quantify it on your own data; every ranking in the
sample data survives every filter width tested.

---

## Findings on the sample data

Reproduce with `python verify_model.py`. **These come from synthetic data** and are
included to show what the tool produces and how to read it — not as N603 results.

| | |
|---|---|
| 0–30 km/h, full throttle | 4.31 s holding the low gear; 4.72 s for the best schedule that shifts |
| Upshift threshold | 20 km/h, where the handover costs no acceleration |
| Gear-selection prize | 47 Wh of 10.1 kWh (< 0.5 %), and that is with clairvoyance and free shifting |
| Low gear duty | engaged 30 % of the time, does 8 % of the work |
| Realistic regen | 86.3 Wh/km against 91.5 without, at a 70 % brake-torque blend |

The shape of the argument, on this data:

- **The upshift threshold is a driveability decision, not an energy one.** Everything
  from 13 to 20 km/h lies within 8 Wh — 0.08 %. Left unconstrained the energy objective
  drifts to the edge of the search range and returns a 1 km/h hysteresis band, which is
  chatter rather than a schedule. Constrain it on tractive force instead: upshift where
  the handover costs no acceleration.
- **The second ratio does not pay for itself on consumption.** Even perfect per-sample
  gear selection saves under 0.5 %. A speed threshold reaches about two-thirds of that;
  the rest needs a 2-D map on speed and torque demand, because the optimal-gear boundary
  sweeps 34.9 km/h as load varies and a vertical line cannot follow a curve.
- **The low gear earns its place on gradeability and launch.** Roughly twice the
  climbing ability below 20 km/h, and on a brisk launch it is the only ratio that can
  deliver the torque at all — at 0–15 km/h and 1.8 m/s², 11 of 12 shift speeds are
  infeasible and holding the low gear is the only one that works.
- **Under acceleration the low ratio is the efficient one**, which is the opposite of
  the cruise result: under load copper loss dominates and the low ratio asks less of the
  motor, while at cruise the high ratio already sits on the map's efficiency ridge.

Whether these hold for the real N603 depends entirely on where the real map's efficiency
ridge sits. Load it and re-run.

---

## Troubleshooting

### "This figure includes Axes that are not compatible with tight_layout"

**Harmless — ignore it.** It comes from colour bars attached to several axes at once,
it appears on a fully working install, and the figure still draws correctly. It fires
at `canvas.draw()`, which is the *last* line of the render, so seeing it means the
figure was built successfully. It is never the reason a plot is missing.

### The numbers appear but the plot area is blank

The figure is being drawn into a widget that has no visible height on your display —
usually a smaller screen or higher display scaling than the machine it was built on.

Run the diagnostic and read the last few lines:

```bash
python diagnose_ui.py
```

It prints library versions, display scaling, and the real on-screen geometry of the plot
widgets after an actual render, then tells you which case you are in. It needs no data
files.

Two things to try:

1. **Press `Auto`** next to the *Plot height* slider. If a tall height was set manually
   (or carried over from a taller analysis), the figure can sit below the fold inside
   the scroll area.
2. **Force the simple layout**, which packs the canvas straight into the panel instead
   of into a scroll container:

   ```bat
   set SHIFT_APP_SIMPLE_LAYOUT=1
   python shift_app.py
   ```

   ```bash
   SHIFT_APP_SIMPLE_LAYOUT=1 python shift_app.py     # macOS / Linux
   ```

   The only thing you give up is scrolling a very tall stack of signal panels — the
   figure is scaled to fit the panel instead.

If neither helps, send the full output of `diagnose_ui.py`.

### Other things worth checking

- **`No module named customtkinter`** (or matplotlib, scipy, …) —
  `python -m pip install -r requirements.txt`.
- **matplotlib older than 3.6** — the app falls back to a one-shot `tight_layout()`
  automatically, but `requirements.txt` asks for 3.7+ and the layout is better there.
- **Nothing happens when you press Run** — check the panel underneath the plot. Errors
  are printed there in full, and the status bar at the bottom says what went wrong.

---

## Requirements

Python 3.10+. Pinned in `requirements.txt` to match the environment this was developed
against (numpy 1.26 — the code deliberately uses `np.trapz`, not `np.trapezoid`, so it
runs on numpy 1.x and 2.x alike).

```
customtkinter  matplotlib  numpy  pandas  scipy  openpyxl
```

## Porting into a larger suite

`shift_core.py` was written to drop into a vehicle-modelling suite as an analysis
module: no globals, no UI imports, every parameter passed in as a dataclass, and every
result returned as one. `shift_app.py` demonstrates the call pattern; the GUI's palette
and layout follow the conventions of the suite it was written for.

## Licence

MIT — see [`LICENSE`](LICENSE).
