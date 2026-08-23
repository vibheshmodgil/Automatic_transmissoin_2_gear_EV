"""
Build a self-contained HTML report for the two-speed shift-speed study.

Deliberately written in the Vehicle-Motor Integration Suite (VMI) report idiom so it
can later drop into that tool's report system:

  - same palette   : primary #4f46e5 (indigo-600), header #1e1b4b (indigo-950)
  - same structure : header / inputs card / assumptions card / TOC nav / figure cards
  - same CSS class names: card, toc, obs, interp, summary, assume
  - same figure embedding: <img src="data:image/png;base64,...">  (fully self-contained)

VMI reference: vmi/enhancements.py -> _MULTI_REPORT_TEMPLATE, _REPORT_SECTION_TEMPLATE

Usage:
    python make_shift_report.py                 # -> shift_study_report.html
    python make_shift_report.py --body-only     # -> shift_study_report.body.html
"""

from __future__ import annotations

import argparse
import base64
import datetime as _dt
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIGDIR = HERE / "figures"

PRIMARY = "#4f46e5"   # VMI COLORS["primary"]
DARK = "#1e1b4b"      # VMI COLORS["header_bg"]

# ----------------------------------------------------------------------------
# Figure inventory. Order = reading order. Each entry drives one card.
# ----------------------------------------------------------------------------
FIGURES = [
    ("fig01_cell9.png", "Drive cycle — vehicle speed profile", "cycle",
     "The full N603 log: 28 368 samples at 1 Hz, 7.88 h, peak 42.26 km/h.",
     "Plotted across the full figure width this is ~30 samples per pixel. Useful as an "
     "overview only — no individual event is readable at this density."),

    ("fig02_cell11.png", "Vehicle longitudinal dynamics", "dynamics",
     "Speed, wheel torque and wheel power from the road-load decomposition "
     "(inertia + rolling + aerodynamic + grade).",
     "Wheel torque looks like a hairball because acceleration comes from np.gradient of "
     "an unfiltered 1 Hz log. That noise is roughly zero-mean in power, but the objective "
     "integrates max(P, 0), which rectifies it and adds energy that was never demanded."),

    ("fig03_cell15.png", "Selected hysteresis path — gear, RPM, torque, power", "path",
     "Five stacked panels showing the controller's actual selection at the 32/22 baseline.",
     "Shows the selected path only, not both hypothetical ratios at every sample — which is "
     "the correct way to present this."),

    ("fig04_cell15.png", "Selected operating regions, gear 1 vs gear 2", "regions",
     "Torque-speed scatter for the points the controller actually selected in each ratio.",
     "Gear 1 spreads to ~8 600 rpm at low torque; gear 2 sits lower and left. Both clouds "
     "hug the bottom of the plane — the cycle never uses more than ~15 % of motor torque."),

    ("fig05_cell17.png", "Operating points, shift thresholds and gear state", "points",
     "Motoring region with the 45 Nm / 11 kW envelope overlaid, plus the speed trace and "
     "gear state.",
     "The red envelope is drawn here but never enforced in the validity check — see finding "
     "P1. Panels 2 and 3 plot the whole 7.9 h log and are unreadable at this width."),

    ("fig06_cell20.png", "Motor efficiency map (motoring region)", "effmap",
     "Supplied map, 0-41 Nm x 0-9 000 rpm, with the high-efficiency contour and the "
     "45 Nm / 11 kW envelope.",
     "Peak 94.13 % at 4 260 rpm / 13.0 Nm. The blank upper-right region is the 14 672 "
     "missing cells — they correspond almost exactly to the area above the constant-power "
     "curve, which is why the nearest-neighbour fallback ends up filling infeasible points."),

    ("fig07_cell28.png", "Selected points on the signed efficiency map", "signed",
     "Gear 1 and gear 2 selections over the mirrored (signed-torque) map.",
     "The negative-torque half is an assumption, not data: no separate regenerative map was "
     "supplied. For a no-regen vehicle these points are fictitious."),

    ("fig08_cell28.png", "Actual gear-transition movement", "transitions",
     "Where each shift moves the operating point, with vehicle speed labelled.",
     "Upshifts drop motor speed by the ratio gap (19 -> 11) and raise torque — moving the "
     "point toward the high-efficiency island. This is the real mechanism behind the trend."),

    ("fig09_cell32.png", "Upshift threshold comparison, 20-40 km/h", "compare",
     "Six panels, same downshift (22 km/h), showing how the selected cloud shifts between "
     "ratios as the upshift threshold moves.",
     "Each panel draws only ~4 labelled shift arrows. The 20 km/h panel actually contains "
     "639 upshifts — a reader seeing four arrows will badly misjudge the strategy."),

    ("fig10_cell39.png", "Consumed motor input energy vs upshift speed", "sweep",
     "The optimisation curve. Red marker = reported minimum.",
     "The marker sits at 22 km/h — equal to the downshift threshold, i.e. a zero-width "
     "hysteresis band. The curve rises monotonically from there to the top of the range. "
     "This is an optimum pinned to the boundary of its own search box, and nothing on the "
     "chart signals that."),

    ("fig11_cell42.png", "Three-case threshold optimisation", "optimise",
     "Upshift-only, downshift-only, and the combined grid.",
     "All three land on a range edge: upshift 22 (floor region), downshift 5 (floor of "
     "5-31), combined (22, 11). The heatmap shades continuously darker toward the "
     "bottom-left corner — the optimiser is being pulled out of the searched box. Note the "
     "red 'Best' and black 'Manager' markers in panels 1 and 2 are drawn under the series "
     "markers with default size and no zorder, so they are effectively invisible."),

    ("fig12_cell45.png", "Selected optimum strategy", "optimum",
     "Speed, gear state, motor RPM and battery power for the reported best strategy.",
     "The gear-state panel is a solid block: this strategy shifts 366 times. No AMT can do "
     "that, and the model charges nothing for it."),
]

# ----------------------------------------------------------------------------
# Content blocks
# ----------------------------------------------------------------------------
INPUTS_TABLE = """<table>
<tr><th>Parameter</th><th>Value</th><th>Note</th></tr>
<tr><td>Vehicle mass</td><td>995 kg</td><td></td></tr>
<tr><td>Wheel radius</td><td>0.247 m</td><td></td></tr>
<tr><td>Cd &times; frontal area</td><td>1.104 &times; 1.0 m&sup2;</td><td>lumped CdA &mdash; do not "correct" the area</td></tr>
<tr><td>Rolling resistance C<sub>rr</sub></td><td>0.02</td><td></td></tr>
<tr><td>Road grade</td><td>0.0&deg;</td><td>gradeability never tested</td></tr>
<tr><td>Gear ratios G1 / G2</td><td>19 / 11</td><td>G1 is the low gear</td></tr>
<tr><td>Gearbox efficiency</td><td>0.97 both ratios</td><td></td></tr>
<tr><td>Motor limits</td><td>&plusmn;45 Nm, 10 000 rpm, 11 kW</td><td>map only covers 41 Nm / 9 000 rpm</td></tr>
<tr><td>Battery</td><td>52 V, 15 kW limit</td><td>voltage declared but never used</td></tr>
<tr><td>Baseline strategy</td><td>upshift 32 / downshift 22 km/h</td><td>manager baseline</td></tr>
</table>"""

ASSUMPTIONS = """<ul>
<li>Quasi-static longitudinal model; no rotational inertia of rotor or driveline.</li>
<li>Gearbox efficiency applied directionally: torque divided by &eta; when motoring, multiplied when regenerating.</li>
<li>Battery power built from <em>wheel</em> power, so the gearbox loss is counted exactly once.</li>
<li>Motor efficiency from the supplied map; the negative-torque half is mirrored from the positive half (assumption, not measurement).</li>
<li>Shifting is instantaneous and lossless &mdash; no torque interruption, synchronisation or inertia cost.</li>
<li>No auxiliary load, no pack I&sup2;R loss, no separate inverter model.</li>
<li>Objective is consumed motor input energy, &int;max(P<sub>batt</sub>,&nbsp;0)&nbsp;dt, with regeneration excluded.</li>
</ul>"""

RESULTS_TABLE = """<table>
<tr><th>Case</th><th>Upshift</th><th>Downshift</th><th>Consumed</th><th>vs baseline</th><th>Shifts</th></tr>
<tr><td>Manager baseline</td><td>32.0</td><td>22.0</td><td>8.840483 kWh</td><td>&mdash;</td><td>44</td></tr>
<tr><td><b>Upshift-only optimum</b></td><td><b>22.0</b></td><td><b>22.0</b></td><td>8.727364 kWh</td><td>&minus;113 Wh (1.28 %)</td><td><b>366</b></td></tr>
<tr><td>Downshift-only optimum</td><td>32.0</td><td><b>5.0</b></td><td>8.732647 kWh</td><td>&minus;108 Wh</td><td>&mdash;</td></tr>
<tr><td>Combined optimum</td><td><b>22.0</b></td><td>11.0</td><td>8.642366 kWh</td><td>&minus;198 Wh (2.24 %)</td><td>&mdash;</td></tr>
</table>"""

VERDICT = """<p><b>The reported optimum is 22.0 / 22.0 km/h &mdash; upshift equal to downshift.</b>
That is a zero-width hysteresis band, not a gear strategy: the controller upshifts and
downshifts on consecutive samples, 366 times over the cycle, against 44 for the baseline.</p>
<p>Immediately below that result the notebook prints
<em>"Optimization validation passed: every combined candidate has downshift speed below
upshift speed."</em> That assertion guards only the combined grid; the upshift-only case is
never checked. The manual-verification cell also passes on this strategy, because it asserts
only internal energy consistency (consumed &minus; recovered = net), never whether the
strategy is physically possible.</p>
<p>Every one of the three optimisation cases lands on an edge of its own search range. An
optimiser that returns the corner of its box has not found an optimum.</p>"""

CLOSING = """<p>Cruise sits at <b>3&ndash;7 Nm on a 45 Nm motor</b> (6&ndash;15 % load) in
<em>both</em> ratios. Peak map efficiency is 13.0 Nm &times; 4 260 rpm = <b>5.80 kW</b> of
shaft power, while the cycle's mean battery power is 8.73 kWh / 7.88 h = <b>1.10 kW</b>
&mdash; 19 % of that.</p>
<p>A gear ratio moves the operating point <em>along</em> an iso-power hyperbola, trading
torque against speed. It cannot move the point <em>between</em> hyperbolas. So no shift
schedule and no ratio choice reaches the high-efficiency island: the demand is a 1&ndash;2 kW
point and the island is a 5.8 kW point.</p>
<p><b>The motor is oversized for this duty by roughly a factor of five.</b> That is the
finding this study should report &mdash; not an 85&ndash;113 Wh shift-threshold saving that
sits inside the model's own error bar.</p>"""

CAVEAT = """<p><b>These numbers come from reconstructed stand-in data, not the real N603 log.</b>
The original CSV and XLSX were unavailable, so both inputs were rebuilt by fitting to every
diagnostic the notebook printed. Sample count, duration, peak speed, map shape, invalid-cell
count, peak efficiency and its location, and the 44 baseline upshifts all match exactly;
energy and fallback counts match within a few percent. The conclusions reproduce, but do not
quote these figures as N603 results.</p>"""

CSS = """
 body{font-family:'Segoe UI',Arial,sans-serif;margin:0;background:#eef1f7;color:#0f172a}
 header{background:%(dark)s;color:#fff;padding:22px 32px}
 header h1{margin:0;font-size:22px} header p{margin:4px 0 0;color:#a5b4fc;font-size:13px}
 main{max-width:1100px;margin:24px auto;padding:0 24px}
 nav.toc{background:#fff;border:1px solid #e2e8f0;border-radius:14px;padding:16px 20px;margin-bottom:20px}
 nav.toc h2{margin:0 0 8px;font-size:13px;text-transform:uppercase;letter-spacing:.04em;color:%(primary)s}
 nav.toc ul{margin:0;padding-left:20px;columns:2;-webkit-columns:2}
 nav.toc a{color:%(primary)s;text-decoration:none}
 nav.toc a:hover{text-decoration:underline}
 .card{background:#fff;border:1px solid #e2e8f0;border-radius:14px;padding:20px;margin-bottom:20px;scroll-margin-top:16px}
 .card h2{margin:0 0 12px;font-size:16px;color:%(primary)s}
 img{max-width:100%%;border-radius:10px;border:1px solid #e2e8f0;display:block}
 .figwrap{overflow-x:auto}
 table{width:100%%;border-collapse:collapse;font-size:14px;margin-top:10px}
 td,th{padding:7px 10px;border-bottom:1px solid #eef1f7;text-align:left}
 th{font-size:12px;text-transform:uppercase;letter-spacing:.04em;color:#64748b}
 td:first-child{color:#475569}
 .summary{font-size:14px;line-height:1.6;margin-top:12px}
 .obs{margin-top:12px;padding:12px 16px;background:#f8fafc;border-left:3px solid %(primary)s;border-radius:0 10px 10px 0}
 .obs ul{margin:6px 0 0;padding-left:20px} .obs li{margin:3px 0}
 .interp{margin-top:12px;padding:12px 16px;background:#fbfcfe;border-left:3px solid #64748b;border-radius:0 10px 10px 0;font-size:14px;line-height:1.6}
 .interp .ihead{margin:0 0 6px;font-size:12px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;color:#64748b}
 .interp p{margin:6px 0}
 .warn{border-left-color:#b91c1c;background:#fef4f2}
 .warn .ihead{color:#b91c1c}
 .assume ul{margin:6px 0 0;padding-left:20px} .assume li{margin:4px 0;line-height:1.5}
 .kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px;margin-top:4px}
 .kpi{background:#f8fafc;border:1px solid #e2e8f0;border-radius:12px;padding:14px 16px}
 .kpi .lab{font-size:11px;text-transform:uppercase;letter-spacing:.07em;color:#64748b;margin-bottom:6px}
 .kpi .val{font-size:22px;font-weight:700;line-height:1;font-variant-numeric:tabular-nums}
 .kpi .sub{font-size:12px;color:#64748b;margin-top:5px;line-height:1.4}
 .bad{color:#b91c1c} .good{color:#15803d} .neut{color:%(primary)s}
 footer{text-align:center;color:#94a3b8;font-size:12px;padding:18px}
""" % {"dark": DARK, "primary": PRIMARY}


def _b64(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def build(body_only: bool = False) -> str:
    if not FIGDIR.is_dir():
        sys.exit(f"figures/ not found at {FIGDIR}. Run the notebook first.")

    missing = [f for f, *_ in FIGURES if not (FIGDIR / f).is_file()]
    if missing:
        sys.exit("missing figures: " + ", ".join(missing))

    stamp = _dt.datetime.now().strftime("%d %b %Y, %H:%M")

    kpis = """<div class="kpis">
 <div class="kpi"><div class="lab">Reported optimum</div><div class="val bad">22 / 22</div>
   <div class="sub">upshift equals downshift &mdash; zero-width band</div></div>
 <div class="kpi"><div class="lab">Shifts in that strategy</div><div class="val bad">366</div>
   <div class="sub">against 44 at the 32/22 baseline</div></div>
 <div class="kpi"><div class="lab">Claimed saving</div><div class="val neut">1.28 %</div>
   <div class="sub">113 Wh of 8 840 Wh</div></div>
 <div class="kpi"><div class="lab">Cycle mean power</div><div class="val neut">1.10 kW</div>
   <div class="sub">10 % of the 11 kW motor rating</div></div>
</div>"""

    toc = ['<li><a href="#results">Results &amp; verdict</a></li>']
    toc += [f'<li><a href="#{a}">{n}</a></li>' for _, n, a, _, _ in FIGURES]
    toc.append('<li><a href="#conclusion">What the study should conclude</a></li>')

    cards = []
    for fname, name, anchor, caption, interp in FIGURES:
        cls = "interp warn" if anchor in ("sweep", "optimise", "optimum") else "interp"
        cards.append(
            f'<div class="card" id="{anchor}">\n'
            f' <h2>{name}</h2>\n'
            f' <div class="figwrap"><img src="data:image/png;base64,{_b64(FIGDIR / fname)}" alt="{name}"></div>\n'
            f' <div class="summary">{caption}</div>\n'
            f' <div class="{cls}"><p class="ihead">Reading this figure</p><p>{interp}</p></div>\n'
            f'</div>'
        )

    body = f"""<header>
 <h1>Two-Speed Shift-Speed Energy Study &mdash; N603</h1>
 <p>Generated {stamp} &middot; 12 figures &middot; stand-in data</p>
</header>
<main>
 <div class="card" id="inputs"><h2>1 &middot; Parameters &mdash; Vehicle &amp; Motor Inputs</h2>{INPUTS_TABLE}</div>
 <div class="card assume" id="assumptions"><h2>2 &middot; Model Assumptions</h2>{ASSUMPTIONS}
   <div class="interp warn"><p class="ihead">Data provenance</p>{CAVEAT}</div></div>
 <div class="card" id="results"><h2>3 &middot; Results &amp; Verdict</h2>
   {kpis}
   {RESULTS_TABLE}
   <div class="interp warn"><p class="ihead">Why this result is not usable</p>{VERDICT}</div></div>
 <nav class="toc"><h2>Contents</h2><ul>{''.join(toc)}</ul></nav>
 {''.join(cards)}
 <div class="card" id="conclusion"><h2>What the study should conclude</h2>
   <div class="summary">{CLOSING}</div></div>
</main>
<footer>Two-Speed Shift-Speed Energy Study &middot; report format matches the Vehicle-Motor Integration Suite</footer>"""

    if body_only:
        return f"<title>N603 Shift Study Figures</title>\n<style>{CSS}</style>\n{body}\n"

    return (
        '<!DOCTYPE html>\n<html lang="en"><head><meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n'
        "<title>Two-Speed Shift-Speed Energy Study - N603</title>\n"
        f"<style>{CSS}</style></head><body>\n{body}\n</body></html>"
    )


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--body-only", action="store_true",
                    help="emit body-only fragment (for embedding)")
    args = ap.parse_args()

    html = build(body_only=args.body_only)
    out = HERE / ("shift_study_report.body.html" if args.body_only
                  else "shift_study_report.html")
    out.write_text(html, encoding="utf-8")
    print(f"wrote {out.name}  ({len(html)/1e6:.2f} MB, {len(FIGURES)} figures embedded)")
