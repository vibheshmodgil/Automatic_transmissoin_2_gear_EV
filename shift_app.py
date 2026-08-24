"""
Two-Speed Shift Optimiser — CustomTkinter application.

Standalone today; built to be lifted into the Vehicle-Motor Integration Suite as a
new analysis type. It follows VMI's conventions so the port is mechanical:

  * customtkinter 5.2.2, matplotlib 3.10.9, numpy 1.26.4 (VMI's pinned versions)
  * VMI's COLORS palette (vmi/theme.py) — falls back to a local copy if VMI isn't importable
  * dark app bar -> analysis selector strip -> scrollable input column + plot canvas -> status bar
  * a data checklist showing which files the selected analysis needs
  * all physics lives in shift_core.py; this file is presentation only

Run:  python shift_app.py
"""

from __future__ import annotations

import os
import queue
import threading
import traceback
from pathlib import Path
import tkinter as tk
from tkinter import filedialog

import customtkinter as ctk
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

import shift_core as sc

# --- VMI palette -----------------------------------------------------------
try:                                                # prefer the real thing
    from vmi.theme import COLORS                    # type: ignore
except Exception:                                   # standalone fallback
    COLORS = {
        "primary": "#4f46e5", "primary_hover": "#4338ca", "secondary": "#0ea5e9",
        "accent": "#6366f1", "success": "#16a34a", "warning": "#ea580c",
        "danger": "#dc2626", "background": "#eef1f7", "card": "#ffffff",
        "section_bg": "#f8fafc", "input_bg": "#ffffff", "border": "#e2e8f0",
        "text": "#0f172a", "text_muted": "#475569", "header_bg": "#1e1b4b",
        "header_bg_soft": "#eef2ff", "on_header": "#f8fafc",
        "on_header_muted": "#a5b4fc", "plot_bg": "#ffffff", "plot_axes_bg": "#ffffff",
    }

ANALYSES = [
    "Single strategy",
    "Points on map",          # operating cloud + shift arrows over the efficiency map
    "Shift movement",         # zoomed: where each shift throws the operating point
    "Gear comparison",        # efficiency vs speed per gear, at several loads
    "Optimal gear map",       # which ratio wins at every (speed, accel) point
    "Upshift sweep",
    "Downshift sweep",
    "Combined grid",
    "Efficiency-only optimum",   # ranked on motor efficiency alone, nothing else
    "Loss breakdown",            # where the energy goes as the threshold moves
    "Energy bins",               # where the energy goes, and where a schedule moves it
    "Gradeability",
    "Acceleration run",       # one 0 -> V (-> 0) manoeuvre, swept over the shift speed
    "Efficiency map",
]
NEEDS = {                        # analysis -> required data keys
    "Single strategy": ("cycle", "map"),
    "Points on map": ("cycle", "map"),
    "Shift movement": ("cycle", "map"),
    "Gear comparison": ("map",),
    "Optimal gear map": ("map",),
    "Upshift sweep": ("cycle", "map"),
    "Downshift sweep": ("cycle", "map"),
    "Combined grid": ("cycle", "map"),
    "Efficiency-only optimum": ("cycle", "map"),
    "Loss breakdown": ("cycle", "map"),
    "Energy bins": ("cycle", "map"),
    "Gradeability": (),
    "Acceleration run": ("map",),
    "Efficiency map": ("map",),
}

# Analyses that need no computation on the worker thread: _work() falls through
# to `out = None` and the draw method reads the map straight from self.emap.
# Declared here so the exemption is visible in the code rather than hidden in a
# test's assumptions.
WORK_EXEMPT = ("Efficiency map",)

_missing = [a for a in ANALYSES if a not in NEEDS]
if _missing:                     # DRAW is checked the same way below its own def
    raise RuntimeError("analyses missing a NEEDS entry: " + ", ".join(_missing))

FIELDS = [
    # (section, label, attr, default, kind)
    ("Vehicle", "Mass [kg]", "mass", "995", "veh"),
    ("Vehicle", "Wheel radius [m]", "wheel_radius", "0.247", "veh"),
    ("Vehicle", "Cd x A [m^2]", "cda", "1.104", "veh"),
    ("Vehicle", "Rolling resistance Crr", "crr", "0.02", "veh"),
    ("Vehicle", "Road grade [deg]", "grade_deg", "0.0", "veh"),
    ("Motor", "Peak torque [Nm]", "peak_torque", "45", "mot"),
    ("Motor", "Peak power [W]", "peak_power", "11000", "mot"),
    ("Motor", "Max speed [rpm]", "max_rpm", "10000", "mot"),
    ("Motor", "Rotor inertia [kg.m^2]", "inertia", "0.005", "mot"),
    ("Gearbox", "Ratio 1 (low)", "ratio_1", "19", "gb"),
    ("Gearbox", "Ratio 2 (high)", "ratio_2", "11", "gb"),
    ("Gearbox", "Efficiency gear 1", "eta_1", "0.97", "gb"),
    ("Gearbox", "Efficiency gear 2", "eta_2", "0.97", "gb"),
    ("Electrical", "Pack voltage [V]", "voltage", "52", "el"),
    ("Electrical", "Auxiliary load [W]", "aux_load", "150", "el"),
    ("Electrical", "Battery power limit [W]", "max_power", "15000", "el"),
    ("Regen", "Regen enabled (0/1)", "regen", "0", "num"),
    ("Regen", "Brake torque to motor [0-1]", "regen_fraction", "0.70", "el"),
    ("Regen", "Charge power limit [W] (0=same)", "regen_max_power", "0", "el"),
    ("Regen", "Blend out below [km/h]", "regen_min_speed", "5", "el"),
    ("Shift cost", "Actuator voltage [V]", "actuator_voltage", "12", "cost"),
    ("Shift cost", "Actuator current [A]", "actuator_current", "20", "cost"),
    ("Shift cost", "Shift duration [s]", "actuator_time_s", "0.5", "cost"),
    ("Shift cost", "Max shifts per hour", "max_shifts_per_hour", "120", "cost"),
    ("Shift cost", "Min hysteresis band [km/h]", "min_band_kmh", "3", "cost"),
    ("Shift cost", "Min accel reserve [m/s2]", "min_accel_reserve", "0.5", "cost"),
    ("Shift cost", "Max accel given up [0-1]", "max_accel_loss", "0.10", "cost"),
    ("Thresholds", "Upshift [km/h]", "upshift", "22", "thr"),
    ("Thresholds", "Downshift [km/h]", "downshift", "10", "thr"),
    ("Sweep range", "Upshift from [km/h]", "up_lo", "8", "sw"),
    ("Sweep range", "Upshift to [km/h]", "up_hi", "42", "sw"),
    ("Sweep range", "Downshift from [km/h]", "dn_lo", "4", "sw"),
    ("Sweep range", "Downshift to [km/h]", "dn_hi", "30", "sw"),
    ("Sweep range", "Step [km/h]", "step", "1", "sw"),
    ("Sweep range", "Minimum band [km/h]", "min_band", "2", "sw"),
    ("Energy bins", "Bin size [rpm]", "bin_rpm", "500", "bin"),
    ("Energy bins", "Reference (0=g1, 1=g2, 2=custom)", "ref_mode", "0", "bin"),
    ("Energy bins", "Bin size [Nm]", "bin_nm", "5", "bin"),
    ("Energy bins", "Compare upshift [km/h]", "cmp_up", "32", "bin"),
    ("Energy bins", "Compare downshift [km/h]", "cmp_dn", "22", "bin"),
    ("Acceleration run", "Target speed [km/h]", "v_target", "30", "run"),
    ("Acceleration run", "Throttle [0-1]", "throttle", "1.0", "run"),
    ("Numerics", "Smoothing window (0=off)", "smooth_window", "0", "num"),

]

# Time-series signals available in the Single-strategy view: (key, label, default on)
SIGNALS = [
    ("speed",  "Road speed",         True),
    ("gear",   "Selected gear",      True),
    ("rpm",    "Motor speed",        True),
    ("torque", "Motor torque",       True),
    ("eff",    "Motor efficiency",   True),
    ("pbatt",  "Battery power",      True),
    ("pmech",  "Shaft power",        False),
    ("accel",  "Acceleration",       False),
    ("energy", "Cumulative energy",  False),
]
WINDOWS = ["full cycle", "3600 s", "1800 s", "600 s", "300 s", "120 s", "60 s"]
# ONE colour per gear, everywhere. A reader should never have to re-learn which
# colour means which ratio between two panels of the same study.
GEAR1 = "#10265e"         # deep blue  - the low ratio
GEAR2 = "#a30f45"         # crimson    - the high ratio
CLOUD_G1, CLOUD_G2 = GEAR1, GEAR2
GOOD = "#1a7f4b"
BAD = "#c62828"
ISO_POWER_KW = [0.25, 0.5, 1, 2, 3, 5, 8, 11]
CMAPS = ["viridis", "plasma", "magma", "cividis", "turbo", "coolwarm", "Greys"]
# how to split the operating cloud in "Points on map" - each answers a different
# question about the same points
BIN_REFERENCE = ["always gear 1 (no shifting)",
                 "always gear 2 (no shifting)",
                 "custom thresholds below"]
POINT_MODES = ["motoring / braking",
               "right or wrong ratio",
               "accelerating / cruising / braking",
               "just shifted / settled"]


class ShiftOptimiserApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Two-Speed Shift Optimiser  -  " + sc.build_stamp())
        self.geometry("1500x950")
        self.minsize(1180, 760)
        ctk.set_appearance_mode("light")
        self.configure(fg_color=COLORS["background"])

        self.cycle = None
        self.emap = None
        self.last_result = None
        self.last_sweep = None
        self.entries = {}
        self.sig = {}                # signal key -> BooleanVar (single-strategy traces)
        self.disp = {}               # display option name -> Var
        self._last_render = None     # (kind, payload, params) so options replot without re-running
        self._manual_h = None        # plot height in px once the user drags the slider
        self._redraw_job = None
        self._q = queue.Queue()

        self._build_header()
        self._build_selector()
        self._build_body()
        self._build_status()
        self._refresh_checklist()
        self.after(120, self._drain)

    # ---------------------------------------------------------------- header
    def _build_header(self):
        bar = ctk.CTkFrame(self, fg_color=COLORS["header_bg"], corner_radius=0, height=74)
        bar.pack(fill="x")
        bar.pack_propagate(False)
        box = ctk.CTkFrame(bar, fg_color="transparent")
        box.pack(side="left", padx=26, pady=13)
        ctk.CTkLabel(box, text="Two-Speed Shift Optimiser",
                     font=ctk.CTkFont(size=20, weight="bold"),
                     text_color=COLORS["on_header"]).pack(anchor="w")
        ctk.CTkLabel(box, text="Shift-schedule energy optimisation with envelope, "
                              "grade and shift-cost constraints",
                     font=ctk.CTkFont(size=12),
                     text_color=COLORS["on_header_muted"]).pack(anchor="w")

        btns = ctk.CTkFrame(bar, fg_color="transparent")
        btns.pack(side="right", padx=22)
        for txt, cmd in (("Load drive cycle", self.load_cycle),
                         ("Load efficiency map", self.load_map),
                         ("Export figure", self.export_figure)):
            ctk.CTkButton(btns, text=txt, command=cmd, width=140, height=32,
                          fg_color=COLORS["primary"], hover_color=COLORS["primary_hover"],
                          font=ctk.CTkFont(size=12)).pack(side="left", padx=5)

    # -------------------------------------------------------------- selector
    def _build_selector(self):
        strip = ctk.CTkFrame(self, fg_color=COLORS["header_bg_soft"], corner_radius=0, height=62)
        strip.pack(fill="x")
        strip.pack_propagate(False)
        ctk.CTkLabel(strip, text="Analysis type", font=ctk.CTkFont(size=12, weight="bold"),
                     text_color=COLORS["text_muted"]).pack(side="left", padx=(26, 10))
        self.analysis = ctk.CTkOptionMenu(
            strip, values=ANALYSES, width=210, command=lambda _=None: self._refresh_checklist(),
            fg_color=COLORS["primary"], button_color=COLORS["primary"],
            button_hover_color=COLORS["primary_hover"])
        self.analysis.set("Single strategy")
        self.analysis.pack(side="left")

        self.checklist = ctk.CTkLabel(strip, text="", font=ctk.CTkFont(size=12),
                                      text_color=COLORS["text_muted"])
        self.checklist.pack(side="left", padx=18)

        self.run_btn = ctk.CTkButton(strip, text="Run analysis", command=self.run, width=150,
                                     height=34, fg_color=COLORS["success"],
                                     hover_color="#15803d",
                                     font=ctk.CTkFont(size=13, weight="bold"))
        self.run_btn.pack(side="right", padx=26)

    # ------------------------------------------------------------------ body
    def _build_body(self):
        body = ctk.CTkFrame(self, fg_color="transparent")
        body.pack(fill="both", expand=True, padx=16, pady=12)

        # Draggable splitters. tk.PanedWindow gives a real sash with the resize
        # cursor on hover, which is what you reach for at a panel edge; the plot
        # height slider stays because it does a different job - making the figure
        # TALLER than the panel so it scrolls.
        self.split_h = tk.PanedWindow(
            body, orient="horizontal", bg=COLORS["border"], sashwidth=7,
            sashrelief="flat", borderwidth=0, showhandle=False, opaqueresize=False)
        self.split_h.pack(fill="both", expand=True)

        left_pane = ctk.CTkFrame(self.split_h, width=330, fg_color=COLORS["card"],
                                 corner_radius=14, border_width=1,
                                 border_color=COLORS["border"])
        self.split_h.add(left_pane, minsize=210, stretch="never", padx=0, pady=0)
        left = ctk.CTkScrollableFrame(left_pane, fg_color=COLORS["card"],
                                      corner_radius=12, border_width=0,
                                      label_text="Parameters",
                                      label_font=ctk.CTkFont(size=13, weight="bold"),
                                      label_text_color=COLORS["primary"])
        left.pack(fill="both", expand=True, padx=2, pady=2)

        section = None
        for sec, label, attr, default, kind in FIELDS:
            if sec != section:
                section = sec
                ctk.CTkLabel(left, text=sec.upper(), font=ctk.CTkFont(size=11, weight="bold"),
                             text_color=COLORS["primary"]).pack(anchor="w", pady=(14, 4), padx=4)
            row = ctk.CTkFrame(left, fg_color="transparent")
            row.pack(fill="x", pady=2)
            ctk.CTkLabel(row, text=label, font=ctk.CTkFont(size=11),
                         text_color=COLORS["text_muted"], width=178,
                         anchor="w").pack(side="left")
            e = ctk.CTkEntry(row, width=94, height=27, fg_color=COLORS["input_bg"],
                             border_color=COLORS["border"], font=ctk.CTkFont(size=11))
            e.insert(0, default)
            e.pack(side="right")
            self.entries[(kind, attr)] = e

        self._build_display(left)

        self.split_v = tk.PanedWindow(
            self.split_h, orient="vertical", bg=COLORS["border"], sashwidth=7,
            sashrelief="flat", borderwidth=0, showhandle=False, opaqueresize=False)
        self.split_h.add(self.split_v, minsize=380, stretch="always", padx=0, pady=0)

        right = ctk.CTkFrame(self.split_v, fg_color=COLORS["card"], corner_radius=14,
                             border_width=1, border_color=COLORS["border"])
        self.split_v.add(right, minsize=180, stretch="always", padx=0, pady=0)

        # The plot normally lives in a scrollable host so a tall stack of signal
        # panels can be scrolled rather than squeezed flat. On some machines -
        # high-DPI displays, other CustomTkinter builds - a Tk canvas with an
        # explicit pixel height inside CTkScrollableFrame ends up with no visible
        # height, and the figure draws into a widget nobody can see. The simple
        # layout packs the canvas straight into the panel, which always lays out;
        # the cost is that a very tall figure is scaled to fit instead of scrolling.
        self.simple_layout = os.environ.get("SHIFT_APP_SIMPLE_LAYOUT", "") == "1"
        if self.simple_layout:
            self.plot_host = ctk.CTkFrame(right, fg_color=COLORS["plot_bg"],
                                          corner_radius=0)
        else:
            self.plot_host = ctk.CTkScrollableFrame(right, fg_color=COLORS["plot_bg"],
                                                    corner_radius=0)
        self.plot_host.pack(fill="both", expand=True, padx=10, pady=(10, 0))

        self.fig = plt.Figure(figsize=(11, 6.2), dpi=100, facecolor=COLORS["plot_bg"])
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_host)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        tb = NavigationToolbar2Tk(self.canvas, right, pack_toolbar=False)
        tb.update()
        tb.pack(fill="x", padx=10)

        # plot-size strip: every analysis gets a manual height, because what is
        # readable depends on the panel count and on the screen, not on a constant
        bar = ctk.CTkFrame(right, fg_color="transparent")
        bar.pack(fill="x", padx=10, pady=(2, 0))
        ctk.CTkLabel(bar, text="Plot height", font=ctk.CTkFont(size=11),
                     text_color=COLORS["text_muted"]).pack(side="left")
        self.height_slider = ctk.CTkSlider(bar, from_=360, to=2600, number_of_steps=112,
                                           width=230, height=14,
                                           command=self._set_plot_height,
                                           button_color=COLORS["primary"],
                                           button_hover_color=COLORS["primary_hover"],
                                           progress_color=COLORS["accent"])
        self.height_slider.set(620)
        self.height_slider.pack(side="left", padx=8)
        self.height_label = ctk.CTkLabel(bar, text="auto", width=86, anchor="w",
                                         font=ctk.CTkFont(size=11),
                                         text_color=COLORS["text_muted"])
        self.height_label.pack(side="left")
        ctk.CTkButton(bar, text="Auto", width=58, height=24,
                      command=self._auto_plot_height, font=ctk.CTkFont(size=11),
                      fg_color=COLORS["primary"],
                      hover_color=COLORS["primary_hover"]).pack(side="left", padx=(0, 8))
        ctk.CTkLabel(bar, text="drag to resize; the panel scrolls when it is taller "
                              "than the window", font=ctk.CTkFont(size=10),
                     text_color=COLORS["text_muted"]).pack(side="left")

        res_pane = ctk.CTkFrame(self.split_v, height=176, fg_color="transparent")
        self.split_v.add(res_pane, minsize=64, stretch="never", padx=0, pady=0)
        self.results = ctk.CTkTextbox(res_pane, fg_color=COLORS["section_bg"],
                                      border_color=COLORS["border"], border_width=1,
                                      font=ctk.CTkFont(family="Consolas", size=11))
        self.results.pack(fill="both", expand=True, pady=(8, 0))

        # a sash move changes the viewport, so the figure has to be refitted
        for pane in (self.split_h, self.split_v):
            pane.bind("<ButtonRelease-1>", self._on_sash_moved)
        # PanedWindow picks its own first split from the requested sizes, which comes
        # out narrower than the parameter rows need; set it once, then leave it alone
        self.after(150, self._place_initial_sashes)
        self._welcome()

    # --------------------------------------------------------- display panel
    def _sec(self, parent, title):
        ctk.CTkLabel(parent, text=title, font=ctk.CTkFont(size=11, weight="bold"),
                     text_color=COLORS["primary"]).pack(anchor="w", pady=(16, 4), padx=4)

    def _check(self, parent, label, default, cmd=True):
        v = ctk.BooleanVar(value=default)
        ctk.CTkCheckBox(parent, text=label, variable=v,
                        command=(self.redraw if cmd else None),
                        font=ctk.CTkFont(size=11), checkbox_width=16, checkbox_height=16,
                        border_width=2, text_color=COLORS["text_muted"],
                        fg_color=COLORS["primary"],
                        hover_color=COLORS["primary_hover"]).pack(anchor="w", padx=6, pady=1)
        return v

    def _slider(self, parent, label, lo, hi, default, unit=""):
        cap = ctk.CTkLabel(parent, text=f"{label}: {default:g}{unit}",
                           font=ctk.CTkFont(size=11), text_color=COLORS["text_muted"],
                           anchor="w")
        cap.pack(anchor="w", padx=6, pady=(6, 0))
        v = ctk.IntVar(value=int(default))

        def on(x):
            v.set(int(round(x)))
            cap.configure(text=f"{label}: {v.get():g}{unit}")
            self.redraw()

        sl = ctk.CTkSlider(parent, from_=lo, to=hi, number_of_steps=int(hi - lo), command=on,
                           button_color=COLORS["primary"], button_hover_color=COLORS["primary_hover"],
                           progress_color=COLORS["accent"], height=14)
        sl.set(default)
        sl.pack(fill="x", padx=6, pady=(0, 2))
        return v

    def _menu(self, parent, values, default):
        v = ctk.StringVar(value=default)
        ctk.CTkOptionMenu(parent, values=values, variable=v, height=26,
                          command=lambda _=None: self.redraw(),
                          font=ctk.CTkFont(size=11),
                          fg_color=COLORS["primary"], button_color=COLORS["primary"],
                          button_hover_color=COLORS["primary_hover"]).pack(fill="x", padx=6, pady=2)
        return v

    def _build_display(self, left):
        """Plot-display controls. Everything here replots the cached result -
        no analysis is re-run, so toggling is instant."""
        self._sec(left, "SIGNALS  (single strategy)")
        ctk.CTkLabel(left, text="one stacked panel per ticked signal",
                     font=ctk.CTkFont(size=10), text_color=COLORS["text_muted"],
                     anchor="w").pack(anchor="w", padx=6)
        for key, label, default in SIGNALS:
            self.sig[key] = self._check(left, label, default)
        self.disp["marks"] = self._check(left, "Mark gear changes", False)

        self._sec(left, "TIME WINDOW")
        self.disp["window"] = self._menu(left, WINDOWS, WINDOWS[0])
        self.disp["start"] = self._slider(left, "Window start", 0, 100, 0, " %")

        self._sec(left, "SWEEP ANCHOR")
        # A 1-D sweep has to hold the other threshold somewhere. Holding it at a
        # typed number makes the answer conditional on that number, which is what
        # makes the upshift sweep, the downshift sweep and the grid look like they
        # disagree. Off by default would reproduce that, so it is on.
        self.disp["self_anchor"] = self._check(
            left, "Self-consistent anchor (fixed point)", True, cmd=False)
        ctk.CTkLabel(left, text=("  alternates the two sweeps until neither moves;"
                                 + chr(10)
                                 + "  untick to hold the Thresholds value instead"),
                     font=ctk.CTkFont(size=10), text_color=COLORS["text_muted"],
                     justify="left").pack(anchor="w", padx=6, pady=(0, 2))

        self._sec(left, "EFFICIENCY-MAP DISPLAY")
        # "Colour bar" is only the legend strip; the colours ON the map are the
        # filled shading, so that gets its own switch rather than hiding inside a
        # level count of zero.
        self.disp["fill_on"] = self._check(left, "Filled colour shading", True)
        self.disp["lines_on"] = self._check(left, "Contour lines", False)
        self.disp["cbar"] = self._check(left, "Colour bar (legend strip)", True)
        self.disp["clabels"] = self._check(left, "Label contour lines", False)
        self.disp["envelope"] = self._check(left, "Envelope + peak marker", True)
        self.disp["negative"] = self._check(left, "Negative-torque (braking) half", True)
        self.disp["isopower"] = self._check(left, "Iso-power lines [kW]", False)
        self.disp["ridge"] = self._check(left, "Efficiency ridge (best rpm per torque)",
                                         False)
        ctk.CTkLabel(left, text="Split the operating cloud by",
                     font=ctk.CTkFont(size=11), text_color=COLORS["text_muted"],
                     anchor="w").pack(anchor="w", padx=6, pady=(8, 0))
        self.disp["ptmode"] = self._menu(left, POINT_MODES, POINT_MODES[0])
        # Spread the whole colormap over the band the vehicle actually works in.
        # Across 0-100 % everything above ~80 % is the same yellow and a 3-point
        # efficiency difference - which is the entire result of this study - simply
        # cannot be seen.
        self.disp["eff_lo"] = self._slider(left, "Colour scale from", 0, 92, 78, " %")
        self.disp["eff_hi"] = self._slider(left, "Colour scale to", 80, 100, 95, " %")

        self._sec(left, "SHIFT-MOVEMENT LAYERS")
        self.disp["lay_g1"] = self._check(left, "All gear 1 points", True)
        self.disp["lay_g2"] = self._check(left, "All gear 2 points", True)
        self.disp["lay_arrow"] = self._check(left, "Shift arrows (gearbox)", True)
        self.disp["lay_driver"] = self._check(left, "Driver arrows (dotted)", True)
        self.disp["lay_marks"] = self._check(left, "Before / after markers", True)
        self.disp["fill_levels"] = self._slider(left, "Shading levels", 2, 40, 22)
        self.disp["line_levels"] = self._slider(left, "Number of contour lines", 2, 25, 10)
        self.disp["cmap"] = self._menu(left, CMAPS, "viridis")

    # -------------------------------------------------------- replot helpers
    def redraw(self, *_):
        """Re-render the cached payload after a display option changes (debounced)."""
        if self._last_render is None:
            return
        if self._redraw_job is not None:
            try:
                self.after_cancel(self._redraw_job)
            except Exception:
                pass
        self._redraw_job = self.after(180, self._redraw_now)

    def _redraw_now(self):
        self._redraw_job = None
        kind, out, p = self._last_render
        try:
            self._render(kind, out, p)
        except Exception:
            self.log(traceback.format_exc())
            self.say("Plotting failed", "err")

    def _time_window(self, t):
        """(lo, hi) of the visible time slice from the window menu + start slider."""
        w = self.disp["window"].get()
        t0, t1 = float(t[0]), float(t[-1])
        if not w[0].isdigit():
            return t0, t1
        span = min(float(w.split()[0]), t1 - t0)
        lo = t0 + (t1 - t0 - span) * self.disp["start"].get() / 100.0
        return lo, lo + span

    def _map_style(self):
        lo = float(self.disp["eff_lo"].get())
        hi = float(self.disp["eff_hi"].get())
        if hi <= lo + 1:
            hi = lo + 1
        return dict(lo=lo, hi=hi,
                    cbar=self.disp["cbar"].get(),
                    fill=int(self.disp["fill_levels"].get()) if self.disp["fill_on"].get() else 0,
                    lines=int(self.disp["line_levels"].get()) if self.disp["lines_on"].get() else 0,
                    labels=self.disp["clabels"].get(),
                    envelope=self.disp["envelope"].get(),
                    cmap=self.disp["cmap"].get())

    def _ridge(self, ax, signed=False):
        """The best rpm for each torque - the curve a schedule should sit on.

        Not the map's peak cell. That cell is only the optimum at its own torque,
        and a cycle running at a third of that torque wants a completely different
        speed. Judging a shift schedule against the peak instead of against this
        curve is the single most common way to reach the wrong conclusion.
        """
        if not self.disp["ridge"].get():
            return
        try:
            t, n = sc.efficiency_ridge(self.emap)
        except Exception:
            return
        for sign in ((1, -1) if signed else (1,)):
            ax.plot(n, sign * t, color="#d81b60", lw=2.4, ls="-", alpha=.95, zorder=6,
                    label="efficiency ridge" if sign == 1 else None)

    def _iso_power(self, ax, signed=False):
        """Constant-shaft-power curves, P = T * n * 2pi/60.

        A gear change slides the operating point ALONG one of these and can never
        move it to another. Draw them and the recurring question answers itself:
        the map's best point sits on a 5.8 kW curve, the cycle's cruise band on the
        0.5-2 kW curves, and no choice of ratio crosses between them.
        """
        if not self.disp["isopower"].get():
            return
        em = self.emap
        n = np.linspace(1, em.rpm.max(), 400)
        t_lim = ax.get_ylim()
        for kw_level in ISO_POWER_KW:
            t = kw_level * 1000.0 / (n * 2.0 * np.pi / 60.0)
            for sign in ((1, -1) if signed else (1,)):
                ax.plot(n, sign * t, color="k", lw=.8, ls=":", alpha=.55, zorder=4)
            j = int(np.argmin(np.abs(t - (t_lim[1] * .62))))
            if 0 < j < len(n) - 1 and t_lim[0] <= t[j] <= t_lim[1]:
                ax.annotate(f"{kw_level:g} kW", (n[j], t[j]), fontsize=7, color="k",
                            alpha=.75, zorder=5,
                            bbox=dict(boxstyle="round,pad=.12", fc="w", ec="none",
                                      alpha=.55))
        ax.set_ylim(*t_lim)

    def _colorbar(self, mappable, ax, label, **kw):
        """Colour bar, unless the user switched it off (or nothing was drawn)."""
        if mappable is None or not self.disp["cbar"].get():
            return None
        if not (self.disp["fill_on"].get() or self.disp["lines_on"].get()):
            return None
        return self.fig.colorbar(mappable, ax=ax, label=label, **kw)

    def _contours(self, ax, X, Y, Z, alpha, st):
        """Filled shading and/or contour lines, honouring the level counts.

        Returns whatever can carry a colour bar - the filled set if there is one,
        otherwise the line set, otherwise None.
        """
        filled = lines = None
        lo, hi = st.get("lo", 0.0), st.get("hi", 100.0)
        if st["fill"] > 0:
            filled = ax.contourf(X, Y, Z, levels=np.linspace(lo, hi, st["fill"] + 1),
                                 cmap=st["cmap"], alpha=alpha, extend="both")
        if st["lines"] > 0:
            lines = ax.contour(X, Y, Z, levels=np.linspace(lo, hi, st["lines"] + 1),
                               colors="k" if filled is not None else None,
                               cmap=None if filled is not None else st["cmap"],
                               linewidths=.6, alpha=.6 if filled is not None else 1.0,
                               zorder=3)
            if st["labels"]:
                ax.clabel(lines, inline=True, fontsize=7, fmt="%.0f")
        return filled if filled is not None else lines

    def _build_status(self):
        bar = ctk.CTkFrame(self, fg_color=COLORS["card"], corner_radius=0, height=30,
                           border_width=1, border_color=COLORS["border"])
        bar.pack(fill="x")
        bar.pack_propagate(False)
        self.status = ctk.CTkLabel(bar, text="Ready", font=ctk.CTkFont(size=11),
                                   text_color=COLORS["text_muted"])
        self.status.pack(side="left", padx=18)

    # ------------------------------------------------------------- utilities
    def _welcome(self):
        self.log("Two-Speed Shift Optimiser\n"
                 "=" * 74 + "\n"
                 "Load a drive cycle (CSV) and an efficiency map (XLSX), pick an analysis,\n"
                 "then press Run.\n\n"
                 "Constraints enforced that the original notebook did not:\n"
                 "  - downshift must be below upshift (no inverted or zero-width bands)\n"
                 "  - motor torque limited by the constant-power envelope, not a flat cap\n"
                 "  - shifting costs energy and is rate-limited\n"
                 "  - an optimum landing on a search-range edge is reported as NOT converged\n")

    def log(self, text, clear=True):
        if clear:
            self.results.delete("1.0", "end")
        self.results.insert("end", text)
        self.results.see("end")

    def say(self, msg, kind="info"):
        self.status.configure(text=msg, text_color={
            "info": COLORS["text_muted"], "ok": COLORS["success"],
            "warn": COLORS["warning"], "err": COLORS["danger"]}[kind])

    def _refresh_checklist(self, *_):
        need = NEEDS.get(self.analysis.get(), ("cycle", "map"))
        have = {"cycle": self.cycle is not None, "map": self.emap is not None}
        name = {"cycle": "drive cycle", "map": "efficiency map"}
        if not need:
            self.checklist.configure(text="no data files required", text_color=COLORS["success"])
            return
        parts = [("[ok] " if have[k] else "[--] ") + name[k] for k in need]
        ready = all(have[k] for k in need)
        self.checklist.configure(text="   ".join(parts),
                                 text_color=COLORS["success"] if ready else COLORS["text_muted"])

    def _f(self, kind, attr, default=0.0):
        try:
            return float(self.entries[(kind, attr)].get().strip())
        except Exception:
            return default

    def params(self):
        veh = sc.Vehicle(self._f("veh", "mass", 995), self._f("veh", "wheel_radius", .247),
                         self._f("veh", "cda", 1.104), self._f("veh", "crr", .02),
                         grade_deg=self._f("veh", "grade_deg", 0.0))
        mot = sc.Motor(self._f("mot", "peak_torque", 45), self._f("mot", "peak_power", 11000),
                       self._f("mot", "max_rpm", 10000), self._f("mot", "inertia", .005))
        gb = sc.Gearbox(self._f("gb", "ratio_1", 19), self._f("gb", "ratio_2", 11),
                        self._f("gb", "eta_1", .97), self._f("gb", "eta_2", .97))
        el = sc.Electrical(self._f("el", "voltage", 52),
                           self._f("el", "aux_load", 150), self._f("el", "max_power", 15000),
                           regen_enabled=bool(self._f("num", "regen", 0)),
                           regen_fraction=self._f("el", "regen_fraction", 0.70),
                           regen_max_power=self._f("el", "regen_max_power", 0.0),
                           regen_min_speed_kmh=self._f("el", "regen_min_speed", 5.0))
        cost = sc.ShiftCost(
            max_shifts_per_hour=self._f("cost", "max_shifts_per_hour", np.inf) or np.inf,
            min_band_kmh=self._f("cost", "min_band_kmh", 2.0),
            min_accel_reserve=self._f("cost", "min_accel_reserve", 0.0),
            max_accel_loss=self._f("cost", "max_accel_loss", 1.0),
            actuator_voltage=self._f("cost", "actuator_voltage", 12.0),
            actuator_current=self._f("cost", "actuator_current", 20.0),
            actuator_time_s=self._f("cost", "actuator_time_s", 0.5))
        num = sc.Numerics(smooth_window=int(self._f("num", "smooth_window", 0)))
        return dict(veh=veh, motor=mot, gb=gb, elec=el, cost=cost, num=num)

    # ------------------------------------------------------------------ data
    def load_cycle(self):
        p = filedialog.askopenfilename(title="Drive cycle",
                                       filetypes=[("CSV", "*.csv"), ("All", "*.*")])
        if not p:
            return
        try:
            self.cycle = sc.load_cycle(p)
            self.say(f"Cycle: {self.cycle.name} — {len(self.cycle.time):,} samples, "
                     f"{self.cycle.duration/3600:.2f} h, {self.cycle.distance_km:.1f} km", "ok")
        except Exception as e:
            self.cycle = None
            self.say(f"Could not read cycle: {e}", "err")
        self._refresh_checklist()

    def load_map(self):
        p = filedialog.askopenfilename(title="Efficiency map",
                                       filetypes=[("Excel", "*.xlsx *.xls"), ("All", "*.*")])
        if not p:
            return
        try:
            self.emap = sc.load_efficiency_map(p)
            pk, pr, pt = self.emap.peak
            self.say(f"Map: peak {pk:.2%} at {pr:.0f} rpm / {pt:.1f} Nm, "
                     f"{self.emap.missing.sum():,} blank cells", "ok")
        except Exception as e:
            self.emap = None
            self.say(f"Could not read map: {e}", "err")
        self._refresh_checklist()

    def export_figure(self):
        p = filedialog.asksaveasfilename(defaultextension=".png",
                                         filetypes=[("PNG", "*.png"), ("PDF", "*.pdf")])
        if p:
            self.fig.savefig(p, dpi=200, bbox_inches="tight")
            self.say(f"Saved {Path(p).name}", "ok")

    # ------------------------------------------------------------------- run
    def run(self):
        kind = self.analysis.get()
        for k in NEEDS.get(kind, ("cycle", "map")):
            if (k == "cycle" and self.cycle is None) or (k == "map" and self.emap is None):
                self.say(f"Load the {'drive cycle' if k == 'cycle' else 'efficiency map'} first",
                         "err")
                return
        try:
            inputs = self._inputs()            # main thread - see _inputs
        except Exception:
            self.log(traceback.format_exc())
            self.say("Could not read the parameters", "err")
            return
        self.run_btn.configure(state="disabled", text="Running...")
        self.say(f"Running {kind}...")
        threading.Thread(target=self._work, args=(kind, inputs), daemon=True).start()

    def _inputs(self):
        """Read every widget the analysis needs - on the main thread, before the
        worker starts.

        Tk is not thread-safe. An entry read from the worker raises
        "main thread is not in main loop" whenever the main loop is not currently
        running, and `_f` swallows that and substitutes its own call-site default -
        i.e. the analysis silently runs on something other than what was typed.
        Reading here removes the whole class of failure.
        """
        return dict(
            p=self.params(),
            upshift=self._f("thr", "upshift", 22),
            downshift=self._f("thr", "downshift", 10),
            up_lo=self._f("sw", "up_lo", 8), up_hi=self._f("sw", "up_hi", 42),
            dn_lo=self._f("sw", "dn_lo", 4), dn_hi=self._f("sw", "dn_hi", 30),
            step=self._f("sw", "step", 1), min_band=self._f("sw", "min_band", 2),
            bin_rpm=self._f("bin", "bin_rpm", 500), bin_nm=self._f("bin", "bin_nm", 5),
            ref_mode=int(self._f("bin", "ref_mode", 0)),
            cmp_up=self._f("bin", "cmp_up", 32), cmp_dn=self._f("bin", "cmp_dn", 22),
            self_anchor=bool(self.disp["self_anchor"].get())
            if "self_anchor" in self.disp else True,
            v_target=self._f("run", "v_target", 30),
            throttle=self._f("run", "throttle", 1.0),
        )

    def _work(self, kind, I):
        """Runs on a worker thread: no widget access beyond this point."""
        try:
            p = I["p"]
            if kind == "Single strategy":
                out = sc.simulate(self.cycle, self.emap, I["upshift"], I["downshift"],
                                  keep_arrays=True, **p)
            elif kind in ("Upshift sweep", "Downshift sweep"):
                # A 1-D sweep must hold the OTHER threshold somewhere, and holding
                # it at a typed number makes the answer conditional on that number
                # - which is why sweeping the downshift against a typed 22 and the
                # upshift against a typed 10 produced two answers that agreed with
                # each other and with the grid only by luck. Anchor instead on the
                # coordinate-wise fixed point, where each threshold is optimal
                # GIVEN the other. Falls back to the typed value on request.
                anchor = None
                if I.get("self_anchor", True):
                    anchor = sc.fixed_point_thresholds(
                        self.cycle, self.emap, I["up_lo"], I["up_hi"], I["step"],
                        I["dn_lo"], I["dn_hi"], I["step"],
                        start_downshift=I["downshift"], **p)
                    # if the gates leave nothing feasible anywhere, the iteration
                    # has no optimum to anchor on - fall back rather than pass NaN
                    if not (np.isfinite(anchor["upshift"])
                            and np.isfinite(anchor["downshift"])):
                        anchor = None
                if kind == "Upshift sweep":
                    held = anchor["downshift"] if anchor else I["downshift"]
                    out = sc.sweep_upshift(self.cycle, self.emap, held,
                                           I["up_lo"], I["up_hi"], I["step"], **p)
                else:
                    held = anchor["upshift"] if anchor else I["upshift"]
                    out = sc.sweep_downshift(self.cycle, self.emap, held,
                                             I["dn_lo"], I["dn_hi"], I["step"], **p)
                out.anchor = anchor
                out.typed_anchor = (I["downshift"] if kind == "Upshift sweep"
                                    else I["upshift"])
            elif kind == "Combined grid":
                out = sc.sweep_grid(self.cycle, self.emap,
                                    I["up_lo"], I["up_hi"], I["step"],
                                    I["dn_lo"], I["dn_hi"], I["step"],
                                    min_band=I["min_band"], **p)
            elif kind == "Acceleration run":
                out = sc.wot_sweep(self.emap, I["v_target"], I["up_lo"], I["up_hi"],
                                   I["step"], throttle=I["throttle"], **p)
            elif kind == "Efficiency-only optimum":
                # The SAME cost model as every other analysis. This used to run with
                # shifting forced free, on the reasoning that mean_efficiency should
                # not be polluted by shift costs - but mean_efficiency is shaft output
                # over electrical input across the motoring samples, and neither the
                # actuator energy nor the traction cut appears in either integral, so
                # they cancel out of it anyway. Forcing them to zero changed nothing
                # about the efficiency ranking and put the net-energy figures this
                # panel also prints on a different footing from the sweeps - which is
                # exactly the inconsistency it looked like.
                out = sc.sweep_efficiency(self.cycle, self.emap,
                                          I["up_lo"], I["up_hi"], max(I["step"], 1.0),
                                          I["dn_lo"], I["dn_hi"], max(I["step"], 1.0),
                                          min_band=I["min_band"], **p)
            elif kind == "Loss breakdown":
                # Both thresholds, each anchored at the fixed point, so the two
                # columns are slices through ONE self-consistent operating point.
                a = sc.fixed_point_thresholds(
                    self.cycle, self.emap, I["up_lo"], I["up_hi"], I["step"],
                    I["dn_lo"], I["dn_hi"], I["step"],
                    start_downshift=I["downshift"], **p)
                hu = a["upshift"] if np.isfinite(a["upshift"]) else I["upshift"]
                hd = a["downshift"] if np.isfinite(a["downshift"]) else I["downshift"]
                su = sc.sweep_upshift(self.cycle, self.emap, hd,
                                      I["up_lo"], I["up_hi"], I["step"], **p)
                sd = sc.sweep_downshift(self.cycle, self.emap, hu,
                                        I["dn_lo"], I["dn_hi"], I["step"], **p)
                out = dict(
                    anchor=a, held_up=hu, held_dn=hd, sweeps={"upshift": su,
                                                              "downshift": sd},
                    terms={"upshift": sc.sweep_energy_terms(su, self.cycle,
                                                            self.emap, "upshift", **p),
                           "downshift": sc.sweep_energy_terms(sd, self.cycle,
                                                              self.emap, "downshift", **p)})
            elif kind == "Energy bins":
                kw = dict(veh=p["veh"], motor=p["motor"], gb=p["gb"], num=p["num"],
                          rpm_step=max(50.0, I["bin_rpm"]),
                          torque_step=max(0.5, I["bin_nm"]))
                # REFERENCE first, then the schedule from the Thresholds inputs.
                # The natural baseline is a single ratio - it is what the gearbox has
                # to beat - so "always gear 1" is the default rather than some other
                # arbitrary threshold pair.
                v_top = float(self.cycle.speed_kmh.max())
                mode = int(I.get("ref_mode", 0))
                if mode == 1:
                    ref_u, ref_d, ref_lab = 0.5, 0.2, "always gear 2"
                elif mode == 2:
                    ref_u, ref_d, ref_lab = I["cmp_up"], I["cmp_dn"], "custom"
                else:
                    ref_u, ref_d, ref_lab = v_top + 10, v_top + 5, "always gear 1"
                # the reference is a baseline, not a candidate: it is not required to
                # satisfy the driveability gates a real schedule must
                open_cost = sc.ShiftCost(p["cost"].energy_per_shift,
                                         p["cost"].interrupt_s, np.inf,
                                         min_band_kmh=0.0, min_accel_reserve=0.0,
                                         max_accel_loss=1.0)
                runs, bins = {}, {}
                runs["R"] = sc.simulate(self.cycle, self.emap, ref_u, ref_d,
                                        keep_arrays=True, **{**p, "cost": open_cost})
                runs["A"] = sc.simulate(self.cycle, self.emap, I["upshift"],
                                        I["downshift"], keep_arrays=True, **p)
                runs["_label"] = ref_lab
                ok_runs = [x for x in runs.values()
                           if hasattr(x, "gear") and x.gear is not None]
                kw["rpm_max"] = max((float(np.nanmax(np.abs(x.motor_rpm)))
                                     for x in ok_runs), default=0.0)
                kw["torque_max"] = max((float(np.nanmax(np.abs(x.motor_torque)))
                                        for x in ok_runs), default=0.0)
                for tag in ("R", "A"):
                    rr = runs[tag]
                    bins[tag] = sc.energy_bins(rr, self.cycle, **kw) \
                        if rr.gear is not None else None
                out = (runs, bins)
            elif kind == "Gradeability":
                out = sc.gradeability_table(veh=p["veh"], motor=p["motor"], gb=p["gb"])
            elif kind in ("Points on map", "Shift movement"):
                out = sc.simulate(self.cycle, self.emap, I["upshift"], I["downshift"],
                                  keep_arrays=True, **p)
            elif kind == "Gear comparison":
                out = sc.gear_efficiency_curves(self.emap, veh=p["veh"], motor=p["motor"],
                                                gb=p["gb"])
            elif kind == "Optimal gear map":
                out = sc.optimal_gear_map(self.emap, veh=p["veh"], motor=p["motor"],
                                          gb=p["gb"])
            else:
                out = None
            self._q.put(("ok", kind, out, p))
        except Exception:
            self._q.put(("err", kind, traceback.format_exc(), None))

    def _drain(self):
        try:
            while True:
                tag, kind, payload, p = self._q.get_nowait()
                self.run_btn.configure(state="normal", text="Run analysis")
                if tag == "err":
                    self.log(payload)
                    self.say("Analysis failed — see the panel below", "err")
                else:
                    try:
                        self._render(kind, payload, p)
                    except Exception:
                        self.log(traceback.format_exc())
                        self.say("Plotting failed", "err")
        except queue.Empty:
            pass
        self.after(120, self._drain)

    # --------------------------------------------------------------- drawing
    # Explicit, because deriving the method from the first word of the analysis
    # name silently collides as soon as two analyses share one ("Efficiency map"
    # and "Efficiency-only optimum" both give "_draw_efficiency").
    DRAW = {
        "Single strategy": "_draw_single",
        "Points on map": "_draw_points",
        "Shift movement": "_draw_shift",
        "Gear comparison": "_draw_gear",
        "Optimal gear map": "_draw_optimal",
        "Upshift sweep": "_draw_upshift",
        "Downshift sweep": "_draw_downshift",
        "Combined grid": "_draw_combined",
        "Efficiency-only optimum": "_draw_effopt",
        "Loss breakdown": "_draw_lossshare",
        "Energy bins": "_draw_bins",
        "Gradeability": "_draw_gradeability",
        "Acceleration run": "_draw_acceleration",
        "Efficiency map": "_draw_efficiency",
    }

    def _render(self, kind, out, p):
        self._last_render = (kind, out, p)
        self.fig.clear()
        getattr(self, self.DRAW[kind])(out, p, kind)
        # Layout engine rather than a one-shot tight_layout: the figure is resized
        # by the host after this call and must re-fit itself. set_layout_engine
        # needs matplotlib >= 3.6; fall back so older installs still draw.
        try:
            self.fig.set_layout_engine("tight")
        except Exception:
            try:
                self.fig.tight_layout()
            except Exception:
                pass
        self._footnote(kind)
        self._fit_canvas(kind)
        self.canvas.draw()

    def _footnote(self, kind):
        """Stamp every figure with what produced it.

        A figure pasted into a deck is separated from the app that made it. Without
        the schedule, the source files and the data caveat travelling with it, the
        reader has no way to know what they are looking at - and the synthetic-data
        warning is exactly the thing that must not get lost.
        """
        bits = []
        if self.cycle is not None:
            bits.append(f"cycle {self.cycle.name}  ({self.cycle.distance_km:.0f} km, "
                        f"{self.cycle.duration/3600:.1f} h)")
        if self.emap is not None:
            bits.append(f"map peak {self.emap.peak[0]:.1%} @ {self.emap.peak[1]:.0f} rpm "
                        f"/ {self.emap.peak[2]:.1f} Nm")
        if kind not in ("Gradeability", "Efficiency map", "Gear comparison",
                        "Optimal gear map", "Acceleration run"):
            bits.append(f"upshift {self._f('thr','upshift',22):g} / downshift "
                        f"{self._f('thr','downshift',10):g} km/h")
        p_txt = (f"m {self._f('veh','mass',995):g} kg  "
                 f"ratios {self._f('gb','ratio_1',19):g}/{self._f('gb','ratio_2',11):g}  "
                 f"motor {self._f('mot','peak_torque',45):g} Nm / "
                 f"{self._f('mot','peak_power',11000)/1000:g} kW")
        line = " | ".join(bits) if bits else ""
        self.fig.text(.008, .012, line, fontsize=6.6, color="#7a848d", ha="left",
                      va="bottom")
        self.fig.text(.992, .012, p_txt, fontsize=6.6, color="#7a848d", ha="right",
                      va="bottom")

    def _fit_canvas(self, kind):
        """Canvas height: the user's if they set one, otherwise fitted to the view.

        In the simple layout the canvas fills its panel, so forcing a height would
        only fight the geometry manager - leave it alone there.
        """
        if self.simple_layout:
            return
        if self._manual_h:
            self.canvas.get_tk_widget().configure(height=self._manual_h)
            self._warn_if_invisible()
            return

        # Fit the viewport unless the content genuinely needs more room. Forcing a
        # fixed height taller than the visible area pushes the figure below the fold
        # and the panel reads as blank - which is exactly what happens on a smaller
        # screen or at higher display scaling.
        want = 155 * len(self.fig.axes) if kind == "Single strategy" else 0
        self.canvas.get_tk_widget().configure(height=max(want, self._viewport_h()))
        self._warn_if_invisible()

    def _place_initial_sashes(self):
        """One-off: open at a width that fits the parameter rows without clipping."""
        try:
            self.split_h.sash_place(0, 348, 0)
            h = self.split_v.winfo_height()
            if h > 300:
                self.split_v.sash_place(0, 0, h - 186)
            self._on_sash_moved()
        except Exception:
            pass

    def _on_sash_moved(self, _event=None):
        """Refit the figure to the panel after a splitter drag."""
        if self._last_render is None:
            return
        self.after_idle(lambda: (self._fit_canvas(self._last_render[0]),
                                 self.canvas.draw_idle()))

    def _viewport_h(self) -> int:
        """Visible height of the scroll area, in pixels, with a sane fallback."""
        try:
            self.update_idletasks()
            for widget in (getattr(self.plot_host, "_parent_canvas", None),
                           self.plot_host):
                if widget is not None:
                    h = widget.winfo_height()
                    if h > 1:
                        return int(h)
        except Exception:
            pass
        return 420

    def _warn_if_invisible(self):
        """Say something useful if the plot widget has no height on this machine.

        A silent blank panel is the worst failure mode: every number is right, the
        figure is drawn, and there is nothing on screen to explain it.
        """
        if self.simple_layout or getattr(self, "_layout_warned", False):
            return
        try:
            self.update_idletasks()
            w = self.canvas.get_tk_widget()
            if w.winfo_ismapped() and w.winfo_height() <= 1:
                self._layout_warned = True
                self.say("Plot area has no height on this display - restart with "
                         "SHIFT_APP_SIMPLE_LAYOUT=1 (see README)", "err")
        except Exception:
            pass

    def _set_plot_height(self, px):
        self._manual_h = int(round(float(px)))
        self.height_label.configure(text=f"{self._manual_h} px")
        self.canvas.get_tk_widget().configure(height=self._manual_h)
        self.canvas.draw_idle()

    def _auto_plot_height(self):
        self._manual_h = None
        self.height_label.configure(text="auto")
        self._fit_canvas(self._last_render[0] if self._last_render else "")
        self.canvas.draw_idle()

    def _draw_single(self, r, p, _kind):
        if not r.feasible and r.gear is None:
            self.fig.text(.5, .5, "Infeasible strategy\n\n" + "\n".join(r.reasons),
                          ha="center", va="center", fontsize=13, color=COLORS["danger"])
            self.log("INFEASIBLE — " + f"upshift {r.upshift:g} / downshift {r.downshift:g}\n"
                     + "=" * 74 + "\n" + "\n".join("  - " + x for x in r.reasons))
            self.say("Strategy rejected", "err")
            return
        t = self.cycle.time
        keys = [k for k, _, _ in SIGNALS if self.sig[k].get()] or ["speed"]
        lo, hi = self._time_window(t)
        m = (t >= lo) & (t <= hi)
        tt = t[m]

        ax = np.atleast_1d(self.fig.subplots(len(keys), 1, sharex=True, squeeze=False)[:, 0])
        shifts = np.flatnonzero(np.diff(r.gear) != 0)
        shifts = shifts[(t[shifts] >= lo) & (t[shifts] <= hi)]

        for a, k in zip(ax, keys):
            y, lab, col = self._signal(r, k, t)
            if k == "gear":
                a.step(tt, y[m], where="post", lw=.8, color=col)
                a.set_yticks([1, 2]); a.set_ylim(.7, 2.3)
            else:
                a.plot(tt, y[m], lw=.7, color=col)
            if k == "speed":
                a.axhline(r.upshift, ls="--", color=COLORS["success"], lw=1,
                          label=f"upshift {r.upshift:g}")
                a.axhline(r.downshift, ls="--", color=COLORS["warning"], lw=1,
                          label=f"downshift {r.downshift:g}")
                a.legend(fontsize=8, ncol=2, loc="upper right")
            if k in ("torque", "pbatt", "pmech", "accel"):
                a.axhline(0, color="k", lw=.7)
            if k == "rpm":
                a.axhline(p["motor"].max_rpm, ls=":", color=COLORS["danger"], lw=1)
            if k == "torque":
                a.axhline(p["motor"].peak_torque, ls=":", color=COLORS["danger"], lw=1)
                a.axhline(-p["motor"].peak_torque, ls=":", color=COLORS["danger"], lw=1)
            # gear-change markers, only while few enough to read
            if self.disp["marks"].get() and 0 < len(shifts) <= 120:
                for i in shifts:
                    a.axvline(t[i], color=COLORS["accent"], lw=.6, alpha=.35, zorder=0)
            # Horizontal, outside the axes. A rotated label is taller than a 90 px
            # panel, so with six stacked traces the neighbouring labels overlap each
            # other - which is what made this figure unreadable.
            a.set_ylabel(lab, fontsize=9, rotation=0, ha="right", va="center")
            a.yaxis.set_label_coords(-0.055, 0.5)   # clear of the tick numbers
            a.tick_params(labelsize=8.5)
            a.grid(alpha=.25)
        ax[-1].set_xlabel("Time [s]", fontsize=10)
        ax[0].set_xlim(lo, hi)

        full_view = not self.disp["window"].get()[0].isdigit()
        win = ("the full cycle" if full_view
               else f"{lo:.0f}-{hi:.0f} s of {t[-1]:.0f} s")
        self.fig.suptitle(f"Cycle traces at upshift {r.upshift:g} / downshift "
                          f"{r.downshift:g} km/h   —   {r.net_kwh:.3f} kWh, "
                          f"{r.wh_per_km:.1f} Wh/km", fontweight="bold", fontsize=12.5)
        ax[0].set_title(f"{win} · {len(shifts)} gear changes in view"
                        + ("  ·  at this scale ~30 samples fall on each pixel — use the "
                           "TIME WINDOW control to read a slice" if full_view else ""),
                        fontsize=8.5, color=COLORS["text_muted"], loc="left", pad=4)
        self.log(self._summary(r))
        self.say(f"{r.consumed_kwh:.3f} kWh — {r.wh_per_km:.1f} Wh/km "
                 f"— {len(keys)} signals plotted", "ok")

    def _signal(self, r, key, t):
        """(values, axis label, colour) for one time-series signal.

        Labels are short and two-line because they are drawn horizontally beside a
        stacked panel: a rotated label is taller than a 90 px panel and neighbouring
        ones then overlap each other.
        """
        if key == "speed":
            return self.cycle.speed_kmh, "Speed\n[km/h]", COLORS["primary"]
        if key == "gear":
            return r.gear.astype(float), "Gear", COLORS["text"]
        if key == "rpm":
            return r.motor_rpm, "Motor\n[rpm]", COLORS["accent"]
        if key == "torque":
            return r.motor_torque, "Torque\n[Nm]", COLORS["warning"]
        if key == "eff":
            # eff is filled with 1.0 wherever the motor is idle - blank those out
            act = (np.abs(r.motor_torque) > 1e-9) & (r.motor_eff < 1.0)
            return (np.where(act, r.motor_eff * 100, np.nan), "Efficiency\n[%]",
                    COLORS["success"])
        if key == "pbatt":
            return r.battery_power / 1000.0, "Battery\n[kW]", COLORS["secondary"]
        if key == "pmech":
            return ((r.motor_torque * r.motor_rpm * 2 * np.pi / 60.0) / 1000.0,
                    "Shaft\n[kW]", "#0891b2")
        if key == "accel":
            return (np.gradient(self.cycle.speed_kmh / 3.6, t, edge_order=2),
                    "Accel\n[m/s2]", "#7c3aed")
        if key == "energy":
            pos = np.maximum(np.nan_to_num(r.battery_power), 0.0)
            e = np.concatenate([[0.0], np.cumsum(.5 * (pos[1:] + pos[:-1]) * np.diff(t))]) / 3.6e6
            return e, "Cumulative\n[kWh]", COLORS["danger"]
        raise KeyError(key)

    def _draw_upshift(self, sw, p, kind):
        self._sweep_plot(sw, "upshift", "1-2 Upshift speed [km/h]", kind)

    def _draw_downshift(self, sw, p, kind):
        self._sweep_plot(sw, "downshift", "2-1 Downshift speed [km/h]", kind)

    def _sweep_plot(self, sw, col, xlabel, kind):
        ax = self.fig.subplots()
        df = sw.table
        ok = df[df["feasible"]]
        bad = df[~df["feasible"]]
        ax.plot(ok[col], ok["net_kwh"], "o-", color=COLORS["primary"],
                label="net energy", ms=5, zorder=4)
        # rejected candidates as bands, not as points: plotted at an arbitrary height
        # they read as data and invite the eye to compare them against the curve
        for xv in bad[col]:
            ax.axvline(xv, color=COLORS["danger"], lw=6, alpha=.13, zorder=0)
        if len(bad):
            ax.plot([], [], lw=6, color=COLORS["danger"], alpha=.3,
                    label=f"rejected by the schedule gates ({len(bad)})")
        if sw.best:
            b = getattr(sw.best, col)
            ax.scatter([b], [sw.best.net_kwh], s=180, zorder=6,
                       facecolor=COLORS["success"], edgecolor="k", lw=1.2,
                       label=f"best {b:g} km/h")
            ax.annotate(f"{b:g} km/h\n{sw.best.net_kwh:.3f} kWh",
                        (b, sw.best.net_kwh), textcoords="offset points",
                        xytext=(10, 14), fontsize=8.5, fontweight="bold",
                        color=COLORS["success"],
                        bbox=dict(boxstyle="round,pad=.25", fc=COLORS["plot_bg"],
                                  ec=COLORS["success"], lw=.8, alpha=.9))
        lo, hi = df[col].min(), df[col].max()
        for v in (lo, hi):
            ax.axvline(v, color=COLORS["text_muted"], ls=":", lw=1.1, alpha=.6)
        ax.annotate("searched range", (lo, ax.get_ylim()[0]),
                    textcoords="offset points", xytext=(4, 4), fontsize=7.5,
                    color=COLORS["text_muted"])
        # The efficiency curve is the whole point and used to be invisible here: the
        # energy axis alone cannot tell you whether a candidate won by moving the
        # operating points somewhere better or by some other term.
        if "mean_efficiency" in df.columns and len(ok):
            ax2 = ax.twinx()
            ax2.plot(ok[col], ok["mean_efficiency"] * 100, "s--", ms=3, alpha=.85,
                     color=COLORS["warning"], label="mean motor efficiency")
            ax2.set_ylabel("Mean motor efficiency [%]", color=COLORS["warning"])
            ax2.tick_params(axis="y", labelcolor=COLORS["warning"])
            if sw.best is not None:
                ax2.scatter([getattr(sw.best, col)], [sw.best.mean_efficiency * 100],
                            s=70, zorder=6, facecolor=COLORS["warning"],
                            edgecolor="k", lw=1)
        ax.set_xlabel(xlabel); ax.set_ylabel("Net battery energy [kWh]")
        ax.grid(alpha=.25)
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = (ax2.get_legend_handles_labels() if "mean_efficiency" in df.columns
                  and len(ok) else ([], []))
        ax.legend(h1 + h2, l1 + l2, fontsize=8.5, loc="upper left", framealpha=.92)
        if sw.best is not None and len(ok) > 1:
            spread = 1000 * (ok["net_kwh"].max() - ok["net_kwh"].min())
            head = (f"{kind}: best at {getattr(sw.best, col):g} km/h  ·  "
                    f"{spread:.0f} Wh across the searched range  ·  efficiency "
                    f"{ok['mean_efficiency'].min():.1%} to {ok['mean_efficiency'].max():.1%}")
        else:
            head = kind
        ax.set_title(head + ("" if sw.converged else "   —   NOT CONVERGED"),
                     fontweight="bold", fontsize=11.5,
                     color=COLORS["text"] if sw.converged else COLORS["danger"])
        self.log(self._sweep_summary(sw, col))
        self.say(("Converged: " if sw.converged else "NOT converged: ") +
                 (f"best {getattr(sw.best, col):g} km/h" if sw.best else "no feasible candidate"),
                 "ok" if sw.converged else "warn")

    def _draw_combined(self, sw, p, kind):
        ax = self.fig.subplots()
        df = sw.table[sw.table["feasible"]]
        if df.empty:
            self.fig.text(.5, .5, "No feasible candidate", ha="center", color=COLORS["danger"])
            self.log("No feasible candidate in the grid.")
            return
        # NET, to match the objective every sweep minimises - the colour bar used to
        # say "consumed" while the ranking used net, which is a quiet contradiction
        piv = df.pivot_table(index="downshift", columns="upshift", values="net_kwh")
        m = ax.pcolormesh(piv.columns, piv.index, piv.to_numpy(), shading="auto",
                          cmap=self._map_style()["cmap"])
        self._colorbar(m, ax, "Net battery energy [kWh]")
        if sw.best:
            ax.scatter([sw.best.upshift], [sw.best.downshift], marker="X", s=230,
                       facecolor=COLORS["success"], edgecolor="k", lw=1.4, zorder=5)
            # label at the marker, not in a corner legend the eye has to pair up
            ax.annotate(f"best  {sw.best.upshift:g}/{sw.best.downshift:g}\n"
                        f"{sw.best.net_kwh:.3f} kWh",
                        (sw.best.upshift, sw.best.downshift),
                        textcoords="offset points", xytext=(12, 12), fontsize=9,
                        fontweight="bold", color=COLORS["success"],
                        bbox=dict(boxstyle="round,pad=.3", fc=COLORS["plot_bg"],
                                  ec=COLORS["success"], lw=1, alpha=.92),
                        arrowprops=dict(arrowstyle="-", color=COLORS["success"], lw=1))
        ax.set_xlabel("1-2 upshift speed [km/h]")
        ax.set_ylabel("2-1 downshift speed [km/h]")
        spread = 1000 * (df["net_kwh"].max() - df["net_kwh"].min())
        ax.set_title(f"Every threshold pair  ·  best "
                     f"{sw.best.upshift:g}/{sw.best.downshift:g}  ·  {spread:.0f} Wh "
                     f"between best and worst"
                     + ("" if sw.converged else "   —   NOT CONVERGED"),
                     fontweight="bold", fontsize=11.5,
                     color=COLORS["text"] if sw.converged else COLORS["danger"])
        # label the empty region in place rather than under the x-axis, where it
        # collided with the provenance footer
        ax.text(.02, .97,
                "white = rejected by the schedule gates" + chr(10)
                + "(band width, acceleration reserve, shift rate)",
                transform=ax.transAxes, ha="left", va="top", fontsize=8,
                color=COLORS["text_muted"],
                bbox=dict(boxstyle="round,pad=.3", fc=COLORS["plot_bg"],
                          ec=COLORS["border"], lw=.7, alpha=.85))
        self.log(self._sweep_summary(sw, "both thresholds"))
        self.say(("Converged: " if sw.converged else "NOT converged: ") +
                 f"best {sw.best.upshift:g}/{sw.best.downshift:g}",
                 "ok" if sw.converged else "warn")

    def _map_background(self, ax, p, alpha=0.75):
        """Efficiency contours + envelope, shared by the map-overlay views."""
        em = self.emap
        st = self._map_style()
        R, T = np.meshgrid(em.rpm, em.torque)
        c = self._contours(ax, R, T, np.where(em.missing, np.nan, em.eff) * 100, alpha, st)
        pk, pr, pt = em.peak
        if st["envelope"]:
            n = np.linspace(1, p["motor"].max_rpm, 400)
            ax.plot(n, p["motor"].envelope(n), color=COLORS["danger"], lw=2.2, zorder=4,
                    label=f"{p['motor'].peak_torque:g} Nm / "
                          f"{p['motor'].peak_power/1000:g} kW limit")
            ax.scatter([pr], [pt], s=150, marker="*", color="w", edgecolor="k", zorder=6,
                       label=f"peak {pk:.1%}")
        ax.set_xlim(0, em.rpm.max())
        ax.set_ylim(0, min(em.torque.max(), p["motor"].peak_torque))
        ax.set_xlabel("Motor speed [rpm]")
        ax.set_ylabel("Motor torque [Nm]")
        self._iso_power(ax)
        self._ridge(ax)
        return c

    def _draw_points(self, r, p, _kind):
        """Where the cycle actually sits on the efficiency map, per gear."""
        if r.gear is None:
            self.fig.text(.5, .5, "Infeasible strategy\n\n" + "\n".join(r.reasons),
                          ha="center", va="center", color=COLORS["danger"])
            self.log("INFEASIBLE\n" + "\n".join("  - " + x for x in r.reasons)); return
        signed = self.disp["negative"].get()
        em = self.emap
        act = np.abs(r.motor_torque) > 1e-9
        ax = self.fig.subplots(1, 2, sharex=True, sharey=True)
        if signed:
            lim = min(em.torque.max(), p["motor"].peak_torque)
            pad = .08 * lim
            tlo, thi = -lim - pad, lim + pad
        for a, gear, col, mk in ((ax[0], 1, "#ffffff", "o"), (ax[1], 2, "#ffd400", "^")):
            c = (self._signed_background(a, p, tlo, thi, alpha=.8) if signed
                 else self._map_background(a, p, alpha=.8))
            groups = self._point_groups(r, p, gear, act, signed, col, mk)
            for m, lab, alpha, face, marker in groups:
                idx = np.flatnonzero(m)
                if not len(idx):
                    continue
                if len(idx) > 4000:
                    idx = idx[:: int(np.ceil(len(idx) / 4000))]
                a.scatter(r.motor_rpm[idx], r.motor_torque[idx], s=7, marker=marker,
                          facecolor=face, edgecolor="k", linewidth=.15, alpha=alpha,
                          zorder=5, label=f"Gear {gear} {lab}  ({int(m.sum()):,} pts)")
            a.set_title(f"Gear {gear} (ratio "
                        f"{p['gb'].ratio_1 if gear == 1 else p['gb'].ratio_2:g})",
                        fontweight="bold")
            # upper-left: the high-torque/low-rpm corner is empty in both panels,
            # and on the signed axes "upper right" collides with the colour bar
            # upper-left always: the high-torque/low-rpm corner is outside the
            # envelope in both modes, and "upper right" collides with the colour bar
            a.legend(fontsize=8, loc="upper left", framealpha=.9)
        if signed:
            ax[0].set_ylabel("Signed motor torque [Nm]\n(+ motoring,  - braking)")
        self._colorbar(c, ax, "Efficiency [%]" + ("  (negative half mirrored)" if signed
                                                  else ""), fraction=.04)
        self.fig.suptitle(f"Where the cycle sits on the map — upshift {r.upshift:g} / "
                          f"downshift {r.downshift:g} km/h", fontweight="bold")

        L = ["Operating points on the efficiency map", "=" * 74, ""]
        L += self._gear_table(r)
        L += self._choice_score(r, p)
        L += ["",
              "  These are the same figures the Single-strategy summary prints; both are",
              "  built by shift_core.gear_breakdown() so they cannot drift apart."]
        L += self._duty_insight(r, p)
        L += ["", f"  Split shown: {self.disp['ptmode'].get()}."]
        if signed:
            L += ["",
                  "  The braking half (negative torque) is drawn from the map MIRRORED about",
                  "  zero torque - eta(-T, n) assumed equal to eta(|T|, n). That is an",
                  "  assumption, not measured data. With regen disabled those points carry no",
                  "  energy at all, so they change nothing in the totals above."]
        self.log("\n".join(L))
        self.say("Operating cloud plotted" + (" with the braking half" if signed else ""),
                 "ok")

    def _point_groups(self, r, p, gear, act, signed, col, mk):
        """Split one gear's operating points into the groups the chosen mode implies.

        Returns [(mask, label, alpha, facecolour, marker), ...]. Each mode answers a
        different question about the same cloud:

          motoring / braking      is the motor pulling or being driven
          right or wrong ratio    was this the more efficient ratio at that instant
          accel / cruise / brake  which regime - and the gear preference INVERTS
                                  between them, which is the study's central finding
          just shifted / settled  is this a transient right after a change, or the
                                  steady operation the schedule actually buys
        """
        mode = self.disp["ptmode"].get()
        base = act & (r.gear == gear)
        pull = base & (r.motor_torque > 0)

        if mode == POINT_MODES[1]:                       # right or wrong ratio
            better, valid = sc.better_gear_per_sample(
                self.cycle, self.emap, veh=p["veh"], motor=p["motor"], gb=p["gb"],
                num=p["num"])
            ok = pull & valid
            return [(ok & (better == gear), "RIGHT ratio", .6, "#1a7f4b", mk),
                    (ok & (better != gear), "wrong ratio", .6, "#c62828", mk)]

        if mode == POINT_MODES[2]:                       # regime
            a = np.gradient(self.cycle.speed_kmh / 3.6, self.cycle.time)
            return [(pull & (a > .15), "accelerating", .6, "#c62828", mk),
                    (pull & (np.abs(a) <= .15), "cruising", .6, "#1a7f4b", mk),
                    (base & (r.motor_torque < 0), "braking", .5, "#2b5fa8", mk)]

        if mode == POINT_MODES[3]:                       # transient or settled
            sh = np.flatnonzero(np.diff(r.gear) != 0)
            fresh = np.zeros(len(r.gear), dtype=bool)
            for off in range(0, 6):                      # ~5 s after a change
                k = sh + off
                fresh[k[k < len(fresh)]] = True
            return [(pull & fresh, "just shifted (<5 s)", .75, "#c62828", mk),
                    (pull & ~fresh, "settled in gear", .5, "#1a7f4b", mk)]

        groups = [(pull, "motoring", .55, col, mk)]      # default
        if signed:
            groups.append((base & (r.motor_torque < 0), "braking", .5, "none", mk))
        return groups

    def _duty_insight(self, r, p):
        """What this cycle asks of THIS motor, computed - never asserted.

        The claims that belong here (how hard the motor is worked, how far the duty
        sits from the map's best point, how much efficiency the ridge is still
        holding) are all properties of the loaded data. Hard-coding them means the
        tool keeps printing the old map's numbers after someone loads a new one.
        """
        try:
            em, mot, gb, num = self.emap, p["motor"], p["gb"], p["num"]
            ratios = np.where(r.gear == 1, gb.ratio_1, gb.ratio_2)
            etas = np.where(r.gear == 1, gb.eta_1, gb.eta_2)
            t_w, n_w, p_w = sc.road_load(self.cycle, p["veh"], mot, gb, num, ratios)
            use = ((p_w > num.power_epsilon) & np.isfinite(r.motor_eff)
                   & (r.motor_eff < 1.0) & (r.motor_eff > 0))
            if not use.any():
                return []
            w = np.where(use, p_w, 0.0)
            tq = np.abs(r.motor_torque)
            rpm = np.abs(r.motor_rpm)
            shaft = tq * rpm * 2 * np.pi / 60.0

            t_mean = float(np.average(tq[use], weights=w[use]))
            p_mean = float(np.average(shaft[use], weights=w[use]))
            pk, pr, pt = em.peak
            p_peak = pt * pr * 2 * np.pi / 60.0

            # how much efficiency the RIDGE still holds at the torque actually used
            t_r, n_r = sc.efficiency_ridge(em)
            i_near = int(np.argmin(np.abs(t_r - t_mean)))
            ridge_rpm = n_r[i_near]
            e_here = float(np.average(r.motor_eff[use], weights=w[use]))
            e_ridge = em.query(np.array([ridge_rpm]), np.array([t_mean]),
                               np.array([True]))[0][0]
        except Exception:
            return []

        L = ["", "  WHAT THIS CYCLE ASKS OF THIS MOTOR", "  " + "-" * 58,
             f"    energy-weighted mean torque   {t_mean:6.2f} Nm   "
             f"= {100*t_mean/mot.peak_torque:.0f} % of the {mot.peak_torque:g} Nm rating",
             f"    energy-weighted mean power    {p_mean/1000:6.2f} kW   "
             f"= {100*p_mean/mot.peak_power:.0f} % of the {mot.peak_power/1000:g} kW rating",
             f"    the map's best cell           {pk:6.2%} at {pr:.0f} rpm / {pt:.1f} Nm"
             f"  = {p_peak/1000:.2f} kW"]
        if p_mean > 0:
            L.append(f"    -> the map's best point is a {p_peak/p_mean:.1f}x bigger load "
                     f"than this duty averages.")
            L.append("       A ratio slides the operating point ALONG a constant-power")
            L.append("       curve, so that cell is not reachable from this cycle at any")
            L.append("       ratio. Judge the schedule against the RIDGE, not the peak.")
        L += ["",
              f"    at {t_mean:.1f} Nm the ridge sits at {ridge_rpm:.0f} rpm and reaches "
              f"{e_ridge:.2%};",
              f"    this schedule averages {e_here:.2%}, so {100*(e_ridge-e_here):+.2f} "
              f"points are still on the table",
              "    at that torque - the gap a better schedule could close."]
        return L

    def _choice_score(self, r, p):
        """How often this schedule engaged the ratio that was actually better.

        A shift schedule is a guess. This scores the guess against the per-sample
        right answer, weighted by the work being done - which is the audit the
        'Shift movement' view cannot provide, because a shift happens under
        acceleration where the LOW ratio wins, while the benefit is collected
        afterwards at cruise where the HIGH ratio wins.
        """
        try:
            better, valid = sc.better_gear_per_sample(
                self.cycle, self.emap, veh=p["veh"], motor=p["motor"], gb=p["gb"],
                num=p["num"])
            ratios = np.where(r.gear == 1, p["gb"].ratio_1, p["gb"].ratio_2)
            t_w, n_w, p_w = sc.road_load(self.cycle, p["veh"], p["motor"], p["gb"],
                                         p["num"], ratios)
            use = valid & (p_w > 0)
            w = np.where(use, p_w, 0.0)
            right = use & (r.gear == better)
            e_all = sc._trapz(w, self.cycle.time) / 3.6e6
            e_right = sc._trapz(np.where(right, p_w, 0.0), self.cycle.time) / 3.6e6
            if e_all <= 0:
                return []
        except Exception:
            return []
        return ["", "  DID THIS SCHEDULE PICK THE RIGHT RATIO?", "  " + "-" * 56,
                f"    energy delivered through the better ratio  "
                f"{100*e_right/e_all:5.1f} %",
                f"    energy delivered through the worse ratio   "
                f"{e_all-e_right:5.3f} kWh",
                "    Tick 'Colour points by gear choice' to see which points those are.",
                "",
                "    A shift happens under acceleration, where the LOW ratio is the",
                "    efficient one, so the arrow in 'Shift movement' points the wrong",
                "    way at the instant. The cycle then settles to cruise, where the",
                "    HIGH ratio is worth several points, and that is where the gain is",
                "    collected. That is why the shift view cannot justify a schedule",
                "    and this score can."]

    def _signed_background(self, ax, p, tmin, tmax, alpha=0.72):
        """Efficiency contours mirrored about zero torque, so braking points show.

        The map is measured for motoring only; the negative-torque half assumes
        eta(-T, n) = eta(|T|, n). That is an assumption, not data — it is labelled
        as such on the axis.
        """
        em = self.emap
        pos = em.torque > 0
        t_signed = np.concatenate([-em.torque[pos][::-1], em.torque])
        e_signed = np.vstack([np.where(em.missing, np.nan, em.eff)[pos][::-1],
                              np.where(em.missing, np.nan, em.eff)])
        R, T = np.meshgrid(em.rpm, t_signed)
        st = self._map_style()
        c = self._contours(ax, R, T, e_signed * 100, alpha, st)
        if st["envelope"]:
            n = np.linspace(1, p["motor"].max_rpm, 400)
            env = p["motor"].envelope(n)
            ax.plot(n, env, color=COLORS["danger"], lw=2, zorder=4)
            ax.plot(n, -env, color=COLORS["danger"], lw=2, zorder=4)
        ax.axhline(0, color="k", lw=.9, zorder=4)
        ax.set_xlim(0, em.rpm.max())
        ax.set_ylim(tmin, tmax)
        ax.set_xlabel("Motor speed [rpm]")
        self._iso_power(ax, signed=True)
        self._ridge(ax, signed=True)
        return c

    def _draw_shift(self, r, p, _kind):
        """Where every gear change throws the operating point — one panel each.

        Upshifts and downshifts are drawn separately because they live in different
        parts of the plane: upshifts happen under power (positive torque), downshifts
        almost always during braking (negative torque). Sharing one axis hid most of
        the downshift arrows below y = 0.
        """
        if r.gear is None:
            self.fig.text(.5, .5, "Infeasible strategy", ha="center", color=COLORS["danger"])
            return
        sh = np.flatnonzero(np.diff(r.gear) != 0)
        ups = sh[r.gear[sh + 1] == 2]
        dns = sh[r.gear[sh + 1] == 1]

        # one shared y-range that actually contains every arrow endpoint
        allt = np.concatenate([r.motor_torque[np.concatenate([ups, dns, ups + 1, dns + 1])]]) \
            if len(sh) else np.array([0.0, 1.0])
        pad = max(2.0, .1 * (allt.max() - allt.min()))
        tmin, tmax = allt.min() - pad, allt.max() + pad

        if not self.disp["negative"].get():
            tmin = 0.0                       # motoring half only, by request
        ax = self.fig.subplots(1, 2, sharex=True, sharey=True)
        act_all = np.abs(r.motor_torque) > 1e-9
        for A, grp, col, lab in ((ax[0], ups, COLORS["success"], "Upshift  1 -> 2"),
                                 (ax[1], dns, COLORS["warning"], "Downshift  2 -> 1")):
            c = self._signed_background(A, p, tmin, tmax)

            # The whole cloud underneath, so the shift points can be read in context.
            # DARK colours on purpose: the cycle lives in the bright yellow-green part
            # of viridis, where white and gold at low alpha are invisible.
            for gear, on, colr, mk in (
                    (1, self.disp["lay_g1"].get(), CLOUD_G1, "o"),
                    (2, self.disp["lay_g2"].get(), CLOUD_G2, "^")):
                if not on:
                    continue
                m = act_all & (r.gear == gear)
                idx = np.flatnonzero(m)
                if not len(idx):
                    continue
                if len(idx) > 6000:
                    idx = idx[:: int(np.ceil(len(idx) / 6000))]
                A.scatter(r.motor_rpm[idx], r.motor_torque[idx], s=9, marker=mk,
                          facecolor=colr, edgecolor="none", alpha=.55, zorder=3,
                          label=f"all gear {gear}  ({int(m.sum()):,})")
            # every shift, not a sample of thirty: thin the ink instead of the data
            take = grp
            dense = max(1, len(take))
            a_lw = 1.7 if dense <= 40 else (1.1 if dense <= 150 else 0.7)
            a_al = .95 if dense <= 40 else (.7 if dense <= 150 else .45)
            new_gear = 2 if lab.startswith("Upshift") else 1
            if len(take):
                cf_n, cf_t = sc.counterfactual_point(r, take, new_gear, p["gb"])
            for k, i in enumerate(take if self.disp["lay_arrow"].get() else []):
                # solid: what the GEARBOX does - same instant, ratio swapped
                A.annotate("", xy=(cf_n[k], cf_t[k]),
                           xytext=(r.motor_rpm[i], r.motor_torque[i]),
                           arrowprops=dict(arrowstyle="-|>,head_width=.28,head_length=.6",
                                           color=col, lw=a_lw, alpha=a_al,
                                           shrinkA=0, shrinkB=0), zorder=7)
                # dotted: what the DRIVER does in the same step - demand moving on
                if not self.disp["lay_driver"].get():
                    continue
                A.annotate("", xy=(r.motor_rpm[i + 1], r.motor_torque[i + 1]),
                           xytext=(cf_n[k], cf_t[k]),
                           arrowprops=dict(arrowstyle="-|>,head_width=.18,head_length=.4",
                                           color=COLORS["text_muted"],
                                           lw=min(1.0, a_lw), ls=":",
                                           alpha=min(.8, a_al), shrinkA=0, shrinkB=0),
                           zorder=6)
            if len(grp):
                # computed whether or not the markers are drawn - the panel title
                # quotes it, and turning a layer off must not change the numbers
                gn, gt = sc.counterfactual_point(r, grp, new_gear, p["gb"])
                if self.disp["lay_marks"].get():
                    A.scatter(r.motor_rpm[grp], r.motor_torque[grp], s=26, marker="o",
                              facecolor="none", edgecolor=col, lw=1.3, zorder=8,
                              label="before the shift")
                    A.scatter(gn, gt, s=42, marker="X", color=col, edgecolor="k", lw=.5,
                              zorder=8, label="after the shift (same instant)")
                    A.scatter(r.motor_rpm[grp + 1], r.motor_torque[grp + 1], s=18,
                              marker=".", color=COLORS["text_muted"], alpha=.7, zorder=7,
                              label="next sample (driver moved on)")
                # the gearbox delta, not the temporal one: same instant, ratio swapped
                d_n = np.mean(gn - r.motor_rpm[grp])
                d_t = np.mean(gt - r.motor_torque[grp])
                A.set_title(f"{lab}   ({len(grp)} events, all drawn)\n"
                            f"the ratio alone moves it  {d_n:+.0f} rpm,  {d_t:+.2f} Nm",
                            fontweight="bold", fontsize=11)
                A.legend(fontsize=7, loc="lower left", framealpha=.9)
            else:
                A.set_title(f"{lab}   (none)", fontweight="bold", fontsize=11)
        ax[0].set_ylabel("Signed motor torque [Nm]\n(+ motoring,  - braking)")
        self._colorbar(c, ax, "Efficiency [%]  (negative half mirrored)", fraction=.04)
        self.fig.suptitle(f"How each shift moves the operating point  —  upshift "
                          f"{r.upshift:g} / downshift {r.downshift:g} km/h", fontweight="bold")

        d = sc.shift_decomposition(r, self.cycle, self.emap, veh=p["veh"],
                                   motor=p["motor"], gb=p["gb"], num=p["num"])
        L = ["What a gear change actually does to the operating point", "=" * 74, "",
             "  The solid arrow is the GEARBOX: same instant, ratio swapped.",
             "  The dotted arrow is the DRIVER: the demand moving on to the next sample.",
             "  Drawn as one arrow (the old view) those two are indistinguishable.", ""]
        for name in ("upshift", "downshift"):
            x = d.get(name)
            if not x:
                L.append(f"  {name}: none"); continue
            L += [f"  {name.upper()}  ({x['events']} events)",
                  f"    before -> next sample   {x['arrow_pts']:+6.2f} efficiency points",
                  f"      of which the gearbox  {x['gear_pts']:+6.2f}"
                  f"   (the new ratio is better at {x['gear_helps_pct']:.0f} % of them)",
                  f"      of which the driver   {x['driver_pts']:+6.2f}"]
            if x["crossed_zero"]:
                L.append(f"    {x['crossed_zero']} of {x['events']} arrows CROSS ZERO "
                         f"TORQUE - braking at one end, pulling at the other")
            L += [f"    under traction : {x['traction_events']:3d} events, gearbox worth "
                  f"{x['gear_pts_traction']:+.2f} pts",
                  f"    braking/coast  : {x['events']-x['traction_events']:3d} events, "
                  f"gearbox worth {x['gear_pts_braking']:+.2f} pts",
                  f"    WEIGHTED BY THE POWER ACTUALLY FLOWING: "
                  f"{x['gear_pts_power_weighted']:+.2f} pts",
                  ""]
        L += ["  That last line is the one that decides energy. An efficiency gain at a",
              "  sample carrying no traction power multiplies a zero: with regen off the",
              "  motor supplies nothing while braking, and that is where nearly every",
              "  downshift happens.", ""]
        try:
            o = sc.oracle_bound(self.cycle, self.emap, **p)
            L += ["  CEILING ON THE WHOLE QUESTION", "  " + "-" * 56,
                  f"    always ratio {p['gb'].ratio_1:g}      {o['gear1_only']:.4f} kWh",
                  f"    always ratio {p['gb'].ratio_2:g}      {o['gear2_only']:.4f} kWh",
                  f"    perfect per-sample choice {o['oracle']:.4f} kWh",
                  f"    -> the entire prize is {o['prize_wh']:.1f} Wh "
                  f"({100*o['prize_wh']/1000/o['best_single']:.2f} %), and that is with",
                  f"       clairvoyance, free shifting and no rate limit. The better ratio",
                  f"       is gear 1 on {o['gear1_share']:.1f} % of moving samples."]
        except Exception:
            pass
        self.log("\n".join(L))
        self.say(f"{len(ups)} upshifts and {len(dns)} downshifts — solid arrow = gearbox, "
                 f"dotted = driver", "ok")

    def _draw_gear(self, payload, p, _kind):
        """Efficiency vs road speed for both ratios, at several load levels."""
        speeds, data = payload
        accels = sorted(data)
        ax = (self.fig.subplots(1, len(accels), sharey=True)
              if len(accels) > 1 else [self.fig.subplots()])
        ax = np.atleast_1d(ax)
        lines = []
        for a_i, acc in enumerate(accels):
            A = ax[a_i]
            rec = data[acc]
            A.plot(speeds, rec[1]["eff"] * 100, "-", lw=2.2, color=GEAR1,
                   label=f"gear 1  (ratio {p['gb'].ratio_1:g} - low, high rpm)")
            A.plot(speeds, rec[2]["eff"] * 100, "-", lw=2.2, color=GEAR2,
                   label=f"gear 2  (ratio {p['gb'].ratio_2:g} - high, low rpm)")
            d = rec[1]["eff"] - rec[2]["eff"]
            with np.errstate(invalid="ignore"):
                cross = np.flatnonzero(np.diff(np.sign(np.nan_to_num(d))) != 0)
            for x in speeds[cross]:
                A.axvline(x, color=COLORS["text"], ls="--", lw=1.1, alpha=.6)
                A.annotate(f"crossover\n{x:.0f} km/h", (x, A.get_ylim()[0]),
                           textcoords="offset points", xytext=(4, 8), fontsize=8,
                           color=COLORS["text"],
                           bbox=dict(boxstyle="round,pad=.2", fc=COLORS["plot_bg"],
                                     ec=COLORS["border"], lw=.7, alpha=.9))
            A.set_title(f"acceleration = {acc:g} m/s²", fontsize=10.5,
                        fontweight="bold")
            A.set_xlabel("Road speed [km/h]", fontsize=9.5)
            A.set_xlim(speeds.min(), speeds.max())      # same axis on every panel
            A.tick_params(labelsize=8.5)
            A.grid(alpha=.25)
            if a_i == 0:
                A.set_ylabel("Motor efficiency [%]")
                lines = A.get_legend_handles_labels()
        # inside the first panel, not below the figure: subplots_adjust is ignored by
        # the tight layout engine, so a figure-level legend lands on the x-labels
        if lines and lines[0]:
            ax[0].legend(*lines, loc="lower left", fontsize=8, framealpha=.92)
        self.fig.suptitle("Which ratio is more efficient — and how that flips with load",
                          fontweight="bold", fontsize=12.5, y=.98)
        txt = ["Gear efficiency vs load", "=" * 74,
               f"{'accel':>8} | {'winner at 10':>14}{'20':>10}{'30':>10}{'40':>10}"]
        for acc in accels:
            rec = data[acc]
            row = []
            for v in (10, 20, 30, 40):
                i = int(np.argmin(np.abs(speeds - v)))
                e1, e2 = rec[1]["eff"][i], rec[2]["eff"][i]
                row.append("gear 1" if (np.isfinite(e1) and (not np.isfinite(e2) or e1 >= e2))
                           else "gear 2")
            txt.append(f"{acc:>8g} | {row[0]:>14}{row[1]:>10}{row[2]:>10}{row[3]:>10}")
        # the crossover speed for each load, measured off the curves themselves
        txt += ["", "  WHERE THE TWO RATIOS SWAP PLACES", "  " + "-" * 60,
                f"  {'acceleration':>13} {'crossover':>11} {'gear 1 torque':>15}"
                f" {'gear 2 torque':>15}"]
        cross_pts = []
        for acc in accels:
            rec = data[acc]
            d = rec[1]["eff"] - rec[2]["eff"]
            with np.errstate(invalid="ignore"):
                k = np.flatnonzero(np.diff(np.sign(np.nan_to_num(d))) != 0)
            if len(k):
                v_c = float(speeds[k[0] + 1])
                cross_pts.append((acc, v_c))
                i_c = k[0] + 1
                txt.append(f"  {acc:>10g} m/s2 {v_c:>8.0f} km/h "
                           f"{rec[1]['torque'][i_c]:>13.1f} Nm {rec[2]['torque'][i_c]:>13.1f} Nm")
            else:
                w = "gear 1" if np.nanmean(d) > 0 else "gear 2"
                txt.append(f"  {acc:>10g} m/s2 {'none':>8}       {w} wins at every speed")
        if len(cross_pts) > 1:
            lo_a, lo_v = cross_pts[0]
            hi_a, hi_v = cross_pts[-1]
            txt += ["",
                    f"  The crossover moves {abs(hi_v-lo_v):.0f} km/h across this load range"
                    f" ({lo_v:.0f} km/h at {lo_a:g} m/s2 -> {hi_v:.0f} at {hi_a:g}).",
                    "  A shift threshold is ONE number. Set it for the cruise crossover and",
                    "  every acceleration is in the wrong ratio; set it for acceleration and",
                    "  every cruise sample is. That is the cost a speed-only schedule cannot",
                    "  avoid, and the size of it is the number above."]
        txt += ["",
                "  Why the ranking inverts: copper loss grows with torque^2 and iron and",
                "  windage grow with speed. Under load the torque term dominates and the",
                "  LOW ratio wins by asking for less torque; at cruise the speed term",
                "  dominates and the HIGH ratio wins by turning slower. The crossover is",
                "  wherever those two trade places, which is why it moves with load."]
        self.log("\n".join(txt))
        self.say("Gear preference flips with load — see the crossover lines", "warn")

    def _draw_optimal(self, payload, p, _kind):
        """2-D map of which ratio wins, with the 1-D shift line drawn on top."""
        sp, ac, better, delta, f1, f2 = payload
        ax = self.fig.subplots(1, 2)
        from matplotlib.colors import ListedColormap
        cm = ListedColormap([GEAR1, GEAR2])       # the same two colours as everywhere
        ax[0].pcolormesh(sp, ac, np.ma.masked_invalid(better), cmap=cm, shading="auto",
                         vmin=1, vmax=2)
        ax[0].contour(sp, ac, np.nan_to_num(better, nan=0), levels=[1.5],
                      colors="k", linewidths=2)
        up = self._f("thr", "upshift", 22)
        dn = self._f("thr", "downshift", 10)
        ax[0].axvline(up, color="#ffffff", lw=3.2)
        ax[0].axvline(up, color=COLORS["success"], lw=2, label=f"your upshift {up:g} km/h")
        ax[0].axvline(dn, color="#ffffff", lw=3.2)
        ax[0].axvline(dn, color=COLORS["success"], lw=2, ls="--",
                      label=f"your downshift {dn:g} km/h")
        ax[0].set_xlabel("Road speed [km/h]"); ax[0].set_ylabel("Acceleration [m/s²]")
        # the label used to say "yellow" while the map drew orange
        ax[0].set_title("Which ratio is more efficient at each operating point",
                        fontweight="bold", fontsize=11)
        from matplotlib.patches import Patch
        ax[0].legend(handles=[Patch(facecolor=GEAR1, label="gear 1 wins"),
                              Patch(facecolor=GEAR2, label="gear 2 wins")]
                     + ax[0].get_legend_handles_labels()[0],
                     fontsize=8.5, loc="lower right", framealpha=.92)
        ax[0].text(.5, -.135, "black line = the true boundary. Your thresholds are "
                              "vertical lines and cannot follow a curve.",
                   transform=ax[0].transAxes, ha="center", fontsize=8,
                   color=COLORS["text_muted"])
        m = ax[1].pcolormesh(sp, ac, np.ma.masked_invalid(delta), cmap="magma", shading="auto")
        self._colorbar(m, ax[1], "Advantage of the better ratio [pts]")
        ax[1].axvline(up, color="#ffffff", lw=3.2); ax[1].axvline(up, color=COLORS["success"], lw=2)
        ax[1].axvline(dn, color="#ffffff", lw=3.2)
        ax[1].axvline(dn, color=COLORS["success"], lw=2, ls="--")
        ax[1].set_xlabel("Road speed [km/h]"); ax[1].set_ylabel("Acceleration [m/s²]")
        ax[1].set_title("How much the choice is worth", fontweight="bold", fontsize=11)
        self.log(self._optimal_summary(sp, ac, better, delta, up, dn,
                                       self._f('cost', 'actuator_voltage', 12.0)
                                       * self._f('cost', 'actuator_current', 20.0)
                                       * self._f('cost', 'actuator_time_s', 0.5)))
        self.say("Optimal-gear boundary is a curve; a speed-only threshold is a line", "warn")

    # --------------------------------------- wide-open-throttle acceleration
    def _draw_acceleration(self, sw, p, _kind):
        """0 -> V at full throttle: the motor sits on its peak curve and the
        acceleration is an OUTPUT, so the answer is a TIME, not an energy.

            a(v) = (F_traction(v, gear) - F_road(v)) / (m + J*ratio^2/r^2)

        The 1-2 upshift is swept. Shifting costs `interrupt_s` of zero traction,
        which is why the fastest schedule is usually either the tractive-force
        crossover or no shift at all.
        """
        gs = self.fig.add_gridspec(2, 2, hspace=.34, wspace=.26)
        a_v = self.fig.add_subplot(gs[0, 0])
        a_t = self.fig.add_subplot(gs[0, 1])
        a_f = self.fig.add_subplot(gs[1, 0])
        a_m = self.fig.add_subplot(gs[1, 1])
        ok = sw.table[sw.table["reached"]]

        # --- 1. speed traces: fastest, slowest, and no-shift ------------------
        runs = [r for r in sw.runs if r.reached]
        show = []
        if runs:
            fastest = min(runs, key=lambda r: r.time_s)
            slowest = max(runs, key=lambda r: r.time_s)
            shifting = [r for r in runs if r.shift_times]
            picks = [(fastest, COLORS["success"], "fastest"),
                     (slowest, COLORS["danger"], "slowest")]
            # when the fastest schedule is "never shift", the best schedule that DOES
            # shift is the interesting third trace - it isolates the interruption
            if shifting:
                picks.insert(1, (min(shifting, key=lambda r: r.time_s),
                                 COLORS["secondary"], "best that shifts"))
            for r, col, lab in picks:
                if r.shift_speed not in [x[0].shift_speed for x in show]:
                    show.append((r, col, lab))
        for r, col, lab in show:
            tag = ("hold gear 1" if not r.shift_times
                   else f"shift {r.shift_speed:g} km/h")
            a_v.plot(r.t, r.v_kmh, lw=1.8, color=col,
                     label=f"{lab}: {tag} — {r.time_s:.2f} s")
            for ts in r.shift_times:
                a_v.axvline(ts, color=col, ls=":", lw=1.2)
        a_v.axhline(sw.v_target, color=COLORS["text_muted"], ls="--", lw=1)
        a_v.set_xlabel("Time [s]"); a_v.set_ylabel("Speed [km/h]")
        a_v.set_title(f"0 - {sw.v_target:g} km/h at "
                      f"{sw.throttle*100:.0f} % throttle", fontweight="bold", fontsize=11)
        a_v.grid(alpha=.3); a_v.legend(fontsize=8, loc="lower right")

        # --- 2. the answer: time (and energy) against the shift speed --------
        if len(ok):
            a_t.plot(ok["shift_speed"], ok["time_s"], "o-", ms=4,
                     color=COLORS["primary"], label="0-V time")
            if sw.best_time is not None:
                a_t.scatter([sw.best_time.shift_speed], [sw.best_time.time_s], s=150,
                            zorder=6, facecolor=COLORS["success"], edgecolor="k", lw=1.2,
                            label=f"fastest {sw.best_time.time_s:.2f} s")
            # the sentinel is the highest candidate overall, reached or not: when
            # gear 1 hits its rpm limit before the target, "hold gear 1" is the
            # infeasible bar, not the last point on the curve
            sentinel = sw.table["shift_speed"].max()
            a_t.axvline(sentinel, color=COLORS["text_muted"], ls=":", lw=1.2)
            lo_y, hi_y = a_t.get_ylim()
            a_t.annotate("hold gear 1", (sentinel, lo_y + .97 * (hi_y - lo_y)),
                         fontsize=8, rotation=90, va="top", ha="right",
                         color=COLORS["text_muted"])
            e = a_t.twinx()
            e.plot(ok["shift_speed"], ok["energy_wh"], "s--", ms=3, alpha=.8,
                   color=COLORS["warning"], label="energy")
            e.set_ylabel("Energy per run [Wh]", color=COLORS["warning"])
            e.tick_params(axis="y", labelcolor=COLORS["warning"])
        bad = sw.table[~sw.table["reached"]]
        for v in bad["shift_speed"]:
            a_t.axvline(v, color=COLORS["danger"], lw=4, alpha=.16)
        a_t.set_xlabel("1-2 upshift speed [km/h]"); a_t.set_ylabel("0-V time [s]")
        a_t.set_title("Acceleration time vs shift speed", fontweight="bold", fontsize=11)
        a_t.grid(alpha=.3); a_t.legend(fontsize=8, loc="upper right")

        # --- 3. why: the tractive-force diagram ------------------------------
        v, F1, F2, Fr = sw.curves
        a_f.plot(v, F1, lw=2, color=COLORS["primary"], label="gear 1 at full throttle")
        a_f.plot(v, F2, lw=2, color=COLORS["warning"], label="gear 2 at full throttle")
        a_f.plot(v, Fr, lw=1.6, ls="--", color=COLORS["text"], label="road load")
        cross = self._force_crossover(v, F1, F2)
        if cross:
            a_f.axvline(cross, color=COLORS["success"], lw=1.6, ls="-.",
                        label=f"force crossover {cross:.1f} km/h")
        a_f.axvline(sw.v_target, color=COLORS["text_muted"], ls=":", lw=1.2)
        a_f.set_xlim(0, v.max()); a_f.set_ylim(0, max(F1.max(), F2.max()) * 1.08)
        a_f.set_xlabel("Road speed [km/h]"); a_f.set_ylabel("Force at the wheel [N]")
        a_f.set_title("Why: tractive force per ratio", fontweight="bold", fontsize=11)
        a_f.grid(alpha=.3); a_f.legend(fontsize=8)

        # --- 4. where full throttle puts the motor on the map ----------------
        c = self._map_background(a_m, p, alpha=.75)
        if sw.best_time is not None:
            r = sw.best_time
            for gear, col, mk in ((1, "#ffffff", "o"), (2, "#ffd400", "^")):
                m = (r.gear == gear) & (r.torque > 1e-9)
                if m.any():
                    a_m.scatter(r.rpm[m], r.torque[m], s=14, marker=mk, facecolor=col,
                                edgecolor="k", linewidth=.25, alpha=.85, zorder=5,
                                label=f"gear {gear}")
        a_m.set_title("Full-throttle operating line", fontweight="bold", fontsize=11)
        a_m.legend(fontsize=8, loc="upper left")
        self._colorbar(c, a_m, "Efficiency [%]", fraction=.045)

        self.fig.suptitle(f"Wide-open-throttle acceleration 0 - {sw.v_target:g} km/h",
                          fontweight="bold")
        self.log(self._wot_summary(sw, cross))
        if sw.best_time is not None:
            b = sw.best_time
            tag = "no shift" if not b.shift_times else f"shift at {b.shift_speed:g} km/h"
            self.say(f"0-{sw.v_target:g} km/h in {b.time_s:.2f} s ({tag})", "ok")
        else:
            self.say(f"{sw.v_target:g} km/h is not reachable at this throttle", "err")

    @staticmethod
    def _force_crossover(v, F1, F2):
        """Lowest speed above the gear-1 base speed where gear 2 matches gear 1."""
        d = np.asarray(F1) - np.asarray(F2)
        sign = np.sign(d)
        idx = np.flatnonzero((np.diff(sign) != 0) & (np.asarray(F2)[:-1] > 0))
        return float(v[idx[0] + 1]) if len(idx) else None

    def _wot_summary(self, sw, cross):
        NL = chr(10)
        t = sw.table
        ok = t[t["reached"]]
        L = [f"Wide-open-throttle acceleration 0 - {sw.v_target:g} km/h", "=" * 74,
             f"  throttle {sw.throttle*100:.0f} %   "
             f"{len(t)} shift speeds tried, {len(ok)} reached the target", ""]
        if not len(ok):
            L.append("  The target speed is not reachable:")
            for r in sw.runs[:1]:
                L += [f"    - {x}" for x in r.notes]
            L += ["", "  At full throttle the motor sits on min(peak torque, constant",
                  "  power). If that curve meets road load below the target, no shift",
                  "  schedule can get there - it is a powertrain limit, not a strategy."]
            return NL.join(L)

        L += [f"{'shift at':>9} {'0-V time':>10} {'vs best':>9} {'distance':>10}"
              f" {'energy':>9} {'shifts':>7}"]
        tb = sw.best_time.time_s
        for _, x in ok.iterrows():
            tag = "no shift" if x["shifts"] == 0 else f"{x['shift_speed']:.0f} km/h"
            L.append(f"{tag:>9} {x['time_s']:9.3f} s {x['time_s']-tb:+8.3f}s"
                     f" {x['distance_m']:9.1f} m {x['energy_wh']:8.3f} Wh"
                     f" {int(x['shifts']):7d}")
        b = sw.best_time
        shifted = ok[ok["shifts"] > 0]
        L += ["",
              f"  FASTEST : {b.time_s:.3f} s "
              + ("holding gear 1 all the way" if not b.shift_times
                 else f"shifting at {b.shift_speed:g} km/h")
              + f"   ({b.distance_m:.1f} m, {b.energy_wh:.3f} Wh)",
              f"  SLOWEST : {ok['time_s'].max():.3f} s  -> across the searched range the "
              f"shift speed is worth {ok['time_s'].max()-tb:.3f} s",
              f"  LEAST ENERGY: {sw.best_energy.energy_wh:.3f} Wh at "
              f"{sw.best_energy.shift_speed:g} km/h"]
        if len(shifted):
            bs = shifted.loc[shifted["time_s"].idxmin()]
            L += ["",
                  f"  Among schedules that DO shift, the best is {bs['shift_speed']:g} km/h "
                  f"at {bs['time_s']:.3f} s;",
                  f"  the spread across those is "
                  f"{shifted['time_s'].max()-shifted['time_s'].min():.3f} s "
                  f"({100*(shifted['time_s'].max()/shifted['time_s'].min()-1):.1f} %),",
                  f"  so the shift speed on its own is worth that much."]
            if not b.shift_times:
                gap = bs["time_s"] - b.time_s
                cut = self._f("cost", "actuator_time_s", 0.5)
                L.append(f"  Not shifting at all is {gap:.3f} s faster still.")
                if cut > 0:
                    L.append(f"  The torque interruption is {cut:.3f} s per shift, so at "
                             f"the best shift speed the")
                    L.append(f"  ratio change itself costs only {gap-cut:+.3f} s - the "
                             f"interruption is the whole cost.")
                    L.append(f"  That is the model checking itself: at the tractive-force "
                             f"crossover the two")
                    L.append(f"  ratios pull the same, so nothing but the cut should "
                             f"remain.")
        L += ["",
              "  Times come from quadrature over speed (t = INT dv/a), not a time-stepped",
              "  integration, and are converged to well under a millisecond - so the",
              "  differences between adjacent shift speeds above are real, not solver noise."]
        if cross:
            L += ["", f"  Tractive-force crossover: {cross:.1f} km/h.",
                  "  Below it gear 1 pulls harder, above it the two ratios are both on",
                  "  constant power and pull the same. Shifting before the crossover",
                  "  throws away force; shifting after it gains nothing."]
        L += ["",
              "  Each shift also costs a torque interruption (ShiftCost.interrupt_s),",
              "  during which traction is zero and the vehicle decelerates against road",
              "  load. That is why 'never shift' can beat every shift speed over a short",
              "  run, and why the penalty for shifting early is bigger than the diagram",
              "  alone suggests.",
              "",
              "  Acceleration is an output here: the motor is on its peak curve, so this",
              "  is a performance test, not the energy question the drive-cycle analyses",
              "  answer. Read them together - the fastest schedule is rarely the",
              "  most efficient one."]
        return NL.join(L)

    # ------------------------------------------------------- loss breakdown
    TERMS = [("wheel", "road work", "#5b6b7f"),
             ("gearbox", "gearbox loss", "#8fa3b8"),
             ("motor", "MOTOR loss", "#a30f45"),
             ("aux", "auxiliary", "#c9a227"),
             ("shift", "shift actuator + cut", "#1a7f4b")]

    def _draw_lossshare(self, out, p, _kind):
        """Where the energy goes as each threshold moves.

        This exists because a sweep summary quotes only the winner. It says the
        winner made N changes and nothing about how N moved across the sweep, so
        the shift penalty is invisible and "peak efficiency here but minimum
        energy there" reads as a contradiction rather than as the trade it is.
        Column per threshold; the middle row is the answer.
        """
        gs = self.fig.add_gridspec(3, 2, hspace=.38, wspace=.30)
        any_data = False
        for c, col in enumerate(("upshift", "downshift")):
            df = out["terms"][col]
            if df is None or df.empty:
                ax = self.fig.add_subplot(gs[:, c])
                ax.text(.5, .5, "no feasible " + col + " candidate",
                        ha="center", va="center", color=COLORS["danger"])
                ax.set_xticks([])
                ax.set_yticks([])
                continue
            any_data = True
            x = df["threshold"].to_numpy()
            held = out["held_dn"] if col == "upshift" else out["held_up"]
            other = "downshift" if col == "upshift" else "upshift"
            i_best = int(np.argmin(df["net_kwh"].to_numpy()))
            i_eff = int(np.argmax(df["mean_efficiency"].to_numpy()))

            # --- row 1: the LOSSES, stacked ------------------------------
            # Road work is excluded on purpose. It is ~8x every loss combined and
            # barely moves with the threshold, so stacking it flattens the four
            # terms that actually differ into an unreadable sliver. Its value is
            # in the title instead.
            a1 = self.fig.add_subplot(gs[0, c])
            loss_terms = [t for t in self.TERMS if t[0] != "wheel"]
            ys = [1000 * df[k].to_numpy() for k, _, _ in loss_terms]
            a1.stackplot(x, *ys, labels=[lb for _, lb, _ in loss_terms],
                         colors=[cc for _, _, cc in loss_terms], alpha=.92)
            a1.set_ylabel("loss [Wh]")
            a1.set_title(col + " sweep - share of the LOSSES" + chr(10)
                         + "(" + other + " held at " + format(held, "g")
                         + " km/h; road work "
                         + format(1000 * df["wheel"].mean(), ".0f")
                         + " Wh excluded)", fontsize=9.5)
            if c == 0:
                a1.legend(fontsize=7, loc="upper left", ncol=2, framealpha=.9)

            # --- row 2: THE answer - efficiency against the penalty ------
            a2 = self.fig.add_subplot(gs[1, c])
            a2b = a2.twinx()
            a2b.bar(x, df["shifts"], width=.55, color=self.TERMS[4][2], alpha=.16,
                    zorder=0)
            a2b.set_ylabel("gear changes", color=self.TERMS[4][2], fontsize=9)
            a2b.tick_params(axis="y", labelcolor=self.TERMS[4][2], labelsize=8)
            a2.set_zorder(a2b.get_zorder() + 1)
            a2.patch.set_visible(False)
            a2.plot(x, df["shift_wh"], "-o", ms=3.5, color=self.TERMS[4][2], lw=2,
                    label="shift cost [Wh]")
            a2.plot(x, df["motor_wh"] - df["motor_wh"].min(), "-s", ms=3.5,
                    color=self.TERMS[2][2], lw=2,
                    label="MOTOR loss above its own min")
            a2.set_ylabel("Wh")
            a2.axvline(x[i_best], color="k", ls="--", lw=1.2)
            a2.axvline(x[i_eff], color=self.TERMS[2][2], ls=":", lw=1.6)
            a2.legend(fontsize=7, loc="upper left", framealpha=.9)
            a2.set_title("what each threshold buys, and what it costs", fontsize=9)

            # --- row 3: net and efficiency, both optima marked -----------
            a3 = self.fig.add_subplot(gs[2, c])
            a3.plot(x, 1000 * df["net_kwh"], "-o", ms=3.5, color="k", lw=2,
                    label="net energy")
            a3.scatter([x[i_best]], [1000 * df["net_kwh"].iloc[i_best]], s=110,
                       color="k", zorder=3,
                       label="least energy " + format(x[i_best], "g"))
            a3.set_xlabel(col + " speed [km/h]")
            a3.set_ylabel("net [Wh]")
            a3e = a3.twinx()
            a3e.plot(x, 100 * df["mean_efficiency"], "-s", ms=3.5,
                     color=self.TERMS[2][2], lw=1.6, alpha=.85)
            a3e.scatter([x[i_eff]], [100 * df["mean_efficiency"].iloc[i_eff]], s=110,
                        marker="X", color=self.TERMS[2][2], zorder=3)
            a3e.set_ylabel("mean motor efficiency [%]", color=self.TERMS[2][2],
                           fontsize=9)
            a3e.tick_params(axis="y", labelcolor=self.TERMS[2][2], labelsize=8)
            a3.legend(fontsize=7, loc="center left", framealpha=.9)
            gap = ("the same point" if i_best == i_eff
                   else format(abs(x[i_eff] - x[i_best]), "g") + " km/h apart")
            a3.set_title("least energy " + format(x[i_best], "g")
                         + " vs best efficiency " + format(x[i_eff], "g")
                         + " - " + gap, fontsize=9)

        self.fig.suptitle("Loss breakdown - what moves when a threshold moves",
                          fontsize=12, fontweight="bold")
        if any_data:
            self.log(self._lossshare_summary(out))
            self.say("Loss breakdown drawn", "ok")

    def _lossshare_summary(self, out):
        NL = chr(10)
        L = ["Loss breakdown - what each threshold actually buys", "=" * 74,
             "  " + sc.build_stamp(), "",
             "  Anchored at the self-consistent pair "
             + format(out["held_up"], "g") + "/" + format(out["held_dn"], "g")
             + " km/h, so the two",
             "  columns are slices through ONE operating point, not two unrelated",
             "  searches.", ""]
        for col in ("upshift", "downshift"):
            df = out["terms"][col]
            if df is None or df.empty:
                L += ["  " + col.upper() + ": no feasible candidate.", ""]
                continue
            i_b = int(np.argmin(df["net_kwh"].to_numpy()))
            i_e = int(np.argmax(df["mean_efficiency"].to_numpy()))
            xb = df["threshold"].iloc[i_b]
            xe = df["threshold"].iloc[i_e]
            L += ["  " + col.upper() + " SWEEP", "  " + "-" * 66,
                  f"    {'thr':>5}{'changes':>9}{'shift Wh':>10}{'motor Wh':>10}"
                  f"{'mean eff':>10}{'net Wh':>10}"]
            for k in range(len(df)):
                mark = ("  <- least energy" if k == i_b else
                        ("  <- best efficiency" if k == i_e else ""))
                L.append(f"    {df['threshold'].iloc[k]:5.0f}"
                         f"{int(df['shifts'].iloc[k]):9d}"
                         f"{df['shift_wh'].iloc[k]:10.1f}"
                         f"{df['motor_wh'].iloc[k]:10.1f}"
                         f"{100*df['mean_efficiency'].iloc[k]:9.3f}%"
                         f"{1000*df['net_kwh'].iloc[k]:10.1f}{mark}")
            if i_b == i_e:
                L += ["",
                      "    Least energy and best efficiency are both at "
                      + format(xb, "g") + " km/h - nothing is",
                      "    being traded here.", ""]
            else:
                d_eff = df["motor_wh"].iloc[i_b] - df["motor_wh"].iloc[i_e]
                d_sh = df["shift_wh"].iloc[i_b] - df["shift_wh"].iloc[i_e]
                n_e = int(df["shifts"].iloc[i_e])
                n_b = int(df["shifts"].iloc[i_b])
                L += ["",
                      "    Best efficiency is at " + format(xe, "g")
                      + " km/h; least energy is at " + format(xb, "g") + " km/h.",
                      f"    Moving from {xe:g} to {xb:g}:",
                      f"      motor loss   {d_eff:+7.1f} Wh   "
                      f"(the map really is worse there)",
                      f"      shift cost   {d_sh:+7.1f} Wh   "
                      f"({n_e} changes -> {n_b})",
                      f"      NET          {d_eff + d_sh:+7.1f} Wh"]
                # the sentence has to match which mechanism actually moved
                if n_b < n_e:
                    L += ["",
                          "    THAT is why the energy optimum is not where the map is",
                          f"    happiest: the cheaper threshold makes {n_e - n_b} fewer gear",
                          "    changes. The map prefers the busier schedule; the battery",
                          "    refuses to pay for it."]
                elif abs(d_sh) > 1e-9:
                    L += ["",
                          "    Note the change COUNT is the same at both. The shift cost",
                          "    still differs because the traction cut is charged at the",
                          "    power actually flowing when each change happens - shifting",
                          "    at a higher speed interrupts a bigger load. So this is not",
                          "    a shift-count effect, it is a shift-TIMING effect."]
                else:
                    L += ["",
                          "    The shift cost is identical at both, so the difference here",
                          "    is entirely in the motor term - read this one as a genuine",
                          "    efficiency-map result."]
                L.append("")
        L += ["  Reading the panels:",
              "    row 1  every Wh in the cycle, stacked. The green band is the shift",
              "           cost - watch it grow as the threshold moves.",
              "    row 2  the trade on one axis: shift cost against motor loss above",
              "           its own minimum, with the gear-change count as bars behind.",
              "           Whichever curve is higher decides the answer.",
              "    row 3  net energy (black, left) and mean efficiency (red, right),",
              "           each with its own optimum marked. When the dot and the cross",
              "           are not at the same x, row 2 says why."]
        return NL.join(L)

    # -------------------------------------------- efficiency-only optimum
    def _draw_effopt(self, sw, p, _kind):
        """Rank every schedule on motor efficiency alone.

        mean_efficiency is shaft output over electrical input across the motoring
        samples, so the auxiliary load, the shift actuation
        energy and the torque interruption all cancel out of it. What is left is the
        one question the energy sweeps keep burying: which schedule keeps the motor
        in the best part of its map over this cycle.
        """
        full = sw.table
        df = full[full["feasible"]]
        if df.empty or sw.best is None:
            self.fig.text(.5, .5, "No feasible schedule in this grid",
                          ha="center", va="center", color=COLORS["danger"])
            self.log("No feasible schedule."); return

        b = sw.best
        # The energy optimum of the SAME grid, chosen by the same tie-break the
        # combined-grid sweep uses - idxmin would return whichever of the many
        # bit-identical ties came first in row order, which is what made this
        # panel and the sweeps quote different pairs for one answer.
        _e = sc.best_by_energy(sw.details)
        e_row = (df[(df["upshift"] == _e.upshift) & (df["downshift"] == _e.downshift)]
                 .iloc[0] if _e is not None else df.loc[df["net_kwh"].idxmin()])

        gs = self.fig.add_gridspec(2, 2, hspace=.34, wspace=.26)
        a_map = self.fig.add_subplot(gs[0, 0])
        a_up = self.fig.add_subplot(gs[0, 1])
        a_dn = self.fig.add_subplot(gs[1, 0])
        a_pts = self.fig.add_subplot(gs[1, 1])

        # --- 1. the efficiency surface over both thresholds ------------------
        # pivot the FULL grid, not just the survivors: a rejected candidate must
        # leave a visible hole at the speed you asked for, not silently shrink the
        # axis so the plot looks like it started somewhere else
        shown = full.copy()
        shown.loc[~shown["feasible"], "mean_efficiency"] = np.nan
        piv = shown.pivot_table(index="downshift", columns="upshift",
                                values="mean_efficiency", dropna=False)
        m = a_map.pcolormesh(piv.columns, piv.index, piv.to_numpy() * 100,
                             shading="auto", cmap=self._map_style()["cmap"])
        self._colorbar(m, a_map, "Mean motor efficiency [%]")
        a_map.scatter([b.upshift], [b.downshift], marker="*", s=280, zorder=6,
                      facecolor="w", edgecolor="k", lw=1.4,
                      label=f"best efficiency {b.upshift:g}/{b.downshift:g}")
        a_map.scatter([e_row["upshift"]], [e_row["downshift"]], marker="X", s=150,
                      zorder=6, facecolor=COLORS["danger"], edgecolor="k", lw=1.2,
                      label=f"best energy {e_row['upshift']:g}/{e_row['downshift']:g}")
        a_map.set_xlabel("Upshift [km/h]"); a_map.set_ylabel("Downshift [km/h]")
        a_map.set_title("Motor efficiency over the schedule grid",
                        fontweight="bold", fontsize=11)
        a_map.legend(fontsize=8, loc="upper left")

        # --- 2/3. slices through the optimum --------------------------------
        for ax, col, fixed_col, fixed_val, xlabel in (
                (a_up, "upshift", "downshift", b.downshift, "1-2 upshift speed [km/h]"),
                (a_dn, "downshift", "upshift", b.upshift, "2-1 downshift speed [km/h]")):
            sl_all = full[np.isclose(full[fixed_col], fixed_val)].sort_values(col)
            sl = sl_all[sl_all["feasible"]]
            if sl.empty:
                ax.axis("off"); continue
            rej = sl_all[~sl_all["feasible"]]
            for xv in rej[col]:
                ax.axvline(xv, color=COLORS["danger"], lw=5, alpha=.15, zorder=0)
            if len(rej):
                ax.plot([], [], lw=5, color=COLORS["danger"], alpha=.3,
                        label=f"rejected ({len(rej)})")
                ax.set_xlim(sl_all[col].min() - .5, sl_all[col].max() + .5)
            ax.plot(sl[col], sl["mean_efficiency"] * 100, "o-", ms=4,
                    color=COLORS["warning"], label="motor efficiency")
            ax.scatter([getattr(b, col)], [b.mean_efficiency * 100], s=150, zorder=6,
                       facecolor=COLORS["warning"], edgecolor="k", lw=1.2)
            ax.set_ylabel("Mean motor efficiency [%]", color=COLORS["warning"])
            ax.tick_params(axis="y", labelcolor=COLORS["warning"])
            ax2 = ax.twinx()
            ax2.plot(sl[col], sl["net_kwh"], "s--", ms=3, alpha=.75,
                     color=COLORS["primary"], label="net energy")
            ax2.set_ylabel("Net energy [kWh]", color=COLORS["primary"])
            ax2.tick_params(axis="y", labelcolor=COLORS["primary"])
            ax.set_xlabel(xlabel)
            ax.set_title(f"{col} at {fixed_col} {fixed_val:g} km/h",
                         fontweight="bold", fontsize=11)
            ax.grid(alpha=.3)
            if len(rej):
                ax.legend(fontsize=8, loc="lower left")

        # --- 4. where that schedule puts the motor ---------------------------
        r = sc.simulate(self.cycle, self.emap, b.upshift, b.downshift,
                        keep_arrays=True, **p)
        c = self._map_background(a_pts, p, alpha=.75)
        if r.gear is not None:
            act = (np.abs(r.motor_torque) > 1e-9) & (r.motor_torque > 0)
            for gear, colr, mk in ((1, "#ffffff", "o"), (2, "#ffd400", "^")):
                mm = act & (r.gear == gear)
                idx = np.flatnonzero(mm)
                if len(idx) > 4000:
                    idx = idx[:: int(np.ceil(len(idx) / 4000))]
                if len(idx):
                    a_pts.scatter(r.motor_rpm[idx], r.motor_torque[idx], s=7, marker=mk,
                                  facecolor=colr, edgecolor="k", linewidth=.15,
                                  alpha=.55, zorder=5,
                                  label=f"gear {gear} ({int(mm.sum()):,})")
        a_pts.set_title(f"Operating cloud at {b.upshift:g}/{b.downshift:g}",
                        fontweight="bold", fontsize=11)
        a_pts.legend(fontsize=8, loc="upper right")
        self._colorbar(c, a_pts, "Efficiency [%]", fraction=.045)

        self.fig.suptitle("Shift schedule ranked on motor efficiency alone",
                          fontweight="bold")
        self.log(self._effopt_summary(sw, b, e_row, df))
        self.say(f"Best motor efficiency {b.mean_efficiency:.2%} at "
                 f"{b.upshift:g}/{b.downshift:g} km/h", "ok")

    def _effopt_summary(self, sw, b, e_row, df):
        NL = chr(10)
        full = sw.table          # every candidate tried, rejected ones included
        lo, hi = df["mean_efficiency"].min(), df["mean_efficiency"].max()
        L = ["Shift schedule ranked on MOTOR EFFICIENCY alone", "=" * 74,
             "  " + sc.build_stamp(), "",
             "  mean_efficiency = shaft output energy / electrical input energy over",
             "  the motoring samples. The auxiliary load, the shift actuation energy",
             "  and the torque interruption all cancel out of",
             "  that ratio, so nothing here is competing with the map.", "",
             f"  {len(df)} feasible schedules of {len(full)} tried",
             f"  efficiency spread {lo:.3%} to {hi:.3%}  ({100*(hi-lo):.2f} points)", "",
             f"  BEST EFFICIENCY   upshift {b.upshift:g} / downshift {b.downshift:g}"
             f"   ->  {b.mean_efficiency:.3%}",
             f"                    net energy there {b.net_kwh:.4f} kWh",
             f"  BEST ENERGY       upshift {e_row['upshift']:g} / downshift "
             f"{e_row['downshift']:g}   ->  {e_row['mean_efficiency']:.3%}",
             f"                    net energy there {e_row['net_kwh']:.4f} kWh", ""]
        d_eff = 100 * (b.mean_efficiency - e_row["mean_efficiency"])
        d_net = 1000 * (b.net_kwh - e_row["net_kwh"])
        if abs(b.upshift - e_row["upshift"]) < 1e-9 and \
           abs(b.downshift - e_row["downshift"]) < 1e-9:
            L.append("  The two agree: on this cycle the most efficient schedule is also")
            L.append("  the cheapest one, so nothing is being traded away.")
        else:
            L += [f"  They differ. Going for peak efficiency buys {d_eff:+.3f} points and",
                  f"  costs {d_net:+.1f} Wh of net energy - that gap is the price the rest",
                  "  of the system charges for sitting in the efficient place (shift",
                  "  actuation and interruption, reflected inertia).",
                  "  Both figures are computed with the SAME shift cost the sweeps use,",
                  "  so they are directly comparable with them."]
        rej = full[~full["feasible"]]
        if len(rej):
            L += ["", f"  {len(rej)} of {len(full)} candidates were REJECTED before being",
                  "  scored, which is why a curve can start above the speed you asked for:"]
            reasons = rej["reasons"].astype(str).str.split(":").str[0]
            for why, n in reasons.value_counts().head(4).items():
                L.append(f"    {n:>5} x  {why.strip()[:62]}")
            L += ["  Relax them under SHIFT COST (minimum band, max acceleration given",
                  "  up, minimum reserve) if you want the whole range scored."]
        L += ["",
              "  Read the two slices together: the orange curve is what the motor map",
              "  wants, the blue dashed curve is what the battery pays. Where they",
              "  disagree, the difference is not efficiency - look at 'What the optimum",
              "  actually wins on' in the energy sweeps to see which term it is.",
              "",
              f"  CONVERGED : {sw.converged}"]
        if sw.boundary_note:
            L.append("  NOTE      : " + sw.boundary_note)
        L += self._indifference_lines(sw)
        L += self._crossover_lines(self.params(), "downshift")
        return NL.join(L)

    # ------------------------------------------------- energy bins
    def _draw_bins(self, payload, p, _kind):
        """Side by side: a single ratio, then the shift schedule, then the difference.

        The reference is what the gearbox has to beat. Putting the two bin grids next
        to each other on one colour scale shows the energy physically moving between
        cells, and the fourth panel collapses that to one dimension - how much energy
        sits in each efficiency band before and after.
        """
        runs, bins = payload
        ref_lab = runs.get("_label", "reference")
        R, A = bins["R"], bins["A"]
        rR, rA = runs["R"], runs["A"]
        if A is None:
            self.fig.text(.5, .5, "The schedule is infeasible\n\n"
                          + "\n".join(rA.reasons), ha="center", va="center",
                          color=COLORS["danger"])
            self.log("Infeasible: " + "; ".join(rA.reasons)); return

        gs = self.fig.add_gridspec(2, 2, hspace=.32, wspace=.24)
        ax = [self.fig.add_subplot(gs[i, j]) for i in (0, 1) for j in (0, 1)]
        X, Y = A["rpm_edges"], A["torque_edges"]
        st = self._map_style()

        def frame(a):
            em = self.emap
            Rm, Tm = np.meshgrid(em.rpm, em.torque)
            Z = np.where(em.missing, np.nan, em.eff) * 100
            a.contourf(Rm, Tm, Z, levels=np.linspace(st["lo"], st["hi"], 24),
                       cmap="Greys_r", alpha=.28, extend="both", zorder=0)
            n = np.linspace(1, p["motor"].max_rpm, 400)
            a.plot(n, p["motor"].envelope(n), color=COLORS["danger"], lw=1.6, zorder=6)
            try:
                t_r, n_r = sc.efficiency_ridge(em)
                a.plot(n_r, t_r, color="#d81b60", lw=2, zorder=6)
            except Exception:
                pass
            a.set_xlim(0, X[-1]); a.set_ylim(0, Y[-1])
            a.set_xlabel("Motor speed [rpm]"); a.set_ylabel("Motor torque [Nm]")

        # one colour scale for both grids, or they cannot be compared by eye
        top = max(np.nanmax(A["e_in"]), np.nanmax(R["e_in"]) if R is not None else 0)

        frame(ax[0])
        if R is not None:
            m = ax[0].pcolormesh(X, Y, np.where(R["e_in"] > 0, R["e_in"], np.nan),
                                 cmap="viridis", shading="flat", vmin=0, vmax=top,
                                 zorder=2)
            self._colorbar(m, ax[0], "Energy drawn [kWh]")
            ax[0].set_title(f"BEFORE - {ref_lab}   ({R['total_in']:.3f} kWh, "
                            f"{1000*R['loss'].sum():.0f} Wh lost)",
                            fontweight="bold", fontsize=10)
        else:
            ax[0].set_title("reference infeasible", fontweight="bold", fontsize=10)

        frame(ax[1])
        m = ax[1].pcolormesh(X, Y, np.where(A["e_in"] > 0, A["e_in"], np.nan),
                             cmap="viridis", shading="flat", vmin=0, vmax=top, zorder=2)
        self._colorbar(m, ax[1], "Energy drawn [kWh]")
        ax[1].set_title(f"AFTER - shifting {rA.upshift:g}/{rA.downshift:g}   "
                        f"({A['total_in']:.3f} kWh, {1000*A['loss'].sum():.0f} Wh lost)",
                        fontweight="bold", fontsize=10)

        # --- 3. what moved ---------------------------------------------------
        frame(ax[2])
        if R is not None:
            d = A["e_in"] - R["e_in"]
            lim = float(np.nanmax(np.abs(d))) or 1.0
            m = ax[2].pcolormesh(X, Y, np.where(np.abs(d) > 1e-9, d * 1000, np.nan),
                                 cmap="RdBu_r", shading="flat", vmin=-1000 * lim,
                                 vmax=1000 * lim, zorder=2)
            self._colorbar(m, ax[2], "Energy moved [Wh]")
            ax[2].set_title("WHAT MOVED   red = gained, blue = lost",
                            fontweight="bold", fontsize=10)

        # --- 4. the same thing in one dimension -------------------------------
        ax[3].set_facecolor(COLORS["plot_bg"])
        edges = np.arange(np.floor(st["lo"] / 2) * 2, 96.1, 2.0)
        def spectrum(b):
            e = np.zeros(len(edges) - 1)
            eff = b["eff"] * 100
            for k in range(len(edges) - 1):
                m2 = np.isfinite(eff) & (eff >= edges[k]) & (eff < edges[k + 1])
                e[k] = b["e_in"][m2].sum()
            return e
        centres = (edges[:-1] + edges[1:]) / 2
        wbar = 0.8
        if R is not None:
            ax[3].bar(centres - wbar / 2, spectrum(R), width=wbar,
                      color=COLORS["text_muted"], alpha=.85, label=f"before ({ref_lab})")
        ax[3].bar(centres + wbar / 2, spectrum(A), width=wbar,
                  color=COLORS["primary"], alpha=.9,
                  label=f"after ({rA.upshift:g}/{rA.downshift:g})")
        ax[3].set_xlabel("Efficiency of the cell [%]")
        ax[3].set_ylabel("Energy drawn there [kWh]")
        ax[3].set_title("Energy by efficiency band - the shift, in one dimension",
                        fontweight="bold", fontsize=10)
        ax[3].grid(alpha=.3, axis="y")
        ax[3].legend(fontsize=8)

        self.fig.suptitle(f"Where the energy goes, and what the shift schedule moves"
                          f"   ({X[1]-X[0]:.0f} rpm x {Y[1]-Y[0]:.0f} Nm bins)",
                          fontweight="bold")
        self.log(self._bins_summary(R, A, rR, rA, ref_lab))
        if R is not None:
            self.say(f"{ref_lab}: {1000*R['loss'].sum():.0f} Wh lost  ->  shifting: "
                     f"{1000*A['loss'].sum():.0f} Wh", "ok")

    @staticmethod
    def _bins_summary(R, A, rR, rA, ref_lab):
        NL = chr(10)
        X, Y = A["rpm_edges"], A["torque_edges"]

        def cell(i, j):
            return f"{X[j]:.0f}-{X[j+1]:.0f} rpm, {Y[i]:.0f}-{Y[i+1]:.0f} Nm"

        L = [f"Energy bins   BEFORE = {ref_lab}   AFTER = shifting "
             f"{rA.upshift:g}/{rA.downshift:g} km/h", "=" * 74, ""]
        if R is not None:
            L += [f"  {'':22}{'BEFORE':>12}{'AFTER':>12}{'change':>12}",
                  f"  {'energy drawn':22}{R['total_in']:11.4f}{A['total_in']:12.4f}"
                  f"{1000*(A['total_in']-R['total_in']):+11.1f} Wh",
                  f"  {'delivered at shaft':22}{R['total_out']:11.4f}"
                  f"{A['total_out']:12.4f}"
                  f"{1000*(A['total_out']-R['total_out']):+11.1f} Wh",
                  f"  {'MOTOR LOSS':22}{1000*R['loss'].sum():10.1f} Wh"
                  f"{1000*A['loss'].sum():10.1f} Wh"
                  f"{1000*(A['loss'].sum()-R['loss'].sum()):+11.1f} Wh",
                  f"  {'mean efficiency':22}"
                  f"{R['total_out']/R['total_in']:11.2%}"
                  f"{A['total_out']/A['total_in']:12.2%}"
                  f"{100*(A['total_out']/A['total_in']-R['total_out']/R['total_in']):+10.2f} pts",
                  ""]

        L += ["  BIGGEST ENERGY CELLS AFTER SHIFTING", "  " + "-" * 62,
              f"  {'cell':<28}{'drawn':>9}{'loss':>10}{'eff':>9}{'share':>8}"]
        for k in np.argsort(A["e_in"].ravel())[::-1][:8]:
            i, j = np.unravel_index(k, A["e_in"].shape)
            if A["e_in"][i, j] <= 0:
                break
            L.append(f"  {cell(i,j):<28}{A['e_in'][i,j]:8.4f} {1000*A['loss'][i,j]:8.1f} Wh"
                     f"{A['eff'][i,j]:8.2%}{100*A['e_in'][i,j]/A['total_in']:7.1f} %")

        if R is not None:
            d = A["e_in"] - R["e_in"]
            out_m, in_m = d < -1e-9, d > 1e-9
            moved = float(-d[out_m].sum()); gained = float(d[in_m].sum())
            eff_from = (np.nansum(-d[out_m] * R["eff"][out_m]) / moved) if moved > 0 else np.nan
            eff_to = (np.nansum(d[in_m] * A["eff"][in_m]) / gained) if gained > 0 else np.nan
            L += ["", "  WHAT THE SHIFT SCHEDULE MOVED", "  " + "-" * 62,
                  f"    {moved:.4f} kWh left cells averaging {eff_from:.2%}",
                  f"    {gained:.4f} kWh arrived in cells averaging {eff_to:.2%}",
                  f"    that is {100*moved/R['total_in']:.1f} % of the energy, "
                  f"relocated {100*(eff_to-eff_from):+.2f} points better"
                  if np.isfinite(eff_from) and np.isfinite(eff_to) else ""]
            L += ["", "  CELLS THAT LOST THE MOST", "  " + "-" * 62]
            for k in np.argsort(d.ravel())[:4]:
                i, j = np.unravel_index(k, d.shape)
                if d[i, j] >= -1e-9:
                    break
                L.append(f"    {cell(i,j):<28}{1000*d[i,j]:8.1f} Wh   was running at "
                         f"{R['eff'][i,j]:.2%}")
            L += ["", "  CELLS THAT GAINED THE MOST", "  " + "-" * 62]
            for k in np.argsort(d.ravel())[::-1][:4]:
                i, j = np.unravel_index(k, d.shape)
                if d[i, j] <= 1e-9:
                    break
                L.append(f"    {cell(i,j):<28}{1000*d[i,j]:+8.1f} Wh   now running at "
                         f"{A['eff'][i,j]:.2%}")

        L += ["",
              "  The bottom-right panel is the same story in one dimension: how much",
              "  energy sits in each efficiency band before and after. Bars moving to",
              "  the right is the schedule earning its keep.",
              "",
              "  Cells carry ENERGY, not sample counts - an idling sample at 2 Nm and",
              "  one pulling 25 Nm are not comparable populations, but they are",
              "  comparable amounts of energy."]

        # These bins are (rpm, torque) cells of the MOTOR. Two real costs have no
        # operating point to live in and so cannot appear on this map at all - say
        # so and reconcile, rather than letting the totals silently disagree with
        # the ones every other view quotes.
        shift_kwh = rA.shift_energy_kwh + rA.interrupt_energy_kwh
        aux_kwh = rA.consumed_kwh - A["total_in"] - shift_kwh
        L += ["", "  WHAT IS NOT ON THIS MAP", "  " + "-" * 62,
              f"    binned into cells        {A['total_in']:9.4f} kWh   (traction only)",
              f"    auxiliary load           {aux_kwh:9.4f} kWh   (no rpm, no torque)",
              f"    shift actuator + cut     {shift_kwh:9.4f} kWh   "
              f"({rA.upshifts + rA.downshifts} changes)",
              f"    {'':25}{'-' * 9}",
              f"    consumed (headline)      {rA.consumed_kwh:9.4f} kWh",
              "",
              "    The actuator runs off its own 12 V supply and the traction cut is",
              "    the ABSENCE of an operating point, so neither is a cell on this map.",
              "    They are in every energy total and every sweep ranking - just not",
              "    here, because there is nowhere on a torque-speed map to draw them."]
        return NL.join(L)

    @staticmethod
    def _optimal_summary(sp, ac, better, delta, up, dn, j_per_shift=0.0):
        """Measure the boundary rather than describing it."""
        NL = chr(10)
        g1 = int(np.nansum(better == 1)); g2 = int(np.nansum(better == 2))
        tot = max(g1 + g2, 1)
        fin = np.isfinite(delta)
        L = ["Optimal gear map", "=" * 74, "",
             f"  gear 1 is the better ratio over {100*g1/tot:.0f} % of the "
             f"(speed, acceleration) plane",
             f"  gear 2 over {100*g2/tot:.0f} %", ""]

        # where the boundary sits at each load - this is the number that matters
        L += ["  WHERE THE BOUNDARY SITS, LOAD BY LOAD", "  " + "-" * 58,
              f"  {'acceleration':>13} {'gear 1 wins up to':>20} {'worth at the edge':>19}"]
        edges = []
        for k in range(0, len(ac), max(1, len(ac) // 7)):
            row = better[k]
            ok = np.flatnonzero(np.isfinite(row) & (row == 1))
            if len(ok):
                v = sp[ok.max()]
                edges.append(v)
                L.append(f"  {ac[k]:>10.2f} m/s2 {v:>17.0f} km/h "
                         f"{np.nanmax(delta[k]):>16.1f} pts")
            else:
                L.append(f"  {ac[k]:>10.2f} m/s2 {'never':>17}")
        if len(edges) > 1:
            L += ["",
                  f"  The boundary travels {max(edges)-min(edges):.0f} km/h "
                  f"({min(edges):.0f} to {max(edges):.0f}) across this load range.",
                  f"  Your thresholds are vertical lines at {up:g} and {dn:g} km/h. A",
                  "  vertical line cannot follow a curve that moves that far, so on one",
                  "  side of the plane or the other the controller is knowingly in the",
                  "  wrong ratio. That is the structural limit of a speed-only schedule -",
                  "  not the choice of number."]

        # how much it is worth, so nobody over-reads the boundary
        if fin.any():
            L += ["", "  HOW MUCH THE CHOICE IS WORTH", "  " + "-" * 58]
            for thr in (1, 3, 5, 10):
                L.append(f"    worth more than {thr:2d} point{'s' if thr > 1 else ' '} "
                         f"over {100*np.nansum(delta > thr)/fin.sum():5.1f} % of the plane")
            med = float(np.nanmedian(delta[fin]))
            share3 = 100 * np.nansum(delta > 3) / fin.sum()
            L.append(f"    median advantage where a choice exists: {med:.2f} points")
            # the verdict has to follow the numbers, not a fixed opinion
            if share3 >= 50:
                L += [f"    The choice is decisive over most of the plane - {share3:.0f} %",
                      "    of it is worth more than 3 points. On this map the ratio",
                      "    selection is not a rounding error; a 2-D shift map on speed and",
                      "    torque demand would collect what a speed threshold cannot.",
                      "",
                      "    Note the tension with the drive-cycle result: the plane is",
                      "    weighted by AREA, the cycle by where it actually spends energy.",
                      "    A region can be worth 25 points and still be worth nothing if",
                      "    the vehicle never goes there. Read this against 'Energy bins'."]
            else:
                L += [f"    Most of the plane is nearly indifferent - only {share3:.0f} %",
                      "    of it is worth more than 3 points. Getting the boundary wrong",
                      "    costs little except in that minority, which is the only region",
                      "    worth designing a 2-D shift map for."]

        # This plane is drawn from the efficiency map alone. It says which ratio is
        # better AT a point; it is silent on what it costs to GET there, and a
        # controller that tracked this boundary exactly would be changing gear
        # constantly. That cost is real and it is what decides the answer.
        L += ["", "  WHAT THIS MAP DOES NOT CHARGE", "  " + "-" * 58,
              "    Which ratio is better at a point says nothing about what changing",
              "    to it costs. A controller following this boundary exactly is the",
              "    per-sample oracle in the sweep summaries: on this cycle it makes",
              f"    ~1,600 gear changes, which at {j_per_shift:.0f} J each is more actuator",
              "    energy than the entire efficiency prize it collects.",
              "    So read this map as WHERE a 2-D shift schedule could help, never as",
              "    a controller. Any real schedule has to be smooth enough to be worth",
              "    executing - see the CEILING block in any sweep."]
        return NL.join(L)

    def _draw_gradeability(self, df, p, _kind):
        ax = self.fig.subplots()
        ax.plot(df["Speed [km/h]"], df["Gear 1 max grade [deg]"], "o-", lw=2.2,
                color=GEAR1, label=f"gear 1  (ratio {p['gb'].ratio_1:g})")
        ax.plot(df["Speed [km/h]"], df["Gear 2 max grade [deg]"], "s-", lw=2.2,
                color=GEAR2, label=f"gear 2  (ratio {p['gb'].ratio_2:g})")
        ax.fill_between(df["Speed [km/h]"], df["Gear 2 max grade [deg]"],
                        df["Gear 1 max grade [deg]"], alpha=.16,
                        color=GEAR1, label="what the low ratio adds")
        ax.set_xlabel("Steady speed [km/h]")
        ax.set_ylabel("Maximum sustainable grade [deg]")
        adv_lim = df[df["Gear 1 max grade [deg]"]
                     > df["Gear 2 max grade [deg]"] + 1e-9]["Speed [km/h]"]
        head = "Gradeability — the low ratio's real job"
        if len(adv_lim):
            head += (f"  ·  it adds climbing ability only below "
                     f"{adv_lim.max():.0f} km/h")
        ax.set_title(head, fontweight="bold", fontsize=11.5)
        ax.grid(alpha=.3); ax.legend(fontsize=9)
        adv = df[df["Gear 1 max grade [deg]"] > df["Gear 2 max grade [deg]"] + 1e-9]
        g1c, g2c = "Gear 1 max grade [deg]", "Gear 2 max grade [deg]"
        txt = ["Gradeability - the steepest slope each ratio can hold at a steady speed",
               "=" * 74, "",
               df.to_string(index=False, float_format=lambda x: f"{x:7.2f}"), "",
               "  Steady state, so no inertia term: this is the slope the vehicle can",
               "  HOLD, not one it can accelerate up.", ""]
        txt += ["  WHAT THE LOW RATIO BUYS", "  " + "-" * 58]
        for _, row in df.iterrows():
            gap = row[g1c] - row[g2c]
            if gap > 1e-9:
                txt.append(f"    at {row['Speed [km/h]']:>4.0f} km/h  "
                           f"{row[g2c]:5.2f}deg -> {row[g1c]:5.2f}deg   "
                           f"(x{row[g1c]/max(row[g2c],1e-9):.2f}, "
                           f"{100*np.tan(np.radians(row[g1c])):.0f} % vs "
                           f"{100*np.tan(np.radians(row[g2c])):.0f} % slope)")
        if len(adv):
            v_lim = adv["Speed [km/h]"].max()
            txt += ["",
                    f"  Above {v_lim:.0f} km/h the two ratios are identical - both are",
                    "  power-limited there, and a ratio cannot create power. The low gear",
                    "  therefore has exactly one job: climbing below that speed.",
                    "",
                    f"  => an upshift near {v_lim:.0f} km/h keeps the low ratio through its",
                    "     whole useful range and hands over the moment it stops helping."]
        else:
            txt += ["", "  The two ratios give the same grade at every speed tested -",
                    "  the low ratio adds no climbing ability on this vehicle."]
        best = df[g1c].max()
        txt += ["",
                f"  Steepest slope the vehicle can hold at all: {best:.2f} deg "
                f"({100*np.tan(np.radians(best)):.0f} % gradient), in gear 1 at "
                f"{df.loc[df[g1c].idxmax(), 'Speed [km/h]']:.0f} km/h.",
                "  Check that against the steepest ramp the vehicle has to serve - that",
                "  is the requirement the low ratio exists to meet."]
        self.log(chr(10).join(txt))
        self.say("Gradeability computed", "ok")

    def _draw_efficiency(self, _out, p, _kind):
        em = self.emap
        st = self._map_style()
        ax = self.fig.subplots()
        R, T = np.meshgrid(em.rpm, em.torque)
        eff = np.where(em.missing, np.nan, em.eff) * 100
        c = self._contours(ax, R, T, eff, 1.0, st)
        self._colorbar(c, ax, "Efficiency [%]")
        pk, pr, pt = em.peak
        pw = pt * pr * 2 * np.pi / 60      # shaft power at the peak cell
        if st["envelope"]:
            n = np.linspace(1, p["motor"].max_rpm, 500)
            ax.plot(n, p["motor"].envelope(n), color=COLORS["danger"], lw=2.5,
                    label=f"{p['motor'].peak_torque:g} Nm / "
                          f"{p['motor'].peak_power/1000:g} kW envelope")
            ax.scatter([pr], [pt], s=140, marker="*", color="w", edgecolor="k", zorder=6,
                       label=f"peak {pk:.2%} @ {pr:.0f} rpm / {pt:.1f} Nm")
            ax.legend(fontsize=8, loc="upper right")
        ax.set_xlim(0, em.rpm.max()); ax.set_ylim(0, em.torque.max())
        ax.set_xlabel("Motor speed [rpm]"); ax.set_ylabel("Motor torque [Nm]")
        ax.set_title(f"Motor efficiency map  ·  peak {pk:.2%} at {pr:.0f} rpm / "
                     f"{pt:.1f} Nm  =  {pw/1000:.2f} kW of shaft power",
                     fontweight="bold", fontsize=11.5)
        self.log(self._map_summary(em, p, pk, pr, pt, pw))
        self.say(f"Peak {pk:.2%} at {pr:.0f} rpm / {pt:.1f} Nm", "ok")

    def _map_summary(self, em, p, pk, pr, pt, pw):
        """Describe THIS map: its shape, its ridge, and how peaky it really is."""
        NL = chr(10)
        eff = np.where(em.missing, np.nan, em.eff)
        finite = np.isfinite(eff)
        L = [f"Efficiency map", "=" * 74, "",
             f"  grid              {em.eff.shape[0]} torque rows x {em.eff.shape[1]} "
             f"speed columns",
             f"  torque axis       {em.torque.min():.1f} to {em.torque.max():.1f} Nm",
             f"  speed axis        {em.rpm.min():.0f} to {em.rpm.max():.0f} rpm",
             f"  blank cells       {em.missing.sum():,} of {em.eff.size:,} "
             f"({100*em.missing.sum()/em.eff.size:.1f} %)",
             f"  peak              {pk:.2%} at {pr:.0f} rpm / {pt:.1f} Nm "
             f"= {pw/1000:.2f} kW of shaft power", ""]

        # how peaky: what fraction of the measured plane is within N points of peak
        L += ["  HOW PEAKY IS IT?", "  " + "-" * 58]
        for band in (0.5, 1.0, 2.0, 5.0):
            frac = np.nansum(finite & (eff >= pk - band / 100)) / max(finite.sum(), 1)
            L.append(f"    within {band:3.1f} points of peak : {100*frac:5.1f} % of the "
                     f"measured cells")
        L += ["    A flat map means the ratio choice cannot matter much; a peaky one",
              "    means it can. This is the single best predictor of whether a shift",
              "    study on this motor is worth running."]

        # the ridge - where the best speed sits for each load
        try:
            t_r, n_r = sc.efficiency_ridge(em)
            L += ["", "  THE RIDGE - best speed at each torque", "  " + "-" * 58,
                  f"    {'torque':>9} {'best rpm':>10} {'efficiency':>12}"]
            for want in (2, 5, 10, 20, 30):
                i = int(np.argmin(np.abs(t_r - want)))
                e = em.query(np.array([n_r[i]]), np.array([t_r[i]]),
                             np.array([True]))[0][0]
                L.append(f"    {t_r[i]:8.1f} Nm {n_r[i]:9.0f} {e:11.2%}")
            L += ["    The best SPEED is not a property of the motor - it moves with",
                  "    load. A duty that never reaches the peak's torque should be judged",
                  "    against this curve, not against the peak."]
        except Exception:
            pass

        # what the envelope actually allows
        try:
            R, T = np.meshgrid(em.rpm, em.torque)
            under = T <= p["motor"].envelope(R)
            usable = finite & under
            share = 100 * np.sum(usable) / max(np.sum(finite), 1)
            L += ["", "  REACHABLE REGION", "  " + "-" * 58,
                  f"    {share:.1f} % of the measured cells sit under the "
                  f"{p['motor'].peak_torque:g} Nm / "
                  f"{p['motor'].peak_power/1000:g} kW envelope"]
            if share < 99.5:
                L.append("    the rest is mapped but the motor cannot hold it "
                         "continuously")
            else:
                L.append("    the whole measured map is deliverable - the envelope is not "
                         "the")
                L.append("    binding constraint on this motor over the mapped region")
            L.append(f"    best reachable point: "
                     f"{np.nanmax(np.where(usable, eff, np.nan)):.2%}")
        except Exception:
            pass
        return NL.join(L)

    # -------------------------------------------------------------- summaries
    def _summary(self, r):
        L = [f"Strategy: upshift {r.upshift:g} / downshift {r.downshift:g} km/h",
             "=" * 74, "  " + sc.build_stamp(),
             f"  consumed            {r.consumed_kwh:10.4f} kWh",
             f"  specific            {r.wh_per_km:10.1f} Wh/km over {r.distance_km:.1f} km",
             f"  recovered           {r.recovered_kwh:10.4f} kWh"
             + (f"   ({r.regen_samples:,} pts, mean {r.regen_torque_mean:.1f} Nm "
                f"to the motor)" if r.regen_samples else "   (regen off)"),
             f"  net                 {r.net_kwh:10.4f} kWh   <- the objective",
             f"  shift actuation     {r.shift_energy_kwh*1000:10.1f} Wh   "
             f"({self._f('cost','actuator_voltage',12):g} V x "
             f"{self._f('cost','actuator_current',20):g} A x "
             f"{self._f('cost','actuator_time_s',0.5):g} s = "
             f"{self._f('cost','actuator_voltage',12)*self._f('cost','actuator_current',20)*self._f('cost','actuator_time_s',0.5):.0f} J "
             f"x {r.upshifts+r.downshifts} shifts)",
             f"  torque interruption {r.interrupt_energy_kwh*1000:10.1f} Wh "
             f"({r.shifts_under_traction} of {r.upshifts+r.downshifts} shifts under traction)",
             "",
             f"  shifts              {r.upshifts} up + {r.downshifts} down "
             f"({r.shifts_per_hour:.1f}/h)",
             f"  hysteresis band     {r.upshift-r.downshift:10.1f} km/h",
             f"  accel reserve       {r.reserve_at_upshift:10.2f} m/s2 in gear 2 at the "
             f"upshift",
             f"                      {r.reserve_before_upshift:10.2f} m/s2 in gear 1 at the "
             f"same speed",
             f"  acceleration given up {100*r.accel_loss:8.0f} %  by handing over there",
             ""]
        L += self._gear_table(r)
        L += ["",
             f"  envelope violations {r.envelope_violations:10,}",
             f"  rpm violations      {r.rpm_violations:10,}",
             f"  battery violations  {r.battery_violations:10,}",
             f"  map fallback points {r.fallback_points:10,}",
             f"  outside-map points  {r.outside_map_points:10,}"]
        if r.reasons:
            L += ["", "  NOTES:"] + [f"    - {x}" for x in r.reasons]
        return "\n".join(L)

    def _gear_table(self, r):
        """Per-gear block. Identical text in every view - one call, one definition."""
        try:
            b = sc.gear_breakdown(r, self.cycle)
        except Exception:
            return ["  (per-gear breakdown needs a run that kept its arrays)"]
        L = ["  PER GEAR" + " " * 12 + "gear 1     gear 2        all",
             "  " + "-" * 56]
        def row(label, key, fmt):
            v = [fmt(b.loc[i, key]) for i in range(3)]
            return f"  {label:<18}{v[0]:>10}{v[1]:>11}{v[2]:>11}"
        pc = lambda x: f"{x:.1f} %"
        L += [row("time share", "time_pct", pc),
              row("traction share", "traction_pct", pc),
              row("output energy", "energy_pct", pc),
              row("efficiency", "efficiency", lambda x: f"{x:.2%}"),
              row("motor rpm from", "rpm_lo", lambda x: f"{x:.0f}"),
              row("motor rpm to", "rpm_hi", lambda x: f"{x:.0f}")]
        idle = 100.0 * (1 - b["traction_samples"].iloc[2] / b["samples"].iloc[2])
        L += [f"  time share counts every sample (this cycle spends {idle:.0f} % of it not",
              "  pulling - idling or braking - and all the idle is in gear 1);",
              "  traction share counts only the samples where the motor is pulling.",
              "  efficiency is output energy / input energy - the 'all' column is exactly",
              "  ShiftResult.mean_efficiency, so every panel quotes the same number."]
        return L

    @staticmethod
    def _indifference_lines(sw, col=None):
        """The band of candidates the objective cannot separate.

        Every search in this tool minimises the same quantity on the same cycle,
        so when the upshift sweep, the downshift sweep and the combined grid name
        different winners the reason is never that they disagree about physics -
        it is that the objective is flat and each search reported the argmin of a
        different slice through the same plateau. Quoting the band makes that
        visible instead of leaving three numbers to look contradictory.
        """
        try:
            band = sw.indifference()
        except Exception:
            return []
        if not band["n"]:
            return []
        L = ["", "  WHAT THE OBJECTIVE CAN ACTUALLY RESOLVE", "  " + "-" * 56,
             f"    {band['n']} of {int(sw.table['feasible'].sum())} feasible candidates "
             f"are within {1000*band['tol_kwh']:.1f} Wh (0.1 %) of the best,",
             f"    spanning upshift {band['upshift'][0]:g}-{band['upshift'][1]:g} km/h "
             f"and downshift {band['downshift'][0]:g}-{band['downshift'][1]:g} km/h",
             f"    over {1000*(band['worst_kwh']-band['best_kwh']):.1f} Wh of net energy."]
        if band["n"] > 1:
            L += ["    Inside that band the argmin is decided by the last digit, not by",
                  "    the physics, so a different sweep picking a different point in it",
                  "    is not a disagreement. Choose within the band on driveability:",
                  "    tractive-force continuity for the upshift, shift count for the",
                  "    downshift."]
        else:
            L.append("    The optimum is isolated - this one really is a distinct winner.")
        return L

    def _crossover_lines(self, p, col):
        """Why the efficiency curve slopes the way it does.

        Without this, an efficiency curve that RISES with the downshift speed
        reads as a bug - the expectation is that handing more of the cycle to the
        low ratio must cost efficiency. Whether it does is decided by one number
        from the map: the speed at which the two ratios swap places. Below it the
        low ratio is genuinely better, so raising the downshift towards it moves
        samples into the BETTER gear and mean efficiency climbs. Above it the same
        move costs efficiency. And the crossover is not one speed - it walks up
        with load, which is why a cycle with a lot of low-speed acceleration can
        show efficiency rising far higher than a steady-cruise argument predicts.
        """
        try:
            cr = sc.ratio_crossover(self.emap, veh=p["veh"], motor=p["motor"],
                                    gb=p["gb"])
        except Exception:
            return []
        L = ["", "  WHY THE EFFICIENCY CURVE SLOPES THIS WAY", "  " + "-" * 56,
             "    Speed at which the two ratios swap places on the map, by load:"]
        for a, r in cr.items():
            c = r["crossover_kmh"]
            if np.isfinite(c):
                L.append(f"      at {a:.1f} m/s2 accel : ratio {p['gb'].ratio_1:g} is "
                         f"better below {c:.1f} km/h, ratio {p['gb'].ratio_2:g} above")
            else:
                who = (p["gb"].ratio_1 if r["low_ratio_better_below"]
                       else p["gb"].ratio_2)
                L.append(f"      at {a:.1f} m/s2 accel : ratio {who:g} is better at "
                         f"every speed searched")
        # The steady-load numbers above are the map's opinion. What decides the
        # curve is the load mix of THIS cycle, so ask the cycle too - this is why
        # the same map under a dense city cycle and under a long mixed cycle
        # produces efficiency curves that slope opposite ways.
        try:
            ec = sc.effective_crossover(self.cycle, self.emap, veh=p["veh"],
                                        motor=p["motor"], gb=p["gb"], num=p["num"])
        except Exception:
            ec = None
        if ec is not None and len(ec):
            half = ec[ec["low_ratio_energy_pct"] < 50.0]
            v50 = float(half["speed_lo"].iloc[0]) if len(half) else float("nan")
            below = ec[ec["speed_hi"] <= (v50 if np.isfinite(v50) else 0)]
            share = float(below["energy_pct"].sum()) if len(below) else 0.0
            L += ["", "    On THIS cycle, weighted by the energy actually flowing:",
                  f"      {'band':>12}{'energy':>9}{'low ratio wins':>16}"
                  f"{'low gain':>11}"]
            for _, r in ec.iterrows():
                if r["energy_pct"] < 0.4:
                    continue
                L.append(f"      {r['speed_lo']:4.0f}-{r['speed_hi']:<7.0f}"
                         f"{r['energy_pct']:7.1f}%{r['low_ratio_energy_pct']:14.1f}%"
                         f"{r['low_ratio_gain_pts']:+10.2f} pts")
            if np.isfinite(v50):
                L += ["",
                      f"    The low ratio stops winning above about {v50:.0f} km/h on this",
                      f"    cycle, and everything below that carries only {share:.1f} % of the",
                      "    motoring energy. So raising the downshift towards that speed",
                      "    lifts mean efficiency, and past it the curve turns over -",
                      "    which is exactly the shape you should see in the sweep."]
        finite = [r["crossover_kmh"] for r in cr.values()
                  if np.isfinite(r["crossover_kmh"])]
        if finite:
            L += ["",
                  f"    The crossover WALKS UP WITH LOAD, {min(finite):.0f} to "
                  f"{max(finite):.0f} km/h here. A drive cycle",
                  "    mixes loads, so the effective crossover is somewhere in that",
                  "    span, not at the cruise value."]
            if col == "downshift":
                L += ["",
                      "    So: raising the downshift threshold TOWARDS the crossover",
                      "    moves low-speed samples into the better ratio and mean",
                      "    efficiency RISES - that is correct, not a fault. Past the",
                      "    crossover it falls again. Either way the energy answer can",
                      "    still go the other way, because efficiency is the motor's",
                      "    share alone and the shift cost sits outside it.",
                      "    Use 'Gear comparison' to see the two curves themselves."]
            else:
                L += ["",
                      "    An upshift hands the vehicle to the high ratio, so upshifting",
                      "    below the crossover gives up efficiency as well as tractive",
                      "    force. Use 'Gear comparison' to see the two curves."]
        return L

    def _sweep_summary(self, sw, col):
        j = (self._f("cost", "actuator_voltage", 12.0)
             * self._f("cost", "actuator_current", 20.0)
             * self._f("cost", "actuator_time_s", 0.5))
        L = [f"Sweep over {col}", "=" * 74, "  " + sc.build_stamp(),
             f"  each gear change costs {j:.0f} J at the actuator "
             f"({self._f('cost','actuator_voltage',12):g} V x "
             f"{self._f('cost','actuator_current',20):g} A x "
             f"{self._f('cost','actuator_time_s',0.5):g} s) plus "
             f"{self._f('cost','actuator_time_s',0.5):g} s of zero traction,",
             "  so a schedule that shifts more has to earn it back",
             f"  candidates evaluated : {len(sw.table)}",
             f"  feasible             : {int(sw.table['feasible'].sum())}",
             f"  rejected             : {int((~sw.table['feasible']).sum())}"]
        if sw.best:
            b = sw.best
            L += ["",
                  f"  BEST  upshift {b.upshift:g} / downshift {b.downshift:g} km/h",
                  (f"        net {b.net_kwh:.4f} kWh  = consumed {b.consumed_kwh:.4f}"
                   f"  - recovered {b.recovered_kwh:.4f}   (net is the objective)"
                   if b.recovered_kwh > 0 else
                   f"        {b.consumed_kwh:.4f} kWh   {b.wh_per_km:.1f} Wh/km"),
                  f"        {b.upshifts+b.downshifts} shifts ({b.shifts_per_hour:.1f}/h), "
                  f"gear 2 {b.time_gear2_pct:.1f} % of the time",
                  f"        mean efficiency {b.mean_efficiency:.2%}",
                  f"        shift cost paid {1000*(b.shift_energy_kwh+b.interrupt_energy_kwh):.1f} Wh "
                  f"({1000*b.shift_energy_kwh:.1f} Wh actuator "
                  f"+ {1000*b.interrupt_energy_kwh:.1f} Wh traction cut)"]
            # net, not consumed: net is what _objective() ranks on, and quoting a
            # consumed spread beside a net best is how two numbers that describe
            # one sweep end up disagreeing
            spread = sw.table.loc[sw.table["feasible"], "net_kwh"]
            if len(spread) > 1:
                L.append(f"        spread across feasible set: "
                         f"{spread.min():.4f} - {spread.max():.4f} kWh "
                         f"({100*(spread.max()/spread.min()-1):.2f} %)")
        if sw.best is not None and col in ("upshift", "downshift"):
            other = "downshift" if col == "upshift" else "upshift"
            held = getattr(sw.best, other)
            if all(abs(getattr(d, other) - held) < 1e-9 for d in sw.details):
                a = getattr(sw, "anchor", None)
                typed = getattr(sw, "typed_anchor", None)
                L += ["", f"  ANCHOR: the {other} is held at {held:g} km/h."]
                if a:
                    L += [f"  That is the SELF-CONSISTENT value, not a typed one: alternating",
                          f"  the two sweeps from {typed:g} km/h settled on "
                          f"{a['upshift']:g}/{a['downshift']:g} after {a['rounds']} rounds",
                          f"  ({'stable' if a['converged'] else 'STILL MOVING - raise max_rounds'}), "
                          "so each threshold here is optimal",
                          "  given the other. Anchoring on a typed number instead is what made",
                          "  this sweep, the other sweep and the grid appear to disagree."]
                    if typed is not None and abs(typed - held) > 1e-9:
                        L.append(f"  Your Thresholds box says {typed:g} km/h; that value is not "
                                 f"self-consistent here.")
                else:
                    L += [f"  Taken from the Thresholds box, so this answer is CONDITIONAL on it.",
                          "  Tick 'Self-consistent anchor' to alternate the two sweeps to a",
                          "  fixed point instead - that is what makes them agree with the grid."]
                L += ["  Use 'Combined grid' to search both at once; the two agree whenever",
                      "  the band below covers the grid's answer."]
        L += ["", f"  CONVERGED : {sw.converged}"]
        if sw.boundary_note:
            L += ["  WARNING   : " + sw.boundary_note]
        rej = sw.table[~sw.table["feasible"]]
        if len(rej) and "reasons" in rej:
            L += ["", "  Why candidates were rejected:"]
            for why, n in rej["reasons"].value_counts().head(5).items():
                L.append(f"    {n:>4} x  {why if why else '(band constraint)'}")
        L += self._indifference_lines(sw, col)
        L += self._crossover_lines(self.params(), col)
        L += self._where_it_wins(sw, col)
        L += self._ceiling(sw)
        return "\n".join(L)

    def _where_it_wins(self, sw, col):
        """Split the best-vs-worst difference into physical terms.

        Without this the sweep says only that two schedules differ by N Wh. This
        says whether the optimum earned it by putting the motor somewhere more
        efficient, or by some unrelated term.
        """
        if sw.best is None or self.cycle is None:
            return []
        ok = sw.table[sw.table["feasible"]]
        if len(ok) < 2:
            return []
        try:
            p = self.params()
            worst_val = ok.loc[ok["net_kwh"].idxmax()]
            runs = {}
            for tag, up, dn in (("best", sw.best.upshift, sw.best.downshift),
                                ("worst", worst_val["upshift"], worst_val["downshift"])):
                r = sc.simulate(self.cycle, self.emap, up, dn, keep_arrays=True, **p)
                runs[tag] = (r, sc.energy_breakdown(r, self.cycle, veh=p["veh"],
                                                    motor=p["motor"], gb=p["gb"],
                                                    elec=p["elec"], num=p["num"]))
        except Exception:
            return []

        (rb, bb), (rw, bw) = runs["best"], runs["worst"]
        # The five terms are CONSUMED-side. Dividing them by a NET difference is
        # only valid with regen off; with regen on the two schedules also recover
        # different amounts, so shares came out over 100 % (a "116 % of the gain"
        # was reported). Attribute against the consumed difference the terms
        # actually sum to, and reconcile to net on its own line underneath.
        total = 1000 * (bw["battery"] - bb["battery"])
        d_rec = 1000 * (rw.recovered_kwh - rb.recovered_kwh)
        net_gap = 1000 * (rw.net_kwh - rb.net_kwh)
        if abs(total) < 1e-9:
            return []
        L = ["", "  WHAT THE OPTIMUM ACTUALLY WINS ON", "  " + "-" * 56,
             f"    best {rb.upshift:g}/{rb.downshift:g} vs worst "
             f"{rw.upshift:g}/{rw.downshift:g}  ->  {net_gap:.1f} Wh apart on NET",
             f"    of which {total:.1f} Wh is consumed"
             + (f" and {d_rec:+.1f} Wh is a difference in what was recovered"
                if abs(d_rec) > 0.05 else ""),
             "",
             f"    {'term':<26}{'best':>10}{'worst':>10}{'diff':>11}"]
        for key, label in (("wheel", "demanded at the wheel"),
                           ("gearbox", "gearbox loss"),
                           ("motor", "MOTOR loss (efficiency)"),
                           ("aux", "auxiliary load"),
                           ("shift", "shift actuator + cut")):
            d = 1000 * (bw[key] - bb[key])
            L.append(f"    {label:<26}{bb[key]:10.4f}{bw[key]:10.4f}{d:+10.1f} Wh")
        L += [f"    {'':<26}{'':>10}{'':>10}{'-' * 10:>11}",
              f"    {'sum = consumed':<26}{bb['battery']:10.4f}{bw['battery']:10.4f}"
              f"{total:+10.1f} Wh",
              f"    {'recovered (regen)':<26}{rb.recovered_kwh:10.4f}"
              f"{rw.recovered_kwh:10.4f}{-d_rec:+10.1f} Wh",
              f"    {'NET':<26}{rb.net_kwh:10.4f}{rw.net_kwh:10.4f}"
              f"{net_gap:+10.1f} Wh",
              "",
              f"    {'mean motor efficiency':<26}{bb['efficiency']:9.2%}"
              f"{bw['efficiency']:10.2%}"
              f"{100*(bb['efficiency']-bw['efficiency']):+9.2f} pts", ""]

        # Name the term that ACTUALLY carried it. Asserting "the operating points
        # moved into a better part of the map" while the motor-loss column shows
        # the optimum losing MORE in the motor prints a negative percentage next
        # to a sentence contradicting it.
        terms = {k: 1000 * (bw[k] - bb[k])
                 for k in ("wheel", "gearbox", "motor", "aux", "shift")}
        key, dom = max(terms.items(), key=lambda kv: abs(kv[1]))
        names = {"wheel": "less work demanded at the wheel",
                 "gearbox": "lower gearbox loss", "motor": "lower MOTOR loss",
                 "aux": "the auxiliary load", "shift": "fewer/cheaper gear changes"}
        L.append(f"    -> {100*dom/total:.0f} % of the {total:+.1f} Wh comes from "
                 f"{names[key]}.")
        if key == "motor":
            L += ["       The operating points genuinely moved into a better part of the",
                  "       map. That gain is spread over every loaded sample, which is why",
                  "       it is not visible in 'Shift movement' - that view draws only the",
                  f"       {rb.upshifts + rb.downshifts} shift events. Use 'Points on map' "
                  "to see the cloud move."]
        elif key == "shift":
            m = terms["motor"]
            n_b, n_w = rb.upshifts + rb.downshifts, rw.upshifts + rw.downshifts
            L.append(f"       The optimum makes {n_b} gear changes against {n_w}.")
            if m < 0:
                # the map actively prefers the schedule that LOST
                L += [f"       The motor term went {m:+.1f} Wh, so the efficiency map is",
                      "       AGAINST the winner - it wins purely by shifting less.",
                      "       Read this as a shift-count result, not a map result: on",
                      "       this cycle the threshold is worth more for what it avoids",
                      "       doing than for where it puts the motor."]
            else:
                L += [f"       The motor term agrees ({m:+.1f} Wh), so both effects point",
                      "       the same way - but shift count is the larger of the two.",
                      "       Read it as a shift-count result with the map along for the",
                      "       ride, not as an efficiency finding."]
        else:
            L += [f"       The efficiency map is not what separates these two schedules;",
                  f"       the motor term is only {terms['motor']:+.1f} Wh of the "
                  f"{total:+.1f} Wh.",
                  "       Do not read this sweep as an efficiency result."]
        return L

    def _ceiling(self, sw):
        """How much any schedule could ever win - the answer the sweep cannot give.

        A sweep only ever compares the candidates in it. This compares the best of
        them against a controller allowed to pick the better ratio at EVERY sample
        with perfect foresight and no shift cost, which no causal controller can
        beat. When the gap between "best single ratio" and that oracle is small,
        the shift schedule cannot matter however it is chosen.

        Every row carries its GEAR-CHANGE COUNT, because that is what makes the
        rows comparable: the single-ratio strategies make none, so they pay no
        shift cost; the sweep's best pays for the changes it makes; and the oracle
        makes thousands. The oracle is therefore shown twice - free, which is the
        true mathematical bound, and charged its own actuator energy, which is what
        the comparison actually has to be made against.
        """
        if self.cycle is None or self.emap is None:
            return []
        try:
            o = sc.oracle_bound(self.cycle, self.emap, **self.params())
        except Exception:
            return []
        n = (sw.best.upshifts + sw.best.downshifts) if sw.best is not None else 0
        best_kwh = sw.best.net_kwh if sw.best is not None else float("nan")
        j = (self._f("cost", "actuator_voltage", 12.0)
             * self._f("cost", "actuator_current", 20.0)
             * self._f("cost", "actuator_time_s", 0.5))
        L = ["", "  CEILING - what is on the table at all", "  " + "-" * 56,
             f"    {'strategy':<36}{'net kWh':>9}{'changes':>9}",
             f"    {'always the low ratio':<36}{o['gear1_only']:9.4f}{0:9d}",
             f"    {'always the high ratio':<36}{o['gear2_only']:9.4f}{0:9d}",
             f"    {chr(34) + 'this sweep' + chr(34) + ' best':<36}{best_kwh:9.4f}{n:9d}",
             f"    {'perfect per-sample, free shifting':<36}{o['oracle']:9.4f}"
             f"{o['oracle_shifts']:9d}",
             f"    {'the same, charged for its changes':<36}{o['oracle_charged']:9.4f}"
             f"{o['oracle_shifts']:9d}",
             "",
             f"    Free-shifting prize over the better single ratio: {o['prize_wh']:.1f} Wh "
             f"({100*o['prize_wh']/1000/o['best_single']:.2f} %).",
             f"    But the oracle buys it with {o['oracle_shifts']:,} gear changes, and at "
             f"{j:.0f} J each that is",
             f"    {o['oracle_shift_wh']:.1f} Wh of actuator alone - "
             f"{'MORE than the whole prize' if o['oracle_shift_wh'] > o['prize_wh'] else 'less than the prize'}."]
        if o["prize_charged_wh"] < 0:
            L += [f"    Charged, the prize is {o['prize_charged_wh']:+.1f} Wh: perfect "
                  "clairvoyant gear selection",
                  "    loses to simply staying in one ratio. And that is the GENEROUS",
                  "    reading - only the actuator is charged, not the 0.5 s of lost",
                  "    traction each change also costs.",
                  "    => the ratio choice cannot pay for the act of changing ratio."]
        else:
            L.append(f"    Charged, {o['prize_charged_wh']:+.1f} Wh of prize survives.")
        if sw.best is not None:
            got = (o["best_single"] - sw.best.net_kwh) * 1000.0   # both are NET
            paid = 1000 * (sw.best.shift_energy_kwh + sw.best.interrupt_energy_kwh)
            L += ["",
                  f"    This sweep's best is {got:+.1f} Wh against the better single ratio,",
                  f"    paying {paid:.1f} Wh of shift cost over {n} changes."]
            if got < 0:
                L.append("    Negative: the second ratio does not earn its keep on this duty.")
        L += ["",
              "    The oracle has clairvoyance and no rate limit, so it is not achievable",
              f"    - it is the bound. The better ratio is the low one on {o['gear1_share']:.1f} % of",
              "    moving samples, and those samples are selected by LOAD, not by speed -",
              "    which is why a speed threshold cannot collect them (see 'Optimal gear",
              "    map')."]
        return L


# Every analysis name keys THREE registries - ANALYSES (the menu), NEEDS (what
# data it requires) and DRAW (how to plot it) - plus a branch in _work(). Missing
# one of them surfaced as a KeyError inside a Tk click handler, i.e. as a
# traceback in the console and a dead button in the UI. Check at import instead.
_no_draw = [a for a in ANALYSES if a not in ShiftOptimiserApp.DRAW]
_no_method = [a for a in ANALYSES
              if a in ShiftOptimiserApp.DRAW
              and not hasattr(ShiftOptimiserApp, ShiftOptimiserApp.DRAW[a])]
if _no_draw or _no_method:
    raise RuntimeError(
        "analysis registry incomplete - "
        + (f"no DRAW entry: {', '.join(_no_draw)}. " if _no_draw else "")
        + (f"DRAW names a missing method: {', '.join(_no_method)}." if _no_method else ""))


if __name__ == "__main__":
    ShiftOptimiserApp().mainloop()
