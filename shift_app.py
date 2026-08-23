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
    "Gradeability": (),
    "Acceleration run": ("map",),
    "Efficiency map": ("map",),
}

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
    ("Electrical", "Pack resistance [ohm]", "pack_resistance", "0.020", "el"),
    ("Electrical", "Auxiliary load [W]", "aux_load", "150", "el"),
    ("Electrical", "Battery power limit [W]", "max_power", "15000", "el"),
    ("Regen", "Regen enabled (0/1)", "regen", "0", "num"),
    ("Regen", "Brake torque to motor [0-1]", "regen_fraction", "0.70", "el"),
    ("Regen", "Charge power limit [W] (0=same)", "regen_max_power", "0", "el"),
    ("Regen", "Blend out below [km/h]", "regen_min_speed", "5", "el"),
    ("Shift cost", "Energy per shift [J]", "energy_per_shift", "500", "cost"),
    ("Shift cost", "Torque interruption [s]", "interrupt_s", "0.4", "cost"),
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
CMAPS = ["viridis", "plasma", "magma", "cividis", "turbo", "coolwarm", "Greys"]


class ShiftOptimiserApp(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Two-Speed Shift Optimiser")
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

    def _check(self, parent, label, default):
        v = ctk.BooleanVar(value=default)
        ctk.CTkCheckBox(parent, text=label, variable=v, command=self.redraw,
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
        return dict(cbar=self.disp["cbar"].get(),
                    fill=int(self.disp["fill_levels"].get()) if self.disp["fill_on"].get() else 0,
                    lines=int(self.disp["line_levels"].get()) if self.disp["lines_on"].get() else 0,
                    labels=self.disp["clabels"].get(),
                    envelope=self.disp["envelope"].get(),
                    cmap=self.disp["cmap"].get())

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
        if st["fill"] > 0:
            filled = ax.contourf(X, Y, Z, levels=st["fill"], cmap=st["cmap"], alpha=alpha)
        if st["lines"] > 0:
            lines = ax.contour(X, Y, Z, levels=st["lines"],
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
        need = NEEDS[self.analysis.get()]
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
        el = sc.Electrical(self._f("el", "voltage", 52), self._f("el", "pack_resistance", .02),
                           self._f("el", "aux_load", 150), self._f("el", "max_power", 15000),
                           regen_enabled=bool(self._f("num", "regen", 0)),
                           regen_fraction=self._f("el", "regen_fraction", 0.70),
                           regen_max_power=self._f("el", "regen_max_power", 0.0),
                           regen_min_speed_kmh=self._f("el", "regen_min_speed", 5.0))
        cost = sc.ShiftCost(self._f("cost", "energy_per_shift", 0),
                            self._f("cost", "interrupt_s", 0),
                            self._f("cost", "max_shifts_per_hour", np.inf) or np.inf,
                            min_band_kmh=self._f("cost", "min_band_kmh", 2.0),
                            min_accel_reserve=self._f("cost", "min_accel_reserve", 0.0),
                            max_accel_loss=self._f("cost", "max_accel_loss", 1.0))
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
        for k in NEEDS[kind]:
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
            elif kind == "Upshift sweep":
                out = sc.sweep_upshift(self.cycle, self.emap, I["downshift"],
                                       I["up_lo"], I["up_hi"], I["step"], **p)
            elif kind == "Downshift sweep":
                out = sc.sweep_downshift(self.cycle, self.emap, I["upshift"],
                                         I["dn_lo"], I["dn_hi"], I["step"], **p)
            elif kind == "Combined grid":
                out = sc.sweep_grid(self.cycle, self.emap,
                                    I["up_lo"], I["up_hi"], I["step"],
                                    I["dn_lo"], I["dn_hi"], I["step"],
                                    min_band=I["min_band"], **p)
            elif kind == "Acceleration run":
                out = sc.wot_sweep(self.emap, I["v_target"], I["up_lo"], I["up_hi"],
                                   I["step"], throttle=I["throttle"], **p)
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
    def _render(self, kind, out, p):
        self._last_render = (kind, out, p)
        self.fig.clear()
        getattr(self, "_draw_" + kind.split()[0].lower())(out, p, kind)
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
        self._fit_canvas(kind)
        self.canvas.draw()

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
            a.set_ylabel(lab, fontsize=10)
            a.grid(alpha=.3)
        ax[-1].set_xlabel("Time [s]")
        ax[0].set_xlim(lo, hi)

        win = (f"{lo:.0f}-{hi:.0f} s of {t[-1]:.0f} s"
               if self.disp["window"].get()[0].isdigit() else "full cycle")
        self.fig.suptitle(f"Upshift {r.upshift:g} / downshift {r.downshift:g} km/h"
                          f"   —   {win}   —   {len(shifts)} gear changes shown",
                          fontweight="bold")
        self.log(self._summary(r))
        self.say(f"{r.consumed_kwh:.3f} kWh — {r.wh_per_km:.1f} Wh/km "
                 f"— {len(keys)} signals plotted", "ok")

    def _signal(self, r, key, t):
        """(values, axis label, colour) for one time-series signal."""
        if key == "speed":
            return self.cycle.speed_kmh, "Speed [km/h]", COLORS["primary"]
        if key == "gear":
            return r.gear.astype(float), "Gear", COLORS["text"]
        if key == "rpm":
            return r.motor_rpm, "Motor speed [rpm]", COLORS["accent"]
        if key == "torque":
            return r.motor_torque, "Motor torque [Nm]", COLORS["warning"]
        if key == "eff":
            # eff is filled with 1.0 wherever the motor is idle - blank those out
            act = (np.abs(r.motor_torque) > 1e-9) & (r.motor_eff < 1.0)
            return np.where(act, r.motor_eff * 100, np.nan), "Motor eff [%]", COLORS["success"]
        if key == "pbatt":
            return r.battery_power / 1000.0, "Battery [kW]", COLORS["secondary"]
        if key == "pmech":
            return (r.motor_torque * r.motor_rpm * 2 * np.pi / 60.0) / 1000.0,                    "Shaft power [kW]", "#0891b2"
        if key == "accel":
            return np.gradient(self.cycle.speed_kmh / 3.6, t, edge_order=2),                    "Accel [m/s2]", "#7c3aed"
        if key == "energy":
            pos = np.maximum(np.nan_to_num(r.battery_power), 0.0)
            e = np.concatenate([[0.0], np.cumsum(.5 * (pos[1:] + pos[:-1]) * np.diff(t))]) / 3.6e6
            return e, "Consumed [kWh]", COLORS["danger"]
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
        ax.plot(ok[col], ok["consumed_kwh"], "o-", color=COLORS["primary"],
                label="feasible", ms=5)
        if len(bad):
            ax.scatter(bad[col], np.full(len(bad), ok["consumed_kwh"].min()),
                       marker="x", color=COLORS["danger"], s=40, label="rejected")
        if sw.best:
            b = getattr(sw.best, col)
            ax.scatter([b], [sw.best.consumed_kwh], s=180, zorder=5,
                       facecolor=COLORS["success"], edgecolor="k", lw=1.2,
                       label=f"best {b:g} km/h")
        lo, hi = df[col].min(), df[col].max()
        for v in (lo, hi):
            ax.axvline(v, color=COLORS["danger"], ls=":", lw=1.2, alpha=.7)
        ax.text(lo, ax.get_ylim()[1], " search bound", color=COLORS["danger"],
                fontsize=8, va="top")
        ax.set_xlabel(xlabel); ax.set_ylabel("Consumed energy [kWh]")
        ax.grid(alpha=.3)
        ax.legend(fontsize=9)
        title = kind + ("" if sw.converged else "  —  NOT CONVERGED")
        ax.set_title(title, fontweight="bold",
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
        piv = df.pivot_table(index="downshift", columns="upshift", values="consumed_kwh")
        m = ax.pcolormesh(piv.columns, piv.index, piv.to_numpy(), shading="auto",
                          cmap=self._map_style()["cmap"])
        self._colorbar(m, ax, "Consumed energy [kWh]")
        if sw.best:
            ax.scatter([sw.best.upshift], [sw.best.downshift], marker="X", s=210,
                       facecolor=COLORS["success"], edgecolor="k", lw=1.4, zorder=5,
                       label=f"best {sw.best.upshift:g}/{sw.best.downshift:g}")
            ax.legend(fontsize=9)
        ax.set_xlabel("Upshift [km/h]"); ax.set_ylabel("Downshift [km/h]")
        ax.set_title(kind + ("" if sw.converged else "  —  NOT CONVERGED"),
                     fontweight="bold",
                     color=COLORS["text"] if sw.converged else COLORS["danger"])
        self.log(self._sweep_summary(sw, "upshift"))
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
            base = act & (r.gear == gear)
            groups = [(base & (r.motor_torque > 0), "motoring", .55, col, mk)]
            if signed:
                groups.append((base & (r.motor_torque < 0), "braking", .5, "none", mk))
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
            a.legend(fontsize=8, loc="upper left" if signed else "upper right")
        if signed:
            ax[0].set_ylabel("Signed motor torque [Nm]\n(+ motoring,  - braking)")
        self._colorbar(c, ax, "Efficiency [%]" + ("  (negative half mirrored)" if signed
                                                  else ""), fraction=.04)
        self.fig.suptitle(f"Where the cycle sits on the map — upshift {r.upshift:g} / "
                          f"downshift {r.downshift:g} km/h", fontweight="bold")

        L = ["Operating points on the efficiency map", "=" * 74, ""]
        L += self._gear_table(r)
        L += ["",
              "  These are the same figures the Single-strategy summary prints; both are",
              "  built by shift_core.gear_breakdown() so they cannot drift apart.",
              "",
              "  Both clouds hug the bottom of the plane. The star is the map's best point;",
              "  the cycle never gets near it because the demand is a 1-2 kW load and that",
              "  point is a 5.8 kW operating condition."]
        if signed:
            L += ["",
                  "  The braking half (negative torque) is drawn from the map MIRRORED about",
                  "  zero torque - eta(-T, n) assumed equal to eta(|T|, n). That is an",
                  "  assumption, not measured data. With regen disabled those points carry no",
                  "  energy at all, so they change nothing in the totals above."]
        self.log("\n".join(L))
        self.say("Operating cloud plotted" + (" with the braking half" if signed else ""),
                 "ok")

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
        for A, grp, col, lab in ((ax[0], ups, COLORS["success"], "Upshift  1 -> 2"),
                                 (ax[1], dns, COLORS["warning"], "Downshift  2 -> 1")):
            c = self._signed_background(A, p, tmin, tmax)
            take = grp[:: max(1, len(grp) // 30)] if len(grp) else grp
            new_gear = 2 if lab.startswith("Upshift") else 1
            if len(take):
                cf_n, cf_t = sc.counterfactual_point(r, take, new_gear, p["gb"])
            for k, i in enumerate(take):
                # solid: what the GEARBOX does - same instant, ratio swapped
                A.annotate("", xy=(cf_n[k], cf_t[k]),
                           xytext=(r.motor_rpm[i], r.motor_torque[i]),
                           arrowprops=dict(arrowstyle="-|>,head_width=.28,head_length=.6",
                                           color=col, lw=1.7, alpha=.95,
                                           shrinkA=0, shrinkB=0), zorder=7)
                # dotted: what the DRIVER does in the same step - demand moving on
                A.annotate("", xy=(r.motor_rpm[i + 1], r.motor_torque[i + 1]),
                           xytext=(cf_n[k], cf_t[k]),
                           arrowprops=dict(arrowstyle="-|>,head_width=.18,head_length=.4",
                                           color=COLORS["text_muted"], lw=1.0, ls=":",
                                           alpha=.8, shrinkA=0, shrinkB=0), zorder=6)
            if len(grp):
                A.scatter(r.motor_rpm[grp], r.motor_torque[grp], s=26, marker="o",
                          facecolor="none", edgecolor=col, lw=1.3, zorder=8,
                          label="before the shift")
                gn, gt = sc.counterfactual_point(r, grp, new_gear, p["gb"])
                A.scatter(gn, gt, s=42, marker="X", color=col, edgecolor="k", lw=.5,
                          zorder=8, label="after the shift (same instant)")
                A.scatter(r.motor_rpm[grp + 1], r.motor_torque[grp + 1], s=18, marker=".",
                          color=COLORS["text_muted"], alpha=.7, zorder=7,
                          label="next sample (driver moved on)")
                # the gearbox delta, not the temporal one: same instant, ratio swapped
                d_n = np.mean(gn - r.motor_rpm[grp])
                d_t = np.mean(gt - r.motor_torque[grp])
                A.set_title(f"{lab}   ({len(grp)} events)\n"
                            f"the ratio alone moves it  {d_n:+.0f} rpm,  {d_t:+.2f} Nm",
                            fontweight="bold", fontsize=11)
                A.legend(fontsize=7, loc="best", framealpha=.85)
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
            A.plot(speeds, rec[1]["eff"] * 100, "-", lw=2, color=COLORS["primary"],
                   label="Gear 1 (low, high rpm)")
            A.plot(speeds, rec[2]["eff"] * 100, "-", lw=2, color=COLORS["warning"],
                   label="Gear 2 (high, low rpm)")
            d = rec[1]["eff"] - rec[2]["eff"]
            with np.errstate(invalid="ignore"):
                cross = np.flatnonzero(np.diff(np.sign(np.nan_to_num(d))) != 0)
            for x in speeds[cross]:
                A.axvline(x, color=COLORS["danger"], ls="--", lw=1.2)
                A.text(x, A.get_ylim()[0], f" {x:.0f}", color=COLORS["danger"], fontsize=8)
            A.set_title(f"a = {acc:g} m/s²", fontsize=11, fontweight="bold")
            A.set_xlabel("Road speed [km/h]")
            A.grid(alpha=.3)
            if a_i == 0:
                A.set_ylabel("Motor efficiency [%]")
                lines = A.get_legend_handles_labels()
        self.fig.legend(*lines, loc="lower center", ncol=2, fontsize=10,
                        bbox_to_anchor=(.5, -.02))
        self.fig.suptitle("Which ratio is more efficient — and how that flips with load",
                          fontweight="bold")
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
        txt += ["",
                "  At cruise the motor runs at 5-15 % of rated torque, so iron and windage",
                "  losses (which grow with rpm) beat copper loss (which grows with torque^2)",
                "  -> the LOW-rpm ratio wins.",
                "  Under acceleration torque is high, copper loss dominates",
                "  -> the HIGH-rpm ratio wins.",
                "",
                "  A shift schedule based on SPEED ALONE cannot follow that crossover.",
                "  Real AMT controllers use a 2-D map (speed x torque demand)."]
        self.log("\n".join(txt))
        self.say("Gear preference flips with load — see the crossover lines", "warn")

    def _draw_optimal(self, payload, p, _kind):
        """2-D map of which ratio wins, with the 1-D shift line drawn on top."""
        sp, ac, better, delta, f1, f2 = payload
        ax = self.fig.subplots(1, 2)
        from matplotlib.colors import ListedColormap
        cm = ListedColormap([COLORS["primary"], COLORS["warning"]])
        ax[0].pcolormesh(sp, ac, np.ma.masked_invalid(better), cmap=cm, shading="auto",
                         vmin=1, vmax=2)
        ax[0].contour(sp, ac, np.nan_to_num(better, nan=0), levels=[1.5],
                      colors="k", linewidths=2)
        up = self._f("thr", "upshift", 22)
        dn = self._f("thr", "downshift", 10)
        ax[0].axvline(up, color=COLORS["success"], lw=2.5, label=f"upshift {up:g}")
        ax[0].axvline(dn, color=COLORS["danger"], lw=2.5, ls="--", label=f"downshift {dn:g}")
        ax[0].set_xlabel("Road speed [km/h]"); ax[0].set_ylabel("Acceleration [m/s²]")
        ax[0].set_title("Which ratio is more efficient\n"
                        "blue = gear 1   yellow = gear 2", fontweight="bold")
        ax[0].legend(fontsize=9, loc="upper right")
        m = ax[1].pcolormesh(sp, ac, np.ma.masked_invalid(delta), cmap="magma", shading="auto")
        self._colorbar(m, ax[1], "Advantage of the better ratio [pts]")
        ax[1].axvline(up, color=COLORS["success"], lw=2.5)
        ax[1].axvline(dn, color=COLORS["danger"], lw=2.5, ls="--")
        ax[1].set_xlabel("Road speed [km/h]"); ax[1].set_ylabel("Acceleration [m/s²]")
        ax[1].set_title("How much it matters", fontweight="bold")
        g1 = np.nansum(better == 1); g2 = np.nansum(better == 2)
        big = np.nansum(delta > 3)
        self.log(
            "Optimal gear map\n" + "=" * 74 + "\n"
            f"  gear 1 better over {100*g1/(g1+g2):.0f} % of the (speed, accel) plane\n"
            f"  gear 2 better over {100*g2/(g1+g2):.0f} %\n"
            f"  the choice is worth >3 efficiency points over "
            f"{100*big/np.isfinite(delta).sum():.0f} % of the plane\n\n"
            "  The black line is the true optimal-gear boundary. It is a CURVE in the\n"
            "  (speed, acceleration) plane.\n\n"
            "  Your shift thresholds are the two vertical lines. A vertical line cannot\n"
            "  follow a curve: wherever they diverge, the controller is in the wrong gear.\n"
            "  THIS is the real limit of the study — not the exact upshift number.\n"
            "  The fix is a 2-D shift map (speed x torque demand), which is what production\n"
            "  AMT/DCT controllers use.")
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
                cut = self._f("cost", "interrupt_s", 0.0)
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

    def _draw_gradeability(self, df, p, _kind):
        ax = self.fig.subplots()
        ax.plot(df["Speed [km/h]"], df["Gear 1 max grade [deg]"], "o-",
                color=COLORS["primary"], label=f"Gear 1 (ratio {p['gb'].ratio_1:g})")
        ax.plot(df["Speed [km/h]"], df["Gear 2 max grade [deg]"], "s-",
                color=COLORS["warning"], label=f"Gear 2 (ratio {p['gb'].ratio_2:g})")
        ax.fill_between(df["Speed [km/h]"], df["Gear 2 max grade [deg]"],
                        df["Gear 1 max grade [deg]"], alpha=.18,
                        color=COLORS["primary"], label="Gear 1 advantage")
        ax.set_xlabel("Steady speed [km/h]"); ax.set_ylabel("Max sustainable grade [deg]")
        ax.set_title("Gradeability — what the low gear is for", fontweight="bold")
        ax.grid(alpha=.3); ax.legend(fontsize=9)
        adv = df[df["Gear 1 max grade [deg]"] > df["Gear 2 max grade [deg]"] + 1e-9]
        txt = ["Gradeability", "=" * 74,
               df.to_string(index=False, float_format=lambda x: f"{x:7.2f}"), ""]
        if len(adv):
            txt.append(f"Gear 1 provides extra grade only below "
                       f"{adv['Speed [km/h]'].max():.0f} km/h; above that both ratios are "
                       f"power-limited and identical.")
            txt.append("=> set the upshift threshold near that speed: the low gear keeps its "
                       "climbing duty and the high gear takes the rest.")
        self.log("\n".join(txt))
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
        ax.set_title("Motor efficiency map", fontweight="bold")
        pw = pt * pr * 2 * np.pi / 60
        self.log(f"Efficiency map\n{'='*74}\n"
                 f"  grid            : {em.eff.shape[0]} torque x {em.eff.shape[1]} speed\n"
                 f"  blank cells     : {em.missing.sum():,} of {em.eff.size:,} "
                 f"({100*em.missing.sum()/em.eff.size:.1f} %)\n"
                 f"  peak efficiency : {pk:.2%} at {pr:.0f} rpm / {pt:.1f} Nm\n"
                 f"  that point is   : {pw/1000:.2f} kW of shaft power\n\n"
                 f"  A gear ratio moves the operating point ALONG an iso-power curve.\n"
                 f"  If the cycle's mean power is far below {pw/1000:.2f} kW, no shift\n"
                 f"  schedule can reach this island — the motor is oversized for the duty.")
        self.say(f"Peak {pk:.2%} at {pr:.0f} rpm / {pt:.1f} Nm", "ok")

    # -------------------------------------------------------------- summaries
    def _summary(self, r):
        L = [f"Strategy: upshift {r.upshift:g} / downshift {r.downshift:g} km/h", "=" * 74,
             f"  consumed            {r.consumed_kwh:10.4f} kWh",
             f"  specific            {r.wh_per_km:10.1f} Wh/km over {r.distance_km:.1f} km",
             f"  recovered           {r.recovered_kwh:10.4f} kWh"
             + (f"   ({r.regen_samples:,} pts, mean {r.regen_torque_mean:.1f} Nm "
                f"to the motor)" if r.regen_samples else "   (regen off)"),
             f"  net                 {r.net_kwh:10.4f} kWh   <- the objective",
             f"  shift actuation     {r.shift_energy_kwh*1000:10.1f} Wh",
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

    def _sweep_summary(self, sw, col):
        L = [f"Sweep over {col}", "=" * 74,
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
                  f"        mean efficiency {b.mean_efficiency:.2%}"]
            spread = sw.table.loc[sw.table["feasible"], "consumed_kwh"]
            if len(spread) > 1:
                L.append(f"        spread across feasible set: "
                         f"{spread.min():.4f} - {spread.max():.4f} kWh "
                         f"({100*(spread.max()/spread.min()-1):.2f} %)")
        L += ["", f"  CONVERGED : {sw.converged}"]
        if sw.boundary_note:
            L += ["  WARNING   : " + sw.boundary_note]
        rej = sw.table[~sw.table["feasible"]]
        if len(rej) and "reasons" in rej:
            L += ["", "  Why candidates were rejected:"]
            for why, n in rej["reasons"].value_counts().head(5).items():
                L.append(f"    {n:>4} x  {why if why else '(band constraint)'}")
        L += self._ceiling(sw)
        return "\n".join(L)

    def _ceiling(self, sw):
        """How much any schedule could ever win - the answer the sweep cannot give.

        A sweep only ever compares the candidates in it. This compares the best of
        them against a controller allowed to pick the better ratio at EVERY sample
        with perfect foresight and no shift cost, which no causal controller can
        beat. When the gap between "best single ratio" and that oracle is small,
        the shift schedule cannot matter however it is chosen.
        """
        if self.cycle is None or self.emap is None:
            return []
        try:
            o = sc.oracle_bound(self.cycle, self.emap, **self.params())
        except Exception:
            return []
        L = ["", "  CEILING - what is on the table at all", "  " + "-" * 56,
             f"    always the low ratio      {o['gear1_only']:.4f} kWh",
             f"    always the high ratio     {o['gear2_only']:.4f} kWh",
             f"    perfect per-sample choice {o['oracle']:.4f} kWh  (oracle)",
             f"    => the entire prize over the better single ratio is "
             f"{o['prize_wh']:.1f} Wh ({100*o['prize_wh']/1000/o['best_single']:.2f} %)"]
        if sw.best is not None and o["prize_wh"] > 0:
            got = (o["best_single"] - sw.best.net_kwh) * 1000.0   # both are NET
            L.append(f"    this sweep's best captures {got:+.1f} Wh of that "
                     f"({100*got/o['prize_wh']:.0f} % of the prize)")
            if got < 0:
                L.append("    NEGATIVE because the sweep pays the shift cost and the "
                         "oracle does not:")
                L.append("    the schedule spends more on shifting than the ratio choice "
                         "returns.")
        elif sw.best is not None:
            L.append("    the prize is zero: one ratio is better everywhere")
        L += ["    The oracle has clairvoyance, free shifting and no rate limit, so it",
              "    is not achievable - it is the bound. The better ratio is the low one",
              f"    on {o['gear1_share']:.1f} % of moving samples, and those samples are",
              "    selected by LOAD, not by speed - which is why a speed threshold",
              "    cannot collect them (see 'Optimal gear map')."]
        return L


if __name__ == "__main__":
    ShiftOptimiserApp().mainloop()
