"""
diagnose_ui.py - work out why the plot area is blank on a given machine.

Run it on the machine where the figures do not appear:

    python diagnose_ui.py

It prints library versions, display scaling, and the actual on-screen geometry of
the plot widgets after a real render. Send the whole output.

The "Axes that are not compatible with tight_layout" UserWarning is expected and
harmless - it comes from colour bars attached to several axes at once, and the
figure still draws. It is not the cause of a blank plot area.
"""
from __future__ import annotations

import sys
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

print("=" * 70)
print("VERSIONS")
print("=" * 70)
try:
    import platform
    print(f"  python        {sys.version.split()[0]}  {platform.machine()}  "
          f"{platform.system()} {platform.release()}")
except Exception as e:
    print("  platform failed:", e)

for mod in ("matplotlib", "customtkinter", "numpy", "pandas", "scipy"):
    try:
        m = __import__(mod)
        print(f"  {mod:<14}{getattr(m, '__version__', '?')}")
    except Exception as e:
        print(f"  {mod:<14}NOT IMPORTABLE - {e}")

import matplotlib
print(f"  backend       {matplotlib.get_backend()}  (the app forces TkAgg)")
from matplotlib.figure import Figure
print(f"  set_layout_engine available: {hasattr(Figure, 'set_layout_engine')}"
      f"   <- needs matplotlib >= 3.6")

import tkinter as tk
print(f"  tk            {tk.TkVersion}")

print()
print("=" * 70)
print("DISPLAY SCALING  (a mismatch here is the usual cause of a blank canvas)")
print("=" * 70)
try:
    import customtkinter as ctk
    root = ctk.CTk()
    root.geometry("900x600")
    root.update_idletasks()
    print(f"  screen                 {root.winfo_screenwidth()} x "
          f"{root.winfo_screenheight()}")
    print(f"  tk scaling             {root.tk.call('tk', 'scaling'):.3f}")
    try:
        from customtkinter.windows.widgets.scaling import ScalingTracker
        print(f"  CTk widget scaling     "
              f"{ScalingTracker.get_widget_scaling(root):.3f}")
        print(f"  CTk window scaling     "
              f"{ScalingTracker.get_window_scaling(root):.3f}")
    except Exception as e:
        print(f"  CTk scaling            could not read ({e})")
    root.destroy()
except Exception:
    traceback.print_exc()

print()
print("=" * 70)
print("LIVE WIDGET GEOMETRY AFTER A REAL RENDER")
print("=" * 70)
try:
    import shift_core as sc
    import shift_app as sa

    app = sa.ShiftOptimiserApp()
    app.update_idletasks()

    # a render that needs no data files at all
    p = app.params()
    out = sc.gradeability_table(veh=p["veh"], motor=p["motor"], gb=p["gb"])
    app._render("Gradeability", out, p)
    for _ in range(30):
        app.update()

    w = app.canvas.get_tk_widget()
    host = getattr(app, "plot_host", None)

    def geo(name, widget):
        if widget is None:
            print(f"  {name:<22} <missing>")
            return
        try:
            print(f"  {name:<22} size {widget.winfo_width():>5} x "
                  f"{widget.winfo_height():<5}  at ({widget.winfo_x()},"
                  f"{widget.winfo_y()})  mapped={bool(widget.winfo_ismapped())}")
        except Exception as e:
            print(f"  {name:<22} could not read: {e}")

    geo("app window", app)
    geo("plot host (scroll)", host)
    if host is not None:
        geo("  scroll inner canvas", getattr(host, "_parent_canvas", None))
    geo("matplotlib canvas", w)
    print(f"  requested canvas height {w.cget('height')}")
    print(f"  figure size inches      {app.fig.get_size_inches()}")
    print(f"  figure axes             {len(app.fig.axes)}")

    if host is not None and getattr(host, "_parent_canvas", None) is not None:
        try:
            print(f"  scrollregion            "
                  f"{host._parent_canvas.cget('scrollregion')}")
        except Exception:
            pass

    blank = w.winfo_height() <= 1 or not w.winfo_ismapped()
    print()
    if blank:
        print("  >>> DIAGNOSIS: the matplotlib canvas has no visible height on this")
        print("      machine. The figure is being drawn into a widget that is not")
        print("      laid out. Run with the simple layout:")
        print()
        print("          set SHIFT_APP_SIMPLE_LAYOUT=1")
        print("          python shift_app.py")
    else:
        print("  >>> The canvas has a real size here, so the plot area should be")
        print("      visible. If it still looks blank, the figure is drawing off-")
        print("      screen inside the scroll area - scroll up, or press 'Auto'")
        print("      next to the Plot height slider.")
    app.destroy()
except Exception:
    print("  RENDER TEST FAILED:")
    traceback.print_exc()

print()
print("=" * 70)
print("Send everything above.")
print("=" * 70)
