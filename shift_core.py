"""
Two-speed shift-schedule optimisation — physics core.

Pure computation, no UI, no globals. Written to drop into the Vehicle-Motor
Integration Suite as ``vmi/shift_optimizer.py`` (numpy 1.26 / scipy 1.15 compatible).

What this fixes relative to the original notebook
-------------------------------------------------
C1  Boundary detection      -- an optimum on a search-range edge is reported as
                               NOT converged instead of being quoted as an answer.
C2  Hysteresis constraint   -- ``downshift < upshift`` enforced when candidates are
                               generated, so degenerate/inverted bands never enter.
P1  Constant-power envelope -- feasibility uses min(T_peak, 9550*P_peak/n), not a
                               flat +/-T_peak.
P2  Road grade              -- a real parameter; sweeps can run per-grade.
P3  Reflected inertia       -- rotor inertia referred to the wheel (scales with G^2),
                               so the two ratios are compared on equal terms.
P4  Efficiency map          -- genuine low-efficiency cells kept; only blank cells are
                               missing; nearest-neighbour fallback uses NORMALISED axes;
                               above-envelope points are flagged infeasible, not filled.
P5  Electrical losses       -- constant auxiliary load + pack I^2R solved exactly.
P6  Differentiation noise   -- optional Savitzky-Golay smoothing before np.gradient.
P7  Shift cost              -- energy per shift and a torque-interruption window.
B3  numpy<2.0               -- uses np.trapz.

Reporting adds Wh/km, distance, time-in-gear and energy-weighted mean efficiency.
"""

from __future__ import annotations

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd
from scipy.interpolate import RegularGridInterpolator, NearestNDInterpolator

__all__ = [
    "Vehicle", "Motor", "Gearbox", "Electrical", "ShiftCost", "Numerics",
    "EfficiencyMap", "CycleData", "ShiftResult", "SweepResult",
    "load_cycle", "load_efficiency_map", "simulate", "sweep_upshift",
    "sweep_downshift", "sweep_grid", "sweep_efficiency", "gear_breakdown",
    "wot_run", "wot_sweep", "tractive_force", "road_load_force",
    "oracle_bound", "shift_decomposition", "counterfactual_point",
    "accel_capability", "energy_breakdown", "better_gear_per_sample", "efficiency_ridge",
]

# numpy 1.x / 2.x compatible trapezoid rule (VMI pins numpy==1.26.4)
_trapz = getattr(np, "trapezoid", None) or np.trapz


# ---------------------------------------------------------------------------
# Parameter blocks
# ---------------------------------------------------------------------------
@dataclass
class Vehicle:
    mass: float = 995.0             # kg
    wheel_radius: float = 0.247     # m
    cda: float = 1.104              # m^2  (Cd x A as a single lumped value)
    crr: float = 0.02
    air_density: float = 1.225      # kg/m^3
    gravity: float = 9.81
    grade_deg: float = 0.0          # P2


@dataclass
class Motor:
    peak_torque: float = 45.0       # Nm
    peak_power: float = 11_000.0    # W
    max_rpm: float = 10_000.0
    inertia: float = 0.005          # kg.m^2 rotor inertia (P3); 0 disables

    def envelope(self, rpm: np.ndarray) -> np.ndarray:
        """Continuous torque ceiling: flat to base speed, then constant power (P1)."""
        rpm = np.abs(np.asarray(rpm, dtype=float))
        with np.errstate(divide="ignore", invalid="ignore"):
            cp = 9550.0 * (self.peak_power / 1000.0) / np.maximum(rpm, 1e-9)
        return np.minimum(self.peak_torque, np.where(rpm > 0, cp, self.peak_torque))

    @property
    def base_rpm(self) -> float:
        return 9550.0 * (self.peak_power / 1000.0) / self.peak_torque


@dataclass
class Gearbox:
    ratio_1: float = 19.0
    ratio_2: float = 11.0
    eta_1: float = 0.97
    eta_2: float = 0.97

    def ratio(self, gear: int) -> float:
        return self.ratio_1 if gear == 1 else self.ratio_2

    def eta(self, gear: int) -> float:
        return self.eta_1 if gear == 1 else self.eta_2


@dataclass
class Electrical:
    voltage: float = 52.0           # V  (P5 - was declared but unused)
    pack_resistance: float = 0.020  # ohm; 0 disables I^2R
    aux_load: float = 150.0         # W constant draw; 0 disables
    max_power: float = 15_000.0     # W magnitude limit
    regen_enabled: bool = False     # notebook is the "no regen" configuration

    # Realistic recovery, not "the motor takes all of it":
    regen_fraction: float = 0.70    # share of brake torque the strategy sends to the
                                    # motor; the friction brakes take the rest
    regen_max_power: float = 0.0    # W the pack will accept on charge; 0 = max_power
    regen_min_speed_kmh: float = 5.0  # blend regen out below this road speed

    @property
    def charge_limit(self) -> float:
        return self.regen_max_power if self.regen_max_power > 0 else self.max_power


@dataclass
class ShiftCost:
    """P7 - shifting is not free, and not every schedule is a schedule.

    Holds the costs of a gear change and the constraints a usable shift schedule
    must satisfy. The constraints are here rather than in the sweeps because every
    sweep goes through simulate(), so putting them in one place means no search can
    quietly return a candidate the others would reject.
    """
    energy_per_shift: float = 0.0   # J drawn per gear change (sync + actuation)
    interrupt_s: float = 0.0        # s of torque interruption per shift
    max_shifts_per_hour: float = np.inf
    min_band_kmh: float = 2.0       # hysteresis width; a 1 km/h band is chatter, not
                                    # a schedule, and the shifts/h cap does not catch it
    min_accel_reserve: float = 0.0  # m/s^2 the HIGH ratio must still deliver at the
                                    # upshift speed; 0 disables the check
    max_accel_loss: float = 1.0     # fraction of the low ratio's acceleration that may
                                    # be given up by upshifting; 1.0 disables the check


@dataclass
class Numerics:
    power_epsilon: float = 1.0      # W
    min_efficiency: float = 0.01
    smooth_window: int = 0          # P6 - Savitzky-Golay window (odd, 0 = off)
    smooth_order: int = 2


# ---------------------------------------------------------------------------
# Efficiency map (P4)
# ---------------------------------------------------------------------------
class EfficiencyMap:
    """Motor efficiency lookup with honest missing-data handling.

    Differences from the notebook:
      * only blank/non-positive cells are treated as missing - genuine low
        efficiencies near stall are real measurements and are kept;
      * the nearest-neighbour fallback is built on NORMALISED (torque, rpm)
        coordinates, so 1 Nm is not treated as equidistant to 1 rpm;
      * queries outside the grid are reported, never silently clamped-and-filled.
    """

    def __init__(self, rpm: np.ndarray, torque: np.ndarray, eff: np.ndarray,
                 min_efficiency: float = 0.01):
        self.rpm = np.asarray(rpm, dtype=float)
        self.torque = np.asarray(torque, dtype=float)
        self.eff = np.array(eff, dtype=float)
        self.min_efficiency = float(min_efficiency)

        # Missing == blank or non-positive. Efficiency > 1 is corrupt.
        self.missing = ~np.isfinite(self.eff) | (self.eff <= 0.0) | (self.eff > 1.0)
        work = np.where(self.missing, np.nan, self.eff)

        self._lin = RegularGridInterpolator(
            (self.torque, self.rpm), work, method="linear",
            bounds_error=False, fill_value=np.nan)

        # Lowest torque row that carries data. Every query below it has one grid
        # corner on the all-blank T = 0 row, so RegularGridInterpolator returns NaN
        # and the point used to be filled by nearest-neighbour - which handed a
        # near-zero-torque point the efficiency of a properly loaded one (~81 %).
        rows_ok = np.flatnonzero((~self.missing).any(axis=1) & (self.torque > 0))
        self._i_lo = int(rows_ok[0]) if rows_ok.size else 0
        self._t_lo = float(self.torque[self._i_lo]) if rows_ok.size else 0.0
        self._row_lo = work[self._i_lo] if rows_ok.size else None

        rm, tm = np.meshgrid(self.rpm, self.torque)
        ok = ~self.missing
        self._t_scale = float(np.ptp(self.torque)) or 1.0
        self._n_scale = float(np.ptp(self.rpm)) or 1.0
        self._near = NearestNDInterpolator(
            np.column_stack([tm[ok] / self._t_scale, rm[ok] / self._n_scale]),
            work[ok])

    def _below_grid_eff(self, torque, rpm):
        """Efficiency below the map's lowest torque row, by constant-loss extension.

        At a given speed the rpm-dependent losses (iron, windage, bearing) are what
        survive as torque -> 0, so hold the loss at its value on the lowest measured
        torque row and let the output power fall:

            loss(n)  = T_lo * w * (1/eff(T_lo, n) - 1)
            eff(T,n) = P_out / (P_out + loss(n)),   P_out = T * w

        This goes to 0 as torque goes to 0, which is the physical answer, instead of
        the nearest-neighbour value the old code substituted.
        """
        if self._row_lo is None:
            return np.full(np.shape(torque), np.nan)
        good = np.isfinite(self._row_lo)
        if not good.any():
            return np.full(np.shape(torque), np.nan)
        w = np.asarray(rpm, float) * 2.0 * np.pi / 60.0
        e_lo = np.interp(np.asarray(rpm, float), self.rpm[good], self._row_lo[good])
        with np.errstate(divide="ignore", invalid="ignore"):
            loss = self._t_lo * w * (1.0 / e_lo - 1.0)
            p_out = np.asarray(torque, float) * w
            e = p_out / (p_out + loss)
        return np.where(np.isfinite(e) & (e > 0), e, np.nan)

    @property
    def peak(self):
        k = np.unravel_index(np.nanargmax(np.where(self.missing, np.nan, self.eff)),
                             self.eff.shape)
        return float(self.eff[k]), float(self.rpm[k[1]]), float(self.torque[k[0]])

    def query(self, rpm, torque, active):
        """Return (efficiency, used_fallback, outside_grid). |torque| is used."""
        n = np.abs(np.asarray(rpm, dtype=float))
        t = np.abs(np.asarray(torque, dtype=float))

        outside = active & ((n < self.rpm.min()) | (n > self.rpm.max())
                            | (t < self.torque.min()) | (t > self.torque.max()))
        nq = np.clip(n, self.rpm.min(), self.rpm.max())
        tq = np.clip(t, self.torque.min(), self.torque.max())

        e = self._lin(np.column_stack([tq, nq]))
        fallback = active & ~np.isfinite(e)
        low = fallback & (tq < self._t_lo)          # below the lowest measured torque
        if low.any():
            e[low] = self._below_grid_eff(tq[low], nq[low])
        rest = active & ~np.isfinite(e)
        if rest.any():
            e[rest] = self._near(tq[rest] / self._t_scale,
                                 nq[rest] / self._n_scale)
        e[~active] = 1.0
        # Only non-physical values are rejected here. A genuinely low efficiency at
        # near-zero torque is a real number (the motor still pays its no-load loss);
        # screening it out with min_efficiency used to declare the whole strategy
        # infeasible over a coasting sample carrying a fraction of a watt.
        bad = active & (~np.isfinite(e) | (e <= 0.0) | (e > 1.0))
        e[bad] = np.nan
        return e, fallback | outside, outside


@dataclass
class CycleData:
    time: np.ndarray
    speed_kmh: np.ndarray
    name: str = ""

    @property
    def duration(self) -> float:
        return float(self.time[-1] - self.time[0])

    @property
    def distance_km(self) -> float:
        return float(_trapz(self.speed_kmh / 3.6, self.time) / 1000.0)


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------
_TIME_ALIASES = ["Time", "Tiime[s]", "Time[s]", "Time [s]", "t"]
_SPEED_ALIASES = ["Speed", "Speed[kmph]", "Speed [km/h]", "Speed[km/h]", "v"]


def load_cycle(path) -> CycleData:
    path = Path(path)
    df = pd.read_csv(path)
    df.columns = df.columns.astype(str).str.strip()

    def pick(aliases, label):
        for a in aliases:
            if a in df.columns:
                return a
        raise ValueError(f"No {label} column. Found: {list(df.columns)}")

    tcol, scol = pick(_TIME_ALIASES, "time"), pick(_SPEED_ALIASES, "speed")
    out = df[[tcol, scol]].copy()
    out.columns = ["Time", "Speed"]
    out["Time"] = pd.to_numeric(out["Time"], errors="coerce")
    out["Speed"] = pd.to_numeric(out["Speed"], errors="coerce")
    out = out.dropna().sort_values("Time").groupby("Time", as_index=False)["Speed"].mean()

    if len(out) < 3:
        raise ValueError("Cycle needs at least three valid samples.")
    if (out["Speed"] < 0).any():
        raise ValueError("Cycle contains negative speed.")
    return CycleData(out["Time"].to_numpy(float), out["Speed"].to_numpy(float), path.name)


def load_efficiency_map(path, min_efficiency: float = 0.01) -> EfficiencyMap:
    raw = pd.read_excel(Path(path), header=None)
    rpm = pd.to_numeric(raw.iloc[0, 1:], errors="coerce").to_numpy(float)
    trq = pd.to_numeric(raw.iloc[1:, 0], errors="coerce").to_numpy(float)
    eff = raw.iloc[1:, 1:].apply(pd.to_numeric, errors="coerce").to_numpy(float)

    vr, vt = np.isfinite(rpm), np.isfinite(trq)
    rpm, trq, eff = rpm[vr], trq[vt], eff[np.ix_(vt, vr)]
    if eff.shape != (len(trq), len(rpm)):
        raise ValueError("Efficiency-map dimensions do not match its axes.")

    fin = eff[np.isfinite(eff)]
    if fin.size == 0:
        raise ValueError("Efficiency map has no numeric values.")
    if np.nanmedian(fin) > 1.5:           # percent -> fraction
        eff = eff / 100.0

    io, jo = np.argsort(trq), np.argsort(rpm)
    trq, rpm, eff = trq[io], rpm[jo], eff[np.ix_(io, jo)]
    if np.any(np.diff(trq) <= 0) or np.any(np.diff(rpm) <= 0):
        raise ValueError("Map axes must be unique and strictly increasing.")
    return EfficiencyMap(rpm, trq, eff, min_efficiency)


# ---------------------------------------------------------------------------
# Kinematics / road load
# ---------------------------------------------------------------------------
def _smooth(speed_ms, num: Numerics):
    """P6 - optional low-pass before differentiation."""
    w = int(num.smooth_window)
    if w < 3 or w > len(speed_ms):
        return speed_ms
    if w % 2 == 0:
        w += 1
    from scipy.signal import savgol_filter
    return np.maximum(savgol_filter(speed_ms, w, num.smooth_order), 0.0)


def road_load(cycle: CycleData, veh: Vehicle, motor: Motor, gb: Gearbox,
              num: Numerics, gear_ratio_for_inertia: Optional[np.ndarray] = None):
    """Wheel-side torque / speed / power.

    ``gear_ratio_for_inertia`` supplies the engaged ratio per sample so the
    reflected rotor inertia (P3) is charged correctly; None disables it.
    """
    v = _smooth(cycle.speed_kmh / 3.6, num)
    a = np.gradient(v, cycle.time, edge_order=2)
    theta = np.deg2rad(veh.grade_deg)
    moving = v > 0

    # P3: rotor inertia referred to the wheel adds effective mass m_eq = J*G^2/r^2
    m_eff = np.full_like(v, veh.mass)
    if gear_ratio_for_inertia is not None and motor.inertia > 0:
        g = np.asarray(gear_ratio_for_inertia, dtype=float)
        m_eff = veh.mass + motor.inertia * g ** 2 / veh.wheel_radius ** 2

    f = (m_eff * a
         + veh.mass * veh.gravity * veh.crr * np.cos(theta) * moving
         + 0.5 * veh.air_density * veh.cda * v ** 2
         + veh.mass * veh.gravity * np.sin(theta) * moving)

    t_wheel = f * veh.wheel_radius
    w_wheel = v / veh.wheel_radius
    return t_wheel, w_wheel * 60.0 / (2.0 * np.pi), t_wheel * w_wheel


def motor_point(t_wheel, rpm_wheel, p_wheel, ratio, eta):
    n = rpm_wheel * ratio
    t = np.empty_like(t_wheel)
    mot = p_wheel >= 0
    t[mot] = t_wheel[mot] / (ratio * eta)
    t[~mot] = t_wheel[~mot] * eta / ratio
    return n, t, t * n * (2.0 * np.pi / 60.0)


def accel_capability(veh: Vehicle, motor: Motor, gb: Gearbox, gear: int,
                     v_kmh: float, throttle: float = 1.0) -> float:
    """Acceleration still available in ``gear`` at ``v_kmh``, m/s^2, full throttle.

    a = (T_env(n)*G*eta/r - F_road(v)) / (m + J*G^2/r^2)

    This is what an energy objective cannot see. Upshifting hands the vehicle to the
    high ratio, and at low speed that ratio has far less to give: the energy optimum
    will happily shift at a speed where almost no acceleration is left, because the
    cycle it is scored on never happens to ask for any.
    """
    v = max(float(v_kmh), 0.0) / 3.6
    ratio, eta = gb.ratio(gear), gb.eta(gear)
    n = (v / veh.wheel_radius) * 60.0 / (2.0 * np.pi) * ratio
    if n > motor.max_rpm:
        return float("-inf")
    t = float(motor.envelope(np.array([n]))[0]) * float(np.clip(throttle, 0.0, 1.0))
    f = t * ratio * eta / veh.wheel_radius
    m_eff = veh.mass + motor.inertia * ratio ** 2 / veh.wheel_radius ** 2
    return float((f - road_load_force(veh, v)[0]) / m_eff)


def gear_sequence(speed_kmh: np.ndarray, upshift: float, downshift: float) -> np.ndarray:
    """Hysteresis state machine, evaluated only at threshold crossings.

    Identical semantics to the notebook's per-sample loop but O(shifts log n).
    Requires downshift < upshift; callers must not pass a degenerate band (C2).
    """
    n = len(speed_kmh)
    gear = np.ones(n, dtype=np.int8)
    up_pos = np.flatnonzero(speed_kmh >= upshift)
    dn_pos = np.flatnonzero(speed_kmh <= downshift)
    i, cur = 0, 1
    while i < n:
        if cur == 1:
            k = np.searchsorted(up_pos, i)
            if k >= len(up_pos):
                break
            j = up_pos[k]
            cur, i = 2, j
        else:
            k = np.searchsorted(dn_pos, i + 1)
            if k >= len(dn_pos):
                gear[i:] = 2
                break
            j = dn_pos[k]
            gear[i:j] = 2
            cur, i = 1, j
    return gear


# ---------------------------------------------------------------------------
# Battery model (P5)
# ---------------------------------------------------------------------------
def _terminal_power(p_demand: np.ndarray, elec: Electrical) -> np.ndarray:
    """Solve P_term - (P_term/V)^2 R = P_demand exactly.

    Root: P = V^2/(2R) * (1 - sqrt(1 - 4 R P_dem / V^2)). Falls back to the
    lossless value when R = 0 or the demand exceeds what the pack can deliver.
    """
    if elec.pack_resistance <= 0 or elec.voltage <= 0:
        return p_demand
    v2, r = elec.voltage ** 2, elec.pack_resistance
    disc = 1.0 - 4.0 * r * p_demand / v2
    out = np.where(disc >= 0.0,
                   v2 / (2.0 * r) * (1.0 - np.sqrt(np.clip(disc, 0.0, None))),
                   np.nan)
    # Discharging only; on regen the same resistance reduces what is recovered.
    neg = p_demand < 0
    if neg.any():
        i = p_demand[neg] / elec.voltage
        out[neg] = p_demand[neg] + i ** 2 * r
    return out


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------
@dataclass
class ShiftResult:
    upshift: float
    downshift: float
    feasible: bool
    consumed_kwh: float = np.nan
    recovered_kwh: float = np.nan
    net_kwh: float = np.nan
    wh_per_km: float = np.nan
    distance_km: float = np.nan
    shift_energy_kwh: float = 0.0
    interrupt_energy_kwh: float = 0.0
    shifts_under_traction: int = 0
    upshifts: int = 0
    downshifts: int = 0
    shifts_per_hour: float = 0.0
    time_gear1_pct: float = 0.0
    time_gear2_pct: float = 0.0
    mean_efficiency: float = np.nan        # energy-weighted
    envelope_violations: int = 0
    rpm_violations: int = 0
    battery_violations: int = 0
    fallback_points: int = 0
    outside_map_points: int = 0
    regen_samples: int = 0
    regen_torque_mean: float = 0.0
    reserve_at_upshift: float = np.nan     # m/s^2 available in gear 2 at the upshift
    reserve_before_upshift: float = np.nan  # ... and in gear 1 at the same speed
    accel_loss: float = np.nan             # fraction of acceleration given up by upshifting
    reasons: list = field(default_factory=list)
    gear: Optional[np.ndarray] = None
    battery_power: Optional[np.ndarray] = None
    motor_rpm: Optional[np.ndarray] = None
    motor_torque: Optional[np.ndarray] = None
    motor_eff: Optional[np.ndarray] = None

    def row(self) -> dict:
        d = asdict(self)
        for k in ("gear", "battery_power", "motor_rpm", "motor_torque", "motor_eff"):
            d.pop(k, None)
        d["reasons"] = "; ".join(self.reasons)
        return d


@dataclass
class SweepResult:
    table: pd.DataFrame
    best: Optional[ShiftResult]
    converged: bool                  # C1 - False when the optimum touches an edge
    boundary_note: str = ""
    details: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
def simulate(cycle: CycleData, emap: EfficiencyMap, upshift: float, downshift: float,
             veh: Vehicle = None, motor: Motor = None, gb: Gearbox = None,
             elec: Electrical = None, cost: ShiftCost = None, num: Numerics = None,
             keep_arrays: bool = False) -> ShiftResult:
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    elec = elec or Electrical(); cost = cost or ShiftCost(); num = num or Numerics()

    res = ShiftResult(float(upshift), float(downshift), feasible=True)

    # C2: refuse a degenerate or inverted band outright.
    if not (downshift < upshift):
        res.feasible = False
        res.reasons.append(f"invalid band: downshift {downshift:g} >= upshift {upshift:g}")
        return res

    # A band narrower than min_band is chatter, not a schedule. This used to be
    # checked only when generating the 2-D grid, so the 1-D sweeps were free to
    # return the tightest band allowed - and did, because energy always prefers it.
    if (upshift - downshift) < cost.min_band_kmh - 1e-9:
        res.feasible = False
        res.reasons.append(f"hysteresis band {upshift-downshift:g} km/h is below the "
                           f"{cost.min_band_kmh:g} km/h minimum")
        return res

    # Driveability: what is left to accelerate with once the high ratio takes over.
    res.reserve_at_upshift = accel_capability(veh, motor, gb, 2, upshift)
    res.reserve_before_upshift = accel_capability(veh, motor, gb, 1, upshift)
    if cost.min_accel_reserve > 0 and res.reserve_at_upshift < cost.min_accel_reserve:
        res.feasible = False
        res.reasons.append(
            f"only {res.reserve_at_upshift:.2f} m/s2 available in gear 2 at the "
            f"{upshift:g} km/h upshift, below the {cost.min_accel_reserve:.2f} minimum")
        return res

    # The binding constraint at a LOW upshift speed is not the absolute reserve - the
    # high ratio is on flat torque there and has plenty. It is how much tractive
    # capability the upshift throws away. Below the tractive-force crossover the low
    # ratio pulls harder, and handing over early gives that difference up; above it
    # the two ratios pull the same and the handover is free.
    if res.reserve_before_upshift > 0:
        res.accel_loss = max(0.0, 1.0 - res.reserve_at_upshift / res.reserve_before_upshift)
        if cost.max_accel_loss < 1.0 and res.accel_loss > cost.max_accel_loss + 1e-9:
            res.feasible = False
            res.reasons.append(
                f"upshifting at {upshift:g} km/h gives up {100*res.accel_loss:.0f} % of "
                f"the available acceleration ({res.reserve_before_upshift:.2f} -> "
                f"{res.reserve_at_upshift:.2f} m/s2), over the "
                f"{100*cost.max_accel_loss:.0f} % limit")
            return res

    # Gear choice needs road load; road load needs the ratio (inertia). Two passes.
    t_w, n_w, p_w = road_load(cycle, veh, motor, gb, num)
    gear = gear_sequence(cycle.speed_kmh, upshift, downshift)
    ratios = np.where(gear == 1, gb.ratio_1, gb.ratio_2)
    if motor.inertia > 0:
        t_w, n_w, p_w = road_load(cycle, veh, motor, gb, num, ratios)
        gear = gear_sequence(cycle.speed_kmh, upshift, downshift)
        ratios = np.where(gear == 1, gb.ratio_1, gb.ratio_2)

    etas = np.where(gear == 1, gb.eta_1, gb.eta_2)
    n_m = n_w * ratios
    t_m = np.where(p_w >= 0, t_w / (ratios * etas), t_w * etas / ratios)

    active = np.abs(p_w) > num.power_epsilon
    eff, fallback, outside = emap.query(n_m, t_m, active)

    # --- feasibility (P1) --------------------------------------------------
    env = motor.envelope(n_m)
    # The envelope is a limit on what the motor can DELIVER. On the braking side
    # exceeding it is not infeasible - the friction brakes simply take the excess,
    # which is what a real blended-braking controller does. Flagging it as a
    # violation used to reject whole strategies over a hard stop.
    env_bad = active & (p_w > 0) & (np.abs(t_m) > env)
    rpm_bad = active & (np.abs(n_m) > motor.max_rpm)
    eff_ok = np.isfinite(eff) & (eff > 0.0) & (eff <= 1.0)
    low_eff = active & eff_ok & (eff < num.min_efficiency)   # reported, not fatal

    res.envelope_violations = int(env_bad.sum())
    res.rpm_violations = int(rpm_bad.sum())
    res.fallback_points = int((active & fallback).sum())
    res.outside_map_points = int((active & outside).sum())

    point_ok = ~active | (~env_bad & ~rpm_bad & eff_ok)

    # --- battery power -----------------------------------------------------
    p_shaft = np.zeros_like(p_w)
    mot = active & (p_w > 0) & point_ok
    reg = active & (p_w < 0) & point_ok
    p_shaft[mot] = p_w[mot] / (eff[mot] * etas[mot])

    if elec.regen_enabled and reg.any():
        # What the motor ACTUALLY absorbs, in this order:
        #   blend fraction  -> the rest goes to the friction brakes
        #   motor envelope  -> it cannot absorb more than it could deliver
        #   low-speed cut   -> regen is blended out approaching a stop
        # The map is then queried at THAT torque, not at the full brake torque:
        # a motor recovering 4 Nm is not at the operating point of one holding 12.
        t_reg = np.minimum(np.abs(t_m) * elec.regen_fraction, env)
        t_reg = np.where(cycle.speed_kmh >= elec.regen_min_speed_kmh, t_reg, 0.0)
        t_reg = np.where(reg, t_reg, 0.0)

        take = reg & (t_reg > 0)
        eff_r, fb_r, out_r = emap.query(n_m, t_reg, take)
        eff_r = np.where(np.isfinite(eff_r) & (eff_r > 0) & (eff_r <= 1.0), eff_r, 0.0)
        w_m = np.abs(n_m) * 2.0 * np.pi / 60.0
        p_recovered = np.where(take, t_reg * w_m * eff_r, 0.0)     # W into the pack
        p_recovered = np.minimum(p_recovered, elec.charge_limit)
        p_shaft[reg] = -p_recovered[reg]
        res.fallback_points += int((take & fb_r).sum())
        res.regen_samples = int(take.sum())
        res.regen_torque_mean = float(np.mean(t_reg[take])) if take.any() else 0.0
    # with regen off the friction brakes absorb everything and the motor
    # contributes nothing - which is why braking-side efficiency is then irrelevant

    p_dem = p_shaft + elec.aux_load          # P5 aux load
    p_batt = _terminal_power(p_dem, elec)    # P5 pack I^2R
    p_batt[active & ~point_ok] = np.nan

    batt_bad = np.isfinite(p_batt) & (np.abs(p_batt) > elec.max_power)
    res.battery_violations = int(batt_bad.sum())

    if res.envelope_violations:
        res.reasons.append(f"{res.envelope_violations} pts above constant-power envelope")
    if res.rpm_violations:
        res.reasons.append(f"{res.rpm_violations} pts above max rpm")
    if res.battery_violations:
        res.reasons.append(f"{res.battery_violations} pts above battery power limit")
    if int((active & ~eff_ok).sum()):
        res.reasons.append(f"{int((active & ~eff_ok).sum())} pts with no usable efficiency")
    if int(low_eff.sum()):
        res.reasons.append(f"{int(low_eff.sum())} pts below {num.min_efficiency:.0%} "
                           f"efficiency (near-zero torque; counted, not rejected)")

    # --- shift accounting (P7) --------------------------------------------
    sh = np.flatnonzero(np.diff(gear) != 0)
    res.upshifts = int((gear[sh + 1] == 2).sum())
    res.downshifts = int((gear[sh + 1] == 1).sum())
    n_shifts = res.upshifts + res.downshifts
    hours = cycle.duration / 3600.0
    res.shifts_per_hour = n_shifts / hours if hours > 0 else 0.0
    if res.shifts_per_hour > cost.max_shifts_per_hour:
        res.reasons.append(
            f"{res.shifts_per_hour:.0f} shifts/h exceeds limit {cost.max_shifts_per_hour:g}")

    res.feasible = bool(np.all(point_ok) and not batt_bad.any()
                        and res.shifts_per_hour <= cost.max_shifts_per_hour)

    if np.any(~np.isfinite(p_batt)):
        res.feasible = False
        return res

    # --- energy ------------------------------------------------------------
    res.shift_energy_kwh = n_shifts * cost.energy_per_shift / 3.6e6
    interrupt_kwh = 0.0
    if cost.interrupt_s > 0 and n_shifts:
        # A torque cut only costs energy where the motor is actually pulling.
        # ~93 % of downshifts happen under braking, where there is no traction to
        # interrupt; charging those the cycle-mean positive power (the previous
        # model) invented a penalty that scaled with shift count and so masked the
        # low-speed downshift benefit this study exists to measure.
        p_tract = np.maximum(np.nan_to_num(p_batt[sh]) - elec.aux_load, 0.0)
        interrupt_kwh = float(cost.interrupt_s * np.sum(p_tract) / 3.6e6)
        res.interrupt_energy_kwh = interrupt_kwh
        res.shifts_under_traction = int(np.count_nonzero(p_tract > 1.0))

    res.consumed_kwh = float(_trapz(np.maximum(p_batt, 0.0), cycle.time) / 3.6e6) \
        + res.shift_energy_kwh + interrupt_kwh
    res.recovered_kwh = max(0.0, float(-_trapz(np.minimum(p_batt, 0.0), cycle.time) / 3.6e6))
    res.net_kwh = res.consumed_kwh - res.recovered_kwh

    res.distance_km = cycle.distance_km
    # net, not consumed: with regen on, the energy actually taken from the pack over
    # the cycle is what a range figure is built from (identical when regen is off)
    res.wh_per_km = (res.net_kwh * 1000.0 / res.distance_km
                     if res.distance_km > 0 else np.nan)

    res.time_gear1_pct = float(100.0 * np.mean(gear == 1))
    res.time_gear2_pct = 100.0 - res.time_gear1_pct

    # Energy-true mean motor efficiency: total shaft output / total electrical
    # input over the motoring part of the cycle. The previous input-power-weighted
    # arithmetic mean of the map value is not the efficiency of anything, and read
    # ~2.8 points high against this definition.
    use = active & (p_w > 0) & np.isfinite(eff) & point_ok
    e_out = _trapz(np.where(use, p_w / etas, 0.0), cycle.time)
    e_in = _trapz(np.where(use, p_shaft, 0.0), cycle.time)
    res.mean_efficiency = float(e_out / e_in) if e_in > 0 else np.nan

    if keep_arrays:
        res.gear, res.battery_power = gear, p_batt
        res.motor_rpm, res.motor_torque, res.motor_eff = n_m, t_m, eff
    return res


# ---------------------------------------------------------------------------
# Gear-choice analysis - which ratio is better, and where
# ---------------------------------------------------------------------------
def _steady_point(speed_kmh, accel, gear, veh, gb):
    """Wheel-side demand -> motor (rpm, torque) for one gear."""
    v = np.asarray(speed_kmh, dtype=float) / 3.6
    f = (veh.mass * accel
         + veh.mass * veh.gravity * veh.crr * (v > 0)
         + 0.5 * veh.air_density * veh.cda * v ** 2)
    t_w = f * veh.wheel_radius
    rpm_w = v / veh.wheel_radius * 60.0 / (2.0 * np.pi)
    ratio, eta = gb.ratio(gear), gb.eta(gear)
    return rpm_w * ratio, t_w / (ratio * eta)


def gear_efficiency_curves(emap: EfficiencyMap, speeds=None, accels=(0.0, 0.3, 0.6, 1.0),
                           veh: Vehicle = None, motor: Motor = None, gb: Gearbox = None):
    """Motor efficiency in each ratio across road speed, at several load levels.

    This is the chart that shows the gear preference *flipping with load*: at cruise
    the lower-rpm ratio wins (spin losses dominate at 5-10 % torque), while under
    acceleration the higher-rpm ratio wins (it keeps torque, and therefore copper
    loss, down). A speed-only shift line cannot express that.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    speeds = np.asarray(speeds if speeds is not None else np.arange(3, 43, 1.0), dtype=float)
    out = {}
    for a in accels:
        rec = {}
        for gear in (1, 2):
            n, t = _steady_point(speeds, a, gear, veh, gb)
            act = np.ones_like(n, dtype=bool)
            e, _, _ = emap.query(n, t, act)
            over = (np.abs(t) > motor.envelope(n)) | (np.abs(n) > motor.max_rpm)
            rec[gear] = dict(rpm=n, torque=t, eff=np.where(over, np.nan, e), over=over)
        out[a] = rec
    return speeds, out


def optimal_gear_map(emap: EfficiencyMap, speed_range=(2.0, 42.0), accel_range=(-0.2, 1.4),
                     n_speed=110, n_accel=90, veh: Vehicle = None,
                     motor: Motor = None, gb: Gearbox = None):
    """Which ratio is more efficient at every (road speed, acceleration) point.

    Returns (speeds, accels, better, delta, feasible1, feasible2) where ``better`` is
    1 or 2 (NaN where neither ratio is feasible) and ``delta`` is the efficiency
    advantage of the winner in percentage points.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    sp = np.linspace(*speed_range, n_speed)
    ac = np.linspace(*accel_range, n_accel)
    S, A = np.meshgrid(sp, ac)
    eff, feas = {}, {}
    for gear in (1, 2):
        n, t = _steady_point(S.ravel(), A.ravel(), gear, veh, gb)
        e, _, _ = emap.query(n, t, np.ones_like(n, dtype=bool))
        ok = (np.abs(t) <= motor.envelope(n)) & (np.abs(n) <= motor.max_rpm)
        eff[gear] = np.where(ok, e, np.nan).reshape(S.shape)
        feas[gear] = ok.reshape(S.shape)
    e1, e2 = eff[1], eff[2]
    better = np.where(np.isnan(e1) & np.isnan(e2), np.nan,
                      np.where(np.isnan(e2), 1.0,
                               np.where(np.isnan(e1), 2.0,
                                        np.where(e1 >= e2, 1.0, 2.0))))
    with np.errstate(invalid="ignore"):
        stack = np.dstack([e1, e2])
        allnan = np.all(np.isnan(stack), axis=2)
        safe = np.where(allnan[:, :, None], 0.0, stack)
        delta = np.where(allnan, np.nan,
                         (np.nanmax(safe, axis=2) - np.nanmin(safe, axis=2)) * 100.0)
    return sp, ac, better, delta, feas[1], feas[2]


# ---------------------------------------------------------------------------
# Gradeability (P2) - the question a whole-cycle grade sweep answers badly
# ---------------------------------------------------------------------------
def max_grade(speed_kmh: float, gear: int, veh: Vehicle = None, motor: Motor = None,
              gb: Gearbox = None) -> float:
    """Steepest grade (deg) sustainable at a steady ``speed_kmh`` in ``gear``.

    Steady state, so no inertia term. Returns 0.0 if the gear cannot even hold
    the speed on the flat, and NaN if the point is over the speed limit.

    This is the check that shows what gear 1 is *for*. Running a whole 110 km
    drive cycle at a fixed grade (see ``Vehicle.grade_deg``) is a much harsher
    and less meaningful test, because it also demands every acceleration uphill.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    v = speed_kmh / 3.6
    if v <= 0:
        return float("nan")
    ratio, eta = gb.ratio(gear), gb.eta(gear)
    n_motor = (v / veh.wheel_radius) * 60.0 / (2.0 * np.pi) * ratio
    if n_motor > motor.max_rpm:
        return float("nan")

    t_avail = float(motor.envelope(np.array([n_motor]))[0])
    f_avail = t_avail * ratio * eta / veh.wheel_radius
    f_resist = veh.mass * veh.gravity * veh.crr + 0.5 * veh.air_density * veh.cda * v ** 2
    spare = f_avail - f_resist
    if spare <= 0:
        return 0.0
    s = spare / (veh.mass * veh.gravity)
    return float(np.degrees(np.arcsin(min(s, 1.0))))


def gradeability_table(speeds=(5, 10, 15, 20, 25, 30, 35, 40), **kw) -> pd.DataFrame:
    rows = []
    for s in speeds:
        rows.append({"Speed [km/h]": s,
                     "Gear 1 max grade [deg]": max_grade(s, 1, **kw),
                     "Gear 2 max grade [deg]": max_grade(s, 2, **kw)})
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# Sweeps (C1 + C2)
# ---------------------------------------------------------------------------
# A threshold change that never alters the gear sequence produces a bit-identical
# energy. Several such candidates tie exactly; min() then returns whichever came
# first, which is the bottom of the search range, which then trips the boundary
# test. That is a tie-breaking artifact, not a boundary optimum - see _finish.
_TIE_REL = 1e-9


def _objective(r) -> float:
    """What every sweep minimises: NET battery energy.

    Identical to consumed when regen is off (recovered is then zero), and the only
    correct choice when it is on - ranking by consumed alone would prefer whichever
    schedule recovers the least.
    """
    return r.net_kwh if np.isfinite(r.net_kwh) else np.inf


def _tie_plateau(ok, edge_label):
    """Candidates whose energy ties the minimum, and the representative to report.

    Returns (best, tied_values). ``best`` is the MIDDLE member of the tied set:
    reporting an edge member of a flat plateau is what made the old code claim a
    boundary optimum for a threshold that has no effect at all.
    """
    e = np.array([_objective(r) for r in ok], dtype=float)
    tol = abs(e.min()) * _TIE_REL
    tied = [r for r, x in zip(ok, e) if x <= e.min() + tol]
    tied.sort(key=lambda r: getattr(r, edge_label))
    return tied[len(tied) // 2], np.array([getattr(r, edge_label) for r in tied], float)


def _finish(rows, details, values, edge_label):
    table = pd.DataFrame(rows)
    ok = [d for d in details if d.feasible and np.isfinite(_objective(d))]
    if not ok:
        return SweepResult(table, None, False, "no feasible candidate", details)
    best, tied = _tie_plateau(ok, edge_label)
    v = np.asarray(values, dtype=float)
    step = float(np.min(np.diff(np.unique(v)))) if len(np.unique(v)) > 1 else 0.0
    probe = getattr(best, edge_label)

    notes = []
    if len(tied) > 1:
        notes.append(f"{len(tied)} candidates tie exactly over "
                     f"[{tied.min():g}, {tied.max():g}] km/h - the threshold has no "
                     f"effect there (it never changes the gear sequence); reporting "
                     f"the midpoint {probe:g}")
    # only a SINGLE-point optimum sitting on an edge means the search was too narrow
    at_edge = (step > 0 and len(tied) == 1
               and (abs(probe - v.min()) <= step / 2 or abs(probe - v.max()) <= step / 2))
    if at_edge:
        which = "lower" if abs(probe - v.min()) <= step / 2 else "upper"
        notes.append(f"optimum {probe:g} km/h sits on the {which} edge of the searched "
                     f"range [{v.min():g}, {v.max():g}] - widen the range; this is not "
                     f"an interior optimum")
    return SweepResult(table, best, not at_edge, "; ".join(notes), details)


def sweep_upshift(cycle, emap, downshift: float, lo=20.0, hi=42.0, step=1.0, **kw) -> SweepResult:
    """Vary upshift at fixed downshift. Candidates violating C2 are skipped."""
    rows, details, vals = [], [], []
    for u in np.arange(lo, hi + step / 2, step):
        if u <= downshift:                       # C2
            continue
        r = simulate(cycle, emap, u, downshift, **kw)
        rows.append(r.row()); details.append(r); vals.append(u)
    return _finish(rows, details, vals, "upshift")


def sweep_downshift(cycle, emap, upshift: float, lo=5.0, hi=31.0, step=1.0, **kw) -> SweepResult:
    rows, details, vals = [], [], []
    for d in np.arange(lo, hi + step / 2, step):
        if d >= upshift:                         # C2
            continue
        r = simulate(cycle, emap, upshift, d, **kw)
        rows.append(r.row()); details.append(r); vals.append(d)
    return _finish(rows, details, vals, "downshift")


def sweep_grid(cycle, emap, up_lo=20.0, up_hi=42.0, up_step=1.0,
               dn_lo=5.0, dn_hi=31.0, dn_step=1.0, min_band=1.0, **kw) -> SweepResult:
    """Both thresholds. ``min_band`` enforces a usable hysteresis width (C2)."""
    rows, details, ups, dns = [], [], [], []
    for u in np.arange(up_lo, up_hi + up_step / 2, up_step):
        for d in np.arange(dn_lo, dn_hi + dn_step / 2, dn_step):
            if u - d < min_band:
                continue
            r = simulate(cycle, emap, u, d, **kw)
            rows.append(r.row()); details.append(r); ups.append(u); dns.append(d)
    table = pd.DataFrame(rows)
    ok = [d for d in details if d.feasible and np.isfinite(_objective(d))]
    if not ok:
        return SweepResult(table, None, False, "no feasible candidate", details)
    e = np.array([_objective(r) for r in ok], dtype=float)
    tied = [r for r, x in zip(ok, e) if x <= e.min() + abs(e.min()) * _TIE_REL]
    # among exact ties prefer the widest hysteresis band, then the middle upshift
    tied.sort(key=lambda r: (-(r.upshift - r.downshift), r.upshift))
    best = tied[len(tied) // 2]
    u, d = np.asarray(ups), np.asarray(dns)
    notes = []
    if len(tied) > 1:
        notes.append(f"{len(tied)} grid points tie exactly with the optimum "
                     f"(upshift {min(r.upshift for r in tied):g}-{max(r.upshift for r in tied):g}, "
                     f"downshift {min(r.downshift for r in tied):g}-{max(r.downshift for r in tied):g})"
                     f" - the choice is degenerate over that block")
    if len(tied) == 1 and (abs(best.upshift - u.min()) < up_step / 2
                           or abs(best.upshift - u.max()) < up_step / 2):
        notes.append(f"upshift {best.upshift:g} on the edge of [{u.min():g}, {u.max():g}]")
    if len(tied) == 1 and (abs(best.downshift - d.min()) < dn_step / 2
                           or abs(best.downshift - d.max()) < dn_step / 2):
        notes.append(f"downshift {best.downshift:g} on the edge of [{d.min():g}, {d.max():g}]")
    if abs((best.upshift - best.downshift) - min_band) < 1e-9:
        notes.append(f"band width pinned at the {min_band:g} km/h minimum")
    edge = any("on the edge" in n or "pinned" in n for n in notes)
    return SweepResult(table, best, not edge,
                       "; ".join(notes) + (" - widen the range" if edge else ""), details)


# ---------------------------------------------------------------------------
# Per-gear statistics - ONE definition, shared by every view
# ---------------------------------------------------------------------------
def gear_breakdown(res: ShiftResult, cycle: CycleData) -> pd.DataFrame:
    """Per-gear time, traction and efficiency on a single set of definitions.

    Every view must quote these, because the same words meant different things in
    different panels before this existed:

      time_pct      share of ALL samples, idle included. 23 % of this cycle is
                    stationary and every one of those samples is "in gear 1",
                    which is why time share and traction share are far apart.
      traction_pct  share of the MOTORING samples only (motor delivering torque).
      energy_pct    share of the motor's mechanical output energy.
      efficiency    energy-true: output energy / input energy for that gear, the
                    same definition as ShiftResult.mean_efficiency (an arithmetic
                    mean of map values is not the efficiency of anything).

    Derived from the kept arrays, so it reproduces mean_efficiency exactly when the
    two gears are combined - see the "all" row.
    """
    if res.gear is None:
        raise ValueError("gear_breakdown needs a simulate(..., keep_arrays=True) result")
    g = res.gear
    w = np.abs(res.motor_rpm) * 2.0 * np.pi / 60.0
    p_out = res.motor_torque * w                      # motor mechanical output, signed
    e = res.motor_eff
    # e == 1.0 exactly is the marker EfficiencyMap.query() puts on INACTIVE samples
    # (`e[~active] = 1.0`); a real map value is always below 1. Excluding them here
    # is what makes this reproduce ShiftResult.mean_efficiency bit for bit rather
    # than to ~1e-9 - an idle sample would otherwise add equally to output and input.
    mot = (p_out > 0) & np.isfinite(e) & (e > 0) & (e < 1.0)

    tot_out = _trapz(np.where(mot, p_out, 0.0), cycle.time)
    rows = []
    for gear in (1, 2, None):
        m = np.ones(len(g), dtype=bool) if gear is None else (g == gear)
        mm = m & mot
        e_out = _trapz(np.where(mm, p_out, 0.0), cycle.time)
        e_in = _trapz(np.where(mm, p_out / np.where(mm, e, 1.0), 0.0), cycle.time)
        rpm, trq = res.motor_rpm[mm], res.motor_torque[mm]
        rows.append({
            "gear": "all" if gear is None else gear,
            "samples": int(m.sum()),
            "time_pct": 100.0 * m.mean(),
            "traction_samples": int(mm.sum()),
            "traction_pct": 100.0 * mm.sum() / mot.sum() if mot.sum() else np.nan,
            "energy_pct": 100.0 * e_out / tot_out if tot_out > 0 else np.nan,
            "efficiency": e_out / e_in if e_in > 0 else np.nan,
            "rpm_lo": float(rpm.min()) if rpm.size else np.nan,
            "rpm_hi": float(rpm.max()) if rpm.size else np.nan,
            "torque_lo": float(trq.min()) if trq.size else np.nan,
            "torque_hi": float(trq.max()) if trq.size else np.nan,
        })
    return pd.DataFrame(rows)


# ---------------------------------------------------------------------------
# How much is actually on the table, and what a shift really does
# ---------------------------------------------------------------------------
def _both_gears(cycle, emap, veh, motor, gb, num):
    """Operating point and efficiency in EACH ratio, for every sample.

    Road load is evaluated separately per ratio so each column carries its own
    reflected inertia - the two are self-consistent, not one perturbed.
    """
    out = {}
    for gear in (1, 2):
        ratio, eta = gb.ratio(gear), gb.eta(gear)
        t_w, n_w, p_w = road_load(cycle, veh, motor, gb, num,
                                  np.full(len(cycle.time), ratio))
        n_m = n_w * ratio
        t_m = np.where(p_w >= 0, t_w / (ratio * eta), t_w * eta / ratio)
        act = np.abs(p_w) > num.power_epsilon
        eff, _, _ = emap.query(n_m, t_m, act)
        out[gear] = dict(t_wheel=t_w, p_wheel=p_w, rpm=n_m, torque=t_m,
                         eff=eff, active=act, eta=eta)
    return out


def energy_breakdown(res: ShiftResult, cycle: CycleData, veh: Vehicle = None,
                     motor: Motor = None, gb: Gearbox = None,
                     elec: Electrical = None, num: Numerics = None) -> dict:
    """Where the battery energy actually goes, in kWh.

        wheel  -> the work the road demands
        gearbox-> mesh loss
        motor  -> the efficiency term: what the map costs you
        aux    -> constant hotel load
        pack   -> I^2R in the battery

    This is the question a sweep cannot answer on its own: two schedules differ by
    some number of Wh, but until the difference is split into these terms you cannot
    say whether the optimum won on EFFICIENCY (the operating points moved somewhere
    better) or on something else entirely, like the reflected inertia of the ratio it
    happened to spend more time in.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    elec = elec or Electrical(); num = num or Numerics()
    if res.gear is None:
        raise ValueError("energy_breakdown needs a simulate(..., keep_arrays=True) result")

    ratios = np.where(res.gear == 1, gb.ratio_1, gb.ratio_2)
    etas = np.where(res.gear == 1, gb.eta_1, gb.eta_2)
    t_w, n_w, p_w = road_load(cycle, veh, motor, gb, num, ratios)
    use = ((np.abs(p_w) > num.power_epsilon) & (p_w > 0)
           & np.isfinite(res.motor_eff) & (res.motor_eff < 1.0) & (res.motor_eff > 0))

    e_wheel = _trapz(np.where(use, p_w, 0.0), cycle.time) / 3.6e6
    e_shaft = _trapz(np.where(use, p_w / etas, 0.0), cycle.time) / 3.6e6
    e_elec = _trapz(np.where(use, p_w / (res.motor_eff * etas), 0.0), cycle.time) / 3.6e6
    e_aux = elec.aux_load * cycle.duration / 3.6e6
    e_batt = _trapz(np.maximum(np.nan_to_num(res.battery_power), 0.0),
                    cycle.time) / 3.6e6
    return dict(wheel=e_wheel, gearbox=e_shaft - e_wheel, motor=e_elec - e_shaft,
                aux=e_aux, pack=e_batt - e_elec - e_aux, battery=e_batt,
                efficiency=e_shaft / e_elec if e_elec > 0 else np.nan)


def efficiency_ridge(emap: EfficiencyMap):
    """For each torque, the speed that maximises efficiency. Returns (torque, rpm).

    "The high-efficiency zone" is usually pictured as the map's single best cell,
    but that cell is only the best point AT ITS OWN TORQUE. The rpm that maximises
    efficiency moves a long way with load - on a typical map, from a few hundred rpm
    at 2 Nm to several thousand at 13 Nm. A drive cycle that works the motor at 5 Nm
    is not trying to reach the 13 Nm peak; it is trying to sit on the ridge at 5 Nm,
    which is somewhere else entirely.

    This is the curve a shift schedule should be judged against.
    """
    torque, rpm = [], []
    for i in range(len(emap.torque)):
        if emap.torque[i] <= 0:
            continue
        row = np.where(emap.missing[i], np.nan, emap.eff[i])
        if not np.any(np.isfinite(row)):
            continue
        torque.append(emap.torque[i])
        rpm.append(emap.rpm[int(np.nanargmax(row))])
    return np.asarray(torque), np.asarray(rpm)


def better_gear_per_sample(cycle: CycleData, emap: EfficiencyMap, veh: Vehicle = None,
                           motor: Motor = None, gb: Gearbox = None,
                           num: Numerics = None):
    """Which ratio would be more efficient at each sample, judged independently.

    Returns ``(better, valid)``: ``better`` is 1 or 2 per sample, ``valid`` marks the
    samples where both ratios have a usable efficiency and the motor is pulling.

    This is what makes a schedule auditable. A shift schedule is a guess about which
    ratio to engage; this says what the right answer was, sample by sample, so the
    guess can be scored point by point instead of only in aggregate.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    num = num or Numerics()
    G = _both_gears(cycle, emap, veh, motor, gb, num)
    e1, e2 = G[1]["eff"], G[2]["eff"]
    valid = (G[1]["active"] & (G[1]["p_wheel"] > 0)
             & np.isfinite(e1) & np.isfinite(e2) & (e1 < 1.0) & (e2 < 1.0))
    better = np.where(np.nan_to_num(e1, nan=-1.0) >= np.nan_to_num(e2, nan=-1.0), 1, 2)
    return better.astype(np.int8), valid


def oracle_bound(cycle: CycleData, emap: EfficiencyMap, veh: Vehicle = None,
                 motor: Motor = None, gb: Gearbox = None, elec: Electrical = None,
                 cost: ShiftCost = None, num: Numerics = None) -> dict:
    """Energy if the better ratio could be chosen for EVERY sample independently.

    No schedule - no threshold pair, no 2-D map, nothing causal - can beat this,
    because it is allowed to change gear at every sample with perfect foresight
    and no shift cost. It is the ceiling on the whole optimisation:

        prize = always-best-single-ratio  -  oracle

    If that prize is small, the shift schedule cannot matter however it is chosen,
    and the search is over before it starts.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    elec = elec or Electrical(); num = num or Numerics()
    G = _both_gears(cycle, emap, veh, motor, gb, num)

    batt = {}
    for gear in (1, 2):
        d = G[gear]
        p_shaft = np.zeros_like(d["p_wheel"])
        m = d["active"] & (d["p_wheel"] > 0) & np.isfinite(d["eff"]) & (d["eff"] > 0)
        p_shaft[m] = d["p_wheel"][m] / (d["eff"][m] * d["eta"])

        # the braking side counts too once regen is on, and the ratio changes what
        # the motor can take back - leaving it out made the ceiling regen-blind
        if elec.regen_enabled:
            reg = d["active"] & (d["p_wheel"] < 0)
            env = motor.envelope(d["rpm"])
            t_reg = np.minimum(np.abs(d["torque"]) * elec.regen_fraction, env)
            t_reg = np.where((cycle.speed_kmh >= elec.regen_min_speed_kmh) & reg,
                             t_reg, 0.0)
            take = reg & (t_reg > 0)
            e_r, _, _ = emap.query(d["rpm"], t_reg, take)
            e_r = np.where(np.isfinite(e_r) & (e_r > 0) & (e_r <= 1.0), e_r, 0.0)
            w = np.abs(d["rpm"]) * 2.0 * np.pi / 60.0
            rec = np.minimum(np.where(take, t_reg * w * e_r, 0.0), elec.charge_limit)
            p_shaft = p_shaft - rec

        batt[gear] = _terminal_power(p_shaft + elec.aux_load, elec)

    # net energy: what goes out minus what comes back
    e = lambda p: float(_trapz(p, cycle.time) / 3.6e6)
    b1 = np.nan_to_num(batt[1], nan=np.inf)
    b2 = np.nan_to_num(batt[2], nan=np.inf)
    best = np.minimum(b1, b2)
    single = min(e(batt[1]), e(batt[2]))
    oracle = e(best)
    act = G[2]["active"]
    return dict(gear1_only=e(batt[1]), gear2_only=e(batt[2]),
                best_single=single, oracle=oracle, prize_wh=1000.0 * (single - oracle),
                gear1_share=100.0 * float(np.mean((b1 < b2)[act])) if act.any() else np.nan)


def shift_decomposition(res: ShiftResult, cycle: CycleData, emap: EfficiencyMap,
                        veh: Vehicle = None, motor: Motor = None, gb: Gearbox = None,
                        num: Numerics = None) -> dict:
    """Separate what a gear change does from what the driver does.

    The arrow drawn in "Shift movement" runs from sample i to sample i+1, so it
    contains BOTH the ratio change and whatever the demand did in between. Split
    exactly (the two terms sum to the arrow):

        eff(i+1, new) - eff(i, old)  =  [eff(i, new) - eff(i, old)]   <- the gearbox
                                     +  [eff(i+1, new) - eff(i, new)] <- the driver
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    num = num or Numerics()
    G = _both_gears(cycle, emap, veh, motor, gb, num)
    e = {g: G[g]["eff"] for g in (1, 2)}
    p_w = G[2]["p_wheel"]

    sh = np.flatnonzero(np.diff(res.gear) != 0)
    out = {}
    for name, sel in (("upshift", res.gear[sh + 1] == 2),
                      ("downshift", res.gear[sh + 1] == 1)):
        i = sh[sel]
        if not len(i):
            out[name] = None
            continue
        new = 2 if name == "upshift" else 1
        old = 1 if name == "upshift" else 2
        j = i + 1
        ok = (np.isfinite(e[new][i]) & np.isfinite(e[old][i])
              & np.isfinite(e[new][j]))
        i, j = i[ok], j[ok]
        if not len(i):
            out[name] = None
            continue
        gear_term = (e[new][i] - e[old][i]) * 100.0
        driver_term = (e[new][j] - e[new][i]) * 100.0
        w = np.maximum(p_w[i], 0.0)
        crossed = np.sign(res.motor_torque[i]) != np.sign(res.motor_torque[j])
        under_traction = p_w[i] > 0
        out[name] = dict(
            events=len(i),
            arrow_pts=float(np.mean(gear_term + driver_term)),
            gear_pts=float(np.mean(gear_term)),
            driver_pts=float(np.mean(driver_term)),
            gear_helps_pct=100.0 * float(np.mean(gear_term > 0)),
            crossed_zero=int(crossed.sum()),
            traction_events=int(under_traction.sum()),
            gear_pts_traction=float(np.mean(gear_term[under_traction]))
            if under_traction.any() else np.nan,
            gear_pts_braking=float(np.mean(gear_term[~under_traction]))
            if (~under_traction).any() else np.nan,
            gear_pts_power_weighted=float(np.sum(w * gear_term) / np.sum(w))
            if np.sum(w) > 0 else np.nan,
        )
    return out


def counterfactual_point(res: ShiftResult, idx: np.ndarray, new_gear: int,
                         gb: Gearbox) -> tuple:
    """(rpm, torque) at the SAME instant had the other ratio been engaged.

    Same wheel demand, different ratio - so this is the pure gearbox effect,
    with no contribution from the demand moving on between two samples. The
    reflected-inertia difference between ratios is ignored (< 2 % of wheel
    torque here), which is why this is a diagnostic and not the energy path.
    """
    old = np.where(res.gear[idx] == 1, gb.ratio_1, gb.ratio_2)
    old_eta = np.where(res.gear[idx] == 1, gb.eta_1, gb.eta_2)
    new = gb.ratio(new_gear)
    new_eta = gb.eta(new_gear)
    rpm = res.motor_rpm[idx] * new / old
    torque = res.motor_torque[idx] * (old * old_eta) / (new * new_eta)
    return rpm, torque


# ---------------------------------------------------------------------------
# Shift schedule ranked on motor efficiency alone
# ---------------------------------------------------------------------------
def sweep_efficiency(cycle: CycleData, emap: EfficiencyMap,
                     up_lo=8.0, up_hi=42.0, up_step=2.0,
                     dn_lo=4.0, dn_hi=30.0, dn_step=2.0,
                     min_band=1.0, **kw) -> SweepResult:
    """Rank schedules by energy-weighted mean MOTOR efficiency over the cycle.

    ``ShiftResult.mean_efficiency`` is shaft output energy over electrical input
    energy across the motoring samples. It contains nothing but the map: the
    auxiliary load, the pack resistance, the shift actuation energy and the torque
    interruption all cancel out of it, because none of them appear in either
    integral. So maximising it answers exactly one question -

        which shift schedule keeps the motor in the best part of its map,
        over this drive cycle, ignoring every other cost?

    That is a different question from minimising battery energy, and the two do not
    have to agree. Where they disagree, the difference is the price the rest of the
    system charges for sitting in the efficient place.

    The reflected rotor inertia is deliberately still in here: it changes the wheel
    demand, hence the operating point, hence the efficiency actually achieved.
    """
    rows, details, ups, dns = [], [], [], []
    for u in np.arange(up_lo, up_hi + up_step / 2, up_step):
        for d in np.arange(dn_lo, dn_hi + dn_step / 2, dn_step):
            if u - d < min_band:
                continue
            r = simulate(cycle, emap, u, d, **kw)
            rows.append(r.row()); details.append(r); ups.append(u); dns.append(d)

    table = pd.DataFrame(rows)
    ok = [r for r in details if r.feasible and np.isfinite(r.mean_efficiency)]
    if not ok:
        return SweepResult(table, None, False, "no feasible candidate", details)

    top = max(r.mean_efficiency for r in ok)
    tied = [r for r in ok if r.mean_efficiency >= top - 1e-12]
    tied.sort(key=lambda r: (r.upshift, r.downshift))
    best = tied[len(tied) // 2]

    u_arr, d_arr = np.asarray(ups, float), np.asarray(dns, float)
    notes = []
    if len(tied) > 1:
        notes.append(f"{len(tied)} schedules tie at {top:.4%}")
    on_edge = (abs(best.upshift - u_arr.min()) < up_step / 2
               or abs(best.upshift - u_arr.max()) < up_step / 2
               or abs(best.downshift - d_arr.min()) < dn_step / 2
               or abs(best.downshift - d_arr.max()) < dn_step / 2)
    if on_edge:
        notes.append(f"best efficiency {best.upshift:g}/{best.downshift:g} sits on the "
                     f"edge of the searched grid - widen it")
    return SweepResult(table, best, not on_edge, "; ".join(notes), details)


# ---------------------------------------------------------------------------
# Wide-open-throttle acceleration: 0 -> V on the motor's peak curve
# ---------------------------------------------------------------------------
@dataclass
class WOTRun:
    """One full-throttle launch with a given 1-2 upshift speed."""
    shift_speed: float
    reached: bool
    time_s: float = np.nan            # 0 -> target
    distance_m: float = np.nan
    energy_wh: float = np.nan
    top_speed_kmh: float = np.nan     # where tractive force meets road load
    shift_times: list = field(default_factory=list)
    notes: list = field(default_factory=list)
    t: Optional[np.ndarray] = None
    v_kmh: Optional[np.ndarray] = None
    gear: Optional[np.ndarray] = None
    accel: Optional[np.ndarray] = None
    rpm: Optional[np.ndarray] = None
    torque: Optional[np.ndarray] = None
    eff: Optional[np.ndarray] = None

    def row(self) -> dict:
        return dict(shift_speed=self.shift_speed, reached=self.reached,
                    time_s=self.time_s, distance_m=self.distance_m,
                    energy_wh=self.energy_wh, top_speed_kmh=self.top_speed_kmh,
                    shifts=len(self.shift_times), notes="; ".join(self.notes))


@dataclass
class WOTSweep:
    v_target: float
    throttle: float
    table: pd.DataFrame
    runs: list
    best_time: Optional[WOTRun] = None
    best_energy: Optional[WOTRun] = None
    curves: tuple = ()                # (v_kmh, F_gear1, F_gear2, F_resist) in N


def tractive_force(veh: Vehicle, motor: Motor, gb: Gearbox, gear: int,
                   v_ms: np.ndarray, throttle: float = 1.0) -> np.ndarray:
    """Force at the wheel with the motor on its peak curve, N.

    Zero above the ratio's rpm limit - that is what caps top speed in gear 1.
    """
    v = np.atleast_1d(np.asarray(v_ms, dtype=float))
    ratio, eta = gb.ratio(gear), gb.eta(gear)
    n = (v / veh.wheel_radius) * 60.0 / (2.0 * np.pi) * ratio
    t = motor.envelope(n) * float(np.clip(throttle, 0.0, 1.0))
    return np.where(n <= motor.max_rpm, t * ratio * eta / veh.wheel_radius, 0.0)


def road_load_force(veh: Vehicle, v_ms: np.ndarray) -> np.ndarray:
    """Steady resistance at speed: rolling + aero + grade, N."""
    v = np.atleast_1d(np.asarray(v_ms, dtype=float))
    th = np.deg2rad(veh.grade_deg)
    return (veh.mass * veh.gravity * veh.crr * np.cos(th)
            + 0.5 * veh.air_density * veh.cda * v ** 2
            + veh.mass * veh.gravity * np.sin(th))


def _wot_accel(v_ms, gear, veh, motor, gb, elec, throttle):
    """Available acceleration at speed v in one ratio, vectorised.

    a(v) = (F_traction - F_road) / (m + J*ratio^2/r^2), with the motor on
    min(peak torque, constant power), zero above the ratio's rpm limit, and the
    pack ceiling referred to the shaft. Returns (a, rpm, torque).
    """
    v = np.atleast_1d(np.asarray(v_ms, dtype=float))
    ratio, eta = gb.ratio(gear), gb.eta(gear)
    n = (v / veh.wheel_radius) * 60.0 / (2.0 * np.pi) * ratio
    w = n * 2.0 * np.pi / 60.0
    t = motor.envelope(n) * float(np.clip(throttle, 0.0, 1.0))
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.minimum(t, np.where(w > 1e-9, elec.max_power / np.maximum(w, 1e-9), np.inf))
    t = np.where(n <= motor.max_rpm, t, 0.0)
    f = t * ratio * eta / veh.wheel_radius
    m_eff = veh.mass + motor.inertia * ratio ** 2 / veh.wheel_radius ** 2
    return (f - road_load_force(veh, v)) / m_eff, n, t


def _wot_batt_power(n, t, gear, emap, gb, elec, num):
    """Battery power drawn while pulling at (n, t), W."""
    w = n * 2.0 * np.pi / 60.0
    p_mech = t * w
    eta = gb.eta(gear)
    act = p_mech > num.power_epsilon
    eff, _, _ = emap.query(n, t, act)
    eff = np.where(np.isfinite(eff) & (eff > 0), eff, num.min_efficiency)
    p_dem = np.where(act, p_mech / (eff * eta), 0.0) + elec.aux_load
    return _terminal_power(p_dem, elec), eff


def _wot_phase(v0, v1, gear, emap, veh, motor, gb, elec, num, throttle, n_grid=1501):
    """Accelerate from v0 to v1 in one ratio, by quadrature over SPEED.

    a depends only on v inside a phase, so time, distance and energy are integrals
    rather than a time-stepped simulation:

        t = INT dv/a(v)      x = INT v dv/a(v)      E = INT P_batt(v) dv/a(v)

    That removes both the Euler bias and the dt quantisation of the old fixed-step
    integrator - which mattered here, because the differences between candidate
    shift speeds are of the same order as the old time step.

    Returns (ok, t, x, E_joule, trace) where trace = (t_grid, v_grid, rpm, torque).
    """
    if v1 <= v0 + 1e-12:
        z = np.array([v0])
        return True, 0.0, 0.0, 0.0, (np.array([0.0]), z, *(_wot_accel(z, gear, veh, motor,
                                                                     gb, elec, throttle)[1:]))
    v = np.linspace(v0, v1, int(n_grid))
    a, n, t_m = _wot_accel(v, gear, veh, motor, gb, elec, throttle)
    if np.any(a <= 1e-6):
        stall = float(v[np.argmax(a <= 1e-6)])
        return False, np.nan, np.nan, np.nan, (None, stall, None, None)

    p_batt, _ = _wot_batt_power(n, t_m, gear, emap, gb, elec, num)
    inv = 1.0 / a
    dt_dv = inv
    t_grid = np.concatenate([[0.0], np.cumsum(np.diff(v) * (dt_dv[1:] + dt_dv[:-1]) / 2.0)])
    x = float(_trapz(v * inv, v))
    e = float(_trapz(np.nan_to_num(p_batt) * inv, v))
    return True, float(t_grid[-1]), x, e, (t_grid, v, n, t_m)


def _wot_coast(v0, duration, veh, motor, gb, elec, gear, n_steps=400):
    """Torque interrupted: no traction, vehicle decelerates against road load.

    RK4 over the (short) interruption; returns (v_end, distance, trace).
    """
    if duration <= 0:
        return v0, 0.0, (np.array([0.0]), np.array([v0]))
    ratio = gb.ratio(gear)
    m_eff = veh.mass + motor.inertia * ratio ** 2 / veh.wheel_radius ** 2
    f = lambda vv: -float(road_load_force(veh, max(vv, 0.0))[0]) / m_eff
    h = duration / n_steps
    v, x = v0, 0.0
    ts, vs = [0.0], [v0]
    for k in range(n_steps):
        k1 = f(v); k2 = f(v + h * k1 / 2); k3 = f(v + h * k2 / 2); k4 = f(v + h * k3)
        dv = h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        x += (v + dv / 2.0) * h
        v = max(0.0, v + dv)
        ts.append((k + 1) * h); vs.append(v)
    return v, x, (np.array(ts), np.array(vs))


def wot_run(emap: EfficiencyMap, v_target_kmh: float, shift_speed: float,
            veh: Vehicle = None, motor: Motor = None, gb: Gearbox = None,
            elec: Electrical = None, cost: ShiftCost = None, num: Numerics = None,
            throttle: float = 1.0, dt: float = 0.01, t_max: float = 120.0,
            n_grid: int = 1501) -> WOTRun:
    """Time a full-throttle launch from rest to ``v_target_kmh``.

    Three phases: pull in gear 1 to the shift speed, coast through the torque
    interruption, pull in gear 2 to the target. Each pulling phase is a quadrature
    over speed (see ``_wot_phase``), so the answer is not tied to a time step -
    the sweep resolves differences of a millisecond between shift speeds.

    ``dt`` is accepted for call compatibility and is not used.
    """
    veh = veh or Vehicle(); motor = motor or Motor(); gb = gb or Gearbox()
    elec = elec or Electrical(); cost = cost or ShiftCost(); num = num or Numerics()

    v_t = v_target_kmh / 3.6
    v_s = shift_speed / 3.6
    run = WOTRun(float(shift_speed), reached=False)
    shifts = 0 < v_s < v_t

    T, V, G, N, Q = [], [], [], [], []
    t_acc = x_acc = e_acc = 0.0

    def push(trace, gear, t_off):
        tg, vg, ng, qg = trace
        T.append(tg + t_off); V.append(vg); N.append(ng); Q.append(qg)
        G.append(np.full(len(vg), gear, dtype=np.int8))

    # --- phase 1: gear 1 from rest ----------------------------------------
    v_end1 = v_s if shifts else v_t
    ok, t1, x1, e1, tr = _wot_phase(0.0, v_end1, 1, emap, veh, motor, gb, elec, num,
                                    throttle, n_grid)
    if not ok:
        run.top_speed_kmh = tr[1] * 3.6
        run.notes.append(f"gear 1 cannot accelerate past {tr[1]*3.6:.1f} km/h "
                         f"- traction equals road load")
        return run
    push(tr, 1, 0.0)
    t_acc, x_acc, e_acc = t1, x1, e1

    if shifts:
        # --- phase 2: torque interruption ----------------------------------
        v_after, x_cut, ctr = _wot_coast(v_s, cost.interrupt_s, veh, motor, gb, elec, 2)
        e_acc += elec.aux_load * cost.interrupt_s + cost.energy_per_shift
        run.shift_times.append(t_acc)
        ts, vs = ctr
        T.append(ts + t_acc); V.append(vs)
        N.append(np.zeros(len(vs))); Q.append(np.zeros(len(vs)))
        G.append(np.full(len(vs), 2, dtype=np.int8))
        t_acc += cost.interrupt_s
        x_acc += x_cut

        # --- phase 3: gear 2 to the target ---------------------------------
        ok, t3, x3, e3, tr = _wot_phase(v_after, v_t, 2, emap, veh, motor, gb, elec, num,
                                        throttle, n_grid)
        if not ok:
            run.top_speed_kmh = tr[1] * 3.6
            run.notes.append(f"gear 2 cannot accelerate past {tr[1]*3.6:.1f} km/h "
                             f"- traction equals road load")
            run.t = np.concatenate(T); run.v_kmh = np.concatenate(V) * 3.6
            run.gear = np.concatenate(G); run.rpm = np.concatenate(N)
            run.torque = np.concatenate(Q)
            return run
        push(tr, 2, t_acc)
        t_acc += t3; x_acc += x3; e_acc += e3

    run.reached = True
    run.time_s = t_acc
    run.distance_m = x_acc
    run.energy_wh = e_acc / 3600.0
    run.t = np.concatenate(T)
    run.v_kmh = np.concatenate(V) * 3.6
    run.gear = np.concatenate(G)
    run.rpm = np.concatenate(N)
    run.torque = np.concatenate(Q)
    # the phase joins repeat a time point (end of pull == start of coast), which
    # would make np.gradient divide by zero
    step = np.diff(run.t, prepend=run.t[0] - 1.0)
    run.accel = np.zeros(len(run.t))
    good = step > 1e-12
    run.accel[good] = np.gradient(np.concatenate(V)[good], run.t[good])
    run.eff = np.full(len(run.t), np.nan)
    run.top_speed_kmh = float(run.v_kmh.max())
    if t_acc > t_max:
        run.reached = False
        run.notes.append(f"took longer than {t_max:g} s")
    return run


def wot_sweep(emap: EfficiencyMap, v_target_kmh: float, lo: float = 5.0,
              hi: float = 40.0, step: float = 1.0, throttle: float = 1.0,
              dt: float = 0.01, t_max: float = 120.0, **kw) -> WOTSweep:
    """0 -> V full-throttle time and energy against the 1-2 upshift speed."""
    veh = kw.get("veh") or Vehicle(); motor = kw.get("motor") or Motor()
    gb = kw.get("gb") or Gearbox()

    ups = [float(u) for u in np.arange(lo, hi + step / 2, step) if 0 < u < v_target_kmh]
    ups.append(float(v_target_kmh + step))     # "hold gear 1" - see acceleration notes
    runs = [wot_run(emap, v_target_kmh, u, throttle=throttle, dt=dt, t_max=t_max, **kw)
            for u in ups]
    table = pd.DataFrame([r.row() for r in runs])

    ok = [r for r in runs if r.reached]
    best_t = min(ok, key=lambda r: r.time_s) if ok else None
    best_e = min(ok, key=lambda r: r.energy_wh) if ok else None

    v_line = np.linspace(0.1, max(v_target_kmh * 1.6, 45.0), 400) / 3.6
    curves = (v_line * 3.6,
              tractive_force(veh, motor, gb, 1, v_line, throttle),
              tractive_force(veh, motor, gb, 2, v_line, throttle),
              road_load_force(veh, v_line))
    return WOTSweep(float(v_target_kmh), float(throttle), table, runs,
                    best_t, best_e, curves)
