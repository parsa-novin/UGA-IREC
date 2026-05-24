"""
hil_l2200_sim.py — RocketPy-based Hardware-In-the-Loop flight simulator.

Based on mags.py / mags_test_launch.py but adapted for real-time HIL use:
  • Uses the AeroTech L2200G motor (.eng file in this directory)
  • Uses the Deployment_Fits_Output/ Mach-vs-Cd CSV tables for airbrake drag
  • Runs a thread-safe, step-by-step point-mass 1-D ODE so the PC simulation
    pace exactly matches the packet rate injected into the H5
  • Accepts live deployment-level updates from the H5 feedback thread

Public API
----------
    sim = HILFlightSim()
    # (optional) override defaults:
    sim = HILFlightSim(
        motor_file  = "AeroTech_L2200G.eng",
        drag_csv_dir= "Deployment_Fits_Output",
        rocket_mass = 18.7,           # kg — body + motor casing (no propellant)
        target_apogee_m = 3048.0,     # 10 000 ft AGL
    )

    # In the serial-RX callback:
    sim.set_deployment_level(angle_deg / 70.0)   # 0..1

    # In the HIL sender thread:
    for pkt_fields, dt in sim.generate_packets(rate_hz=50.0):
        serial.write(build_packet(pkt_fields))
        serial.flush()

    # After the run completes, save telemetry in the same CSV format
    # the csv_flight() reader in hil_gui.py / hil_test.py expects:
    sim.save_csv("hil_flight_log.csv")

CSV column names (match hil_gui.py csv_flight() reader)
---------------------------------------------------------
    packet_time_s, altitude_m, pressure_kpa,
    imu16_ax_g, imu16_ay_g, imu16_az_g,
    imu4_ax_g,  imu4_ay_g,  imu4_az_g,
    mag_x_gauss, mag_y_gauss, mag_z_gauss,
    bmi_ax_g, bmi_ay_g, bmi_az_g,
    bmi_gx_dps, bmi_gy_dps, bmi_gz_dps,
    ext_temp_c,
    esc_valid, esc_temp_c, esc_voltage_v, esc_current_a,
    esc_consumption_mah, esc_erpm,
    encoder_position_mm,
    sim_deploy_level, sim_velocity_mps   (extra diagnostic columns)
"""

from __future__ import annotations

import csv
import re
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator, List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_BASE_DIR = Path(__file__).resolve().parent

DEFAULT_MOTOR_FILE   = _BASE_DIR / "AeroTech_L2200G.eng"
DEFAULT_DRAG_CSV_DIR = _BASE_DIR / "Deployment_Fits_Output"

# ---------------------------------------------------------------------------
# Rocket / atmosphere constants  (mirrors flight_estimator.h + mags.py)
# ---------------------------------------------------------------------------

G0          = 9.80665        # m/s²  standard gravity
RHO0        = 1.225          # kg/m³ sea-level density
T0_K        = 288.15         # K     sea-level temperature
T_TROPO_K   = 216.65         # K     tropopause floor
L_KPM       = 0.0065         # K/m   lapse rate
R_AIR       = 287.05         # J/kg·K
GAMMA_AIR   = 1.4            # —

ROCKET_RADIUS   = 0.078359   # m  — matches mags.py
REF_AREA        = np.pi * ROCKET_RADIUS ** 2   # m²

# Rocket mass (body + motor casing, NO propellant) — matches firmware
ROCKET_DRY_MASS_KG  = 18.7 + 1.608   # 18.7 kg body + 1.608 kg motor case

# Maximum airbrake deployment angle — must match firmware AIRBRAKE_MAX_ANGLE_DEG
AIRBRAKE_MAX_ANGLE_DEG = 70.0

# Magnetic field reference (Huntsville area, roughly)
MAG_X_GAUSS =  0.18
MAG_Y_GAUSS =  0.02
MAG_Z_GAUSS =  0.46


# ---------------------------------------------------------------------------
# Atmosphere helpers
# ---------------------------------------------------------------------------

def _atm_temp(h_m: float) -> float:
    return max(T0_K - L_KPM * h_m, T_TROPO_K)


def _atm_density(h_m: float) -> float:
    T   = _atm_temp(h_m)
    exp = G0 / (R_AIR * L_KPM) - 1.0   # ≈ 4.256
    return RHO0 * (T / T0_K) ** exp


def _speed_of_sound(h_m: float) -> float:
    return (GAMMA_AIR * R_AIR * _atm_temp(h_m)) ** 0.5


def _atm_pressure(h_m: float) -> float:
    """Approximate barometric pressure [Pa]."""
    T_ratio = _atm_temp(h_m) / T0_K
    # ISA troposphere pressure formula
    return 101325.0 * T_ratio ** (G0 / (R_AIR * L_KPM))


# ---------------------------------------------------------------------------
# .eng motor file parser
# ---------------------------------------------------------------------------

@dataclass
class MotorData:
    name:          str
    prop_mass_kg:  float
    total_mass_kg: float
    dry_mass_kg:   float
    thrust_t:      np.ndarray   # time  [s]
    thrust_f:      np.ndarray   # force [N]
    total_impulse: float        # N·s

    def thrust_at(self, t: float) -> float:
        """Interpolated thrust [N] at time t; 0 before ignition or after burnout."""
        if t < self.thrust_t[0] or t > self.thrust_t[-1]:
            return 0.0
        return float(np.interp(t, self.thrust_t, self.thrust_f))

    @property
    def burnout_time(self) -> float:
        return float(self.thrust_t[-1])


def parse_eng_file(path: str | Path) -> MotorData:
    """
    Parse an RASP .eng thrust-curve file.

    Header line format (space-separated):
        name diameter_mm length_mm delays prop_mass_kg total_mass_kg manufacturer

    Subsequent non-comment lines:
        time_s thrust_n
    """
    path = Path(path)
    times: List[float] = []
    forces: List[float] = []
    prop_mass = 0.0
    total_mass = 0.0
    name = "unknown"
    header_found = False

    with path.open("r") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith(";"):
                continue
            # First non-comment line is the header
            if not header_found:
                parts = line.split()
                if len(parts) >= 7:
                    name       = parts[0]
                    prop_mass  = float(parts[5])
                    total_mass = float(parts[6])
                header_found = True
                continue
            # Data lines
            parts = line.split()
            if len(parts) >= 2:
                try:
                    times.append(float(parts[0]))
                    forces.append(float(parts[1]))
                except ValueError:
                    pass

    if not times:
        raise ValueError(f"No thrust data found in: {path}")

    t_arr = np.array(times,  dtype=float)
    f_arr = np.array(forces, dtype=float)

    # Ensure thrust starts at t=0 and ends at 0 N
    if t_arr[0] > 0.0:
        t_arr  = np.concatenate([[0.0], t_arr])
        f_arr  = np.concatenate([[0.0], f_arr])
    if f_arr[-1] > 0.0:
        t_arr  = np.concatenate([t_arr,  [t_arr[-1]]])
        f_arr  = np.concatenate([f_arr,  [0.0]])

    # Total impulse via trapezoidal integration
    total_impulse = float(np.trapz(f_arr, t_arr))

    return MotorData(
        name          = name,
        prop_mass_kg  = prop_mass,
        total_mass_kg = total_mass,
        dry_mass_kg   = total_mass - prop_mass,
        thrust_t      = t_arr,
        thrust_f      = f_arr,
        total_impulse = total_impulse,
    )


# ---------------------------------------------------------------------------
# Airbrake drag model  (adapted from mags_test_launch.py AirbrakeModel)
# ---------------------------------------------------------------------------

def _normalize_drag_table(df) -> np.ndarray:
    """Return (N,2) float array [mach, Cd] sorted and anchored at mach=0."""
    import pandas as pd  # local import — only needed if AirbrakeModel is used

    # Infer column names
    mach_col = cd_col = None
    for c in df.columns:
        if re.search(r"mach", c, re.IGNORECASE):
            mach_col = c
        if re.search(r"(drag.*coeff|cd\b|c_d\b|dragcoefficient)", c, re.IGNORECASE):
            cd_col = c
    if "MachNumber___" in df.columns and "x_DragCoefficient___" in df.columns:
        mach_col, cd_col = "MachNumber___", "x_DragCoefficient___"
    if mach_col is None or cd_col is None:
        raise ValueError(f"Cannot find Mach/Cd columns in {list(df.columns)}")

    df = df.drop_duplicates(subset=mach_col, keep="first")
    df = df.sort_values(mach_col).reset_index(drop=True)
    if float(df[mach_col].iloc[0]) > 0.0:
        zero_row = {mach_col: 0.0, cd_col: float(df[cd_col].iloc[0])}
        df = pd.concat([pd.DataFrame([zero_row]), df], ignore_index=True)
    return df[[mach_col, cd_col]].astype(float).values


class AirbrakeModel:
    """
    Mach-dependent airbrake drag, exactly mirroring mags_test_launch.py.

    Loads Deployment_<N>deg_CdFit.csv files and interpolates between them.

    drag_coefficient_curve(deployment_level, mach) returns the *delta* Cd
    that RocketPy would add to the body Cd.

    For the HIL physics loop we use total_cd(angle_deg, mach) instead, which
    returns the full Cd so we can compute F_drag directly.
    """

    _AREA_CONST_MM2 = 3982.98097   # exposed area constant from geometry

    def __init__(self, csv_dir: str | Path = DEFAULT_DRAG_CSV_DIR,
                 reference_area: float = REF_AREA):
        import pandas as pd
        self.csv_dir        = Path(csv_dir)
        self.reference_area = reference_area
        self.tables: dict[float, np.ndarray] = {}
        self.deployment_angles: List[float]   = []
        self._load_tables(pd)

    def _load_tables(self, pd) -> None:
        pattern = re.compile(r"Deployment_(\d+(?:\.\d+)?)deg_CdFit\.csv", re.IGNORECASE)
        for path in sorted(self.csv_dir.glob("Deployment_*deg_CdFit.csv")):
            m = pattern.search(path.name)
            if not m:
                continue
            angle = float(m.group(1))
            df    = pd.read_csv(path)
            self.tables[angle] = _normalize_drag_table(df)

        if not self.tables:
            raise FileNotFoundError(
                f"No Deployment_*deg_CdFit.csv files found in: {self.csv_dir}"
            )
        self.deployment_angles = sorted(self.tables.keys())
        print(f"[HIL sim] Loaded {len(self.tables)} airbrake drag tables from {self.csv_dir}")

    # ------------------------------------------------------------------
    def _cd_at_angle_mach(self, angle_deg: float, mach: float) -> float:
        """Total Cd (body + airbrakes) at a given deployment angle and Mach."""
        angles    = self.deployment_angles
        angle_deg = float(np.clip(angle_deg, angles[0], angles[-1]))

        if angle_deg <= angles[0]:
            tab = self.tables[angles[0]]
            return float(np.interp(mach, tab[:, 0], tab[:, 1]))
        if angle_deg >= angles[-1]:
            tab = self.tables[angles[-1]]
            return float(np.interp(mach, tab[:, 0], tab[:, 1]))

        hi = next(i for i, v in enumerate(angles) if v >= angle_deg)
        lo = hi - 1
        t  = (angle_deg - angles[lo]) / (angles[hi] - angles[lo])
        cd0 = float(np.interp(mach, self.tables[angles[lo]][:, 0], self.tables[angles[lo]][:, 1]))
        cd1 = float(np.interp(mach, self.tables[angles[hi]][:, 0], self.tables[angles[hi]][:, 1]))
        return (1.0 - t) * cd0 + t * cd1

    def total_cd(self, deployment_level: float, mach: float) -> float:
        """Full Cd at a given RocketPy deployment_level [0..1] and Mach number."""
        angle_deg = float(np.clip(deployment_level, 0.0, 1.0)) * AIRBRAKE_MAX_ANGLE_DEG
        return self._cd_at_angle_mach(angle_deg, mach)

    # RocketPy-compatible delta-Cd interface (kept for compatibility)
    def drag_coefficient_curve(self, deployment_level: float, mach: float) -> float:
        angle_deg = float(np.clip(deployment_level, 0.0, 1.0)) * AIRBRAKE_MAX_ANGLE_DEG
        cd_total  = self._cd_at_angle_mach(angle_deg, mach)
        cd_zero   = self._cd_at_angle_mach(0.0, mach)
        delta_cd  = cd_total - cd_zero
        delta_area = self._AREA_CONST_MM2 * np.sin(np.deg2rad(angle_deg)) / 1e6
        return delta_cd * (1.0 + delta_area / self.reference_area)


# ---------------------------------------------------------------------------
# Telemetry row  (one row per integration step)
# ---------------------------------------------------------------------------

@dataclass
class _TelemetryRow:
    packet_time_s:      float
    altitude_m:         float
    pressure_kpa:       float
    imu16_ax_g:         float
    imu16_ay_g:         float
    imu16_az_g:         float
    imu4_ax_g:          float
    imu4_ay_g:          float
    imu4_az_g:          float
    mag_x_gauss:        float
    mag_y_gauss:        float
    mag_z_gauss:        float
    bmi_ax_g:           float
    bmi_ay_g:           float
    bmi_az_g:           float
    bmi_gx_dps:         float
    bmi_gy_dps:         float
    bmi_gz_dps:         float
    ext_temp_c:         float
    esc_valid:          int
    esc_temp_c:         int
    esc_voltage_v:      float
    esc_current_a:      float
    esc_consumption_mah:float
    esc_erpm:           int
    encoder_position_mm:float
    sim_deploy_level:   float
    sim_velocity_mps:   float


_CSV_FIELDS = [f.name for f in _TelemetryRow.__dataclass_fields__.values()]


# ---------------------------------------------------------------------------
# PacketFields dataclass  (matches hil_gui.py PacketFields)
# ---------------------------------------------------------------------------

@dataclass
class PacketFields:
    altitude_m:          float = 0.0
    pressure_pa:         int   = 101325
    imu16_ax_g:          float = 0.0
    imu16_ay_g:          float = 0.0
    imu16_az_g:          float = 1.0
    imu4_ax_g:           float = 0.0
    imu4_ay_g:           float = 0.0
    imu4_az_g:           float = 1.0
    mag_x_gauss:         float = 0.0
    mag_y_gauss:         float = 0.0
    mag_z_gauss:         float = 0.5
    bmi_ax_g:            float = 0.0
    bmi_ay_g:            float = 0.0
    bmi_az_g:            float = 1.0
    bmi_gx_dps:          float = 0.0
    bmi_gy_dps:          float = 0.0
    bmi_gz_dps:          float = 0.0
    ext_temp_c:          float = 20.0
    esc_valid:           bool  = False
    esc_temp_c:          int   = 25
    esc_voltage_mv:      int   = 16800
    esc_current_ma:      int   = 0
    esc_consumption_mah: int   = 0
    esc_erpm:            int   = 0
    encoder_position_um: int   = 0


# ---------------------------------------------------------------------------
# Main HIL flight simulator
# ---------------------------------------------------------------------------

class HILFlightSim:
    """
    Real-time step-based 1-D flight simulator for HIL testing.

    Physics model
    -------------
    Vertical (1-D) point mass.  Gravity + motor thrust + aerodynamic drag.
    Motor thrust from the AeroTech L2200G .eng file.
    Drag Cd from the Deployment_Fits_Output CSVs via AirbrakeModel.
    Propellant mass is burned proportionally to impulse delivered.

    Sensor outputs
    --------------
    IMU specific force (az_g) = (F_thrust - F_drag) / (m * G0)
      • ≈ 14 g during burn  →  triggers firmware ARM at |a| > 5 g
      • ≈ 0.3 g during coast  →  triggers firmware FIRE at |a| < 3 g
    Barometric altitude and pressure from ISA model.
    Temperature from troposphere lapse rate.
    """

    def __init__(
        self,
        motor_file:       str | Path = DEFAULT_MOTOR_FILE,
        drag_csv_dir:     str | Path = DEFAULT_DRAG_CSV_DIR,
        rocket_dry_mass:  float      = ROCKET_DRY_MASS_KG,
        target_apogee_m:  float      = 3048.0,    # 10 000 ft AGL
    ):
        self.motor  = parse_eng_file(motor_file)
        print(f"[HIL sim] Motor: {self.motor.name}  "
              f"prop={self.motor.prop_mass_kg:.3f} kg  "
              f"Isp_equiv={self.motor.total_impulse/(self.motor.prop_mass_kg*G0):.0f} s  "
              f"burnout={self.motor.burnout_time:.2f} s")

        self.airbrake = AirbrakeModel(drag_csv_dir)
        self.target_apogee_m = target_apogee_m

        # State
        self._t          = 0.0
        self._alt        = 0.0    # m AGL
        self._vel        = 0.0    # m/s  (positive = upward)
        self._dry_mass   = rocket_dry_mass
        self._prop_mass  = self.motor.prop_mass_kg
        self._cum_imp    = 0.0    # N·s impulse delivered so far
        self._done       = False

        # Thread-safe deployment level (set by H5 feedback)
        self._deploy_lock  = threading.Lock()
        self._deploy_level = 0.0  # 0..1

        # Telemetry log
        self._log: List[_TelemetryRow] = []

        print(f"[HIL sim] Rocket dry mass = {self._dry_mass:.2f} kg  "
              f"(+{self._prop_mass:.3f} kg propellant at ignition)")
        print(f"[HIL sim] Target apogee   = {target_apogee_m:.0f} m  "
              f"({target_apogee_m / 0.3048:.0f} ft)")

    # ------------------------------------------------------------------
    # Thread-safe deployment update (call from H5 feedback thread)
    # ------------------------------------------------------------------

    def set_deployment_level(self, level: float) -> None:
        """Set airbrake deployment fraction [0..1].  Thread-safe."""
        with self._deploy_lock:
            self._deploy_level = max(0.0, min(1.0, float(level)))

    def get_deployment_level(self) -> float:
        with self._deploy_lock:
            return self._deploy_level

    # ------------------------------------------------------------------
    # Current mass
    # ------------------------------------------------------------------

    def _current_mass(self) -> float:
        prop_remaining = self._prop_mass * max(
            0.0, 1.0 - self._cum_imp / max(self.motor.total_impulse, 1e-9)
        )
        return self._dry_mass + prop_remaining

    # ------------------------------------------------------------------
    # Single physics step
    # ------------------------------------------------------------------

    def step(self, dt: float) -> _TelemetryRow:
        """Advance simulation by dt seconds and return the telemetry row."""
        deploy   = self.get_deployment_level()
        mass     = self._current_mass()
        thrust   = self.motor.thrust_at(self._t)

        # Aerodynamic drag
        rho      = _atm_density(self._alt)
        spd      = abs(self._vel)
        mach_num = spd / max(_speed_of_sound(self._alt), 1.0)
        cd_total = self.airbrake.total_cd(deploy, mach_num)
        F_drag   = 0.5 * rho * spd * spd * cd_total * REF_AREA
        # Drag force direction opposes velocity
        F_drag_up = -np.sign(self._vel) * F_drag if self._vel != 0.0 else 0.0

        # Net upward force
        F_net = thrust + F_drag_up - mass * G0
        a_inertial = F_net / mass

        # Integrate state
        self._vel += a_inertial * dt
        self._alt  = max(0.0, self._alt + self._vel * dt)
        self._t   += dt

        # Propellant consumption (proportional to impulse delivered)
        self._cum_imp = min(self._cum_imp + thrust * dt, self.motor.total_impulse)

        # Landing detection (allow at least 3 s of flight first)
        if self._alt <= 0.0 and self._t > 3.0:
            self._done = True

        # ── Sensor data generation ────────────────────────────────────────
        # Specific force (what an accelerometer on the rocket axis reads):
        #   SF = (F_thrust + F_drag_up) / mass
        # This reads ~14 g during burn, ~0-0.5 g during coast, matching
        # the firmware's trigger thresholds (arm >5 g, fire <3 g).
        az_sf_g = (thrust + F_drag_up) / (mass * G0)

        baro_alt   = self._alt
        pressure   = _atm_pressure(baro_alt)
        temp_k     = _atm_temp(baro_alt)
        temp_c     = temp_k - 273.15

        row = _TelemetryRow(
            packet_time_s       = self._t,
            altitude_m          = baro_alt,
            pressure_kpa        = pressure / 1000.0,
            imu16_ax_g          = 0.0,
            imu16_ay_g          = 0.0,
            imu16_az_g          = float(az_sf_g),
            imu4_ax_g           = 0.0,
            imu4_ay_g           = 0.0,
            imu4_az_g           = float(az_sf_g),
            mag_x_gauss         = MAG_X_GAUSS,
            mag_y_gauss         = MAG_Y_GAUSS,
            mag_z_gauss         = MAG_Z_GAUSS,
            bmi_ax_g            = 0.0,
            bmi_ay_g            = 0.0,
            bmi_az_g            = float(az_sf_g),
            bmi_gx_dps          = 0.0,
            bmi_gy_dps          = 0.0,
            bmi_gz_dps          = 0.0,
            ext_temp_c          = float(temp_c),
            esc_valid           = 0,
            esc_temp_c          = 25,
            esc_voltage_v       = 16.8,
            esc_current_a       = 0.0,
            esc_consumption_mah = 0.0,
            esc_erpm            = 0,
            encoder_position_mm = 0.0,
            sim_deploy_level    = float(deploy),
            sim_velocity_mps    = float(self._vel),
        )
        self._log.append(row)
        return row

    # ------------------------------------------------------------------
    # Generator (for HILInjector / hil_gui.py)
    # ------------------------------------------------------------------

    def generate_packets(
        self, rate_hz: float = 50.0
    ) -> Iterator[Tuple[PacketFields, float]]:
        """
        Yield (PacketFields, dt) until landing.

        This is the generator used by HILInjector in hil_gui.py.
        dt is the nominal wall-clock delay between packets (= 1/rate_hz).
        """
        dt = 1.0 / rate_hz
        while not self._done:
            row = self.step(dt)
            pf  = PacketFields(
                altitude_m          = row.altitude_m,
                pressure_pa         = int(row.pressure_kpa * 1000.0),
                imu16_ax_g          = row.imu16_ax_g,
                imu16_ay_g          = row.imu16_ay_g,
                imu16_az_g          = row.imu16_az_g,
                imu4_ax_g           = row.imu4_ax_g,
                imu4_ay_g           = row.imu4_ay_g,
                imu4_az_g           = row.imu4_az_g,
                mag_x_gauss         = row.mag_x_gauss,
                mag_y_gauss         = row.mag_y_gauss,
                mag_z_gauss         = row.mag_z_gauss,
                bmi_ax_g            = row.bmi_ax_g,
                bmi_ay_g            = row.bmi_ay_g,
                bmi_az_g            = row.bmi_az_g,
                bmi_gx_dps          = row.bmi_gx_dps,
                bmi_gy_dps          = row.bmi_gy_dps,
                bmi_gz_dps          = row.bmi_gz_dps,
                ext_temp_c          = row.ext_temp_c,
                esc_valid           = bool(row.esc_valid),
                esc_temp_c          = row.esc_temp_c,
                esc_voltage_mv      = int(row.esc_voltage_v * 1000.0),
                esc_current_ma      = int(row.esc_current_a * 1000.0),
                esc_consumption_mah = int(row.esc_consumption_mah),
                esc_erpm            = row.esc_erpm,
                encoder_position_um = 0,
            )
            yield pf, dt

    # ------------------------------------------------------------------
    # CSV export  (matches hil_gui.py csv_flight() reader format)
    # ------------------------------------------------------------------

    def save_csv(self, output_path: str | Path = "hil_flight_log.csv") -> None:
        """Save the telemetry log to a CSV file."""
        output_path = Path(output_path)
        with output_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            for row in self._log:
                writer.writerow(row.__dict__)
        print(f"[HIL sim] Saved {len(self._log)} rows → {output_path}")

    # ------------------------------------------------------------------
    # Quick stats (call after simulation completes)
    # ------------------------------------------------------------------

    def print_summary(self) -> None:
        if not self._log:
            print("[HIL sim] No data.")
            return
        alts = [r.altitude_m for r in self._log]
        max_alt   = max(alts)
        max_alt_t = self._log[alts.index(max_alt)].packet_time_s
        vels      = [r.sim_velocity_mps for r in self._log]
        max_vel   = max(vels)
        print(f"[HIL sim] Duration     : {self._log[-1].packet_time_s:.2f} s")
        print(f"[HIL sim] Max altitude : {max_alt:.1f} m  ({max_alt/0.3048:.0f} ft AGL)  "
              f"at t = {max_alt_t:.2f} s")
        print(f"[HIL sim] Max velocity : {max_vel:.1f} m/s")
        print(f"[HIL sim] Target apogee: {self.target_apogee_m:.0f} m  "
              f"({self.target_apogee_m/0.3048:.0f} ft)")


# ---------------------------------------------------------------------------
# Quick standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time

    print("Running standalone HIL sim (no serial, no H5 feedback) …\n")
    sim = HILFlightSim()

    # Simulate airbrakes deploying at 70 % after burnout (for testing)
    burnout = sim.motor.burnout_time
    t_start = time.perf_counter()

    for pf, dt in sim.generate_packets(50.0):
        if sim._t > burnout + 0.5:
            sim.set_deployment_level(0.70)  # 70 % deployment after burnout

    elapsed = time.perf_counter() - t_start
    sim.print_summary()
    sim.save_csv("hil_l2200_standalone_test.csv")
    print(f"\nSimulated {sim._t:.1f} s in {elapsed:.2f} s wall-clock time.")
