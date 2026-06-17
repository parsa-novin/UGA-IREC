"""
hil_l2200_sim.py — RocketPy-based HIL flight simulator.

Runs the full mags.py flight model (AeroTech L2200G motor, actual rocket geometry,
Deployment_Fits_Output Mach-Cd tables) to pre-compute a high-fidelity 3D trajectory,
then streams sensor packets derived from that trajectory into the H5 HIL injector.

The H5 is the airbrake controller.  This module provides the sensor data; the H5
processes it and outputs commanded deployment angles.  Those angles are accepted via
set_deployment_level() and logged for comparison, but the pre-computed physics
trajectory is not altered in real time (the same way actual sensor hardware works).

Public API (unchanged from previous version)
--------------------------------------------
    sim = HILFlightSim()
    sim.set_deployment_level(angle_deg / 70.0)    # called from H5 feedback thread
    for pkt, dt in sim.generate_packets(50.0):    # called by HILInjector
        serial.write(build_packet(pkt))
    sim.save_csv("hil_flight_log.csv")
    sim.print_summary()
"""

from __future__ import annotations

import csv
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_BASE_DIR = Path(__file__).resolve().parent

DEFAULT_MOTOR_FILE    = _BASE_DIR / "m3400.eng"
DEFAULT_DRAG_CSV_DIR  = _BASE_DIR / "Deployment_Fits_Output"
DEFAULT_BASE_DRAG_CSV = _BASE_DIR / "hell.csv"

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

G0    = 9.80665    # m/s²
R_AIR = 287.05     # J/(kg·K)

# Magnetic field reference — Midland TX area
MAG_X_GAUSS =  0.18
MAG_Y_GAUSS =  0.02
MAG_Z_GAUSS =  0.46

# ---------------------------------------------------------------------------
# Telemetry row  (CSV schema compatible with hil_gui.py csv_flight() reader)
# ---------------------------------------------------------------------------

@dataclass
class _TelemetryRow:
    packet_time_s:       float
    altitude_m:          float
    pressure_kpa:        float
    imu16_ax_g:          float
    imu16_ay_g:          float
    imu16_az_g:          float
    imu4_ax_g:           float
    imu4_ay_g:           float
    imu4_az_g:           float
    mag_x_gauss:         float
    mag_y_gauss:         float
    mag_z_gauss:         float
    bmi_ax_g:            float
    bmi_ay_g:            float
    bmi_az_g:            float
    bmi_gx_dps:          float
    bmi_gy_dps:          float
    bmi_gz_dps:          float
    ext_temp_c:          float
    esc_valid:           int
    esc_temp_c:          int
    esc_voltage_v:       float
    esc_current_a:       float
    esc_consumption_mah: float
    esc_erpm:            int
    encoder_position_mm: float
    sim_deploy_level:    float
    sim_velocity_mps:    float


_CSV_FIELDS = list(_TelemetryRow.__dataclass_fields__.keys())


# ---------------------------------------------------------------------------
# PacketFields  (matches hil_gui.py exactly)
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
    cam_current_ma:      float = 0.0
    batt_voltage_mv:     int   = 16800


# ---------------------------------------------------------------------------
# HIL flight simulator
# ---------------------------------------------------------------------------

class HILFlightSim:
    """
    Pre-computes a full RocketPy 6-DOF flight (mags.py model) and replays it
    as real-time sensor packets for the H5 HIL injector.

    Specific-force mapping (what the on-board accelerometers measure):
        SF_x/y  = a_body_x/y / G0          [g]   — lateral, no gravity component
        SF_z    = (a_body_z + G0) / G0     [g]   — axial, gravity removed
    Valid for near-vertical flights (inclination ≥ ~85°).

    Angular rate from w1/w2/w3 (rad/s) is converted to deg/s for the gyro channels.
    """

    def __init__(
        self,
        motor_file:       str | Path = DEFAULT_MOTOR_FILE,
        drag_csv_dir:     str | Path = DEFAULT_DRAG_CSV_DIR,
        base_drag_csv:    str | Path = DEFAULT_BASE_DRAG_CSV,
        target_apogee_m:  float      = 3048.0,
        inclination_deg:  float      = 89.0,
        rail_length_m:    float      = 5.5,
    ):
        print("[HIL sim] Running full RocketPy simulation using mags.py model …")
        print("[HIL sim] (first run may take ~10–20 s)")

        # ── ensure mags.py is importable (same directory) ──────────────────
        if str(_BASE_DIR) not in sys.path:
            sys.path.insert(0, str(_BASE_DIR))
        import mags as _m
        import rocketpy

        # ── environment: site coordinates, calm-air atmosphere ──────────────
        env = rocketpy.Environment(
            latitude=_m.DEFAULT_LATITUDE,
            longitude=_m.DEFAULT_LONGITUDE,
            elevation=_m.DEFAULT_ELEVATION,
        )
        env.set_date((2026, 6, 16, 10))
        env.set_atmospheric_model(type="custom_atmosphere", wind_u=0.0, wind_v=0.0)
        elev = float(_m.DEFAULT_ELEVATION)

        # ── rocket: full geometry, no autonomous controller ─────────────────
        # The H5 is the airbrake controller; we don't add a sim-side one.
        drag_table = _m.load_base_drag_curve(str(base_drag_csv))
        rocket = _m.build_rocket(
            drag_table     = drag_table,
            motor_file     = motor_file,
            airbrake_model = None,    # H5 commands flaps — no sim controller
            elevation      = elev,
        )

        # ── run the full 6-DOF RocketPy simulation ──────────────────────────
        flight = rocketpy.Flight(
            rocket      = rocket,
            environment = env,
            rail_length = rail_length_m,
            inclination = inclination_deg,
            heading     = 0.0,
        )

        # ── sample at 200 Hz for smooth interpolation ───────────────────────
        t_full  = np.array(flight.time)
        t_dense = np.arange(0.0, float(t_full[-1]) + 0.005, 0.005)  # 200 Hz

        alt_asl = np.array(flight.altitude(t_dense))
        alt_agl = np.clip(alt_asl - elev, 0.0, None)

        press   = np.array(flight.pressure(t_dense))          # Pa
        rho     = np.array(flight.density(t_dense))            # kg/m³
        ax_b    = np.array(flight.ax_body_frame(t_dense))     # m/s²
        ay_b    = np.array(flight.ay_body_frame(t_dense))
        az_b    = np.array(flight.az_body_frame(t_dense))
        vz      = np.array(flight.vz(t_dense))                # m/s  (inertial vertical)
        w1      = np.array(flight.w1(t_dense))                # rad/s body angular rate
        w2      = np.array(flight.w2(t_dense))
        w3      = np.array(flight.w3(t_dense))

        # ── specific force (accelerometer reading) ──────────────────────────
        # Body frame: +z from tail to nose.  Gravity projects as -G0 onto z
        # when rocket is vertical, so SF_z = a_body_z + G0.
        self._t          = t_dense
        self._alt_agl    = alt_agl
        self._press      = press
        self._temp_c     = press / (np.maximum(rho, 1e-9) * R_AIR) - 273.15
        self._sf_ax      = ax_b / G0
        self._sf_ay      = ay_b / G0
        self._sf_az      = (az_b + G0) / G0
        self._vz         = vz
        self._gx_dps     = w1 * (180.0 / np.pi)
        self._gy_dps     = w2 * (180.0 / np.pi)
        self._gz_dps     = w3 * (180.0 / np.pi)

        # ── stop replay just after the rocket descends past drogue altitude ─
        # (avoids streaming 10+ min of parachute descent)
        apogee_idx = int(np.argmax(alt_agl))
        post_drop  = np.where(alt_agl[apogee_idx:] < 200.0)[0]
        if len(post_drop):
            self._t_stop = float(t_dense[apogee_idx + post_drop[0]])
        else:
            self._t_stop = float(t_dense[-1])

        # ── metadata ────────────────────────────────────────────────────────
        self.target_apogee_m = target_apogee_m
        self._apogee_agl     = float(flight.apogee) - elev

        # thread-safe deployment feedback from H5
        self._deploy_lock  = threading.Lock()
        self._deploy_level = 0.0
        self._done         = False
        self._log: List[_TelemetryRow] = []

        print(f"[HIL sim] Motor    : {motor_file.name if isinstance(motor_file, Path) else motor_file}")
        print(f"[HIL sim] Apogee   : {self._apogee_agl:.0f} m AGL "
              f"({self._apogee_agl / 0.3048:.0f} ft)  —  target {target_apogee_m:.0f} m "
              f"({target_apogee_m / 0.3048:.0f} ft)")
        print(f"[HIL sim] Replay   : 0 → {self._t_stop:.1f} s  "
              f"({len(t_dense)} points at 200 Hz sampled, "
              f"replay capped at 200 m AGL descent)")

    # ── thread-safe deployment update ──────────────────────────────────────

    def set_deployment_level(self, level: float) -> None:
        """Accept commanded deployment fraction [0..1] from the H5 feedback thread."""
        with self._deploy_lock:
            self._deploy_level = float(np.clip(level, 0.0, 1.0))

    def get_deployment_level(self) -> float:
        with self._deploy_lock:
            return self._deploy_level

    # ── packet generator ───────────────────────────────────────────────────

    def generate_packets(
        self, rate_hz: float = 50.0
    ) -> Iterator[Tuple[PacketFields, float]]:
        """
        Yield (PacketFields, dt) at rate_hz packets/s until the flight replay ends.
        Interpolates all sensor channels from the 200 Hz pre-sampled trajectory.
        """
        dt    = 1.0 / rate_hz
        t_sim = 0.0

        while t_sim <= self._t_stop:
            deploy = self.get_deployment_level()

            # interpolate from pre-sampled dense arrays
            alt  = float(np.interp(t_sim, self._t, self._alt_agl))
            prs  = float(np.interp(t_sim, self._t, self._press))
            tmp  = float(np.interp(t_sim, self._t, self._temp_c))
            ax   = float(np.interp(t_sim, self._t, self._sf_ax))
            ay   = float(np.interp(t_sim, self._t, self._sf_ay))
            az   = float(np.interp(t_sim, self._t, self._sf_az))
            vz   = float(np.interp(t_sim, self._t, self._vz))
            gx   = float(np.interp(t_sim, self._t, self._gx_dps))
            gy   = float(np.interp(t_sim, self._t, self._gy_dps))
            gz   = float(np.interp(t_sim, self._t, self._gz_dps))

            pf = PacketFields(
                altitude_m          = alt,
                pressure_pa         = int(prs),
                imu16_ax_g          = ax,   imu16_ay_g  = ay,   imu16_az_g  = az,
                imu4_ax_g           = ax,   imu4_ay_g   = ay,   imu4_az_g   = az,
                mag_x_gauss         = MAG_X_GAUSS,
                mag_y_gauss         = MAG_Y_GAUSS,
                mag_z_gauss         = MAG_Z_GAUSS,
                bmi_ax_g            = ax,   bmi_ay_g    = ay,   bmi_az_g    = az,
                bmi_gx_dps          = gx,   bmi_gy_dps  = gy,   bmi_gz_dps  = gz,
                ext_temp_c          = tmp,
                esc_valid           = False,
                esc_temp_c          = 25,
                esc_voltage_mv      = 16800,
                esc_current_ma      = 0,
                esc_consumption_mah = 0,
                esc_erpm            = 0,
                encoder_position_um = 0,
            )

            self._log.append(_TelemetryRow(
                packet_time_s       = t_sim,
                altitude_m          = alt,
                pressure_kpa        = prs / 1000.0,
                imu16_ax_g          = ax,   imu16_ay_g          = ay,   imu16_az_g = az,
                imu4_ax_g           = ax,   imu4_ay_g           = ay,   imu4_az_g  = az,
                mag_x_gauss         = MAG_X_GAUSS,
                mag_y_gauss         = MAG_Y_GAUSS,
                mag_z_gauss         = MAG_Z_GAUSS,
                bmi_ax_g            = ax,   bmi_ay_g            = ay,   bmi_az_g   = az,
                bmi_gx_dps          = gx,   bmi_gy_dps          = gy,   bmi_gz_dps = gz,
                ext_temp_c          = tmp,
                esc_valid           = 0,
                esc_temp_c          = 25,
                esc_voltage_v       = 16.8,
                esc_current_a       = 0.0,
                esc_consumption_mah = 0.0,
                esc_erpm            = 0,
                encoder_position_mm = 0.0,
                sim_deploy_level    = deploy,
                sim_velocity_mps    = vz,
            ))

            yield pf, dt
            t_sim += dt

        self._done = True

    # ── CSV export ─────────────────────────────────────────────────────────

    def save_csv(self, output_path: str | Path = "hil_flight_log.csv") -> None:
        output_path = Path(output_path)
        with output_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDS)
            writer.writeheader()
            for row in self._log:
                writer.writerow(row.__dict__)
        print(f"[HIL sim] Saved {len(self._log)} rows → {output_path}")

    # ── summary ────────────────────────────────────────────────────────────

    def print_summary(self) -> None:
        if not self._log:
            print("[HIL sim] No data.")
            return
        alts  = [r.altitude_m     for r in self._log]
        vels  = [r.sim_velocity_mps for r in self._log]
        deps  = [r.sim_deploy_level for r in self._log]
        max_alt   = max(alts)
        max_alt_t = self._log[alts.index(max_alt)].packet_time_s
        max_vel   = max(vels)
        max_dep   = max(deps)
        print(f"[HIL sim] Duration     : {self._log[-1].packet_time_s:.2f} s")
        print(f"[HIL sim] Max altitude : {max_alt:.1f} m  ({max_alt/0.3048:.0f} ft AGL)  "
              f"at t = {max_alt_t:.2f} s")
        print(f"[HIL sim] Max velocity : {max_vel:.1f} m/s")
        print(f"[HIL sim] Max H5 deploy: {max_dep:.3f}  ({max_dep * 70:.1f}°)")
        print(f"[HIL sim] Target apogee: {self.target_apogee_m:.0f} m  "
              f"({self.target_apogee_m/0.3048:.0f} ft)")


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import time

    print("Running standalone HIL sim (no serial, no H5 feedback) …\n")
    sim = HILFlightSim()

    t0 = time.perf_counter()
    n  = 0
    for pf, dt in sim.generate_packets(50.0):
        n += 1

    elapsed = time.perf_counter() - t0
    sim.print_summary()
    sim.save_csv("hil_l2200_standalone_test.csv")
    print(f"\nStreamed {n} packets ({n/50.0:.1f} s sim) in {elapsed:.2f} s wall-clock time.")
