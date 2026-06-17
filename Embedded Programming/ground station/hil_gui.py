"""
Hardware-In-the-Loop GUI for the STM32H5 airbrakes controller.

Builds and transmits aggregate packets over UART exactly as the STM32C5 would.
Three data sources are supported:

  synth    — built-in synthetic vertical-flight trajectory (no dependencies)
  csv      — replay a CSV log (same CSV_FIELDS as ground-station logger)
  rocketpy — live RocketPy-based simulation using the AeroTech L2200G motor
             and actual Deployment_Fits_Output/ drag CSVs.  Airbrake commands
             received back from the H5 ("angle=X.Xdeg" in the [HIL] status
             line) update the simulation drag in real time.

When no serial port is selected (or "(sim)" is chosen) the app runs in
simulation mode: a software replica of the H5 state machine generates
responses and the flap encoder tracks the commanded angle with a lag.

Usage:
  python hil_gui.py
  python hil_gui.py --port COM4 --source rocketpy
  python hil_gui.py --port COM4 --source csv --file flight_log.csv

H5 firmware requirement (rocketpy / real-H5 mode):
  Rebuild with HIL_MODE defined in hil_config.h and connect USART2
  (PA2/PA3, 115200 baud) to this PC.
"""

import argparse
import csv
import dataclasses
import math
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import matplotlib
matplotlib.use("TkAgg")
import serial
from serial.tools import list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ── Constants ─────────────────────────────────────────────────────────────────
AGGREGATE_HEADER = 0xA55A
UPSTREAM_HEADER  = 0xAA55
PACKET_LEN       = 108
DEFAULT_BAUD     = 115200
DEFAULT_RATE_HZ  = 50.0
HISTORY_LEN      = 800
UI_REFRESH_MS    = 33

AGGREGATE_FORMAT = "<" + "H" + "Hii" + "f" * 16 + "B" + "B" + "B" + "I" + "I" + "I" + "I" + "i" + "f" + "I" + "B"
PACKET_STRUCT    = struct.Struct(AGGREGATE_FORMAT)

# Path to the RocketPy directory (one level up, then RocketPy/)
_ROCKETPY_DIR = Path(__file__).resolve().parent.parent.parent / "RocketPy"

# Maximum airbrake deployment angle in the H5 firmware (must match
# AIRBRAKE_MAX_ANGLE_DEG in airbrake_control.h = 70°)
H5_MAX_ANGLE_DEG = 70.0

FLIGHT_STATE_COLORS = {
    "IDLE":             "#7f8fb2",
    "ARMED":            "#ffd479",
    "BOOST":            "#ff6b6b",
    "COAST":            "#9ff1c7",
    "AIRBRAKES_ACTIVE": "#8bd3ff",
    "APOGEE":           "#d1b3ff",
    "DESCENT":          "#ffb366",
}


# ── Packet building ───────────────────────────────────────────────────────────

def _xor(data: bytes) -> int:
    v = 0
    for b in data:
        v ^= b
    return v


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class PacketFields:
    altitude_m: float        = 0.0
    pressure_pa: int         = 101325
    imu16_ax_g: float        = 0.0
    imu16_ay_g: float        = 0.0
    imu16_az_g: float        = 1.0
    imu4_ax_g: float         = 0.0
    imu4_ay_g: float         = 0.0
    imu4_az_g: float         = 1.0
    mag_x_gauss: float       = 0.0
    mag_y_gauss: float       = 0.0
    mag_z_gauss: float       = 0.5
    bmi_ax_g: float          = 0.0
    bmi_ay_g: float          = 0.0
    bmi_az_g: float          = 1.0
    bmi_gx_dps: float        = 0.0
    bmi_gy_dps: float        = 0.0
    bmi_gz_dps: float        = 0.0
    ext_temp_c: float        = 20.0
    esc_valid: bool          = False
    esc_temp_c: int          = 25
    esc_voltage_mv: int      = 16800
    esc_current_ma: int      = 0
    esc_consumption_mah: int = 0
    esc_erpm: int            = 0
    encoder_position_um: int = 0
    cam_current_ma: float    = 0.0
    batt_voltage_mv: int     = 16800


def build_packet(f: PacketFields) -> bytes:
    altitude_cm = int(f.altitude_m * 100)
    imu = (
        f.imu16_ax_g, f.imu16_ay_g, f.imu16_az_g,
        f.imu4_ax_g,  f.imu4_ay_g,  f.imu4_az_g,
        f.mag_x_gauss, f.mag_y_gauss, f.mag_z_gauss,
        f.bmi_ax_g,  f.bmi_ay_g,  f.bmi_az_g,
        f.bmi_gx_dps, f.bmi_gy_dps, f.bmi_gz_dps,
        f.ext_temp_c,
    )
    # upstream_checksum = xor(packet[4:76]) = altitude(4) + pressure(4) + 16 floats(64)
    up_cs = _xor(struct.pack("<ii" + "f" * 16, altitude_cm, f.pressure_pa, *imu))
    pkt = bytearray(PACKET_STRUCT.pack(
        AGGREGATE_HEADER, UPSTREAM_HEADER,
        altitude_cm, f.pressure_pa, *imu,
        up_cs,
        int(f.esc_valid), f.esc_temp_c,
        f.esc_voltage_mv, f.esc_current_ma,
        f.esc_consumption_mah, f.esc_erpm,
        f.encoder_position_um,
        f.cam_current_ma, f.batt_voltage_mv,
        0,
    ))
    pkt[-1] = _xor(pkt[2:-1])
    return bytes(pkt)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _enc_to_angle(d_mm: float) -> float:
    if d_mm <= 0.0:
        return 0.0
    a = -0.000513 * d_mm ** 3 + 0.010615 * d_mm ** 2 + 2.3199 * d_mm - 0.4986
    return _clamp(a, 0.0, 70.0)


def _angle_to_enc_approx(deg: float) -> float:
    """Approximate inverse (linear term only) for simulation feedback."""
    return _clamp(deg / 2.3199, 0.0, 50.0)


def _mag3(x, y, z) -> float:
    return math.sqrt(x * x + y * y + z * z)


# ── Data sources ──────────────────────────────────────────────────────────────

def synthetic_flight(rate_hz: float):
    dt = 1.0 / rate_hz
    t = alt = vel = cons = 0.0
    while True:
        accel = (140.0 - t * 10.0) if t < 3.0 else -9.81
        vel  += accel * dt
        alt   = max(0.0, alt + vel * dt)
        az_g  = accel / 9.81
        pa    = int(101325.0 * math.exp(-alt / 8500.0))
        on    = t < 3.0
        curr  = int(_clamp(abs(accel) * 40, 0, 8000)) if on else 0
        cons += curr * dt / 3600.0
        yield dataclasses.replace(PacketFields(),
            altitude_m          = alt,
            pressure_pa         = pa,
            imu16_az_g          = az_g,
            imu4_az_g           = az_g,
            bmi_az_g            = az_g,
            mag_x_gauss         = 0.18,
            mag_y_gauss         = 0.02,
            mag_z_gauss         = 0.46,
            ext_temp_c          = 20.0 - alt * 0.006,
            esc_valid           = on,
            esc_temp_c          = int(_clamp(25 + curr / 200, 25, 80)),
            esc_voltage_mv      = int(_clamp(16800 - cons * 2, 14000, 16800)),
            esc_current_ma      = curr,
            esc_consumption_mah = int(cons),
            esc_erpm            = curr * 50 if on else 0,
        ), dt
        t += dt
        if alt == 0.0 and t > 15.0:
            break


def csv_flight(path: Path, rate_hz: float):
    with open(path, newline="", encoding="utf-8") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError(f"CSV {path} is empty")
    for i, row in enumerate(rows):
        t0 = float(row["packet_time_s"])
        t1 = float(rows[i + 1]["packet_time_s"]) if i + 1 < len(rows) else t0 + 1.0 / rate_hz
        enc = float(row["encoder_position_mm"])
        yield dataclasses.replace(PacketFields(),
            altitude_m          = float(row["altitude_m"]),
            pressure_pa         = int(float(row["pressure_kpa"]) * 1000),
            imu16_ax_g          = float(row["imu16_ax_g"]),
            imu16_ay_g          = float(row["imu16_ay_g"]),
            imu16_az_g          = float(row["imu16_az_g"]),
            imu4_ax_g           = float(row["imu4_ax_g"]),
            imu4_ay_g           = float(row["imu4_ay_g"]),
            imu4_az_g           = float(row["imu4_az_g"]),
            mag_x_gauss         = float(row["mag_x_gauss"]),
            mag_y_gauss         = float(row["mag_y_gauss"]),
            mag_z_gauss         = float(row["mag_z_gauss"]),
            bmi_ax_g            = float(row["bmi_ax_g"]),
            bmi_ay_g            = float(row["bmi_ay_g"]),
            bmi_az_g            = float(row["bmi_az_g"]),
            bmi_gx_dps          = float(row["bmi_gx_dps"]),
            bmi_gy_dps          = float(row["bmi_gy_dps"]),
            bmi_gz_dps          = float(row["bmi_gz_dps"]),
            ext_temp_c          = float(row["ext_temp_c"]),
            esc_valid           = bool(int(row["esc_valid"])),
            esc_temp_c          = int(float(row["esc_temp_c"])),
            esc_voltage_mv      = int(float(row["esc_voltage_v"]) * 1000),
            esc_current_ma      = int(float(row["esc_current_a"]) * 1000),
            esc_consumption_mah = int(float(row["esc_consumption_mah"])),
            esc_erpm            = int(float(row["esc_erpm"])),
            encoder_position_um = int(enc * 1000),
        ), t1 - t0


# ── Simulated H5 state machine ────────────────────────────────────────────────

class SimulatedH5:
    """
    Software replica of the H5 airbrakes controller for simulation mode.
    process() consumes a PacketFields, advances the internal state machine,
    and returns (ascii_messages, updated_pkt) where updated_pkt has a simulated
    encoder position that lags behind the commanded angle.
    """

    def __init__(self):
        self.state            = "IDLE"
        self.tick             = 0
        self.prev_alt         = 0.0
        self.cmd_angle_deg    = 0.0
        self.sim_enc_mm       = 0.0   # first-order lag model
        self._lag_alpha       = 0.04  # per-tick tracking speed

    def process(self, f: PacketFields) -> tuple[list[str], PacketFields]:
        self.tick += 1
        alt = f.altitude_m
        az  = abs(f.imu16_az_g)
        msgs: list[str] = []

        old = self.state
        if self.state == "IDLE":
            if az > 3.5:
                self.state = "BOOST"
        elif self.state == "BOOST":
            if az < 2.5 and alt > 30.0:
                self.state = "COAST"
        elif self.state == "COAST":
            self.cmd_angle_deg = _clamp((alt - 120.0) * 0.13, 0.0, 70.0)
            if alt < self.prev_alt - 8.0 and alt > 80.0:
                self.state = "APOGEE"
        elif self.state == "APOGEE":
            self.cmd_angle_deg = 0.0
            self.state = "DESCENT"
        elif self.state == "DESCENT":
            if alt < 5.0 and self.prev_alt > 5.0:
                self.state = "IDLE"

        if old != self.state:
            msgs.append(f"STATE → {self.state}")

        if self.tick % 25 == 0:
            msgs.append(
                f"TLM  state={self.state}  alt={alt:.0f}m  "
                f"az={f.imu16_az_g:+.2f}g  cmd={self.cmd_angle_deg:.1f}deg"
            )
        if self.state in ("COAST", "AIRBRAKES_ACTIVE") and self.tick % 10 == 0:
            erpm = int(self.cmd_angle_deg * 820)
            msgs.append(f"ABCMD  angle={self.cmd_angle_deg:.1f}deg  erpm_target={erpm}")
        if self.tick % 100 == 0:
            msgs.append(f"ACK  pkt={self.tick}  chk=OK  uptime={self.tick * 20}ms")

        # First-order lag: simulated encoder tracks commanded angle
        target_enc_mm = _angle_to_enc_approx(self.cmd_angle_deg)
        self.sim_enc_mm += (target_enc_mm - self.sim_enc_mm) * self._lag_alpha

        updated = dataclasses.replace(f, encoder_position_um=int(self.sim_enc_mm * 1000))
        self.prev_alt = alt
        return msgs, updated


# ── H5 control-loop mirror ────────────────────────────────────────────────────

class H5ControlLoop:
    """Python mirror of the H5 firmware control stack.

    Processes the same PacketFields sent to the H5 and produces matching
    estimated state + airbrake command.  All constants mirror the firmware
    header files exactly so outputs should converge to the same values.

    flight_trigger.h  → ARM_G=5.0, FIRE_G=3.0
    flight_estimator.h → KF tuning, atmosphere model, drag table
    airbrake_control.h → KP=1/300, DEADBAND=5m, MAX_STEP=0.05
    """

    # Atmosphere (flight_estimator.c)
    G0     = 9.80665
    RHO0   = 1.225
    T0     = 288.15
    L_KPM  = 0.0065
    R_AIR  = 287.05
    GAMMA  = 1.4

    # Rocket parameters (flight_estimator.h)
    PROP_MASS  = 2.866
    CASE_MASS  = 1.608
    BODY_MASS  = 18.7
    M0         = 18.7 + 1.608 + 2.866   # 23.174 kg
    M_DRY      = 18.7 + 1.608            # 20.308 kg
    TOTAL_IMP  = 5910.0
    REF_AREA   = 0.019284

    # Drag table (flight_estimator.c)
    CD_MACH_BP = [0.00, 0.10, 0.30, 0.50, 0.70, 0.85, 0.95, 1.05, 1.20, 1.50, 2.00]
    CD_BODY_BP = [0.45, 0.45, 0.43, 0.42, 0.44, 0.50, 0.65, 0.75, 0.62, 0.52, 0.48]
    CD_AB_FULL = 0.30

    # KF tuning (flight_estimator.c)
    KF_Q_H = 0.0001
    KF_Q_V = 0.01
    KF_R   = 0.25
    KF_P0  = 100.0

    BARO_CAL_N = 50
    PRED_DT    = 0.05
    PRED_TMAX  = 90.0
    APOGEE_V   = -2.0
    MAX_DT     = 0.10

    # Flight trigger (flight_trigger.h)
    ARM_G  = 5.0
    FIRE_G = 3.0

    # Airbrake controller (airbrake_control.h)
    TARGET_ALT  = 3048.0      # default 10 000 ft
    KP          = 1.0 / 300.0
    DEADBAND    = 5.0
    MAX_STEP    = 0.05
    CTRL_PERIOD = 0.050       # 20 Hz
    MAX_ANGLE   = 70.0        # deg

    def __init__(self):
        self._last_t: float | None = None
        self.reset()

    def reset(self):
        self._phase         = "IDLE"
        self._kf_x          = [0.0, 0.0]   # [altitude_m, velocity_mps]
        self._kf_P          = [[self.KF_P0, 0.0], [0.0, self.KF_P0]]
        self._h_gnd         = 0.0
        self._cal_n         = 0
        self._cal_s         = 0.0
        self._mass          = self.M0
        self._imp           = 0.0
        self._apogee_est    = 0.0
        self._deploy_inj    = 0.0
        self._trig          = "IDLE"
        self._ctrl_active   = False
        self._ctrl_level    = 0.0
        self._target_m      = self.TARGET_ALT
        self._last_ctrl_t   = 0.0
        self._last_t        = None
        # Public outputs
        self.altitude_m     = 0.0
        self.velocity_mps   = 0.0
        self.apogee_est_m   = 0.0
        self.deploy_level   = 0.0
        self.cmd_angle_deg  = 0.0
        self.trigger_state  = "IDLE"
        self.phase          = "IDLE"

    def set_target_ft(self, feet: float):
        self._target_m = feet * 0.3048

    # ── Atmosphere ───────────────────────────────────────────────────────────

    def _temp(self, h: float) -> float:
        return max(216.65, self.T0 - self.L_KPM * h)

    def _density(self, h: float) -> float:
        T   = self._temp(h)
        exp = self.G0 / (self.R_AIR * self.L_KPM) - 1.0
        return self.RHO0 * (T / self.T0) ** exp

    def _sos(self, h: float) -> float:
        return math.sqrt(self.GAMMA * self.R_AIR * self._temp(h))

    # ── Aerodynamics ─────────────────────────────────────────────────────────

    def _cd_body(self, mach: float) -> float:
        bp, cd = self.CD_MACH_BP, self.CD_BODY_BP
        if mach <= bp[0]:  return cd[0]
        if mach >= bp[-1]: return cd[-1]
        for i in range(1, len(bp)):
            if mach <= bp[i]:
                t = (mach - bp[i-1]) / (bp[i] - bp[i-1])
                return cd[i-1] + t * (cd[i] - cd[i-1])
        return cd[-1]

    def _drag_accel(self, v: float, h: float, deploy: float, mass: float) -> float:
        rho  = self._density(h)
        spd  = abs(v)
        mach = spd / self._sos(h)
        cd   = self._cd_body(mach) + deploy * deploy * self.CD_AB_FULL
        q    = 0.5 * rho * spd * spd
        return (q * cd * self.REF_AREA) / mass

    # ── Kalman filter ─────────────────────────────────────────────────────────

    def _kf_predict(self, dt: float, deploy: float):
        v  = self._kf_x[1]
        h  = self._kf_x[0]
        ad = self._drag_accel(v, h, deploy, self._mass)
        a  = -self.G0 - (ad if v >= 0 else -ad)
        self._kf_x[0] += v * dt + 0.5 * a * dt * dt
        self._kf_x[1] += a * dt
        P   = self._kf_P
        p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + self.KF_Q_H
        p01 = P[0][1] + dt * P[1][1]
        p10 = P[1][0] + dt * P[1][1]
        p11 = P[1][1] + self.KF_Q_V
        self._kf_P = [[p00, p01], [p10, p11]]

    def _kf_update(self, baro_h: float):
        P    = self._kf_P
        y    = baro_h - self._kf_x[0]
        Sinv = 1.0 / (P[0][0] + self.KF_R)
        K0   = P[0][0] * Sinv
        K1   = P[1][0] * Sinv
        self._kf_x[0] += K0 * y
        self._kf_x[1] += K1 * y
        self._kf_P = [
            [(1.0 - K0) * P[0][0], (1.0 - K0) * P[0][1]],
            [P[1][0] - K1 * P[0][0], P[1][1] - K1 * P[0][1]],
        ]

    # ── Apogee predictor ──────────────────────────────────────────────────────

    def _predict_apogee(self, h0: float, v0: float, deploy: float) -> float:
        h, v = h0, v0
        if v <= 0:
            return h
        t = 0.0
        while t < self.PRED_TMAX:
            ad = self._drag_accel(v, h, deploy, self._mass)
            a  = -self.G0 - ad
            v += a * self.PRED_DT
            h += v * self.PRED_DT
            t += self.PRED_DT
            if v <= 0:
                return h
        return h

    # ── Main update ───────────────────────────────────────────────────────────

    def process(self, f: "PacketFields", now: float) -> list[str]:
        """Process one packet; returns list of log messages emitted this tick."""
        if self._last_t is None:
            self._last_t = now
            dt = 0.020
        else:
            dt = min(max(now - self._last_t, 0.0001), self.MAX_DT)
            self._last_t = now

        baro_m    = f.altitude_m
        ax, ay, az = f.imu16_ax_g, f.imu16_ay_g, f.imu16_az_g
        imu_mag_g  = math.sqrt(ax * ax + ay * ay + az * az)

        msgs: list[str] = []

        # ── Flight trigger ────────────────────────────────────────────────
        if self._trig == "IDLE":
            if imu_mag_g > self.ARM_G:
                self._trig  = "ARMED"
                self._phase = "BOOST"
                self._imp   = 0.0
                self._mass  = self.M0
                self._kf_x[1] = 0.0
                msgs.append(
                    f"[GS-TRIG] ARMED — IMU16 |a| = {imu_mag_g:.2f} g "
                    f"(threshold {self.ARM_G:.1f} g)"
                )

        elif self._trig == "ARMED":
            if imu_mag_g < self.FIRE_G:
                self._trig        = "FIRED"
                self._mass        = self.M_DRY
                self._phase       = "COAST"
                self._ctrl_active = True
                self._ctrl_level  = 0.0
                self._deploy_inj  = 0.0
                self._last_ctrl_t = now
                msgs.append(
                    f"[GS-TRIG] FIRED  — IMU16 |a| = {imu_mag_g:.2f} g "
                    f"(threshold {self.FIRE_G:.1f} g) — starting control"
                )

        # ── Estimator ─────────────────────────────────────────────────────
        if self._phase == "IDLE":
            self._cal_s += baro_m
            self._cal_n += 1
            if self._cal_n >= self.BARO_CAL_N:
                self._h_gnd = self._cal_s / self._cal_n
                self._cal_s = 0.0
                self._cal_n = 0
            self._kf_x[0] = 0.0
            self._kf_x[1] = 0.0

        elif self._phase == "BOOST":
            h_agl  = baro_m - self._h_gnd
            F_est  = self._mass * imu_mag_g * self.G0
            self._imp  += F_est * dt
            prop   = min((self._imp / self.TOTAL_IMP) * self.PROP_MASS, self.PROP_MASS)
            self._mass  = self.M0 - prop
            a_in   = (imu_mag_g - 1.0) * self.G0
            self._kf_x[0] += self._kf_x[1] * dt + 0.5 * a_in * dt * dt
            self._kf_x[1] += a_in * dt
            self._kf_P[0][0] += self.KF_Q_H * 10.0
            self._kf_P[1][1] += self.KF_Q_V * 10.0
            self._kf_update(h_agl)

        elif self._phase == "COAST":
            h_agl = baro_m - self._h_gnd
            self._kf_predict(dt, self._deploy_inj)
            self._kf_update(h_agl)
            self._apogee_est = self._predict_apogee(
                self._kf_x[0], self._kf_x[1], self._deploy_inj)
            if self._kf_x[1] < self.APOGEE_V:
                self._phase = "DESCENT"

        elif self._phase == "DESCENT":
            h_agl = baro_m - self._h_gnd
            self._kf_predict(dt, 0.0)
            self._kf_update(h_agl)

        # ── Airbrake P-controller ─────────────────────────────────────────
        if self._ctrl_active:
            if self._phase == "DESCENT":
                self._ctrl_level  = 0.0
                self._deploy_inj  = 0.0
                self._ctrl_active = False
            elif self._phase == "COAST" and (now - self._last_ctrl_t) >= self.CTRL_PERIOD:
                self._last_ctrl_t = now
                error = self._apogee_est - self._target_m
                if not (-self.DEADBAND < error < self.DEADBAND):
                    cmd   = max(0.0, min(1.0, self.KP * error))
                    delta = max(-self.MAX_STEP, min(self.MAX_STEP, cmd - self._ctrl_level))
                    self._ctrl_level += delta
                self._deploy_inj = self._ctrl_level

        # ── Update public outputs ─────────────────────────────────────────
        self.altitude_m    = self._kf_x[0]
        self.velocity_mps  = self._kf_x[1]
        self.apogee_est_m  = self._apogee_est
        self.deploy_level  = self._ctrl_level
        self.cmd_angle_deg = self._ctrl_level * self.MAX_ANGLE
        self.trigger_state = self._trig
        self.phase         = self._phase

        return msgs


# ── Worker threads ────────────────────────────────────────────────────────────

class HILInjector(threading.Thread):
    def __init__(self, source_fn, rate_mult: float,
                 ser, event_queue: queue.Queue,
                 stop_event: threading.Event,
                 sim_h5: SimulatedH5 | None,
                 hil_sim=None):
        """
        hil_sim : HILFlightSim | None
            Live RocketPy simulation object.  When set, the injector passes
            the PacketFields from its generate_packets() generator directly —
            SimulatedH5 is not used.  The H5's commanded angle (parsed from
            the "[HIL] …angle=X.Xdeg" line) is fed back by the main GUI
            update loop via hil_sim.set_deployment_level().
        """
        super().__init__(daemon=True)
        self.source_fn  = source_fn
        self.rate_mult  = rate_mult
        self.ser        = ser
        self.q          = event_queue
        self.stop       = stop_event
        self.sim_h5     = sim_h5
        self.hil_sim    = hil_sim
        self.tx_count   = 0
        self.tx_rate_hz = 0.0
        self._last_t    = 0.0

    def run(self):
        try:
            self._run_inner()
        except Exception as exc:
            self.q.put(("error", f"Injector crashed: {exc}"))
        finally:
            self.q.put(("done", None))

    def _run_inner(self):
        t_start       = time.perf_counter()
        consec_errors = 0
        MAX_CONSEC_TX_ERRORS = 5

        for pkt_fields, nominal_dt in self.source_fn():
            if self.stop.is_set():
                break

            if self.hil_sim is not None:
                # RocketPy source: physics are already in pkt_fields,
                # SimulatedH5 is bypassed.  H5 feedback is applied by the
                # GUI update loop calling hil_sim.set_deployment_level().
                msgs = []
            elif self.sim_h5 is not None:
                msgs, pkt_fields = self.sim_h5.process(pkt_fields)
                for m in msgs:
                    self.q.put(("rx", m))
            else:
                msgs = []

            packet = build_packet(pkt_fields)
            if self.ser is not None:
                try:
                    self.ser.write(packet)
                    self.ser.flush()
                    consec_errors = 0
                except Exception as exc:
                    consec_errors += 1
                    if consec_errors == 1:
                        self.q.put(("error", f"TX error: {exc}"))
                    elif consec_errors == MAX_CONSEC_TX_ERRORS:
                        self.q.put(("error",
                            f"TX failing consistently — check that no other "
                            f"program (STM32CubeIDE, Tera Term, main.py) has "
                            f"the COM port open, then replug the USB adapter. "
                            f"Stopping injection."))
                        return

            self.tx_count += 1
            now = time.perf_counter()
            if self._last_t:
                dt = now - self._last_t
                r  = 1.0 / dt if dt > 0 else 0.0
                self.tx_rate_hz = 0.9 * self.tx_rate_hz + 0.1 * r
            self._last_t = now

            self.q.put(("tx", pkt_fields))

            ideal  = self.tx_count * nominal_dt / self.rate_mult
            sleep  = ideal - (now - t_start)
            if sleep > 0.001:
                time.sleep(sleep)


class SerialRxListener(threading.Thread):
    def __init__(self, ser: serial.Serial, event_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.ser  = ser
        self.q    = event_queue
        self.stop = stop_event
        self._buf = b""

    def run(self):
        while not self.stop.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            self._buf += chunk
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if text:
                    self.q.put(("rx", text))


# ── History ───────────────────────────────────────────────────────────────────

class HILHistory:
    def __init__(self, maxlen: int):
        self.times       = deque(maxlen=maxlen)
        self.altitude    = deque(maxlen=maxlen)
        self.accel_az    = deque(maxlen=maxlen)
        self.accel_mag   = deque(maxlen=maxlen)
        self.enc_mm      = deque(maxlen=maxlen)
        self.flap_actual = deque(maxlen=maxlen)
        self.flap_cmd    = deque(maxlen=maxlen)
        self.flap_gs     = deque(maxlen=maxlen)  # GS control loop command
        self.esc_curr    = deque(maxlen=maxlen)
        self.esc_volt    = deque(maxlen=maxlen)
        self._t0: float | None = None

    def add(self, f: PacketFields, cmd_angle: float, gs_angle: float):
        now = time.perf_counter()
        if self._t0 is None:
            self._t0 = now
        enc = f.encoder_position_um / 1000.0
        self.times.append(now - self._t0)
        self.altitude.append(f.altitude_m)
        self.accel_az.append(f.imu16_az_g)
        self.accel_mag.append(_mag3(f.imu16_ax_g, f.imu16_ay_g, f.imu16_az_g))
        self.enc_mm.append(enc)
        self.flap_actual.append(_enc_to_angle(enc))
        self.flap_cmd.append(cmd_angle)
        self.flap_gs.append(gs_angle)
        self.esc_curr.append(f.esc_current_ma / 1000.0)
        self.esc_volt.append(f.esc_voltage_mv / 1000.0)

    def x(self):
        return list(self.times)


# ── Metric card widget ────────────────────────────────────────────────────────

class MetricCard(ttk.Frame):
    def __init__(self, master, title: str, accent: str):
        super().__init__(master, padding=(10, 7), style="Card.TFrame")
        self._val = tk.StringVar(value="--")
        self._sub = tk.StringVar(value="")
        ttk.Label(self, text=title,            style="CompactCardTitle.TLabel").pack(anchor="w")
        ttk.Label(self, textvariable=self._val, style=f"{accent}.CompactValue.TLabel").pack(anchor="w", pady=(3, 0))
        ttk.Label(self, textvariable=self._sub, style="CompactCardSub.TLabel").pack(anchor="w", pady=(2, 0))

    def set(self, value: str, sub: str = ""):
        self._val.set(value)
        self._sub.set(sub)


# ── Main application ──────────────────────────────────────────────────────────

def _available_ports() -> list[str]:
    return sorted(p.device for p in list_ports.comports())


class HILApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("UGA Spaceport — HIL Test Console")
        self.root.geometry("1700x1020")
        self.root.configure(bg="#06080d")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._eq: queue.Queue        = queue.Queue()
        self._stop                   = threading.Event()
        self._injector: HILInjector | None = None
        self._ser: serial.Serial | None    = None
        self._sim_h5                 = SimulatedH5()
        self._history                = HILHistory(HISTORY_LEN)
        self._hil_sim                = None   # HILFlightSim when source=="rocketpy"
        self._gs_ctrl                = H5ControlLoop()
        self._gs_ctrl_log_n          = 0      # packet counter for periodic GS log

        # UI state vars
        self.port_var    = tk.StringVar(value="(sim)")
        self.baud_var    = tk.IntVar(value=DEFAULT_BAUD)
        self.rate_var    = tk.DoubleVar(value=1.0)
        self.source_var  = tk.StringVar(value="synth")
        self.csv_var     = tk.StringVar(value="")

        # Runtime counters
        self._tx_n        = 0
        self._rx_n        = 0
        self._flight_state = "IDLE"
        self._cmd_angle   = 0.0

        self._configure_styles()
        self._build_ui()
        self._build_plots()

    # ─── Styles ───────────────────────────────────────────────────────────────

    def _configure_styles(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("TFrame",       background="#06080d")
        s.configure("TLabel",       background="#06080d", foreground="#edf2ff")
        s.configure("Header.TLabel",    font=("Segoe UI Semibold", 18), foreground="#f5f7ff")
        s.configure("SubHeader.TLabel", font=("Segoe UI", 10),          foreground="#aab7d5")
        s.configure("Tab.TFrame",       background="#0a111c")
        s.configure("Divider.TFrame",   background="#1a2535")
        s.configure("Card.TFrame",      background="#0f1726", relief="flat")
        s.configure("CardTitle.TLabel",        background="#0f1726", foreground="#9fb0d3", font=("Segoe UI Semibold", 10))
        s.configure("CompactCardTitle.TLabel", background="#0f1726", foreground="#9fb0d3", font=("Segoe UI Semibold", 9))
        s.configure("CompactCardSub.TLabel",   background="#0f1726", foreground="#7f8fb2", font=("Segoe UI", 8))
        for tag, color in [
            ("Blue",   "#8bd3ff"),
            ("Gold",   "#ffd479"),
            ("Green",  "#9ff1c7"),
            ("Rose",   "#ff9aa7"),
            ("Purple", "#d1b3ff"),
            ("Orange", "#ffb366"),
        ]:
            s.configure(f"{tag}.Value.TLabel",        background="#0f1726", foreground=color, font=("Bahnschrift SemiBold", 24))
            s.configure(f"{tag}.CompactValue.TLabel", background="#0f1726", foreground=color, font=("Bahnschrift SemiBold", 16))
        s.configure("TEntry",    fieldbackground="#0f1726", foreground="#edf2ff", insertcolor="#edf2ff")
        s.configure("TCombobox", fieldbackground="#0f1726", foreground="#edf2ff", selectbackground="#1b2740")
        s.configure("TButton",   background="#1b2740", foreground="#dce7ff", font=("Segoe UI Semibold", 9), padding=(8, 4))
        s.map("TButton", background=[("active", "#253352")])
        s.configure("Start.TButton", background="#163324", foreground="#9ff1c7", font=("Segoe UI Semibold", 10), padding=(10, 5))
        s.map("Start.TButton",  background=[("active", "#1e4a32"), ("disabled", "#0d1f15")])
        s.configure("Stop.TButton",  background="#3a1515", foreground="#ff9aa7", font=("Segoe UI Semibold", 10), padding=(10, 5))
        s.map("Stop.TButton",   background=[("active", "#521e1e"), ("disabled", "#1e0e0e")])

    # ─── UI Layout ────────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Header ──────────────────────────────────────────────────────────
        hdr = ttk.Frame(self.root, padding=(20, 14, 20, 0))
        hdr.pack(fill="x")
        title_row = ttk.Frame(hdr)
        title_row.pack(fill="x")
        ttk.Label(title_row, text="UGA Spaceport", style="Header.TLabel").pack(side="left")
        ttk.Label(title_row, text="  ·  HIL Test Console",
                  font=("Segoe UI", 16), foreground="#4a6080", background="#06080d").pack(side="left", pady=(2, 0))
        ttk.Label(hdr, text="Hardware-In-the-Loop injector for the STM32H5 airbrakes controller — "
                             "select a port or leave on (sim) to run fully simulated",
                  style="SubHeader.TLabel").pack(anchor="w", pady=(2, 8))

        # ── Controls ────────────────────────────────────────────────────────
        ctrl = ttk.Frame(hdr)
        ctrl.pack(fill="x", pady=(0, 10))

        def _lbl(text, padl=0):
            ttk.Label(ctrl, text=text, style="SubHeader.TLabel").pack(side="left", padx=(padl, 4))

        _lbl("Port")
        ports = _available_ports()
        self._port_combo = ttk.Combobox(
            ctrl, textvariable=self.port_var,
            values=["(sim)"] + ports, state="readonly", width=12,
        )
        self._port_combo.pack(side="left", padx=(0, 14))

        _lbl("Baud", 0)
        ttk.Entry(ctrl, textvariable=self.baud_var, width=8).pack(side="left", padx=(0, 14))

        _lbl("Rate")
        ttk.Combobox(ctrl, textvariable=self.rate_var,
                     values=[0.25, 0.5, 1.0, 2.0, 5.0, 10.0],
                     state="readonly", width=6).pack(side="left")
        ttk.Label(ctrl, text="×", style="SubHeader.TLabel").pack(side="left", padx=(2, 14))

        _lbl("Source")
        ttk.Combobox(ctrl, textvariable=self.source_var,
                     values=["synth", "csv", "rocketpy"], state="readonly", width=10).pack(side="left", padx=(0, 6))
        ttk.Entry(ctrl, textvariable=self.csv_var, width=26,
                  ).pack(side="left", padx=(0, 14))

        ttk.Button(ctrl, text="Refresh Ports", command=self._refresh_ports).pack(side="left", padx=(0, 14))

        self._start_btn = ttk.Button(ctrl, text="▶  Start Injection",
                                     style="Start.TButton", command=self._start, width=18)
        self._start_btn.pack(side="left", padx=(0, 6))
        self._stop_btn = ttk.Button(ctrl, text="■  Stop",
                                    style="Stop.TButton", command=self._stop, width=10, state="disabled")
        self._stop_btn.pack(side="left")

        # ── Status bar ───────────────────────────────────────────────────────
        sbar = ttk.Frame(self.root, style="Divider.TFrame", padding=(20, 6, 20, 6))
        sbar.pack(fill="x")

        self._conn_var  = tk.StringVar(value="Idle — press Start Injection")
        self._tx_var    = tk.StringVar(value="TX  0 pkts")
        self._rx_var    = tk.StringVar(value="RX  0 msgs")
        self._hz_var    = tk.StringVar(value="0.0 Hz")
        self._state_var = tk.StringVar(value="IDLE")

        ttk.Label(sbar, textvariable=self._conn_var, style="SubHeader.TLabel").pack(side="left")
        for v in (self._tx_var, self._rx_var, self._hz_var):
            ttk.Label(sbar, text="  |  ", style="SubHeader.TLabel").pack(side="left")
            ttk.Label(sbar, textvariable=v, style="SubHeader.TLabel").pack(side="left")

        self._state_badge = tk.Label(
            sbar, textvariable=self._state_var,
            bg="#111a2a", fg="#7f8fb2",
            font=("Bahnschrift SemiBold", 11),
            padx=14, pady=3,
        )
        self._state_badge.pack(side="right")
        ttk.Label(sbar, text="Flight State ", style="SubHeader.TLabel").pack(side="right", padx=(0, 2))

        # ── Main content split ────────────────────────────────────────────────
        body = ttk.Frame(self.root, padding=(12, 8, 12, 12))
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=0, minsize=440)
        body.rowconfigure(0, weight=1)

        self._plot_host = ttk.Frame(body, style="Tab.TFrame", padding=8)
        self._plot_host.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        right = ttk.Frame(body, style="Tab.TFrame", padding=(10, 8))
        right.grid(row=0, column=1, sticky="nsew")
        self._build_right_panel(right)

    def _build_right_panel(self, parent):
        # ── Metric cards (3 rows × 2 cols) ──────────────────────────────────
        cf = ttk.Frame(parent)
        cf.pack(fill="x", pady=(0, 8))
        cf.columnconfigure(0, weight=1)
        cf.columnconfigure(1, weight=1)

        specs = [
            ("Altitude",     "Blue",   "alt"),
            ("Accel  az",    "Gold",   "accel"),
            ("Flap Actual",  "Green",  "flap_a"),
            ("Flap Cmd",     "Purple", "flap_c"),
            ("TX Rate",      "Blue",   "txrate"),
            ("ESC Current",  "Rose",   "esccurr"),
        ]
        self._cards: dict[str, MetricCard] = {}
        for i, (title, accent, key) in enumerate(specs):
            card = MetricCard(cf, title, accent)
            card.grid(row=i // 2, column=i % 2, sticky="nsew", padx=3, pady=3)
            self._cards[key] = card

        # ── Comm log ─────────────────────────────────────────────────────────
        ttk.Label(parent, text="Communication Log", style="CardTitle.TLabel").pack(anchor="w", pady=(4, 2))

        log_frame = ttk.Frame(parent)
        log_frame.pack(fill="both", expand=True)

        self._log = tk.Text(
            log_frame,
            bg="#07101c", fg="#8fa0bc",
            font=("Consolas", 8),
            state="disabled", relief="flat", bd=0,
            selectbackground="#1b2740",
            wrap="word",
        )
        sb = ttk.Scrollbar(log_frame, command=self._log.yview)
        self._log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self._log.pack(side="left", fill="both", expand=True)

        self._log.tag_config("ts",     foreground="#2a3d55")
        self._log.tag_config("tx",     foreground="#3d5a7a")
        self._log.tag_config("rx",     foreground="#9ff1c7")
        self._log.tag_config("state",  foreground="#ffd479")
        self._log.tag_config("error",  foreground="#ff6b6b")
        self._log.tag_config("info",   foreground="#aab7d5")
        self._log.tag_config("hs_ok",  foreground="#00ff88")
        self._log.tag_config("hs_warn",foreground="#ffcc44")

        # ── Manual command entry ──────────────────────────────────────────────
        cmd_row = ttk.Frame(parent)
        cmd_row.pack(fill="x", pady=(6, 0))
        ttk.Label(cmd_row, text="Send to H5:", style="SubHeader.TLabel").pack(side="left")
        self._cmd_var = tk.StringVar()
        e = ttk.Entry(cmd_row, textvariable=self._cmd_var, width=20)
        e.pack(side="left", padx=(6, 6))
        e.bind("<Return>", lambda _: self._send_cmd())
        ttk.Button(cmd_row, text="Send", command=self._send_cmd).pack(side="left")

    # ─── Plots ────────────────────────────────────────────────────────────────

    def _build_plots(self):
        self._fig = Figure(figsize=(11, 9), facecolor="#0a111c")
        self._fig.subplots_adjust(
            left=0.07, right=0.97, top=0.96, bottom=0.06,
            hspace=0.50, wspace=0.30,
        )
        axs = [self._fig.add_subplot(2, 2, i + 1) for i in range(4)]
        ax_alt, ax_az, ax_flap, ax_esc = axs

        self._style_ax(ax_alt,  "Altitude",              "m")
        self._style_ax(ax_az,   "Axial Acceleration az", "g")
        self._style_ax(ax_flap, "Flap Angle",            "deg")
        self._style_ax(ax_esc,  "ESC Telemetry",         "A  /  V")

        # altitude
        self._ln_alt,  = ax_alt.plot([], [], color="#7cc7ff", lw=2.2)

        # accel: az solid, |a| dashed dim
        self._ln_az,   = ax_az.plot([], [], color="#ffd479", lw=2.0, label="az")
        self._ln_amag, = ax_az.plot([], [], color="#ffd47944", lw=1.3, ls="--", label="|a|")
        ax_az.axhline(0, color="#2a3d55", lw=0.8, ls=":")
        ax_az.legend(facecolor="#0a111c", edgecolor="#22324f", labelcolor="#dce7ff", fontsize=8)

        # flap: actual (encoder) vs H5 commanded vs GS control loop computed
        self._ln_fa,  = ax_flap.plot([], [], color="#9ff1c7", lw=2.2, label="Actual")
        self._ln_fc,  = ax_flap.plot([], [], color="#d1b3ff", lw=1.8, ls="--", label="H5 Cmd")
        self._ln_fgs, = ax_flap.plot([], [], color="#ff9a3c", lw=1.6, ls=":", label="GS Cmd")
        ax_flap.set_ylim(-2, 75)
        ax_flap.legend(facecolor="#0a111c", edgecolor="#22324f", labelcolor="#dce7ff", fontsize=8)

        # ESC current + voltage
        self._ln_curr, = ax_esc.plot([], [], color="#ff8ba7", lw=2.0, label="Current A")
        self._ln_volt, = ax_esc.plot([], [], color="#9ff1c7", lw=1.8, ls="--", label="Voltage V")
        ax_esc.legend(facecolor="#0a111c", edgecolor="#22324f", labelcolor="#dce7ff", fontsize=8)

        self._canvas = FigureCanvasTkAgg(self._fig, master=self._plot_host)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)

    def _style_ax(self, ax, title: str, ylabel: str):
        ax.set_facecolor("#0f1726")
        ax.grid(True, color="#1e2f47", lw=0.6, alpha=0.85)
        ax.tick_params(colors="#c9d5f0", labelsize=8)
        ax.set_title(title, color="#edf2ff", fontsize=11, pad=5, fontweight="semibold")
        ax.set_ylabel(ylabel, color="#9fb0d3", fontsize=8)
        ax.set_xlabel("t [s]", color="#6f84ac", fontsize=8)
        for sp in ax.spines.values():
            sp.set_color("#1e2f47")

    # ─── Injection control ────────────────────────────────────────────────────

    def _start(self):
        self._stop.clear()
        self._sim_h5       = SimulatedH5()
        self._gs_ctrl      = H5ControlLoop()
        self._gs_ctrl_log_n = 0
        self._history      = HILHistory(HISTORY_LEN)
        self._hil_sim      = None
        self._tx_n = self._rx_n = 0
        self._flight_state = "IDLE"
        self._cmd_angle    = 0.0

        port = self.port_var.get()
        dry  = port == "(sim)"
        self._ser = None

        if not dry:
            try:
                self._ser = serial.Serial(port, self.baud_var.get(), timeout=0.1)
                self._conn_var.set(f"Connected — {port} @ {self.baud_var.get()} baud")
            except serial.SerialException as exc:
                messagebox.showerror("Serial Error", str(exc))
                return
        else:
            ports = _available_ports()
            if len(ports) == 1:
                try:
                    self._ser = serial.Serial(ports[0], self.baud_var.get(), timeout=0.1)
                    self._conn_var.set(f"Auto-connected — {ports[0]} @ {self.baud_var.get()} baud")
                    dry = False
                except serial.SerialException:
                    pass

            if dry:
                self._conn_var.set("Simulation mode — no serial port (H5 responses are synthetic)")

        self._start_btn.config(state="disabled")
        self._stop_btn.config(state="normal")

        if self._ser is not None:
            # Real H5: run 3-step handshake on a background thread, then begin injection
            self._log_msg("Serial port open — starting H5 handshake...", "state")
            threading.Thread(target=self._run_handshake, daemon=True).start()
        else:
            # Simulation mode: skip handshake, start immediately
            self._begin_injection()

    def _run_handshake(self):
        """Background thread: 3-step handshake with H5, then kick off injection.

        Step 1 — wait for H5 to announce itself ([HIL] HELLO).
        Step 2 — send HILSYN, wait for [HIL] SYN-ACK.
        Step 3 — send HILGO,  wait for [HIL] GO.
        """
        ser     = self._ser
        leftover = b""

        def read_until(keyword: str, timeout_s: float) -> str | None:
            """Drain serial, route lines to event queue, return first line matching keyword."""
            nonlocal leftover
            deadline = time.perf_counter() + timeout_s
            while time.perf_counter() < deadline:
                if self._stop.is_set():
                    return None
                try:
                    n = max(1, ser.in_waiting or 1)
                    chunk = ser.read(min(n, 256))
                except Exception:
                    return None
                leftover += chunk
                while b"\n" in leftover:
                    line, leftover = leftover.split(b"\n", 1)
                    text = line.decode("ascii", errors="replace").strip()
                    if text:
                        self._eq.put(("rx", text))
                        if keyword in text:
                            return text
            return None

        # ── Step 1: H5 PRESENT ───────────────────────────────────────────
        self._eq.put(("info_log", "Waiting for H5 HELLO (up to 5 s)..."))
        result = read_until("[HIL] HELLO", 5.0)
        if result:
            self._eq.put(("hs_ok",  "✓  H5 PRESENT — firmware detected and online"))
        else:
            self._eq.put(("hs_warn",
                "⚠  H5 HELLO not received — confirm HIL_MODE is enabled in firmware"))

        # ── Step 2: H5 ACKNOWLEDGES GROUND STATION ───────────────────────
        try:
            ser.write(b"HILSYN\n")
            ser.flush()
        except Exception as exc:
            self._eq.put(("error", f"HILSYN send failed: {exc}"))
        result = read_until("SYN-ACK", 3.0)
        if result:
            self._eq.put(("hs_ok",  "✓  H5 ACKNOWLEDGED — ground station recognized by H5"))
        else:
            self._eq.put(("hs_warn",
                "⚠  SYN-ACK not received — H5 may be running older firmware"))

        # ── Step 3: LINK VERIFIED ─────────────────────────────────────────
        try:
            ser.write(b"HILGO\n")
            ser.flush()
        except Exception as exc:
            self._eq.put(("error", f"HILGO send failed: {exc}"))
        result = read_until("[HIL] GO", 3.0)
        if result:
            self._eq.put(("hs_ok",  "✓  H5 COMM OK — bidirectional link verified, starting injection"))
        else:
            self._eq.put(("hs_warn",
                "⚠  GO not received — starting injection anyway"))

        # Signal main thread to launch RX listener + injector
        self._eq.put(("hs_begin", None))

    def _begin_injection(self):
        """Launch the SerialRxListener and HILInjector threads."""
        if self._ser is not None:
            SerialRxListener(self._ser, self._eq, self._stop).start()

        src_fn, hil_sim = self._make_source()
        self._hil_sim = hil_sim

        sim_h5 = self._sim_h5 if (self._ser is None and hil_sim is None) else None

        self._injector = HILInjector(
            source_fn   = src_fn,
            rate_mult   = self.rate_var.get(),
            ser         = self._ser,
            event_queue = self._eq,
            stop_event  = self._stop,
            sim_h5      = sim_h5,
            hil_sim     = hil_sim,
        )
        self._injector.start()
        self._log_msg("HIL injection started.", "state")

    def _stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
        self._start_btn.config(state="normal")
        self._stop_btn.config(state="disabled")
        self._conn_var.set("Stopped")
        self._log_msg("Injection stopped.", "state")

    def _make_source(self):
        """Return (source_fn, hil_sim_or_None).

        source_fn  : callable → Iterator[(PacketFields, float)]
        hil_sim    : HILFlightSim instance when source=="rocketpy", else None
        """
        src = self.source_var.get()

        if src == "csv":
            path = Path(self.csv_var.get())
            if not path.exists():
                messagebox.showerror("CSV Error", f"File not found: {path}")
                return lambda: synthetic_flight(DEFAULT_RATE_HZ), None
            return lambda: csv_flight(path, DEFAULT_RATE_HZ), None

        if src == "rocketpy":
            # Lazy import — only requires numpy / pandas / rocketpy if selected
            try:
                if str(_ROCKETPY_DIR) not in sys.path:
                    sys.path.insert(0, str(_ROCKETPY_DIR))
                from hil_l2200_sim import HILFlightSim  # type: ignore
            except ImportError as exc:
                messagebox.showerror(
                    "RocketPy Import Error",
                    f"Could not import hil_l2200_sim:\n{exc}\n\n"
                    "Falling back to synthetic source."
                )
                return lambda: synthetic_flight(DEFAULT_RATE_HZ), None

            try:
                hil_sim = HILFlightSim()
            except Exception as exc:
                messagebox.showerror(
                    "RocketPy Init Error",
                    f"HILFlightSim failed to initialise:\n{exc}\n\n"
                    "Falling back to synthetic source."
                )
                return lambda: synthetic_flight(DEFAULT_RATE_HZ), None

            self._log_msg(
                f"RocketPy source: L2200G motor · drag CSVs loaded · "
                f"target apogee {hil_sim.target_apogee_m:.0f} m "
                f"({hil_sim.target_apogee_m / 0.3048:.0f} ft)",
                "state",
            )
            return lambda: hil_sim.generate_packets(DEFAULT_RATE_HZ), hil_sim

        # Default: synthetic
        return lambda: synthetic_flight(DEFAULT_RATE_HZ), None

    def _refresh_ports(self):
        ports = _available_ports()
        self._port_combo["values"] = ["(sim)"] + ports

    # ─── Manual command ───────────────────────────────────────────────────────

    def _send_cmd(self):
        cmd = self._cmd_var.get().strip()
        if not cmd:
            return
        sent = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write((cmd + "\n").encode("ascii", errors="ignore"))
                sent = True
            except Exception as exc:
                self._log_msg(f"Send failed: {exc}", "error")
        tag = "tx" if sent else "info"
        suffix = "" if sent else "  [sim — not transmitted]"
        self._log_msg(f"→ {cmd}{suffix}", tag)
        self._cmd_var.set("")

    # ─── Comm log ─────────────────────────────────────────────────────────────

    def _log_msg(self, text: str, tag: str = "rx"):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.config(state="normal")
        self._log.insert("end", f"[{ts}]  ", "ts")
        self._log.insert("end", text + "\n", tag)
        lines = int(self._log.index("end-1c").split(".")[0])
        if lines > 600:
            self._log.delete("1.0", "60.0")
        self._log.see("end")
        self._log.config(state="disabled")

    # ─── Update loop ──────────────────────────────────────────────────────────

    def update(self):
        dirty = False
        while True:
            try:
                kind, data = self._eq.get_nowait()
            except queue.Empty:
                break

            if kind == "tx":
                self._tx_n += 1
                now_t = time.perf_counter()

                # Run GS control loop mirror on the same packet
                gs_msgs = self._gs_ctrl.process(data, now_t)
                for m in gs_msgs:
                    self._log_msg(m, "state")

                self._history.add(data, self._cmd_angle, self._gs_ctrl.cmd_angle_deg)

                if self._tx_n % 50 == 0:
                    f = data
                    self._log_msg(
                        f"→ PKT {self._tx_n:5d}  "
                        f"alt={f.altitude_m:6.0f} m  "
                        f"az={f.imu16_az_g:+5.2f} g  "
                        f"enc={f.encoder_position_um / 1000:.1f} mm",
                        "tx",
                    )

                # Periodic GS control loop status line (every 100 packets ≈ 2 s at 50 Hz)
                self._gs_ctrl_log_n += 1
                if self._gs_ctrl_log_n % 100 == 0:
                    gc = self._gs_ctrl
                    self._log_msg(
                        f"[GS-CTRL] phase={gc.phase}  "
                        f"alt={gc.altitude_m:.0f}m  vel={gc.velocity_mps:.1f}mps  "
                        f"apogee={gc.apogee_est_m:.0f}m  "
                        f"deploy={gc.deploy_level:.3f}  angle={gc.cmd_angle_deg:.1f}deg",
                        "info",
                    )
                dirty = True

            elif kind == "rx":
                msg = str(data)
                self._rx_n += 1
                # parse flight state
                for key in FLIGHT_STATE_COLORS:
                    if key in msg:
                        self._flight_state = key
                        break
                # parse commanded angle (handles "cmd=45.1deg" or "angle=45.1deg")
                for part in msg.replace("deg", "").replace("°", "").split():
                    for prefix in ("cmd=", "angle="):
                        if part.startswith(prefix):
                            try:
                                angle_deg = float(part[len(prefix):])
                                self._cmd_angle = angle_deg
                                if self._hil_sim is not None:
                                    self._hil_sim.set_deployment_level(
                                        angle_deg / H5_MAX_ANGLE_DEG
                                    )
                            except ValueError:
                                pass
                self._log_msg(f"← {msg}", "rx")

            elif kind == "hs_ok":
                self._log_msg(str(data), "hs_ok")

            elif kind == "hs_warn":
                self._log_msg(str(data), "hs_warn")

            elif kind == "info_log":
                self._log_msg(str(data), "info")

            elif kind == "hs_begin":
                self._begin_injection()

            elif kind == "error":
                self._log_msg(str(data), "error")

            elif kind == "done":
                self._log_msg("Injection sequence complete.", "state")
                self._start_btn.config(state="normal")
                self._stop_btn.config(state="disabled")
                # Auto-save RocketPy telemetry log
                if self._hil_sim is not None:
                    ts  = datetime.now().strftime("%Y%m%d_%H%M%S")
                    csv_path = Path(__file__).parent / f"hil_rocketpy_{ts}.csv"
                    try:
                        self._hil_sim.save_csv(csv_path)
                        self._hil_sim.print_summary()
                        self._log_msg(f"RocketPy log saved → {csv_path.name}", "state")
                    except Exception as exc:
                        self._log_msg(f"CSV save failed: {exc}", "error")

        # ── Status bar ───────────────────────────────────────────────────────
        hz = self._injector.tx_rate_hz if self._injector else 0.0
        self._tx_var.set(f"TX  {self._tx_n} pkts")
        self._rx_var.set(f"RX  {self._rx_n} msgs")
        self._hz_var.set(f"{hz:.1f} Hz")
        self._state_var.set(self._flight_state)
        self._state_badge.config(fg=FLIGHT_STATE_COLORS.get(self._flight_state, "#7f8fb2"))

        # ── Metric cards ─────────────────────────────────────────────────────
        if self._history.times:
            alt  = self._history.altitude[-1]
            az   = self._history.accel_az[-1]
            fa   = self._history.flap_actual[-1]
            fc   = self._history.flap_cmd[-1]
            curr = self._history.esc_curr[-1]
            volt = self._history.esc_volt[-1]
            enc  = self._history.enc_mm[-1]
            mag  = self._history.accel_mag[-1]
            self._cards["alt"    ].set(f"{alt:,.0f} m")
            self._cards["accel"  ].set(f"{az:+.2f} g",  f"|a| {mag:.2f} g")
            self._cards["flap_a" ].set(f"{fa:.1f}°",    f"{enc:.2f} mm travel")
            self._cards["flap_c" ].set(f"{fc:.1f}°",    "commanded by H5")
            self._cards["txrate" ].set(f"{hz:.1f} Hz",  f"{self._tx_n} total")
            self._cards["esccurr"].set(f"{curr:.2f} A", f"{volt:.2f} V bus")

        # ── Plots ─────────────────────────────────────────────────────────────
        if dirty and self._history.times:
            self._redraw_plots()

        self.root.after(UI_REFRESH_MS, self.update)

    def _redraw_plots(self):
        x  = self._history.x()
        if not x:
            return
        win = max(0.0, x[-1] - 40.0), max(40.0, x[-1])

        def _update(line, y):
            line.set_data(x, list(y))
            ax = line.axes
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(*win)

        _update(self._ln_alt,  self._history.altitude)
        _update(self._ln_az,   self._history.accel_az)
        _update(self._ln_amag, self._history.accel_mag)
        _update(self._ln_fa,   self._history.flap_actual)
        _update(self._ln_fc,   self._history.flap_cmd)
        _update(self._ln_fgs,  self._history.flap_gs)
        _update(self._ln_curr, self._history.esc_curr)
        _update(self._ln_volt, self._history.esc_volt)

        # Keep flap axis anchored even when both series are near zero
        fa_ax = self._ln_fa.axes
        ylo, yhi = fa_ax.get_ylim()
        if yhi < 5:
            fa_ax.set_ylim(-2, 75)

        self._canvas.draw_idle()

    # ─── Lifecycle ────────────────────────────────────────────────────────────

    def _on_close(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.root.destroy()

    def run(self):
        self.root.after(UI_REFRESH_MS, self.update)
        self.root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="HIL GUI for STM32H5 airbrakes")
    ap.add_argument("--port",   default=None)
    ap.add_argument("--baud",   type=int,   default=DEFAULT_BAUD)
    ap.add_argument("--source", choices=["synth", "csv", "rocketpy"], default="synth")
    ap.add_argument("--file",   default=None)
    ap.add_argument("--rate",   type=float, default=1.0)
    args = ap.parse_args()

    app = HILApp()
    if args.port:
        app.port_var.set(args.port)
    if args.baud != DEFAULT_BAUD:
        app.baud_var.set(args.baud)
    if args.source:
        app.source_var.set(args.source)
    if args.file:
        app.csv_var.set(args.file)
    if args.rate != 1.0:
        app.rate_var.set(args.rate)

    app.run()


if __name__ == "__main__":
    main()
