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
PACKET_LEN       = 100
DEFAULT_BAUD     = 115200
DEFAULT_RATE_HZ  = 50.0
HISTORY_LEN      = 800
UI_REFRESH_MS    = 33

AGGREGATE_FORMAT = "<" + "H" + "Hii" + "f" * 16 + "B" + "B" + "B" + "I" + "I" + "I" + "I" + "i" + "B"
PACKET_STRUCT    = struct.Struct(AGGREGATE_FORMAT)

# Path to the RocketPy directory (one level up, then RocketPy/)
_ROCKETPY_DIR = Path(__file__).resolve().parent.parent / "RocketPy"

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


def build_packet(f: PacketFields) -> bytes:
    altitude_cm = int(f.altitude_m)
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
        f.encoder_position_um, 0,
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
        t_start = time.perf_counter()
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
                except Exception as exc:
                    self.q.put(("error", f"TX error: {exc}"))

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

        self.q.put(("done", None))


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
        self.esc_curr    = deque(maxlen=maxlen)
        self.esc_volt    = deque(maxlen=maxlen)
        self._t0: float | None = None

    def add(self, f: PacketFields, cmd_angle: float):
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

        self._log.tag_config("ts",    foreground="#2a3d55")
        self._log.tag_config("tx",    foreground="#3d5a7a")
        self._log.tag_config("rx",    foreground="#9ff1c7")
        self._log.tag_config("state", foreground="#ffd479")
        self._log.tag_config("error", foreground="#ff6b6b")
        self._log.tag_config("info",  foreground="#aab7d5")

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

        # flap: actual (encoder) vs commanded (H5 output)
        self._ln_fa, = ax_flap.plot([], [], color="#9ff1c7", lw=2.2, label="Actual")
        self._ln_fc, = ax_flap.plot([], [], color="#d1b3ff", lw=1.8, ls="--", label="Commanded")
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
        self._sim_h5  = SimulatedH5()
        self._history = HILHistory(HISTORY_LEN)
        self._hil_sim = None
        self._tx_n = self._rx_n = 0
        self._flight_state = "IDLE"
        self._cmd_angle    = 0.0

        port = self.port_var.get()
        dry  = port == "(sim)"
        self._ser = None

        if not dry:
            try:
                self._ser = serial.Serial(port, self.baud_var.get(), timeout=0.05)
                self._conn_var.set(f"Connected — {port} @ {self.baud_var.get()} baud")
                SerialRxListener(self._ser, self._eq, self._stop).start()
            except serial.SerialException as exc:
                messagebox.showerror("Serial Error", str(exc))
                return
        else:
            ports = _available_ports()
            if len(ports) == 1:
                # auto-connect if exactly one port is attached
                try:
                    self._ser = serial.Serial(ports[0], self.baud_var.get(), timeout=0.05)
                    self._conn_var.set(f"Auto-connected — {ports[0]} @ {self.baud_var.get()} baud")
                    SerialRxListener(self._ser, self._eq, self._stop).start()
                    dry = False
                except serial.SerialException:
                    pass

            if dry:
                self._conn_var.set("Simulation mode — no serial port (H5 responses are synthetic)")

        src_fn, hil_sim = self._make_source()
        self._hil_sim = hil_sim

        # Use SimulatedH5 only when no real H5 is connected AND no RocketPy sim
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
        self._start_btn.config(state="disabled")
        self._stop_btn.config(state="normal")

    def _stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            self._ser.close()
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
                self._history.add(data, self._cmd_angle)
                if self._tx_n % 50 == 0:
                    f = data
                    self._log_msg(
                        f"→ PKT {self._tx_n:5d}  "
                        f"alt={f.altitude_m:6.0f} m  "
                        f"az={f.imu16_az_g:+5.2f} g  "
                        f"enc={f.encoder_position_um / 1000:.1f} mm",
                        "tx",
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
                                # ── Live feedback into RocketPy simulation ──
                                # Convert the H5's commanded angle to a
                                # deployment fraction and update the physics.
                                if self._hil_sim is not None:
                                    self._hil_sim.set_deployment_level(
                                        angle_deg / H5_MAX_ANGLE_DEG
                                    )
                            except ValueError:
                                pass
                self._log_msg(f"← {msg}", "rx")

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
