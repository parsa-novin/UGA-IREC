import csv
import math
import queue
import struct
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

PORT = "COM3"
BAUD = 115200
SERIAL_TIMEOUT_S = 0.005

AGGREGATE_HEADER = 0xA55A
UPSTREAM_HEADER = 0xAA55
PACKET_LEN = 108
HISTORY_LEN = 1200
UI_REFRESH_MS = 20
RECONNECT_DELAY_S = 1.0
FLOWER_COUNT = 7

LOG_DIR = Path(".")
LOG_FILENAME = f"ground_station_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
LOG_PATH = LOG_DIR / LOG_FILENAME

AGGREGATE_FORMAT = "<" + "H" + "Hii" + "f" * 16 + "B" + "B" + "B" + "I" + "I" + "I" + "I" + "i" + "f" + "I" + "B"
PACKET_STRUCT = struct.Struct(AGGREGATE_FORMAT)

AGGREGATE_FIELDS = [
    "aggregate_header",
    "upstream_header",
    "altitude_cm",
    "pressure_pa",
    "imu16_ax_g",
    "imu16_ay_g",
    "imu16_az_g",
    "imu4_ax_g",
    "imu4_ay_g",
    "imu4_az_g",
    "mag_x_gauss",
    "mag_y_gauss",
    "mag_z_gauss",
    "bmi_ax_g",
    "bmi_ay_g",
    "bmi_az_g",
    "bmi_gx_dps",
    "bmi_gy_dps",
    "bmi_gz_dps",
    "ext_temp_c",
    "upstream_checksum",
    "esc_valid",
    "esc_temp_c",
    "esc_voltage_mv",
    "esc_current_ma",
    "esc_consumption_mah",
    "esc_erpm",
    "encoder_position_um",
    "cam_current_ma",
    "batt_voltage_mv",
    "aggregate_checksum",
]

# Unified CSV schema — identical column names and order as the H5 SD card logger.
# One program can read both the ground-station CSV and the SD card CSV unchanged.
CSV_FIELDS = [
    "epoch_time_s",
    "altitude_m",
    "pressure_kpa",
    "imu16_ax_g",
    "imu16_ay_g",
    "imu16_az_g",
    "imu16_accel_mag_g",
    "imu4_ax_g",
    "imu4_ay_g",
    "imu4_az_g",
    "imu4_accel_mag_g",
    "mag_x_gauss",
    "mag_y_gauss",
    "mag_z_gauss",
    "mag_mag_gauss",
    "bmi_ax_g",
    "bmi_ay_g",
    "bmi_az_g",
    "bmi_accel_mag_g",
    "bmi_gx_dps",
    "bmi_gy_dps",
    "bmi_gz_dps",
    "bmi_gyro_mag_dps",
    "ext_temp_c",
    "esc_valid",
    "esc_temp_c",
    "esc_voltage_v",
    "esc_current_a",
    "esc_consumption_mah",
    "esc_erpm",
    "encoder_position_mm",
    "flap_angle_deg",
    "cam_current_ma",
    "batt_voltage_v",
]

LOW_LEVEL_PLOTS = [
    ("Altitude", "altitude_m", "m"),
    ("Pressure", "pressure_kpa", "kPa"),
    ("IMU16 Ax", "imu16_ax_g", "g"),
    ("IMU16 Ay", "imu16_ay_g", "g"),
    ("IMU16 Az", "imu16_az_g", "g"),
    ("IMU16 |a|", "imu16_accel_mag_g", "g"),
    ("IMU4 Ax", "imu4_ax_g", "g"),
    ("IMU4 Ay", "imu4_ay_g", "g"),
    ("IMU4 Az", "imu4_az_g", "g"),
    ("IMU4 |a|", "imu4_accel_mag_g", "g"),
    ("Mag X", "mag_x_gauss", "G"),
    ("Mag Y", "mag_y_gauss", "G"),
    ("Mag Z", "mag_z_gauss", "G"),
    ("Mag |B|", "mag_mag_gauss", "G"),
    ("BMI Ax", "bmi_ax_g", "g"),
    ("BMI Ay", "bmi_ay_g", "g"),
    ("BMI Az", "bmi_az_g", "g"),
    ("BMI |a|", "bmi_accel_mag_g", "g"),
    ("BMI Gx", "bmi_gx_dps", "dps"),
    ("BMI Gy", "bmi_gy_dps", "dps"),
    ("BMI Gz", "bmi_gz_dps", "dps"),
    ("BMI |gyro|", "bmi_gyro_mag_dps", "dps"),
    ("Ext Temp", "ext_temp_c", "C"),
    ("ESC Temp", "esc_temp_c", "C"),
    ("Battery", "esc_voltage_v", "V"),
    ("ESC Current", "esc_current_a", "A"),
    ("Consumption", "esc_consumption_mah", "mAh"),
    ("eRPM", "esc_erpm", "eRPM"),
    ("Displacement", "encoder_position_mm", "mm"),
    ("Flap Angle", "flap_angle_deg", "deg"),
    ("Cam Current", "cam_current_ma", "mA"),
    ("Batt Voltage", "batt_voltage_v", "V"),
]

# Camera echo message prefixes forwarded by H5 from the H7 echo buffer.
_CAM_CMD_LABELS = {
    "CAM:CMD:REC": ("Start recording dispatched",  "info"),
    "CAM:CMD:STP": ("Stop recording dispatched",   "info"),
    "CAM:CMD:ON":  ("Power-on dispatched",          "info"),
    "CAM:CMD:OFF": ("Power-off dispatched",         "info"),
    "CAM:CMD:CUR": ("Current query dispatched",     "info"),
    "CAM:RC:OK":   ("RunCam OK",                    "ok"),
    "CAM:RC:TMO":  ("Camera not responding (timeout) — enable RunCam Device Protocol in camera OSD: Settings → UART Protocol → On", "err"),
    "CAM:RC:UNS":  ("Feature unsupported by camera", "warn"),
    "CAM:RC:ERR":  ("RunCam error",                 "err"),
}


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def displacement_mm_to_angle_deg(displacement_mm: float) -> float:
    angle = (
        -0.000513 * displacement_mm ** 3
        + 0.010615 * displacement_mm ** 2
        + 2.3199 * displacement_mm
        - 0.4986
    )
    return clamp(angle, 0.0, 70.0)


def magnitude3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def available_com_ports() -> list[str]:
    ports = sorted(port.device for port in list_ports.comports())
    return ports or [PORT]


@dataclass
class TelemetrySample:
    packet_time_s: float
    altitude_m: float
    pressure_kpa: float
    imu16_ax_g: float
    imu16_ay_g: float
    imu16_az_g: float
    imu16_accel_mag_g: float
    imu4_ax_g: float
    imu4_ay_g: float
    imu4_az_g: float
    imu4_accel_mag_g: float
    mag_x_gauss: float
    mag_y_gauss: float
    mag_z_gauss: float
    mag_mag_gauss: float
    bmi_ax_g: float
    bmi_ay_g: float
    bmi_az_g: float
    bmi_accel_mag_g: float
    bmi_gx_dps: float
    bmi_gy_dps: float
    bmi_gz_dps: float
    bmi_gyro_mag_dps: float
    ext_temp_c: float
    esc_valid: bool
    esc_temp_c: float
    esc_voltage_v: float
    esc_current_a: float
    esc_consumption_mah: float
    esc_erpm: float
    encoder_position_mm: float
    flap_angle_deg: float
    cam_current_ma: float
    batt_voltage_v: float
    aggregate_checksum_ok: bool
    upstream_checksum_ok: bool


class PacketParser:
    def __init__(self):
        self.buffer = bytearray()
        self.sync_bytes = struct.pack("<H", AGGREGATE_HEADER)

    def feed(self, data: bytes) -> list[TelemetrySample]:
        self.buffer.extend(data)
        packets = []

        while True:
            if len(self.buffer) < PACKET_LEN:
                break

            idx = self.buffer.find(self.sync_bytes)
            if idx < 0:
                if len(self.buffer) > PACKET_LEN:
                    del self.buffer[:-1]
                break

            if idx > 0:
                del self.buffer[:idx]

            if len(self.buffer) < PACKET_LEN:
                break

            packet_bytes = bytes(self.buffer[:PACKET_LEN])
            sample = self._decode_packet(packet_bytes)
            if sample is not None:
                packets.append(sample)
                del self.buffer[:PACKET_LEN]
                continue

            del self.buffer[0]

        return packets

    def _decode_packet(self, packet_bytes: bytes) -> TelemetrySample | None:
        try:
            values = PACKET_STRUCT.unpack(packet_bytes)
        except struct.error:
            return None

        fields = dict(zip(AGGREGATE_FIELDS, values))
        if fields["aggregate_header"] != AGGREGATE_HEADER:
            return None

        aggregate_checksum_ok = xor_checksum(packet_bytes[2:-1]) == fields["aggregate_checksum"]
        upstream_bytes = packet_bytes[2:77]
        upstream_checksum_ok = (
            struct.unpack_from("<H", upstream_bytes, 0)[0] == UPSTREAM_HEADER
            and xor_checksum(upstream_bytes[2:-1]) == fields["upstream_checksum"]
        )

        # altitude field is integer metres (float cast to int32 on the c5 side)
        altitude_m = float(fields["altitude_cm"])
        pressure_kpa = fields["pressure_pa"] / 1000.0
        # encoder_position_um holds true micrometres; divide by 1000 to get mm
        encoder_position_mm = fields["encoder_position_um"] / 1000.0
        flap_angle_deg = displacement_mm_to_angle_deg(max(0.0, encoder_position_mm))
        batt_voltage_v = fields["batt_voltage_mv"] / 1000.0

        return TelemetrySample(
            packet_time_s=time.time(),
            altitude_m=altitude_m,
            pressure_kpa=pressure_kpa,
            imu16_ax_g=fields["imu16_ax_g"],
            imu16_ay_g=fields["imu16_ay_g"],
            imu16_az_g=fields["imu16_az_g"],
            imu16_accel_mag_g=magnitude3(fields["imu16_ax_g"], fields["imu16_ay_g"], fields["imu16_az_g"]),
            imu4_ax_g=fields["imu4_ax_g"],
            imu4_ay_g=fields["imu4_ay_g"],
            imu4_az_g=fields["imu4_az_g"],
            imu4_accel_mag_g=magnitude3(fields["imu4_ax_g"], fields["imu4_ay_g"], fields["imu4_az_g"]),
            mag_x_gauss=fields["mag_x_gauss"],
            mag_y_gauss=fields["mag_y_gauss"],
            mag_z_gauss=fields["mag_z_gauss"],
            mag_mag_gauss=magnitude3(fields["mag_x_gauss"], fields["mag_y_gauss"], fields["mag_z_gauss"]),
            bmi_ax_g=fields["bmi_ax_g"],
            bmi_ay_g=fields["bmi_ay_g"],
            bmi_az_g=fields["bmi_az_g"],
            bmi_accel_mag_g=magnitude3(fields["bmi_ax_g"], fields["bmi_ay_g"], fields["bmi_az_g"]),
            bmi_gx_dps=fields["bmi_gx_dps"],
            bmi_gy_dps=fields["bmi_gy_dps"],
            bmi_gz_dps=fields["bmi_gz_dps"],
            bmi_gyro_mag_dps=magnitude3(fields["bmi_gx_dps"], fields["bmi_gy_dps"], fields["bmi_gz_dps"]),
            ext_temp_c=fields["ext_temp_c"],
            esc_valid=bool(fields["esc_valid"]),
            esc_temp_c=float(fields["esc_temp_c"]),
            esc_voltage_v=fields["esc_voltage_mv"] / 1000.0,
            esc_current_a=fields["esc_current_ma"] / 1000.0,
            esc_consumption_mah=float(fields["esc_consumption_mah"]),
            esc_erpm=float(fields["esc_erpm"]),
            encoder_position_mm=encoder_position_mm,
            flap_angle_deg=flap_angle_deg,
            cam_current_ma=fields["cam_current_ma"],
            batt_voltage_v=batt_voltage_v,
            aggregate_checksum_ok=aggregate_checksum_ok,
            upstream_checksum_ok=upstream_checksum_ok,
        )


class SerialTelemetryReader(threading.Thread):
    def __init__(self, get_port, baud: int, sample_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.get_port = get_port
        self.baud = baud
        self.sample_queue = sample_queue
        self.stop_event = stop_event
        self.parser = PacketParser()
        self.status = "Disconnected"
        self.serial_lock = threading.Lock()
        self.serial_handle = None
        # Camera echo messages forwarded by H5 from the H7 echo buffer.
        self.camera_queue: queue.Queue[str] = queue.Queue()
        self._cam_line_buf: bytearray = bytearray()

    def run(self):
        while not self.stop_event.is_set():
            port = self.get_port()
            try:
                self.status = f"Connecting to {port} @ {self.baud}"
                with serial.Serial(port, self.baud, timeout=SERIAL_TIMEOUT_S) as ser:
                    ser.reset_input_buffer()
                    # Send Unix epoch to H5 so SD card timestamps are absolute.
                    epoch_cmd = f"EPOCH{int(time.time())}\n".encode("ascii")
                    ser.write(epoch_cmd)
                    ser.flush()
                    with self.serial_lock:
                        self.serial_handle = ser
                    self.status = f"Connected to {port} @ {self.baud}"
                    while not self.stop_event.is_set():
                        if self.get_port() != port:
                            self.status = f"Switching to {self.get_port()}..."
                            break
                        pending = ser.in_waiting
                        chunk = ser.read(max(512, pending))
                        if not chunk:
                            continue

                        for sample in self.parser.feed(chunk):
                            self.sample_queue.put(sample)
                        self._scan_cam_lines(chunk)

                        # Drain any immediate backlog so UI tracks live data.
                        for _ in range(6):
                            pending = ser.in_waiting
                            if pending <= 0:
                                break
                            extra = ser.read(min(4096, pending))
                            if not extra:
                                break
                            for sample in self.parser.feed(extra):
                                self.sample_queue.put(sample)
                            self._scan_cam_lines(extra)
            except Exception as exc:
                self.status = f"Serial error: {exc}"
                with self.serial_lock:
                    self.serial_handle = None
                time.sleep(RECONNECT_DELAY_S)
            finally:
                with self.serial_lock:
                    self.serial_handle = None

    def _scan_cam_lines(self, data: bytes):
        """Extract CAM:... lines from the byte stream alongside binary packets."""
        for byte in data:
            if byte == ord('\n'):
                if self._cam_line_buf:
                    line = self._cam_line_buf.decode("ascii", errors="ignore").strip()
                    if line.startswith("CAM:"):
                        self.camera_queue.put(line)
                    self._cam_line_buf.clear()
            elif byte == ord('\r'):
                pass
            elif 0x20 <= byte <= 0x7E:
                # Only accumulate printable ASCII; binary packet bytes will
                # appear as non-printable and reset the buffer, preventing
                # false matches inside binary telemetry frames.
                if len(self._cam_line_buf) < 128:
                    self._cam_line_buf.append(byte)
            else:
                self._cam_line_buf.clear()

    def send_line(self, line: str) -> bool:
        payload = (line.strip() + "\n").encode("ascii", errors="ignore")
        with self.serial_lock:
            ser = self.serial_handle
            if ser is None or not ser.is_open:
                return False
            ser.write(payload)
            ser.flush()
            return True


class TelemetryHistory:
    def __init__(self, maxlen: int):
        self.times = deque(maxlen=maxlen)
        self.series = {field: deque(maxlen=maxlen) for _, field, _ in LOW_LEVEL_PLOTS}
        self.latest: TelemetrySample | None = None
        self.packet_count = 0
        self.packet_rate_hz = 0.0
        self.last_packet_time = 0.0

    def add(self, sample: TelemetrySample):
        self.latest = sample
        self.packet_count += 1
        self.times.append(sample.packet_time_s)
        for _, field, _ in LOW_LEVEL_PLOTS:
            self.series[field].append(getattr(sample, field))

        if self.last_packet_time:
            dt = sample.packet_time_s - self.last_packet_time
            if dt > 0:
                rate = 1.0 / dt
                self.packet_rate_hz = rate if self.packet_rate_hz == 0.0 else (0.85 * self.packet_rate_hz + 0.15 * rate)
        self.last_packet_time = sample.packet_time_s

    def x_axis(self) -> list[float]:
        if not self.times:
            return []
        start = self.times[0]
        return [t - start for t in self.times]


class TelemetryLogger:
    def __init__(self, path: Path):
        self.fp = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.fp, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.fp.flush()
        self.lock = threading.Lock()

    def write(self, sample: TelemetrySample):
        # Unified schema — identical column names and order as the H5 SD card CSV.
        row = {
            "epoch_time_s": f"{sample.packet_time_s:.3f}",
            "altitude_m": f"{sample.altitude_m:.3f}",
            "pressure_kpa": f"{sample.pressure_kpa:.3f}",
            "imu16_ax_g": f"{sample.imu16_ax_g:.4f}",
            "imu16_ay_g": f"{sample.imu16_ay_g:.4f}",
            "imu16_az_g": f"{sample.imu16_az_g:.4f}",
            "imu16_accel_mag_g": f"{sample.imu16_accel_mag_g:.4f}",
            "imu4_ax_g": f"{sample.imu4_ax_g:.4f}",
            "imu4_ay_g": f"{sample.imu4_ay_g:.4f}",
            "imu4_az_g": f"{sample.imu4_az_g:.4f}",
            "imu4_accel_mag_g": f"{sample.imu4_accel_mag_g:.4f}",
            "mag_x_gauss": f"{sample.mag_x_gauss:.4f}",
            "mag_y_gauss": f"{sample.mag_y_gauss:.4f}",
            "mag_z_gauss": f"{sample.mag_z_gauss:.4f}",
            "mag_mag_gauss": f"{sample.mag_mag_gauss:.4f}",
            "bmi_ax_g": f"{sample.bmi_ax_g:.4f}",
            "bmi_ay_g": f"{sample.bmi_ay_g:.4f}",
            "bmi_az_g": f"{sample.bmi_az_g:.4f}",
            "bmi_accel_mag_g": f"{sample.bmi_accel_mag_g:.4f}",
            "bmi_gx_dps": f"{sample.bmi_gx_dps:.4f}",
            "bmi_gy_dps": f"{sample.bmi_gy_dps:.4f}",
            "bmi_gz_dps": f"{sample.bmi_gz_dps:.4f}",
            "bmi_gyro_mag_dps": f"{sample.bmi_gyro_mag_dps:.4f}",
            "ext_temp_c": f"{sample.ext_temp_c:.3f}",
            "esc_valid": int(sample.esc_valid),
            "esc_temp_c": f"{sample.esc_temp_c:.3f}",
            "esc_voltage_v": f"{sample.esc_voltage_v:.3f}",
            "esc_current_a": f"{sample.esc_current_a:.3f}",
            "esc_consumption_mah": f"{sample.esc_consumption_mah:.3f}",
            "esc_erpm": f"{sample.esc_erpm:.3f}",
            "encoder_position_mm": f"{sample.encoder_position_mm:.4f}",
            "flap_angle_deg": f"{sample.flap_angle_deg:.3f}",
            "cam_current_ma": f"{sample.cam_current_ma:.3f}",
            "batt_voltage_v": f"{sample.batt_voltage_v:.3f}",
        }
        with self.lock:
            self.writer.writerow(row)
            self.fp.flush()

    def close(self):
        with self.lock:
            self.fp.flush()
            self.fp.close()


class MetricCard(ttk.Frame):
    def __init__(self, master, title: str, accent: str, compact: bool = False):
        super().__init__(master, padding=(10, 7) if compact else (14, 10), style="Card.TFrame")
        self.value_var = tk.StringVar(value="--")
        self.sub_var = tk.StringVar(value="")
        value_style = f"{accent}.CompactValue.TLabel" if compact else f"{accent}.Value.TLabel"
        title_style = "CompactCardTitle.TLabel" if compact else "CardTitle.TLabel"
        sub_style = "CompactCardSub.TLabel" if compact else "CardSub.TLabel"

        ttk.Label(self, text=title, style=title_style).pack(anchor="w")
        ttk.Label(self, textvariable=self.value_var, style=value_style).pack(anchor="w", pady=(3 if compact else 6, 0))
        ttk.Label(self, textvariable=self.sub_var, style=sub_style).pack(anchor="w", pady=(2 if compact else 4, 0))

    def set(self, value: str, sub: str = ""):
        self.value_var.set(value)
        self.sub_var.set(sub)


class GroundStationApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("UGA Rocketry Ground Station")
        self.root.geometry("1620x980")
        self.root.configure(bg="#06080d")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.sample_queue: queue.Queue[TelemetrySample] = queue.Queue()
        self.stop_event = threading.Event()
        self.port_var = tk.StringVar(value=PORT)
        self.port_values = available_com_ports()
        if PORT not in self.port_values:
            self.port_values.insert(0, PORT)
        self.port_var.set(self.port_values[0] if self.port_values else PORT)
        self.reader = SerialTelemetryReader(self.port_var.get, BAUD, self.sample_queue, self.stop_event)
        self.history = TelemetryHistory(HISTORY_LEN)
        self.logger = TelemetryLogger(LOG_PATH)

        self.status_var = tk.StringVar(value="Waiting for serial data...")
        self.packet_var = tk.StringVar(value="Packets: 0")
        self.log_var = tk.StringVar(value=f"CSV: {LOG_PATH}")
        self.command_var = tk.StringVar(value="Controls idle")

        # Camera state: UNKNOWN | ONLINE | RECORDING | IDLE | OFFLINE
        self._cam_state = "UNKNOWN"

        self.metric_cards: dict[str, MetricCard] = {}
        self.low_level_axes = []
        self.low_level_lines = {}
        self.low_level_base_titles = {}
        self.latest_sample: TelemetrySample | None = None
        self._frame_count = 0
        self._canvas_size: tuple[int, int] | None = None

        self._configure_styles()
        self._build_ui()
        self._build_high_level_figure()
        self._build_low_level_figure()
        self._build_airbrake_figure()

    def _configure_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#06080d")
        style.configure("TLabel", background="#06080d", foreground="#edf2ff")
        style.configure("Header.TLabel", font=("Segoe UI Semibold", 18), foreground="#f5f7ff")
        style.configure("SubHeader.TLabel", font=("Segoe UI", 10), foreground="#aab7d5")
        style.configure("Tab.TFrame", background="#0a111c")
        style.configure("Card.TFrame", background="#0f1726", relief="flat")
        style.configure("CardTitle.TLabel", background="#0f1726", foreground="#9fb0d3", font=("Segoe UI Semibold", 10))
        style.configure("CardSub.TLabel", background="#0f1726", foreground="#7f8fb2", font=("Segoe UI", 9))
        style.configure("CompactCardTitle.TLabel", background="#0f1726", foreground="#9fb0d3", font=("Segoe UI Semibold", 9))
        style.configure("CompactCardSub.TLabel", background="#0f1726", foreground="#7f8fb2", font=("Segoe UI", 8))
        style.configure("Blue.Value.TLabel", background="#0f1726", foreground="#8bd3ff", font=("Bahnschrift SemiBold", 24))
        style.configure("Gold.Value.TLabel", background="#0f1726", foreground="#ffd479", font=("Bahnschrift SemiBold", 24))
        style.configure("Green.Value.TLabel", background="#0f1726", foreground="#9ff1c7", font=("Bahnschrift SemiBold", 24))
        style.configure("Rose.Value.TLabel", background="#0f1726", foreground="#ff9aa7", font=("Bahnschrift SemiBold", 24))
        style.configure("Blue.CompactValue.TLabel", background="#0f1726", foreground="#8bd3ff", font=("Bahnschrift SemiBold", 17))
        style.configure("Gold.CompactValue.TLabel", background="#0f1726", foreground="#ffd479", font=("Bahnschrift SemiBold", 17))
        style.configure("Green.CompactValue.TLabel", background="#0f1726", foreground="#9ff1c7", font=("Bahnschrift SemiBold", 17))
        style.configure("Rose.CompactValue.TLabel", background="#0f1726", foreground="#ff9aa7", font=("Bahnschrift SemiBold", 17))
        style.configure("Notebook.TNotebook", background="#06080d", borderwidth=0)
        style.configure("Notebook.TNotebook.Tab", background="#111a2a", foreground="#dce7ff", padding=(14, 8), font=("Segoe UI Semibold", 10))
        style.map("Notebook.TNotebook.Tab", background=[("selected", "#1b2740")], foreground=[("selected", "#ffffff")])

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=(18, 16, 18, 8))
        top.pack(fill="x")
        ttk.Label(top, text="UGA Spaceport Telemetry Console", style="Header.TLabel").pack(anchor="w")
        ttk.Label(top, text="Four-tab live dashboard for the aggregate H5 packet stream", style="SubHeader.TLabel").pack(anchor="w", pady=(2, 0))

        controls = ttk.Frame(top)
        controls.pack(fill="x", pady=(10, 0))
        ttk.Label(controls, text="COM Port", style="SubHeader.TLabel").pack(side="left")
        self.port_combo = ttk.Combobox(
            controls,
            textvariable=self.port_var,
            values=self.port_values,
            state="readonly",
            width=18,
        )
        self.port_combo.pack(side="left", padx=(10, 8))
        self.port_combo.bind("<<ComboboxSelected>>", self._on_port_selected)
        ttk.Button(controls, text="Refresh Ports", command=self._refresh_ports).pack(side="left")
        ttk.Button(controls, text="Reset H5", command=self._confirm_reset).pack(side="left", padx=(12, 0))

        status_bar = ttk.Frame(self.root, padding=(18, 0, 18, 10))
        status_bar.pack(fill="x")
        ttk.Label(status_bar, textvariable=self.status_var, style="SubHeader.TLabel").pack(side="left")
        ttk.Label(status_bar, textvariable=self.packet_var, style="SubHeader.TLabel").pack(side="left", padx=(20, 0))
        ttk.Label(status_bar, textvariable=self.log_var, style="SubHeader.TLabel").pack(side="right")

        self.notebook = ttk.Notebook(self.root, style="Notebook.TNotebook")
        self.notebook.pack(fill="both", expand=True, padx=12, pady=(0, 12))

        self.high_tab = ttk.Frame(self.notebook, style="Tab.TFrame", padding=12)
        self.low_tab = ttk.Frame(self.notebook, style="Tab.TFrame", padding=12)
        self.airbrake_tab = ttk.Frame(self.notebook, style="Tab.TFrame", padding=12)
        self.camera_tab = ttk.Frame(self.notebook, style="Tab.TFrame", padding=12)

        self.notebook.add(self.high_tab, text="1. High-Level")
        self.notebook.add(self.low_tab, text="2. Low-Level")
        self.notebook.add(self.airbrake_tab, text="3. Airbrakes")
        self.notebook.add(self.camera_tab, text="4. Camera")

        self._build_high_level_tab()
        self._build_low_level_tab()
        self._build_airbrake_tab()
        self._build_camera_tab()

        self.root.bind("<Up>",   lambda _e: self._send_command("F", "Up arrow jog"))
        self.root.bind("<Down>", lambda _e: self._send_command("B", "Down arrow jog"))

    def _build_high_level_tab(self):
        cards = ttk.Frame(self.high_tab)
        cards.pack(fill="x", pady=(0, 10))
        for idx in range(6):
            cards.columnconfigure(idx, weight=1)

        specs = [
            ("Altitude", "Blue", "altitude"),
            ("Primary Accel", "Gold", "accel"),
            ("External Temp", "Rose", "ext_temp"),
            ("ESC Battery", "Green", "battery"),
            ("Flap Angle", "Gold", "flap_angle"),
            ("Packet Health", "Blue", "health"),
        ]
        for col, (title, accent, key) in enumerate(specs):
            card = MetricCard(cards, title, accent)
            card.grid(row=0, column=col, sticky="nsew", padx=6, pady=6)
            self.metric_cards[key] = card

        self.high_plot_host = ttk.Frame(self.high_tab)
        self.high_plot_host.pack(fill="both", expand=True)

    def _build_low_level_tab(self):
        self.low_plot_host = ttk.Frame(self.low_tab)
        self.low_plot_host.pack(fill="both", expand=True)

    def _build_airbrake_tab(self):
        left = ttk.Frame(self.airbrake_tab)
        left.pack(side="left", fill="both", expand=True)
        right = ttk.Frame(self.airbrake_tab, width=500)
        right.pack(side="right", fill="y", padx=(12, 0))
        right.pack_propagate(False)

        self.rocket_canvas = tk.Canvas(left, bg="#05070b", highlightthickness=0, width=760, height=880)
        self.rocket_canvas.pack(fill="both", expand=True)

        metrics = ttk.Frame(right)
        metrics.pack(fill="x")
        for i in range(2):
            metrics.columnconfigure(i, weight=1)

        metric_specs = [
            ("Displacement", "Blue", "airbrake_displacement"),
            ("Flap Angle", "Gold", "airbrake_angle"),
            ("ESC Current", "Rose", "airbrake_current"),
            ("Battery", "Green", "airbrake_battery"),
            ("ESC Temp", "Rose", "airbrake_temp"),
            ("eRPM", "Blue", "airbrake_erpm"),
        ]
        for idx, (title, accent, key) in enumerate(metric_specs):
            card = MetricCard(metrics, title, accent, compact=True)
            card.grid(row=idx // 2, column=idx % 2, sticky="nsew", padx=6, pady=6)
            self.metric_cards[key] = card

        ttk.Label(
            right,
            text="Angle model: -0.000513x^3 + 0.010615x^2 + 2.3199x - 0.4986, clamped to 0-70 deg",
            style="SubHeader.TLabel",
            wraplength=470,
            justify="left",
        ).pack(fill="x", padx=6, pady=(8, 8))

        command_frame = ttk.Frame(right)
        command_frame.pack(fill="x", padx=6, pady=(0, 10))
        for i in range(3):
            command_frame.columnconfigure(i, weight=1)

        ttk.Button(command_frame, text="Auto Home", command=lambda: self._send_command("H", "Auto-homing")).grid(row=0, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(command_frame, text="Max (70 deg)", command=lambda: self._send_command("UP", "Go to max (70 deg)")).grid(row=0, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(command_frame, text="Min (0 deg)", command=lambda: self._send_command("DOWN", "Go to min (0 deg)")).grid(row=0, column=2, sticky="ew", padx=4, pady=4)
        ttk.Button(command_frame, text="Stop", command=lambda: self._send_command("S", "Stop")).grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(command_frame, text="Zero Encoder", command=lambda: self._send_command("O", "Zero encoder")).grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(command_frame, text="Deploy Seq", command=lambda: self._send_command("67", "Deploy sequence")).grid(row=1, column=2, sticky="ew", padx=4, pady=4)

        ttk.Label(
            right,
            textvariable=self.command_var,
            style="SubHeader.TLabel",
            wraplength=470,
            justify="left",
        ).pack(fill="x", padx=6, pady=(0, 8))

        self.airbrake_plot_host = ttk.Frame(right)
        self.airbrake_plot_host.pack(fill="both", expand=True)

    def _build_camera_tab(self):
        left = ttk.Frame(self.camera_tab, width=300)
        left.pack(side="left", fill="y", padx=(0, 12))
        left.pack_propagate(False)

        right = ttk.Frame(self.camera_tab)
        right.pack(side="right", fill="both", expand=True)

        # ── Status card ────────────────────────────────────────────────────────
        status_card = ttk.Frame(left, style="Card.TFrame", padding=(14, 10))
        status_card.pack(fill="x", pady=(0, 12))
        ttk.Label(status_card, text="Camera Status", style="CardTitle.TLabel").pack(anchor="w")
        self._cam_state_var = tk.StringVar(value="UNKNOWN")
        self._cam_sub_var = tk.StringVar(value="No device info received")
        ttk.Label(status_card, textvariable=self._cam_state_var,
                  style="Gold.Value.TLabel").pack(anchor="w", pady=(6, 0))
        ttk.Label(status_card, textvariable=self._cam_sub_var,
                  style="CardSub.TLabel").pack(anchor="w", pady=(4, 0))

        # ── Control buttons ────────────────────────────────────────────────────
        ctrl_card = ttk.Frame(left, style="Card.TFrame", padding=(14, 10))
        ctrl_card.pack(fill="x", pady=(0, 12))
        ttk.Label(ctrl_card, text="Controls", style="CardTitle.TLabel").pack(anchor="w", pady=(0, 8))

        btn_frame = ttk.Frame(ctrl_card, style="Card.TFrame")
        btn_frame.pack(fill="x")
        btn_frame.columnconfigure(0, weight=1)
        btn_frame.columnconfigure(1, weight=1)

        ttk.Button(btn_frame, text="Power On",
                   command=lambda: self._send_cam_command("CAMON", "Power on")).grid(
            row=0, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(btn_frame, text="Power Off",
                   command=lambda: self._send_cam_command("CAMOFF", "Power off")).grid(
            row=0, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(btn_frame, text="Start Recording",
                   command=lambda: self._send_cam_command("CAMSTART", "Start recording")).grid(
            row=1, column=0, sticky="ew", padx=4, pady=4)
        ttk.Button(btn_frame, text="Stop Recording",
                   command=lambda: self._send_cam_command("CAMSTOP", "Stop recording")).grid(
            row=1, column=1, sticky="ew", padx=4, pady=4)
        ttk.Button(btn_frame, text="Read Current",
                   command=lambda: self._send_cam_command("CAMCURR", "Read current")).grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=4, pady=4)

        # ── Note about RunCam protocol ─────────────────────────────────────────
        ttk.Label(
            left,
            text="If camera is unresponsive, enable RunCam Device Protocol in camera OSD: Settings → UART Protocol → On",
            style="SubHeader.TLabel",
            wraplength=270,
            justify="left",
        ).pack(fill="x", padx=4, pady=(4, 0))

        # ── Event log ──────────────────────────────────────────────────────────
        log_header = ttk.Frame(right)
        log_header.pack(fill="x", pady=(0, 6))
        ttk.Label(log_header, text="Camera Event Log", style="CardTitle.TLabel").pack(side="left")
        ttk.Button(log_header, text="Clear", command=self._cam_log_clear).pack(side="right")

        log_frame = ttk.Frame(right, style="Card.TFrame")
        log_frame.pack(fill="both", expand=True)

        self._cam_log = tk.Text(
            log_frame,
            state=tk.DISABLED,
            bg="#0a111c",
            fg="#c9d5f0",
            font=("Consolas", 10),
            wrap=tk.WORD,
            borderwidth=0,
            highlightthickness=0,
            relief="flat",
            padx=10,
            pady=8,
        )
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self._cam_log.yview)
        self._cam_log.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self._cam_log.pack(side="left", fill="both", expand=True)

        # Colour tags
        self._cam_log.tag_configure("sent",  foreground="#8bd3ff")   # commands sent from GS
        self._cam_log.tag_configure("ok",    foreground="#9ff1c7")   # success response
        self._cam_log.tag_configure("err",   foreground="#ff8ba7")   # error / timeout
        self._cam_log.tag_configure("warn",  foreground="#ffd479")   # unsupported / warning
        self._cam_log.tag_configure("info",  foreground="#d1b3ff")   # device info / neutral
        self._cam_log.tag_configure("curr",  foreground="#ffb366")   # current readings
        self._cam_log.tag_configure("ts",    foreground="#4a5f80")   # timestamps

        self._cam_log_append("Camera tab ready.\n", "info")
        self._cam_log_append("Click a button to send a command. Echo messages (CAM:...) from H7 appear here once firmware is flashed and serial is connected.\n", "ts")

    # ── Camera helpers ─────────────────────────────────────────────────────────

    def _send_cam_command(self, command: str, label: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        ok = self.reader.send_line(command)
        if ok:
            self._cam_log_append(f"[{ts}]", "ts")
            self._cam_log_append(f" >> {label} ({command})\n", "sent")
        else:
            port = self.port_var.get()
            self._cam_log_append(f"[{ts}]", "ts")
            self._cam_log_append(f" !! Could not send {command} — no connection on {port}\n", "err")
        self.command_var.set(f"{'Sent' if ok else 'Failed'} {label} at {datetime.now().strftime('%H:%M:%S')}")

    def _cam_log_append(self, text: str, tag: str = ""):
        self._cam_log.configure(state=tk.NORMAL)
        if tag:
            self._cam_log.insert(tk.END, text, tag)
        else:
            self._cam_log.insert(tk.END, text)
        self._cam_log.see(tk.END)
        self._cam_log.configure(state=tk.DISABLED)

    def _cam_log_clear(self):
        self._cam_log.configure(state=tk.NORMAL)
        self._cam_log.delete("1.0", tk.END)
        self._cam_log.configure(state=tk.DISABLED)

    def _process_camera_echo(self, line: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._cam_log_append(f"[{ts}]", "ts")

        if line in _CAM_CMD_LABELS:
            label, tag = _CAM_CMD_LABELS[line]
            self._cam_log_append(f" {label}\n", tag)
            # Update camera state from dispatch echoes
            if line == "CAM:CMD:REC":
                self._cam_state = "RECORDING"
            elif line == "CAM:CMD:STP":
                self._cam_state = "IDLE"
            elif line == "CAM:CMD:OFF":
                self._cam_state = "OFFLINE"
            elif line == "CAM:CMD:ON":
                self._cam_state = "POWERING ON"
        elif line.startswith("CAM:INF:"):
            parts = line[8:].split(":")
            ver_str  = parts[0] if parts else "?"
            feat_str = parts[1] if len(parts) > 1 else "?"
            self._cam_log_append(f" Device info — {ver_str}, features {feat_str}\n", "info")
            self._cam_state = "ONLINE"
        elif line.startswith("CAM:CUR:"):
            val = line[8:]
            self._cam_log_append(f" Current: {val} mA\n", "curr")
        else:
            self._cam_log_append(f" {line}\n", "info")

        self._update_cam_state_card()

    def _update_cam_state_card(self):
        colors = {
            "UNKNOWN":    ("Gold",  "No device info received"),
            "ONLINE":     ("Green", "Camera responding on UART"),
            "RECORDING":  ("Rose",  "Recording in progress"),
            "IDLE":       ("Blue",  "Ready, not recording"),
            "OFFLINE":    ("Blue",  "Camera powered off"),
            "POWERING ON": ("Gold", "Waiting for camera boot"),
        }
        accent, sub = colors.get(self._cam_state, ("Gold", ""))
        self._cam_state_var.set(self._cam_state)
        self._cam_sub_var.set(sub)

    # ── Existing UI helpers ────────────────────────────────────────────────────

    def _build_high_level_figure(self):
        self.high_fig = Figure(figsize=(12, 7), facecolor="#0a111c")
        self.high_axes = [self.high_fig.add_subplot(2, 2, i + 1) for i in range(4)]
        self.high_lines = {
            "altitude": self.high_axes[0].plot([], [], color="#7cc7ff", linewidth=2.5)[0],
            "accel": self.high_axes[1].plot([], [], color="#ffd479", linewidth=2.5)[0],
            "ext_temp": self.high_axes[2].plot([], [], color="#ff8ba7", linewidth=2.0, label="Ext Temp")[0],
            "esc_temp": self.high_axes[2].plot([], [], color="#ffd7dd", linewidth=2.0, label="ESC Temp")[0],
            "battery": self.high_axes[3].plot([], [], color="#90f3c6", linewidth=2.0, label="Battery")[0],
            "current": self.high_axes[3].plot([], [], color="#ffb366", linewidth=1.8, label="Current")[0],
        }
        titles = [
            ("Altitude", "m"),
            ("Acceleration Magnitude", "g"),
            ("Temperatures", "C"),
            ("Airbrake Power", "V / A"),
        ]
        for ax, (title, ylabel) in zip(self.high_axes, titles):
            self._style_axis(ax, title, ylabel)
        self.high_axes[2].legend(facecolor="#0a111c", edgecolor="#22324f", labelcolor="#dce7ff")
        self.high_axes[3].legend(facecolor="#0a111c", edgecolor="#22324f", labelcolor="#dce7ff")

        self.high_canvas = FigureCanvasTkAgg(self.high_fig, master=self.high_plot_host)
        self.high_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_low_level_figure(self):
        self.low_fig = Figure(figsize=(16, 12), facecolor="#0a111c")
        self.low_fig.subplots_adjust(left=0.035, right=0.99, top=0.97, bottom=0.05, hspace=0.48, wspace=0.26)

        for idx, (title, field, unit) in enumerate(LOW_LEVEL_PLOTS, start=1):
            ax = self.low_fig.add_subplot(7, 5, idx)
            self._style_axis(ax, title, unit, compact=True)
            line = ax.plot([], [], color=self._plot_color(idx), linewidth=1.4)[0]
            self.low_level_axes.append(ax)
            self.low_level_lines[field] = line
            self.low_level_base_titles[field] = title

        self.low_canvas = FigureCanvasTkAgg(self.low_fig, master=self.low_plot_host)
        self.low_canvas.get_tk_widget().pack(fill="both", expand=True)

    def _build_airbrake_figure(self):
        self.air_fig = Figure(figsize=(5.3, 6.8), facecolor="#0a111c")
        self.air_fig.subplots_adjust(left=0.12, right=0.96, top=0.96, bottom=0.09, hspace=0.4)
        self.transfer_ax = self.air_fig.add_subplot(2, 1, 1)
        self.power_ax = self.air_fig.add_subplot(2, 1, 2)
        self._style_axis(self.transfer_ax, "Displacement -> Flap Angle", "deg")
        self._style_axis(self.power_ax, "ESC Telemetry", "mixed")

        xs = [x / 2.0 for x in range(0, 101)]
        ys = [displacement_mm_to_angle_deg(x) for x in xs]
        self.transfer_ax.plot(xs, ys, color="#8bd3ff", linewidth=2.2)
        self.transfer_point = self.transfer_ax.scatter([0], [0], s=80, color="#ffd479", edgecolors="#ffffff", linewidths=0.8, zorder=5)
        self.transfer_ax.set_xlim(0, 50)
        self.transfer_ax.set_ylim(0, 75)

        self.air_lines = {
            "current": self.power_ax.plot([], [], color="#ff8ba7", linewidth=2.0, label="Current A")[0],
            "battery": self.power_ax.plot([], [], color="#9ff1c7", linewidth=2.0, label="Battery V")[0],
            "angle": self.power_ax.plot([], [], color="#ffd479", linewidth=1.8, label="Angle deg")[0],
        }
        self.power_ax.legend(facecolor="#0a111c", edgecolor="#22324f", labelcolor="#dce7ff", fontsize=8)

        self.air_canvas_fig = FigureCanvasTkAgg(self.air_fig, master=self.airbrake_plot_host)
        self.air_canvas_fig.get_tk_widget().pack(fill="both", expand=True)

    def _style_axis(self, ax, title: str, ylabel: str, compact: bool = False):
        ax.set_facecolor("#0f1726")
        ax.grid(True, color="#23324d", alpha=0.7, linewidth=0.7)
        ax.tick_params(colors="#c9d5f0", labelsize=7 if compact else 9)
        ax.set_title(title, color="#edf2ff", fontsize=9 if compact else 12, pad=6)
        ax.set_ylabel(ylabel, color="#9fb0d3", fontsize=7 if compact else 9)
        ax.set_xlabel("t [s]", color="#6f84ac", fontsize=7 if compact else 9)
        for spine in ax.spines.values():
            spine.set_color("#23324d")

    def _plot_color(self, idx: int) -> str:
        palette = ["#7cc7ff", "#ffd479", "#ff8ba7", "#9ff1c7", "#d1b3ff", "#ffb366"]
        return palette[(idx - 1) % len(palette)]

    def _refresh_ports(self):
        current = self.port_var.get()
        ports = available_com_ports()
        if current and current not in ports:
            ports.insert(0, current)
        self.port_values = ports
        self.port_combo["values"] = self.port_values
        if current:
            self.port_var.set(current)
        self.status_var.set(f"Detected ports: {', '.join(self.port_values)}")

    def _on_port_selected(self, _event=None):
        selected = self.port_var.get()
        self.status_var.set(f"Selected {selected}; reconnecting serial reader...")

    def _confirm_reset(self):
        if tk.messagebox.askyesno("Reset H5", "Send RESET command to H5?\nThe board will reboot immediately."):
            self._send_command("RESET", "H5 reset")

    def _send_command(self, command: str, label: str):
        ok = self.reader.send_line(command)
        port = self.port_var.get()
        if ok:
            self.command_var.set(f"Sent {label} on {port} at {datetime.now().strftime('%H:%M:%S')}")
        else:
            self.command_var.set(f"Could not send {label}; no live serial connection on {port}")

    def update(self):
        updated = False
        while True:
            try:
                sample = self.sample_queue.get_nowait()
            except queue.Empty:
                break
            self.latest_sample = sample
            self.history.add(sample)
            self.logger.write(sample)
            updated = True

        # Drain camera echo messages from the serial reader.
        while True:
            try:
                line = self.reader.camera_queue.get_nowait()
            except queue.Empty:
                break
            self._process_camera_echo(line)

        self.status_var.set(self.reader.status)
        self.packet_var.set(f"Packets: {self.history.packet_count} | Rate: {self.history.packet_rate_hz:5.1f} Hz")

        if updated and self.latest_sample is not None:
            self._frame_count += 1
            active_tab = self.notebook.index(self.notebook.select())
            x = self.history.x_axis()

            self._update_cards(self.latest_sample)
            if active_tab == 0:
                self._update_high_level_plots(x)
            elif active_tab == 1:
                if self._frame_count % 3 == 0:
                    self._update_low_level_plots(x)
            elif active_tab == 2:
                self._update_airbrake_plots(self.latest_sample, x)
                self._draw_airbrake_scene(self.latest_sample)

        self.root.after(UI_REFRESH_MS, self.update)

    def _update_cards(self, sample: TelemetrySample):
        health = "GOOD" if sample.aggregate_checksum_ok and sample.upstream_checksum_ok else "CHECK"
        self.metric_cards["altitude"].set(f"{sample.altitude_m:,.1f} m", f"{sample.pressure_kpa:,.2f} kPa")
        self.metric_cards["accel"].set(f"{sample.imu16_accel_mag_g:,.2f} g", f"IMU16 |a|, BMI |a| {sample.bmi_accel_mag_g:,.2f} g")
        self.metric_cards["ext_temp"].set(f"{sample.ext_temp_c:,.1f} C", f"ESC {sample.esc_temp_c:,.1f} C")
        self.metric_cards["battery"].set(f"{sample.esc_voltage_v:,.2f} V", f"{sample.esc_current_a:,.2f} A | {sample.esc_consumption_mah:,.0f} mAh")
        self.metric_cards["flap_angle"].set(f"{sample.flap_angle_deg:,.1f} deg", f"{sample.encoder_position_mm:,.2f} mm travel")
        self.metric_cards["health"].set(health, f"ESC {'valid' if sample.esc_valid else 'stale'} | {self.history.packet_rate_hz:,.1f} Hz")

        self.metric_cards["airbrake_displacement"].set(f"{sample.encoder_position_mm:,.2f} mm", "Linear displacement")
        self.metric_cards["airbrake_angle"].set(f"{sample.flap_angle_deg:,.1f} deg", "From body tube")
        self.metric_cards["airbrake_current"].set(f"{sample.esc_current_a:,.2f} A", f"{sample.esc_consumption_mah:,.0f} mAh")
        self.metric_cards["airbrake_battery"].set(f"{sample.esc_voltage_v:,.2f} V", "Airbrake bus")
        self.metric_cards["airbrake_temp"].set(f"{sample.esc_temp_c:,.1f} C", "ESC temperature")
        self.metric_cards["airbrake_erpm"].set(f"{sample.esc_erpm:,.0f}", "Electrical RPM")

    def _update_high_level_plots(self, x: list[float]):
        if not x:
            return

        self.high_lines["altitude"].set_data(x, list(self.history.series["altitude_m"]))
        self.high_lines["accel"].set_data(x, list(self.history.series["imu16_accel_mag_g"]))
        self.high_lines["ext_temp"].set_data(x, list(self.history.series["ext_temp_c"]))
        self.high_lines["esc_temp"].set_data(x, list(self.history.series["esc_temp_c"]))
        self.high_lines["battery"].set_data(x, list(self.history.series["esc_voltage_v"]))
        self.high_lines["current"].set_data(x, list(self.history.series["esc_current_a"]))

        for ax in self.high_axes:
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(left=max(0.0, x[-1] - 30.0), right=max(30.0, x[-1]))

        self.high_canvas.draw_idle()

    def _update_low_level_plots(self, x: list[float]):
        if not x:
            return

        for title, field, unit in LOW_LEVEL_PLOTS:
            y = list(self.history.series[field])
            line = self.low_level_lines[field]
            ax = line.axes
            line.set_data(x, y)
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(left=max(0.0, x[-1] - 30.0), right=max(30.0, x[-1]))
            latest = y[-1] if y else 0.0
            ax.set_title(f"{title} | {latest:.2f} {unit}", color="#edf2ff", fontsize=9, pad=6)

        self.low_canvas.draw_idle()

    def _update_airbrake_plots(self, sample: TelemetrySample, x: list[float]):
        if not x:
            return

        self.air_lines["current"].set_data(x, list(self.history.series["esc_current_a"]))
        self.air_lines["battery"].set_data(x, list(self.history.series["esc_voltage_v"]))
        self.air_lines["angle"].set_data(x, list(self.history.series["flap_angle_deg"]))
        self.transfer_point.set_offsets([[max(0.0, sample.encoder_position_mm), sample.flap_angle_deg]])

        self.power_ax.relim()
        self.power_ax.autoscale_view()
        self.power_ax.set_xlim(left=max(0.0, x[-1] - 30.0), right=max(30.0, x[-1]))
        self.air_canvas_fig.draw_idle()

    def _init_airbrake_canvas(self, width: int, height: int):
        canvas = self.rocket_canvas
        canvas.delete("all")
        self._canvas_size = (width, height)

        canvas.create_rectangle(0, 0, width, height, fill="#05070b", outline="")
        for i in range(18):
            shade = int(12 + i * 4)
            color = f"#{shade:02x}{max(0, shade - 6):02x}{max(0, shade - 10):02x}"
            canvas.create_rectangle(0, i * height / 18, width, (i + 1) * height / 18, fill=color, outline="")

        rocket_center = width * 0.44
        rocket_top = height * 0.13
        rocket_bottom = height * 0.86
        body_width = width * 0.17
        nose_height = height * 0.12
        fin_height = height * 0.08
        self._canvas_layout = (rocket_center, rocket_top, rocket_bottom, body_width)

        for i in range(36):
            frac = i / 35.0
            color = self._lerp_color("#c71c1c", "#0d0d12", frac)
            x0 = rocket_center - body_width / 2 + frac * body_width
            x1 = rocket_center - body_width / 2 + (frac + 1 / 35.0) * body_width
            canvas.create_rectangle(x0, rocket_top, x1, rocket_bottom, fill=color, outline="")

        canvas.create_polygon(
            rocket_center, rocket_top - nose_height,
            rocket_center - body_width / 2, rocket_top,
            rocket_center + body_width / 2, rocket_top,
            fill="#9f0f15", outline="#ffbac6", width=2,
        )
        canvas.create_rectangle(
            rocket_center - body_width / 2, rocket_top,
            rocket_center + body_width / 2, rocket_bottom,
            outline="#ffbac6", width=2,
        )
        canvas.create_polygon(
            rocket_center - body_width / 2, rocket_bottom,
            rocket_center - body_width * 0.92, rocket_bottom + fin_height,
            rocket_center - body_width / 2, rocket_bottom - fin_height * 0.2,
            fill="#11161e", outline="#ffbac6", width=1,
        )
        canvas.create_polygon(
            rocket_center + body_width / 2, rocket_bottom,
            rocket_center + body_width * 0.92, rocket_bottom + fin_height,
            rocket_center + body_width / 2, rocket_bottom - fin_height * 0.2,
            fill="#11161e", outline="#ffbac6", width=1,
        )
        self._draw_magnolia_flowers(canvas, rocket_center, rocket_top, rocket_bottom, body_width)

        # Pre-create dynamic items (flap shadow, flap, hinge, arc, text)
        hinge_x = rocket_center + body_width / 2
        hinge_y = rocket_top + (rocket_bottom - rocket_top) * 0.45
        self._canvas_hinge_pos = (hinge_x, hinge_y)

        self._canvas_shadow = canvas.create_polygon([0] * 8, fill="#000000", outline="", stipple="gray50")
        self._canvas_flap = canvas.create_polygon([0] * 8, fill="#9aa2ad", outline="#e1e8f0", width=2)
        self._canvas_hinge = canvas.create_oval(
            hinge_x - 7, hinge_y - 7, hinge_x + 7, hinge_y + 7,
            fill="#f5f7ff", outline="#333d4d",
        )
        self._canvas_arc = canvas.create_arc(
            hinge_x - 90, hinge_y - 90, hinge_x + 90, hinge_y + 90,
            start=90, extent=-2, style=tk.ARC, outline="#ffd479", width=3,
        )
        canvas.create_text(width * 0.76, height * 0.12, text="Airbrake Deployment", fill="#f5f7ff", font=("Segoe UI Semibold", 24))
        self._canvas_angle_text = canvas.create_text(width * 0.76, height * 0.17, text="-- deg from body tube", fill="#ffd479", font=("Bahnschrift SemiBold", 20))
        self._canvas_disp_text = canvas.create_text(width * 0.76, height * 0.22, text="-- mm travel", fill="#9ff1c7", font=("Segoe UI Semibold", 16))

    def _draw_airbrake_scene(self, sample: TelemetrySample):
        canvas = self.rocket_canvas
        width = max(760, canvas.winfo_width())
        height = max(880, canvas.winfo_height())

        if self._canvas_size != (width, height):
            self._init_airbrake_canvas(width, height)

        rocket_center, rocket_top, rocket_bottom, body_width = self._canvas_layout
        hinge_x, hinge_y = self._canvas_hinge_pos
        flap_len = body_width * 1.12
        flap_width = body_width * 0.20
        angle_rad = math.radians(sample.flap_angle_deg)
        cos_a, sin_a = math.cos(angle_rad), -math.sin(angle_rad)

        corners = [(0.0, -flap_len / 2), (flap_width, -flap_len / 2), (flap_width, flap_len / 2), (0.0, flap_len / 2)]
        points = []
        for cx, cy in corners:
            points.extend([hinge_x + cx * cos_a - cy * sin_a, hinge_y + cx * sin_a + cy * cos_a])

        shadow_points = [c + (10 if i % 2 == 0 else 8) for i, c in enumerate(points)]

        canvas.coords(self._canvas_shadow, shadow_points)
        canvas.coords(self._canvas_flap, points)
        canvas.itemconfig(self._canvas_arc, extent=-max(2, int(sample.flap_angle_deg)))
        canvas.itemconfig(self._canvas_angle_text, text=f"{sample.flap_angle_deg:0.1f} deg from body tube")
        canvas.itemconfig(self._canvas_disp_text, text=f"{sample.encoder_position_mm:0.2f} mm travel")

    def _draw_magnolia_flowers(self, canvas, center_x, top_y, bottom_y, body_width):
        span = bottom_y - top_y
        for idx in range(FLOWER_COUNT):
            frac = 0.14 + idx * (0.70 / max(1, FLOWER_COUNT - 1))
            x = center_x + ((-1) ** idx) * body_width * 0.12
            y = top_y + span * frac
            size = body_width * (0.16 if idx % 2 == 0 else 0.13)
            self._draw_flower(canvas, x, y, size)

    def _draw_flower(self, canvas, x, y, size):
        petal_fill = "#fff6f6"
        petal_outline = "#d0d7e6"
        offsets = [(0, -1.1), (0.95, -0.2), (0.65, 0.9), (-0.65, 0.9), (-0.95, -0.2)]
        for ox, oy in offsets:
            canvas.create_oval(
                x + (ox - 0.52) * size, y + (oy - 0.92) * size,
                x + (ox + 0.52) * size, y + (oy + 0.92) * size,
                fill=petal_fill, outline=petal_outline, width=1,
            )
        canvas.create_oval(x - 0.25 * size, y - 0.25 * size, x + 0.25 * size, y + 0.25 * size, fill="#f7db7b", outline="")

    def _lerp_color(self, left: str, right: str, frac: float) -> str:
        lv = tuple(int(left[i:i + 2], 16) for i in (1, 3, 5))
        rv = tuple(int(right[i:i + 2], 16) for i in (1, 3, 5))
        out = tuple(int(l + (r - l) * frac) for l, r in zip(lv, rv))
        return "#{:02x}{:02x}{:02x}".format(*out)

    def run(self):
        self.reader.start()
        self.root.after(UI_REFRESH_MS, self.update)
        self.root.mainloop()

    def on_close(self):
        self.stop_event.set()
        self.logger.close()
        self.root.destroy()


if __name__ == "__main__":
    app = GroundStationApp()
    app.run()
