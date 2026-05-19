"""
Hardware-In-the-Loop test harness for the STM32H5 airbrakes controller.

Builds and transmits aggregate packets over UART exactly as the STM32C5 would,
so the H5 processes them as real sensor data.  Two data sources are supported:

  csv   -- replay a CSV log captured by the ground station (same CSV_FIELDS)
  synth -- generate a synthetic vertical-flight trajectory

Usage:
  python hil_test.py --source synth --port COM4 --baud 115200
  python hil_test.py --source csv --file flight_log.csv --port COM4 --rate 1.0
  python hil_test.py --source synth --port COM4 --rate 2.0   # 2x real-time

The script also listens for ASCII response lines from the H5 and prints them.
"""

import argparse
import csv
import math
import struct
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial
from serial.tools import list_ports

# ── Packet constants (must match ground station / firmware) ──────────────────
AGGREGATE_HEADER = 0xA55A
UPSTREAM_HEADER  = 0xAA55
PACKET_LEN       = 100
DEFAULT_BAUD     = 115200
DEFAULT_RATE_HZ  = 50.0   # matches C5 output rate

AGGREGATE_FORMAT = "<" + "H" + "Hii" + "f" * 16 + "B" + "B" + "B" + "I" + "I" + "I" + "I" + "i" + "B"
PACKET_STRUCT    = struct.Struct(AGGREGATE_FORMAT)


# ── Packet builder ────────────────────────────────────────────────────────────

def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


@dataclass
class PacketFields:
    altitude_cm: int          = 0
    pressure_pa: int          = 101325
    imu16_ax_g: float         = 0.0
    imu16_ay_g: float         = 0.0
    imu16_az_g: float         = 1.0
    imu4_ax_g: float          = 0.0
    imu4_ay_g: float          = 0.0
    imu4_az_g: float          = 1.0
    mag_x_gauss: float        = 0.0
    mag_y_gauss: float        = 0.0
    mag_z_gauss: float        = 0.5
    bmi_ax_g: float           = 0.0
    bmi_ay_g: float           = 0.0
    bmi_az_g: float           = 1.0
    bmi_gx_dps: float         = 0.0
    bmi_gy_dps: float         = 0.0
    bmi_gz_dps: float         = 0.0
    ext_temp_c: float         = 20.0
    esc_valid: bool            = False
    esc_temp_c: int            = 25
    esc_voltage_mv: int        = 16800
    esc_current_ma: int        = 0
    esc_consumption_mah: int   = 0
    esc_erpm: int              = 0
    encoder_position_um: int   = 0


def build_packet(f: PacketFields) -> bytes:
    """Pack a PacketFields into a 100-byte aggregate packet with valid checksums."""
    imu_floats = (
        f.imu16_ax_g, f.imu16_ay_g, f.imu16_az_g,
        f.imu4_ax_g,  f.imu4_ay_g,  f.imu4_az_g,
        f.mag_x_gauss, f.mag_y_gauss, f.mag_z_gauss,
        f.bmi_ax_g,  f.bmi_ay_g,  f.bmi_az_g,
        f.bmi_gx_dps, f.bmi_gy_dps, f.bmi_gz_dps,
        f.ext_temp_c,
    )

    # upstream_bytes = packet[2:77]; upstream_checksum = xor(upstream_bytes[2:-1]) = xor(packet[4:76])
    # packet[4:76] = altitude_cm(4) + pressure_pa(4) + 16 floats(64) = 72 bytes
    upstream_checksum = xor_checksum(struct.pack("<ii" + "f" * 16, f.altitude_cm, f.pressure_pa, *imu_floats))

    # First pass: build packet with placeholder aggregate_checksum = 0
    packet = bytearray(PACKET_STRUCT.pack(
        AGGREGATE_HEADER,
        UPSTREAM_HEADER,
        f.altitude_cm,
        f.pressure_pa,
        *imu_floats,
        upstream_checksum,
        int(f.esc_valid),
        f.esc_temp_c,
        f.esc_voltage_mv,
        f.esc_current_ma,
        f.esc_consumption_mah,
        f.esc_erpm,
        f.encoder_position_um,
        0,  # aggregate_checksum placeholder
    ))

    # aggregate_checksum covers packet[2:-1] (bytes 2 through 98 inclusive)
    packet[-1] = xor_checksum(packet[2:-1])
    return bytes(packet)


# ── Data sources ──────────────────────────────────────────────────────────────

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def synthetic_flight(rate_hz: float):
    """
    Yield (PacketFields, wall_delay_s) for a synthetic vertical flight.

    Timeline (real-time seconds):
      0–3   s  launch / boost  ~15 g axial
      3–12  s  coast           ~1 g (gravity)
      12    s  apogee
      12–30 s  descent
    """
    dt = 1.0 / rate_hz
    t = 0.0

    alt_m = 0.0
    vel_ms = 0.0
    consumption_mah = 0

    while True:
        # Simple point-mass trajectory
        if t < 3.0:
            accel_ms2 = 140.0 - t * 10.0   # ~15 g at launch, tapering
        elif t < 12.0:
            accel_ms2 = -9.81
        else:
            accel_ms2 = -9.81

        vel_ms  += accel_ms2 * dt
        alt_m   = max(0.0, alt_m + vel_ms * dt)

        axial_g = accel_ms2 / 9.81

        # Pressure via barometric formula (rough)
        pressure_pa = int(101325.0 * math.exp(-alt_m / 8500.0))

        esc_on = t < 3.0
        esc_current_ma = int(_clamp(abs(accel_ms2) * 40, 0, 8000)) if esc_on else 0
        consumption_mah += int(esc_current_ma * dt / 3600)

        f = PacketFields(
            altitude_cm          = int(alt_m),
            pressure_pa          = pressure_pa,
            imu16_ax_g           = 0.0,
            imu16_ay_g           = 0.0,
            imu16_az_g           = float(axial_g),
            imu4_ax_g            = 0.0,
            imu4_ay_g            = 0.0,
            imu4_az_g            = float(axial_g),
            mag_x_gauss          = 0.18,
            mag_y_gauss          = 0.02,
            mag_z_gauss          = 0.46,
            bmi_ax_g             = 0.0,
            bmi_ay_g             = 0.0,
            bmi_az_g             = float(axial_g),
            bmi_gx_dps           = 0.0,
            bmi_gy_dps           = 0.0,
            bmi_gz_dps           = 0.0,
            ext_temp_c           = 20.0 - alt_m * 0.006,
            esc_valid            = esc_on,
            esc_temp_c           = int(_clamp(25 + esc_current_ma / 200, 25, 80)),
            esc_voltage_mv       = int(_clamp(16800 - consumption_mah * 2, 14000, 16800)),
            esc_current_ma       = esc_current_ma,
            esc_consumption_mah  = consumption_mah,
            esc_erpm             = int(esc_current_ma * 50) if esc_on else 0,
            encoder_position_um  = 0,
        )

        yield f, dt
        t += dt
        if alt_m == 0.0 and t > 15.0:
            break


def csv_flight(path: Path, rate_hz: float):
    """
    Yield (PacketFields, wall_delay_s) by replaying a ground-station CSV log.
    Timing is derived from the packet_time_s column; wall_delay is scaled by
    rate_hz vs the original sample rate.
    """
    with open(path, newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        rows = list(reader)

    if not rows:
        raise ValueError(f"CSV {path} is empty")

    base_t = float(rows[0]["packet_time_s"])

    for i, row in enumerate(rows):
        pkt_t   = float(row["packet_time_s"])
        next_t  = float(rows[i + 1]["packet_time_s"]) if i + 1 < len(rows) else pkt_t + 1.0 / rate_hz
        real_dt = next_t - pkt_t

        alt_m        = float(row["altitude_m"])
        pressure_kpa = float(row["pressure_kpa"])
        enc_mm       = float(row["encoder_position_mm"])

        f = PacketFields(
            altitude_cm          = int(alt_m),
            pressure_pa          = int(pressure_kpa * 1000),
            imu16_ax_g           = float(row["imu16_ax_g"]),
            imu16_ay_g           = float(row["imu16_ay_g"]),
            imu16_az_g           = float(row["imu16_az_g"]),
            imu4_ax_g            = float(row["imu4_ax_g"]),
            imu4_ay_g            = float(row["imu4_ay_g"]),
            imu4_az_g            = float(row["imu4_az_g"]),
            mag_x_gauss          = float(row["mag_x_gauss"]),
            mag_y_gauss          = float(row["mag_y_gauss"]),
            mag_z_gauss          = float(row["mag_z_gauss"]),
            bmi_ax_g             = float(row["bmi_ax_g"]),
            bmi_ay_g             = float(row["bmi_ay_g"]),
            bmi_az_g             = float(row["bmi_az_g"]),
            bmi_gx_dps           = float(row["bmi_gx_dps"]),
            bmi_gy_dps           = float(row["bmi_gy_dps"]),
            bmi_gz_dps           = float(row["bmi_gz_dps"]),
            ext_temp_c           = float(row["ext_temp_c"]),
            esc_valid            = bool(int(row["esc_valid"])),
            esc_temp_c           = int(float(row["esc_temp_c"])),
            esc_voltage_mv       = int(float(row["esc_voltage_v"]) * 1000),
            esc_current_ma       = int(float(row["esc_current_a"]) * 1000),
            esc_consumption_mah  = int(float(row["esc_consumption_mah"])),
            esc_erpm             = int(float(row["esc_erpm"])),
            encoder_position_um  = int(enc_mm * 1000),
        )

        yield f, real_dt


# ── H5 response listener ──────────────────────────────────────────────────────

class ResponseListener(threading.Thread):
    """Reads ASCII lines from the H5 and prints them."""

    def __init__(self, ser: serial.Serial, stop: threading.Event):
        super().__init__(daemon=True)
        self.ser   = ser
        self.stop  = stop
        self._buf  = b""

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
                    ts = time.strftime("%H:%M:%S")
                    print(f"[{ts}] H5: {text}")


# ── Main ──────────────────────────────────────────────────────────────────────

def list_ports_str() -> list[str]:
    return sorted(p.device for p in list_ports.comports())


def resolve_port(requested: str | None) -> str | None:
    """
    Return the port to use, or None to run in dry-run mode.

    - Explicit --port: use it as-is.
    - No --port, one port found: auto-select it.
    - No --port, multiple ports: print list and ask user to pick.
    - No --port, no ports: warn and return None (dry-run).
    """
    if requested:
        return requested

    available = list_ports_str()

    if not available:
        print("No COM ports detected — running in dry-run mode (packets will not be transmitted).")
        return None

    if len(available) == 1:
        print(f"Auto-selected port: {available[0]}")
        return available[0]

    print("Multiple COM ports detected:")
    for i, p in enumerate(available):
        print(f"  [{i}] {p}")
    while True:
        try:
            choice = input("Select port number (or Enter to dry-run): ").strip()
        except EOFError:
            choice = ""
        if choice == "":
            print("No port selected — running in dry-run mode.")
            return None
        if choice.isdigit() and 0 <= int(choice) < len(available):
            return available[int(choice)]
        print("  Invalid selection, try again.")


def run_hil(args):
    port = resolve_port(args.port)
    dry_run = port is None

    print(f"HIL test  port={'(dry-run)' if dry_run else port}  baud={args.baud}  "
          f"source={args.source}  rate={args.rate}x")

    if args.source == "csv":
        if not args.file:
            raise ValueError("--file is required for --source csv")
        source = csv_flight(Path(args.file), DEFAULT_RATE_HZ)
    else:
        source = synthetic_flight(DEFAULT_RATE_HZ)

    stop = threading.Event()
    ser  = None

    if not dry_run:
        try:
            ser = serial.Serial(port, args.baud, timeout=0.05)
        except serial.SerialException as exc:
            print(f"Cannot open {port}: {exc}")
            print(f"Available ports: {list_ports_str()}")
            return
        ResponseListener(ser, stop).start()

    packet_count = 0
    t_start      = time.perf_counter()

    print("Sending packets — press Ctrl+C to stop.\n")

    try:
        for pkt_fields, nominal_dt in source:
            packet = build_packet(pkt_fields)

            if ser is not None:
                ser.write(packet)
                ser.flush()

            packet_count += 1

            elapsed   = time.perf_counter() - t_start
            ideal_t   = packet_count * nominal_dt / args.rate
            sleep_for = ideal_t - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

            if packet_count % 50 == 0:
                elapsed = time.perf_counter() - t_start
                print(
                    f"  [{elapsed:7.2f}s]  pkt={packet_count:5d}  "
                    f"alt={pkt_fields.altitude_cm:5d} m  "
                    f"az={pkt_fields.imu16_az_g:+.2f} g  "
                    f"enc={pkt_fields.encoder_position_um / 1000.0:.2f} mm"
                    + ("  [dry-run]" if dry_run else "")
                )

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        stop.set()
        if ser is not None:
            ser.close()
        elapsed = time.perf_counter() - t_start
        print(f"\nSent {packet_count} packets in {elapsed:.2f}s  "
              f"(avg {packet_count / max(elapsed, 1e-9):.1f} Hz)")


def main():
    parser = argparse.ArgumentParser(
        description="HIL packet injector for the STM32H5 airbrakes controller"
    )
    parser.add_argument("--port",   default=None,
                        help="Serial port connected to H5 UART (auto-detected if omitted)")
    parser.add_argument("--baud",   type=int, default=DEFAULT_BAUD)
    parser.add_argument("--source", choices=["synth", "csv"], default="synth",
                        help="Data source: synthetic trajectory or CSV replay")
    parser.add_argument("--file",   default=None,
                        help="Path to ground-station CSV (required for --source csv)")
    parser.add_argument("--rate",   type=float, default=1.0,
                        help="Playback rate multiplier (e.g. 2.0 = 2x real-time)")
    parser.add_argument("--list-ports", action="store_true", help="Print available COM ports and exit")

    args = parser.parse_args()

    if args.list_ports:
        print("Available ports:", list_ports_str())
        return

    run_hil(args)


if __name__ == "__main__":
    main()
