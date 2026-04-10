import re
import csv
import time
import threading
from datetime import datetime
from pathlib import Path

import serial
from pynput import keyboard

PORT = "COM3"      # change this
BAUD = 115200

pressed = set()
running = True

neutral_us = 1500
TRIM_STEP_US = 5
TRIM_MIN_US = 1300
TRIM_MAX_US = 1700

ser_lock = threading.Lock()
state_lock = threading.Lock()
log_lock = threading.Lock()

LOG_DIR = Path(".")
LOG_FILENAME = f"serial_rx_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
LOG_PATH = LOG_DIR / LOG_FILENAME

CSV_TELEMETRY_HEADER = "Time_s,Position_um,Temp_C,Voltage_mV,Current_mA,Consumption_mAh,eRPM"
CSV_TELEMETRY_ROW_RE = re.compile(
    r"^\s*-?\d+(?:\.\d+)?,\s*-?\d+,\s*-?\d+,\s*\d+,\s*\d+,\s*\d+,\s*\d+\s*$"
)

rx_log_fp = open(LOG_PATH, "w", newline="", encoding="utf-8")
rx_log_writer = csv.writer(rx_log_fp)
rx_log_writer.writerow(["pc_time_iso", "pc_time_epoch", "kind", "raw_line"])
rx_log_fp.flush()


def open_serial():
    s = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2.0)
    return s


ser = open_serial()


def log_rx_line(kind: str, line: str):
    with log_lock:
        now = time.time()
        rx_log_writer.writerow([datetime.now().isoformat(timespec="milliseconds"), f"{now:.3f}", kind, line])
        rx_log_fp.flush()


def classify_rx_line(line: str) -> str:
    if line == CSV_TELEMETRY_HEADER:
        return "telemetry_header"
    if CSV_TELEMETRY_ROW_RE.match(line):
        return "telemetry_sample"
    return "text"


def send_raw(cmd: str):
    global running
    try:
        with ser_lock:
            ser.write((cmd + "\n").encode("ascii"))
            ser.flush()
    except Exception as e:
        print(f"\n[Serial write error] {e}")
        running = False


def send_neutral_value(value_us: int):
    global neutral_us
    value_us = max(TRIM_MIN_US, min(TRIM_MAX_US, int(value_us)))
    with state_lock:
        neutral_us = value_us
    send_raw(f"NEUTRAL {value_us}")


def send_direct_pulse(value_us: int):
    value_us = max(1000, min(2000, int(value_us)))
    send_raw(f"N{value_us}")


def send_stop():
    send_raw("S")


def send_zero_encoder():
    send_raw("O")
    print("\n[PC] Encoder zero command sent (O)")


def sync_neutral_and_stop():
    with state_lock:
        current = neutral_us
    send_neutral_value(current)
    send_stop()


def send_motion():
    up = keyboard.Key.up in pressed
    down = keyboard.Key.down in pressed

    with state_lock:
        current = neutral_us

    if up and not down:
        send_raw("B")
        print(f"\r[PC] FORWARD  (neutral={current} us) ", end="", flush=True)
    elif down and not up:
        send_raw("F")
        print(f"\r[PC] REVERSE  (neutral={current} us) ", end="", flush=True)
    else:
        send_stop()
        print(f"\r[PC] STOP     (neutral={current} us) ", end="", flush=True)


def send_deploy():
    send_raw("67")
    print("\n[PC] DEPLOY trigger sent (67)")


def send_home():
    send_raw("H")
    print("\n[PC] HOMING trigger sent (H)")


def send_percent(percent: int):
    percent = max(-100, min(100, int(percent)))
    send_raw(str(percent))
    print(f"\n[PC] Percent command sent: {percent}%")


def maybe_update_local_state_from_line(line: str):
    global neutral_us

    m = re.search(r"(?:NEUTRAL set to|Starting neutral:)\s+(\d+)\s*us", line, re.IGNORECASE)
    if m:
        value = int(m.group(1))
        value = max(TRIM_MIN_US, min(TRIM_MAX_US, value))
        with state_lock:
            neutral_us = value

    if "DEPLOYMENT SEQUENCE INITIATED" in line:
        print("\n[PC] Firmware is now in deployment sequence; further input may be ignored until completion.")
    elif "HOMING SEQUENCE INITIATED" in line or "Starting Homing Sequence" in line:
        print("\n[PC] Firmware is now in homing sequence; further input may be ignored until completion.")
    elif "Encoder position set to 0" in line:
        print("\n[PC] Firmware reports encoder position reset to zero.")


def serial_reader():
    global running
    while running:
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    kind = classify_rx_line(line)
                    log_rx_line(kind, line)

                    if kind != "telemetry_sample" and kind != "telemetry_header":
                        maybe_update_local_state_from_line(line)
                        print(f"\n[STM32] {line}")
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"\n[Serial read error] {e}")
            running = False
            break


HELP_TEXT = """
Console commands:
  f / b / s / stop        -> firmware slow forward / reverse / stop
  neutral 1500            -> set neutral trim (1300..1700)
  pulse 1600              -> direct pulse width (1000..2000)
  pct 25                  -> percentage throttle (-100..100)
  deploy                  -> send 67
  home                    -> send H
  o / zero                -> send O (set encoder position to 0)
  raw <anything>          -> send exactly what follows
  help                    -> print this help
  quit / exit             -> stop script
"""


def command_input_thread():
    global running

    while running:
        try:
            raw = input("\ncmd> ").strip()
        except EOFError:
            raw = "quit"
        except Exception as e:
            print(f"\n[Input error] {e}")
            running = False
            break

        if not running:
            break

        if raw == "":
            continue

        lower = raw.lower()

        try:
            if lower in ("help", "h?"):
                print(HELP_TEXT)

            elif lower in ("quit", "exit"):
                pressed.clear()
                send_stop()
                running = False
                print("\nExiting command thread.")

            elif lower == "f":
                send_raw("F")

            elif lower == "b":
                send_raw("B")

            elif lower in ("s", "stop"):
                send_stop()

            elif lower == "deploy":
                send_deploy()

            elif lower == "home":
                send_home()

            elif lower in ("o", "zero"):
                send_zero_encoder()

            elif lower.startswith("neutral "):
                value = int(raw.split(None, 1)[1])
                send_neutral_value(value)
                send_stop()

            elif lower.startswith("pulse "):
                value = int(raw.split(None, 1)[1])
                send_direct_pulse(value)

            elif lower.startswith("pct "):
                value = int(raw.split(None, 1)[1])
                send_percent(value)

            elif lower.startswith("raw "):
                send_raw(raw[4:])

            else:
                send_raw(raw)

        except ValueError:
            print("[PC] Bad numeric value.")
        except Exception as e:
            print(f"[PC] Command handling error: {e}")


def on_press(key):
    global running, neutral_us

    try:
        if key in (keyboard.Key.up, keyboard.Key.down):
            if key not in pressed:
                pressed.add(key)
                send_motion()

        elif key == keyboard.Key.left:
            with state_lock:
                neutral_us = max(TRIM_MIN_US, neutral_us - TRIM_STEP_US)
                current = neutral_us
            send_neutral_value(current)
            print(f"\r[PC] TRIM neutral -> {current} us (LEFT)  ", end="", flush=True)
            if keyboard.Key.up not in pressed and keyboard.Key.down not in pressed:
                send_stop()

        elif key == keyboard.Key.right:
            with state_lock:
                neutral_us = min(TRIM_MAX_US, neutral_us + TRIM_STEP_US)
                current = neutral_us
            send_neutral_value(current)
            print(f"\r[PC] TRIM neutral -> {current} us (RIGHT) ", end="", flush=True)
            if keyboard.Key.up not in pressed and keyboard.Key.down not in pressed:
                send_stop()

        elif key == keyboard.Key.space:
            pressed.clear()
            send_stop()
            with state_lock:
                current = neutral_us
            print(f"\r[PC] SPACE STOP   (neutral={current} us) ", end="", flush=True)

        elif key == keyboard.Key.f5:
            send_deploy()

        elif key == keyboard.Key.f6:
            send_home()

        elif key == keyboard.Key.f7:
            send_zero_encoder()

        elif key == keyboard.Key.f1:
            print_runtime_help()

        elif key == keyboard.Key.esc:
            pressed.clear()
            send_stop()
            running = False
            with state_lock:
                current = neutral_us
            print(f"\nFinal neutral value: {current} us")
            print("Exiting.")
            return False

    except Exception as e:
        print(f"\n[Key press error] {e}")


def on_release(key):
    try:
        if key in pressed:
            pressed.discard(key)
            send_motion()
    except Exception as e:
        print(f"\n[Key release error] {e}")


def print_runtime_help():
    with state_lock:
        current = neutral_us

    print("STM32 ESC / Airbrake console")
    print("────────────────────────────")
    print(f"  Current local neutral = {current} us")
    print(f"  RX log CSV            = {LOG_PATH}")
    print()
    print("Keyboard:")
    print("  UP / DOWN  = hold slow forward / reverse")
    print("  LEFT/RIGHT = trim neutral by 5 us")
    print("  SPACE      = stop")
    print("  F5         = deployment trigger (67)")
    print("  F6         = homing trigger (H)")
    print("  F7         = set encoder position to 0 (O)")
    print("  F1         = print help")
    print("  ESC        = quit")
    print()
    print(HELP_TEXT.strip())
    print()
    print("Notes:")
    print("  - High-rate CSV telemetry rows are suppressed from the console.")
    print("  - Every received serial line is still saved to the RX log CSV.")
    print("  - Deployment and homing are sequence actions.")
    print("  - Firmware may ignore more input until they finish.")


if __name__ == "__main__":
    print_runtime_help()
    print()
    print("Syncing neutral to firmware and sending stop...")
    sync_neutral_and_stop()

    reader_thread = threading.Thread(target=serial_reader, daemon=True)
    reader_thread.start()

    input_thread = threading.Thread(target=command_input_thread, daemon=True)
    input_thread.start()

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    running = False
    try:
        send_stop()
        time.sleep(0.2)
    finally:
        with ser_lock:
            ser.close()
        with log_lock:
            rx_log_fp.flush()
            rx_log_fp.close()