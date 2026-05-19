import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
from datetime import datetime

CSV_PATH = "ground_station_telemetry_20260411_150007.csv"

# ── Load & clean ──────────────────────────────────────────────────────────────
df = pd.read_csv(CSV_PATH, parse_dates=["pc_time_iso"])
df = df.sort_values("pc_time_iso").reset_index(drop=True)

# Relative time in seconds from first row
t0 = df["pc_time_iso"].iloc[0]
df["t_s"] = (df["pc_time_iso"] - t0).dt.total_seconds()

# Filter encoder: drop rows with clearly corrupted values (|enc| > 1e4 mm)
df["enc_clean"] = df["encoder_position_mm"].where(df["encoder_position_mm"].abs() < 1e4)

# ── Identify key events ───────────────────────────────────────────────────────
# Flight window: altitude rises above 0 and then returns
flight_mask = df["altitude_m"] > 0
flight_start_idx = flight_mask.idxmax()
flight_end_idx   = len(df) - 1 - flight_mask[::-1].values.argmax()

df_flight = df.iloc[flight_start_idx:flight_end_idx + 1].copy()

t_launch  = df_flight["t_s"].iloc[0]
t_apogee  = df_flight.loc[df_flight["altitude_m"].idxmax(), "t_s"]
alt_apogee = df_flight["altitude_m"].max()

# Burnout: sustained drop in imu4 accel from ~4 g → <1 g
# Motor burn phase is first ~1.3 s of flight; burnout when accel falls <1.5 g
burn_mask  = (df_flight["t_s"] < t_launch + 3) & (df_flight["imu4_accel_mag_g"] < 1.5)
t_burnout  = df_flight.loc[burn_mask.idxmax(), "t_s"] if burn_mask.any() else None

# Airbrake command: first time ESC erpm rises above a threshold during ascent
AB_RPM_THRESHOLD = 500
ascent_mask = df_flight["altitude_m"] < alt_apogee
ab_mask = ascent_mask & (df_flight["esc_erpm"] > AB_RPM_THRESHOLD)
if ab_mask.any():
    t_airbrake     = df_flight.loc[ab_mask.idxmax(), "t_s"]
    alt_airbrake   = df_flight.loc[ab_mask.idxmax(), "altitude_m"]
else:
    t_airbrake = t_apogee  # fallback
    alt_airbrake = alt_apogee

# Landing: last time altitude > 0 during descent
descent_mask = (df_flight["t_s"] > t_apogee) & (df_flight["altitude_m"] > 0)
t_landing = df_flight.loc[descent_mask[::-1].idxmax(), "t_s"] if descent_mask.any() else df_flight["t_s"].iloc[-1]

# ── Smooth noisy channels ─────────────────────────────────────────────────────
WINDOW = 5
df_flight["alt_smooth"]   = df_flight["altitude_m"].rolling(WINDOW, center=True, min_periods=1).mean()
df_flight["accel_smooth"] = df_flight["imu4_accel_mag_g"].rolling(WINDOW, center=True, min_periods=1).mean()
df_flight["gyro_smooth"]  = df_flight["bmi_gyro_mag_dps"].rolling(WINDOW, center=True, min_periods=1).mean()

# ── Event annotation helper ───────────────────────────────────────────────────
def vline_annotate(ax, t, label, color, y_frac=0.92, linestyle="--"):
    ax.axvline(t, color=color, linestyle=linestyle, linewidth=1.2, alpha=0.8)
    ylim = ax.get_ylim()
    ypos = ylim[0] + y_frac * (ylim[1] - ylim[0])
    ax.text(t + 0.3, ypos, label, color=color, fontsize=7.5, va="top",
            rotation=90, ha="left")

# ── Figure layout ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 11))
fig.suptitle(
    f"Flight Analysis — UGA Spaceport  |  {t0.strftime('%Y-%m-%d %H:%M:%S')}\n"
    f"Apogee: {alt_apogee:.0f} m  |  Airbrake cmd @ {alt_airbrake:.0f} m",
    fontsize=13, fontweight="bold", y=0.98
)

gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.55, wspace=0.35)

t = df_flight["t_s"]

EVENT_COLORS = {
    "Launch":   "#2ecc71",
    "Burnout":  "#e67e22",
    "Airbrakes":"#e74c3c",
    "Apogee":   "#9b59b6",
    "Landing":  "#3498db",
}

# ── 1. Altitude vs time (spans full width) ────────────────────────────────────
ax_alt = fig.add_subplot(gs[0, :])
ax_alt.fill_between(t, df_flight["alt_smooth"], alpha=0.15, color="#3498db")
ax_alt.plot(t, df_flight["alt_smooth"], color="#2980b9", linewidth=1.8, label="Altitude (smoothed)")
ax_alt.scatter(t_apogee, alt_apogee, color=EVENT_COLORS["Apogee"], zorder=5, s=80,
               label=f"Apogee {alt_apogee:.0f} m")
ax_alt.scatter(t_airbrake, alt_airbrake, color=EVENT_COLORS["Airbrakes"], zorder=5,
               s=80, marker="^", label=f"Airbrake cmd {alt_airbrake:.0f} m")
ax_alt.set_ylabel("Altitude (m)", fontsize=9)
ax_alt.set_title("Altitude Profile", fontsize=10)
ax_alt.legend(fontsize=8, loc="upper right")
ax_alt.grid(True, alpha=0.3)
ax_alt.set_xlabel("Time from start (s)", fontsize=9)

for name, t_ev in [("Launch", t_launch), ("Burnout", t_burnout),
                    ("Airbrakes", t_airbrake), ("Apogee", t_apogee), ("Landing", t_landing)]:
    if t_ev is None:
        continue
    vline_annotate(ax_alt, t_ev, name, EVENT_COLORS[name])

# ── 2. Acceleration magnitude ─────────────────────────────────────────────────
ax_acc = fig.add_subplot(gs[1, 0])
ax_acc.plot(t, df_flight["accel_smooth"], color="#e67e22", linewidth=1.5)
ax_acc.set_ylabel("Accel magnitude (g)", fontsize=9)
ax_acc.set_title("IMU Acceleration (imu4)", fontsize=10)
ax_acc.grid(True, alpha=0.3)
ax_acc.set_xlabel("Time (s)", fontsize=9)
for name, t_ev in [("Launch", t_launch), ("Burnout", t_burnout), ("Airbrakes", t_airbrake), ("Apogee", t_apogee)]:
    if t_ev is None: continue
    vline_annotate(ax_acc, t_ev, name, EVENT_COLORS[name])

# ── 3. ESC ERPM (airbrake motor state) ───────────────────────────────────────
ax_esc = fig.add_subplot(gs[1, 1])
esc_clean = df_flight["esc_erpm"].where(df_flight["esc_erpm"].abs() < 50000)
ax_esc.plot(t, esc_clean, color="#e74c3c", linewidth=1.4)
ax_esc.set_ylabel("ESC eRPM", fontsize=9)
ax_esc.set_title("Airbrake Motor ESC (eRPM)", fontsize=10)
ax_esc.grid(True, alpha=0.3)
ax_esc.set_xlabel("Time (s)", fontsize=9)
for name, t_ev in [("Launch", t_launch), ("Airbrakes", t_airbrake), ("Apogee", t_apogee)]:
    if t_ev is None: continue
    vline_annotate(ax_esc, t_ev, name, EVENT_COLORS[name])

# ── 4. Gyro magnitude ─────────────────────────────────────────────────────────
ax_gyro = fig.add_subplot(gs[2, 0])
ax_gyro.plot(t, df_flight["gyro_smooth"], color="#8e44ad", linewidth=1.4)
ax_gyro.set_ylabel("Gyro magnitude (°/s)", fontsize=9)
ax_gyro.set_title("Spin Rate (BMI Gyro)", fontsize=10)
ax_gyro.grid(True, alpha=0.3)
ax_gyro.set_xlabel("Time (s)", fontsize=9)
for name, t_ev in [("Launch", t_launch), ("Burnout", t_burnout), ("Apogee", t_apogee)]:
    if t_ev is None: continue
    vline_annotate(ax_gyro, t_ev, name, EVENT_COLORS[name])

# ── 5. External temperature ───────────────────────────────────────────────────
ax_temp = fig.add_subplot(gs[2, 1])
ax_temp.plot(t, df_flight["ext_temp_c"], color="#16a085", linewidth=1.4)
ax_temp.set_ylabel("Temperature (°C)", fontsize=9)
ax_temp.set_title("External Temperature", fontsize=10)
ax_temp.grid(True, alpha=0.3)
ax_temp.set_xlabel("Time (s)", fontsize=9)
for name, t_ev in [("Launch", t_launch), ("Apogee", t_apogee), ("Landing", t_landing)]:
    if t_ev is None: continue
    vline_annotate(ax_temp, t_ev, name, EVENT_COLORS[name])

# ── 6. Ascent rate (altitude derivative) ─────────────────────────────────────
ax_vel = fig.add_subplot(gs[3, 0])
dt = t.diff().replace(0, np.nan)
vel = df_flight["alt_smooth"].diff() / dt
vel_smooth = vel.rolling(7, center=True, min_periods=1).mean()
ax_vel.plot(t, vel_smooth, color="#27ae60", linewidth=1.5)
ax_vel.axhline(0, color="gray", linewidth=0.8, linestyle=":")
ax_vel.set_ylabel("Vertical velocity (m/s)", fontsize=9)
ax_vel.set_title("Estimated Vertical Velocity", fontsize=10)
ax_vel.grid(True, alpha=0.3)
ax_vel.set_xlabel("Time (s)", fontsize=9)
for name, t_ev in [("Launch", t_launch), ("Burnout", t_burnout), ("Airbrakes", t_airbrake), ("Apogee", t_apogee)]:
    if t_ev is None: continue
    vline_annotate(ax_vel, t_ev, name, EVENT_COLORS[name])

# ── 7. Pressure ───────────────────────────────────────────────────────────────
ax_pres = fig.add_subplot(gs[3, 1])
pres_smooth = df_flight["pressure_kpa"].rolling(WINDOW, center=True, min_periods=1).mean()
ax_pres.plot(t, pres_smooth, color="#c0392b", linewidth=1.4)
ax_pres.set_ylabel("Pressure (kPa)", fontsize=9)
ax_pres.set_title("Barometric Pressure", fontsize=10)
ax_pres.grid(True, alpha=0.3)
ax_pres.set_xlabel("Time (s)", fontsize=9)
for name, t_ev in [("Apogee", t_apogee), ("Landing", t_landing)]:
    if t_ev is None: continue
    vline_annotate(ax_pres, t_ev, name, EVENT_COLORS[name])

# ── Legend strip for event lines ──────────────────────────────────────────────
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], color=EVENT_COLORS[n], linestyle="--", linewidth=1.4, label=n)
    for n in EVENT_COLORS
]
fig.legend(handles=legend_elements, loc="lower center", ncol=5,
           fontsize=8.5, framealpha=0.9, bbox_to_anchor=(0.5, 0.002))

plt.savefig("flight_analysis.png", dpi=150, bbox_inches="tight")
print("Saved flight_analysis.png")

# ── Print summary ─────────────────────────────────────────────────────────────
print("\n== Flight Summary ==")
print(f"  Launch time (abs):    {t0 + pd.to_timedelta(t_launch, unit='s')}")
print(f"  Launch (t=0 offset):  {t_launch:.1f} s from data start")
if t_burnout:
    print(f"  Motor burnout:        t+{t_burnout - t_launch:.1f} s")
print(f"  Airbrake command:     t+{t_airbrake - t_launch:.1f} s  |  {alt_airbrake:.0f} m")
print(f"  Apogee:               t+{t_apogee - t_launch:.1f} s  |  {alt_apogee:.0f} m")
print(f"  Landing:              t+{t_landing - t_launch:.1f} s")
print(f"  Total flight time:    {t_landing - t_launch:.1f} s")

plt.show()
