import rocketpy
import datetime

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
print("hello world")

import os
import re
import numpy as np

# ===================== AoA CSV setup =====================
# Put your AoA fit CSVs in: ./AoA_Fits_Output/
AOA_CSV_FILES = [
    "AoA_0deg_CdFit.csv",
    "AoA_2deg_CdFit.csv",
    "AoA_4deg_CdFit.csv",
    "AoA_6deg_CdFit.csv",
    "AoA_8deg_CdFit.csv",
]

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
AOA_DIR  = os.path.join(BASE_DIR, "AoA_Fits_Output")

def _infer_mach_cd_columns(df_):
    if "MachNumber___" in df_.columns and "x_DragCoefficient___" in df_.columns:
        return "MachNumber___", "x_DragCoefficient___"

    mach_candidates = [c for c in df_.columns if re.search(r"mach", c, re.IGNORECASE)]
    cd_candidates   = [c for c in df_.columns if re.search(r"(drag.*coeff|cd\b|c_d\b|dragcoefficient)", c, re.IGNORECASE)]
    if not mach_candidates or not cd_candidates:
        raise ValueError(f"Could not infer Mach/Cd columns from columns: {list(df_.columns)}")
    return mach_candidates[0], cd_candidates[0]

def _normalize_to_drag_table(df_):
    mach_col, cd_col = _infer_mach_cd_columns(df_)

    df_ = df_.drop_duplicates(subset=mach_col, keep="first")
    df_ = df_.sort_values(mach_col).reset_index(drop=True)

    if float(df_[mach_col].iloc[0]) > 0.0:
        df_ = pd.concat(
            [pd.DataFrame({mach_col: [0.0], cd_col: [float(df_[cd_col].iloc[0])]}), df_],
            ignore_index=True
        )

    return df_[[mach_col, cd_col]].astype(float).values.tolist()

def _parse_aoa_deg(filename):
    m = re.search(r"AoA_(\d+(?:\.\d+)?)deg", filename)
    if not m:
        raise ValueError(f"Filename does not match AoA_<deg>deg_... pattern: {filename}")
    return float(m.group(1))

AOA_DRAG_TABLES = {}
for fn in AOA_CSV_FILES:
    aoa_deg = _parse_aoa_deg(fn)
    path = os.path.join(AOA_DIR, fn)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing AoA CSV: {path}")
    df_aoa = pd.read_csv(path)
    AOA_DRAG_TABLES[aoa_deg] = _normalize_to_drag_table(df_aoa)

_AOA_GRID = sorted(AOA_DRAG_TABLES.keys())

def cd_from_tables_alpha_mach(alpha_rad, mach_value):
    a_deg = abs(float(alpha_rad)) * 180.0 / np.pi
    m = float(mach_value)

    if a_deg <= _AOA_GRID[0]:
        tab = np.array(AOA_DRAG_TABLES[_AOA_GRID[0]], dtype=float)
        return float(np.interp(m, tab[:, 0], tab[:, 1]))
    if a_deg >= _AOA_GRID[-1]:
        tab = np.array(AOA_DRAG_TABLES[_AOA_GRID[-1]], dtype=float)
        return float(np.interp(m, tab[:, 0], tab[:, 1]))

    hi_idx = next(i for i, v in enumerate(_AOA_GRID) if v >= a_deg)
    lo_idx = hi_idx - 1
    a0, a1 = _AOA_GRID[lo_idx], _AOA_GRID[hi_idx]
    t = (a_deg - a0) / (a1 - a0) if a1 != a0 else 0.0

    tab0 = np.array(AOA_DRAG_TABLES[a0], dtype=float)
    tab1 = np.array(AOA_DRAG_TABLES[a1], dtype=float)

    cd0 = float(np.interp(m, tab0[:, 0], tab0[:, 1]))
    cd1 = float(np.interp(m, tab1[:, 0], tab1[:, 1]))
    return (1.0 - t) * cd0 + t * cd1

BASELINE_AOA_DEG = 0.0 if 0.0 in AOA_DRAG_TABLES else _AOA_GRID[0]
_BASE_TAB = np.array(AOA_DRAG_TABLES[BASELINE_AOA_DEG], dtype=float)

def cd_base_mach(mach_value):
    m = float(mach_value)
    return float(np.interp(m, _BASE_TAB[:, 0], _BASE_TAB[:, 1]))

AOA_LOG = []  # (abs_alpha_deg, mach, cd_total, cd0, cd_used)

def generic_surface_cD(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
    abs_alpha_deg = abs(alpha) * 180/np.pi
    mach = float(mach)

    cd_total = cd_from_tables_alpha_mach(alpha, mach)
    cd0      = cd_base_mach(mach)
    cd_used  = cd_total - cd0

    AOA_LOG.append((abs_alpha_deg, mach, cd_total, cd0, cd_used))
    return cd_used

# ===================== Load your original Mach-only Cd curve =====================
# df = pd.read_csv("mach_cdd.csv")
df = pd.read_csv("HELP.csv")

df = df.drop_duplicates(subset='MachNumber___', keep='first')

if df['MachNumber___'].iloc[0] > 0:
    df = pd.concat([
        pd.DataFrame({'MachNumber___':[0], 'x_DragCoefficient___':[df['x_DragCoefficient___'].iloc[0]]}),
        df
    ], ignore_index=True)

df = df.sort_values('MachNumber___').reset_index(drop=True)

drag_table = df[['MachNumber___', 'x_DragCoefficient___']].values.tolist()
power_on_drag_curve = drag_table
power_off_drag_curve = drag_table

from rocketpy import GenericSurface

REF_AREA = np.pi * (0.078359**2)
REF_LEN  = 2 * 0.078359

# ===================== Build Rocket =====================
cloud_test = rocketpy.Rocket(
    radius=0.078359,
    mass=23.243,
    inertia=(7.15, 7.15, 0.095),
    power_off_drag=power_off_drag_curve,
    power_on_drag=power_on_drag_curve,
    center_of_mass_without_motor=1.255014,
    coordinate_system_orientation="nose_to_tail",
)

M1939 = rocketpy.GenericMotor.load_from_eng_file("m3400.eng")
cloud_test.add_motor(M1939, position=2.1971)
print("helloworld2")

nose_cone = cloud_test.add_nose(
    length=0.762,
    kind='ogive',
    position=0,
    base_radius=0.078359
)

tailCone = cloud_test.add_tail(
    top_radius=0.156718/2,
    bottom_radius=0.1143/2,
    length=0.127,
    position=2.0828
)

fin_set = cloud_test.add_trapezoidal_fins(
    n=3,
    root_chord=0.2032,
    tip_chord=0.2032/2,
    span=0.1778,
    position=1.8796
)

# ===================== Environment preview only =====================
env = rocketpy.Environment(
    latitude=31.02403,
    longitude=-103.66157,
    elevation=198.73
)
print("helloworld3")

tomorrow = datetime.date.today() + datetime.timedelta(days=3)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))
env.set_atmospheric_model(
    type="custom_atmosphere",
    wind_u=0,
    wind_v=0
)

print("Backend:", matplotlib.get_backend())

aoa_drag_surface = GenericSurface(
    reference_area=REF_AREA,
    reference_length=REF_LEN,
    coefficients={"cD": generic_surface_cD},
    center_of_pressure=(0, 0, 0),
    name="AoA-dependent delta drag"
)

# cloud_test.add_surfaces(aoa_drag_surface, positions=0.762)

cloud_test.draw()
plt.show()

def get_peak_angular_metrics(flight):
    t = np.array(flight.time)
    print(flight.rocket.motor.burn_out_time)
    burnout_time = float(flight.rocket.motor.burn_out_time)

    t = t[t <= burnout_time]

    y1 = np.array([flight.alpha1(tt) for tt in t], dtype=float)
    y2 = np.array([flight.alpha2(tt) for tt in t], dtype=float)
    y3 = np.array([flight.alpha3(tt) for tt in t], dtype=float)

    alpha_mag = np.sqrt(y1**2 + y2**2 + y3**2)
    alpha_lat = np.sqrt(y1**2 + y2**2)

    i_mag = np.argmax(alpha_mag)
    i_lat = np.argmax(alpha_lat)

    return {
        "peak_alpha1": float(np.max(np.abs(y1))),
        "peak_alpha2": float(np.max(np.abs(y2))),
        "peak_alpha3": float(np.max(np.abs(y3))),
        "peak_alpha_mag": float(alpha_mag[i_mag]),
        "peak_alpha_lat": float(alpha_lat[i_lat]),
        "time_peak_alpha_mag": float(t[i_mag]),
        "time_peak_alpha_lat": float(t[i_lat]),
    }

def build_rocket():
    rocket = rocketpy.Rocket(
        radius=0.078359,
        mass=23.243,
        inertia=(7.15, 7.15, 0.095),
        power_off_drag=power_off_drag_curve,
        power_on_drag=power_on_drag_curve,
        center_of_mass_without_motor=1.255014,
        coordinate_system_orientation="nose_to_tail",
    )

    motor = rocketpy.GenericMotor.load_from_eng_file("m3400.eng")
    rocket.add_motor(motor, position=2.1971)

    rocket.add_nose(
        length=0.762,
        kind="ogive",
        position=0,
        base_radius=0.078359
    )

    rocket.add_tail(
        top_radius=0.156718/2,
        bottom_radius=0.1143/2,
        length=0.127,
        position=2.0828
    )

    rocket.add_trapezoidal_fins(
        n=3,
        root_chord=0.2032,
        tip_chord=0.2032/2,
        span=0.1778,
        position=1.8796
    )

    # AoA-dependent drag intentionally disabled
    # rocket.add_surfaces(aoa_drag_surface, positions=0.762)

    return rocket

# ===================== Environment factory =====================
def build_environment(date_utc):
    env = rocketpy.Environment(
        latitude=31.02403,
        longitude=-103.66157,
        elevation=198.73
    )

    env.set_date(date_utc)

    wind_speed = rng.uniform(0.0, 8.046)          # m/s
    wind_direction_deg = rng.uniform(0.0, 360.0) # math angle convention

    theta = np.deg2rad(wind_direction_deg)
    wind_u = wind_speed * np.cos(theta)
    wind_v = wind_speed * np.sin(theta)

    env.set_atmospheric_model(
        type="custom_atmosphere",
        wind_u=wind_u,
        wind_v=wind_v
    )

    return env, wind_speed, wind_direction_deg

LAUNCH_DATE_UTC = (2025, 6, 10, 12)
N_RUNS = 500
RANDOM_SEED = 12345
rng = np.random.default_rng(RANDOM_SEED)

results = []

for k in range(N_RUNS):
    heading_run = rng.uniform(0.0, 360.0)
    inclination_run = rng.uniform(87.5, 90.0)
    rail_length_run = 5.5

    AOA_LOG.clear()

    env, wind_speed, wind_direction_deg = build_environment(LAUNCH_DATE_UTC)
    rocket = build_rocket()

    try:
        flight = rocketpy.Flight(
            rocket=rocket,
            environment=env,
            rail_length=rail_length_run,
            inclination=inclination_run,
            heading=heading_run
        )

        metrics = get_peak_angular_metrics(flight)

        results.append({
            "run": k,
            "heading_deg": heading_run,
            "inclination_deg": inclination_run,
            "surface_wind_speed": wind_speed,
            "surface_wind_direction": wind_direction_deg,
            **metrics
        })

        print(
            f"[{k+1}/{N_RUNS}] completed | "
            f"peak_alpha_mag={metrics['peak_alpha_mag']:.3f} rad/s^2 | "
            f"peak_alpha_lat={metrics['peak_alpha_lat']:.3f} rad/s^2"
        )

    except Exception as e:
        results.append({
            "run": k,
            "heading_deg": heading_run,
            "inclination_deg": inclination_run,
            "surface_wind_speed": wind_speed,
            "surface_wind_direction": wind_direction_deg,
            "error": str(e)
        })
        print(f"[{k+1}/{N_RUNS}] failed | error: {e}")

results_df = pd.DataFrame(results)
results_df.to_csv("monte_carlo_angular_accel_results.csv", index=False)

good = results_df[results_df["peak_alpha_mag"].notna()].copy()

worst_row = good.loc[good["peak_alpha_mag"].idxmax()]
print("\nWORST CASE BY TOTAL ANGULAR ACCELERATION")
print(worst_row)

plt.figure()
plt.hist(good["peak_alpha_mag"], bins=25)
plt.xlabel("Peak angular acceleration magnitude (rad/s^2)")
plt.ylabel("Count")
plt.title("Monte Carlo distribution of peak angular acceleration")
plt.grid(True)
plt.show()

if len(AOA_LOG) > 0:
    aoa_log_df = pd.DataFrame(AOA_LOG, columns=["abs_alpha_deg", "mach", "cd_total", "cd0", "cd_used"])
    print("AOA_LOG rows:", len(aoa_log_df))
    print(aoa_log_df.describe())

if "surface_wind_speed" in good.columns:
    plt.figure()
    plt.scatter(good["surface_wind_speed"], good["peak_alpha_mag"])
    plt.xlabel("Surface wind speed")
    plt.ylabel("Peak angular acceleration magnitude (rad/s^2)")
    plt.title("Peak angular acceleration vs surface wind speed")
    plt.grid(True)
    plt.show()