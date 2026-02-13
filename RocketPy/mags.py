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
    # Prefer your current naming convention if present
    if "MachNumber___" in df_.columns and "x_DragCoefficient___" in df_.columns:
        return "MachNumber___", "x_DragCoefficient___"

    # Otherwise infer by common keywords
    mach_candidates = [c for c in df_.columns if re.search(r"mach", c, re.IGNORECASE)]
    cd_candidates   = [c for c in df_.columns if re.search(r"(drag.*coeff|cd\b|c_d\b|dragcoefficient)", c, re.IGNORECASE)]
    if not mach_candidates or not cd_candidates:
        raise ValueError(f"Could not infer Mach/Cd columns from columns: {list(df_.columns)}")
    return mach_candidates[0], cd_candidates[0]

def _normalize_to_drag_table(df_):
    mach_col, cd_col = _infer_mach_cd_columns(df_)

    # Remove duplicate Mach numbers (keep first), sort, ensure Mach 0 exists
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

# Load AoA tables: aoa_deg -> [[Mach, Cd], ...]
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
    """Absolute AoA (radians) + Mach -> Cd using AoA-binned Mach-Cd CSVs."""
    a_deg = abs(float(alpha_rad)) * 180.0 / np.pi
    m = float(mach_value)

    # Clamp to available AoA range
    if a_deg <= _AOA_GRID[0]:
        tab = np.array(AOA_DRAG_TABLES[_AOA_GRID[0]], dtype=float)
        return float(np.interp(m, tab[:, 0], tab[:, 1]))
    if a_deg >= _AOA_GRID[-1]:
        tab = np.array(AOA_DRAG_TABLES[_AOA_GRID[-1]], dtype=float)
        return float(np.interp(m, tab[:, 0], tab[:, 1]))

    # Find surrounding AoA bins
    hi_idx = next(i for i, v in enumerate(_AOA_GRID) if v >= a_deg)
    lo_idx = hi_idx - 1
    a0, a1 = _AOA_GRID[lo_idx], _AOA_GRID[hi_idx]
    t = (a_deg - a0) / (a1 - a0) if a1 != a0 else 0.0

    tab0 = np.array(AOA_DRAG_TABLES[a0], dtype=float)
    tab1 = np.array(AOA_DRAG_TABLES[a1], dtype=float)

    cd0 = float(np.interp(m, tab0[:, 0], tab0[:, 1]))
    cd1 = float(np.interp(m, tab1[:, 0], tab1[:, 1]))
    return (1.0 - t) * cd0 + t * cd1

# Baseline Cd(Mach) used so the GenericSurface contributes ONLY delta Cd.
# That avoids needing to overwrite rocket.power_on_drag/power_off_drag (which breaks on your version).
BASELINE_AOA_DEG = 0.0 if 0.0 in AOA_DRAG_TABLES else _AOA_GRID[0]
_BASE_TAB = np.array(AOA_DRAG_TABLES[BASELINE_AOA_DEG], dtype=float)

def cd_base_mach(mach_value):
    m = float(mach_value)
    return float(np.interp(m, _BASE_TAB[:, 0], _BASE_TAB[:, 1]))

# Log AoA & Mach at every aerodynamic evaluation
# Log AoA, Mach, and the Cd value returned to RocketPy at every evaluation
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
df = pd.read_csv("mach_cdd.csv")  # USE THIS ONE FOR SIMS

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

# ===================== Build Rocket =====================
cloud_test = rocketpy.Rocket(
    radius=0.078359,  # meters
    mass=23.243,      # kg
    inertia=(7.15, 7.15, 0.095),  # kgm^2
    power_off_drag=power_off_drag_curve,  # keep baseline; GenericSurface adds delta
    power_on_drag=power_on_drag_curve,
    center_of_mass_without_motor=1.255014,  # 49.41in
    coordinate_system_orientation="nose_to_tail",
)

# Motor
M1939 = rocketpy.GenericMotor.load_from_eng_file("m3400.eng")  # not actually 1939

cloud_test.add_motor(M1939, position=2.1971)
print("helloworld2")

# Geometry
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

# Environment
env = rocketpy.Environment(
    latitude=31.02403,
    longitude=-103.66157,
    elevation=198.73
)
print("helloworld3")

tomorrow = datetime.date.today() + datetime.timedelta(days=3)
env.set_date((tomorrow.year, tomorrow.month, tomorrow.day, 12))
env.set_atmospheric_model(type="Forecast", file="GFS")
print("Backend:", matplotlib.get_backend())

# ===================== Attach AoA-dependent delta drag via GenericSurface =====================
from rocketpy import GenericSurface

REF_AREA = np.pi * (0.078359**2)
REF_LEN  = 2 * 0.078359

aoa_drag_surface = GenericSurface(
    reference_area=REF_AREA,
    reference_length=REF_LEN,
    coefficients={"cD": generic_surface_cD},   # correct key for drag coeff
    center_of_pressure=(0, 0, 0),
    name="AoA-dependent delta drag"
)

cloud_test.add_surfaces(aoa_drag_surface, positions=0.762)

# Visualize rocket
cloud_test.draw()
plt.show()

# ===================== Run Flight =====================
flight = rocketpy.Flight(
    rocket=cloud_test,
    environment=env,
    rail_length=5.2,
    inclination=87.5,
    heading=0
)

flight.info()

aoa_log_df = pd.DataFrame(AOA_LOG, columns=["abs_alpha_deg", "mach", "cd_total", "cd0", "cd_used"])
print("AOA_LOG rows:", len(aoa_log_df))
print(aoa_log_df.describe())

x = np.arange(len(aoa_log_df))

fig, ax1 = plt.subplots()
ax3 = ax1.twinx()

# Left Y: Cd used
ax1.plot(x, aoa_log_df["cd_total"].to_numpy(), 'b')
ax1.set_xlabel("Calculation point (evaluation index)")
ax1.set_ylabel("Cd used (value returned by GenericSurface)")

# # Right Y: AoA
# ax2.plot(x, aoa_log_df["abs_alpha_deg"].to_numpy())
# ax2.set_ylabel("Abs AoA (deg)")

ax3.plot(x, aoa_log_df["mach"].to_numpy(), 'r')

ax1.grid(True)
plt.title("mach vs cd (mach red cd blue)")
plt.show()

# ===================== AoA log results =====================
aoa_log_df = pd.DataFrame(AOA_LOG, columns=["abs_alpha_deg", "mach"])
print("AOA_LOG rows:", len(aoa_log_df))
print(aoa_log_df.describe())
flight.altitude()
flight.attitude_angle()