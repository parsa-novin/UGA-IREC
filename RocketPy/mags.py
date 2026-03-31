import os
import re
import json
import argparse
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import rocketpy
from rocketpy import GenericSurface


# ============================================================
# DEFAULT PATHS / CONSTANTS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

DEFAULT_BASE_DRAG_CSV = BASE_DIR / "Deployment_Fits_Output" / "Deployment_80deg_CdFit.csv"
DEFAULT_MOTOR_FILE = BASE_DIR / "m3400.eng"
DEFAULT_AOA_DIR = BASE_DIR / "AoA_Fits_Output"

ROCKET_RADIUS = 0.078359
ROCKET_MASS = 18.7
ROCKET_INERTIA = (7.15, 7.15, 0.095)
ROCKET_COM_NO_MOTOR = 1.1633
ROCKET_COORD_SYS = "nose_to_tail"

NOSE_LENGTH = 0.7625
TAIL_TOP_RADIUS = 0.157 / 2
TAIL_BOTTOM_RADIUS = 0.12 / 2
TAIL_LENGTH = 0.112
TAIL_POSITION = 2.1467

FIN_COUNT = 3
FIN_ROOT_CHORD = 0.2032
FIN_TIP_CHORD = 0.095
FIN_SPAN = 0.1778
FIN_POSITION = 1.9445

MOTOR_POSITION = 2.262
AOA_SURFACE_POSITION = 0.762

REF_AREA = np.pi * (ROCKET_RADIUS ** 2)
REF_LEN = 2 * ROCKET_RADIUS

DEFAULT_LATITUDE = 34.62906
DEFAULT_LONGITUDE = -86.31311
DEFAULT_ELEVATION = 182.32

OPEN_METEO_ARCHIVE_URL = "https://archive-api.open-meteo.com/v1/archive"
WIND_DATE_STR = "2025-03-28"


# ============================================================
# AOA FILE DISCOVERY
# ============================================================

def _aoa_sort_key(path_obj: Path):
    m = re.search(r"AoA_(\d+(?:\.\d+)?)deg", path_obj.name)
    if not m:
        return float("inf")
    return float(m.group(1))


def find_aoa_csv_files(aoa_dir):
    aoa_dir = Path(aoa_dir)

    if not aoa_dir.exists():
        raise FileNotFoundError(f"AoA directory does not exist: {aoa_dir}")

    files = sorted(aoa_dir.glob("AoA_*deg_CdFit.csv"), key=_aoa_sort_key)

    if not files:
        raise FileNotFoundError(f"No AoA CSV files found in: {aoa_dir}")

    return [f.name for f in files]


# ============================================================
# DRAG TABLE HELPERS
# ============================================================

def infer_mach_cd_columns(df):
    if "MachNumber___" in df.columns and "x_DragCoefficient___" in df.columns:
        return "MachNumber___", "x_DragCoefficient___"

    mach_candidates = [c for c in df.columns if re.search(r"mach", c, re.IGNORECASE)]
    cd_candidates = [
        c for c in df.columns
        if re.search(r"(drag.*coeff|cd\b|c_d\b|dragcoefficient)", c, re.IGNORECASE)
    ]

    if not mach_candidates or not cd_candidates:
        raise ValueError(f"Could not infer Mach/Cd columns from columns: {list(df.columns)}")

    return mach_candidates[0], cd_candidates[0]


def normalize_to_drag_table(df):
    mach_col, cd_col = infer_mach_cd_columns(df)

    df = df.drop_duplicates(subset=mach_col, keep="first")
    df = df.sort_values(mach_col).reset_index(drop=True)

    if float(df[mach_col].iloc[0]) > 0.0:
        df = pd.concat(
            [
                pd.DataFrame({
                    mach_col: [0.0],
                    cd_col: [float(df[cd_col].iloc[0])]
                }),
                df
            ],
            ignore_index=True
        )

    return df[[mach_col, cd_col]].astype(float).values.tolist()


def load_base_drag_curve(csv_path):
    csv_path = Path(csv_path)

    if not csv_path.exists():
        raise FileNotFoundError(f"Missing base drag CSV: {csv_path}")

    df = pd.read_csv(csv_path)
    return normalize_to_drag_table(df)


# ============================================================
# AOA DRAG MODEL
# (USED ONLY FOR MONTE CARLO WHEN --use-aoa-drag IS ENABLED)
# ============================================================

class AoADragModel:
    def __init__(self, aoa_dir, aoa_csv_files, reference_area, reference_length):
        self.aoa_dir = Path(aoa_dir)
        self.aoa_csv_files = aoa_csv_files
        self.reference_area = reference_area
        self.reference_length = reference_length

        self.tables = {}
        self.aoa_grid = []
        self.base_tab = None
        self.log = []
        self.loaded_files = []

        self._load_tables()

    @staticmethod
    def parse_aoa_deg(filename):
        m = re.search(r"AoA_(\d+(?:\.\d+)?)deg", Path(filename).name)
        if not m:
            raise ValueError(f"Filename does not match AoA_<deg>deg_... pattern: {filename}")
        return float(m.group(1))

    def _load_tables(self):
        if self.aoa_csv_files is None:
            self.aoa_csv_files = find_aoa_csv_files(self.aoa_dir)

        print("\nAoA CSV files found:")
        for fn in self.aoa_csv_files:
            print(f"  {fn}")

        for fn in self.aoa_csv_files:
            aoa_deg = self.parse_aoa_deg(fn)
            path = self.aoa_dir / fn

            if not path.exists():
                raise FileNotFoundError(f"Missing AoA CSV: {path}")

            df = pd.read_csv(path)
            table = normalize_to_drag_table(df)
            self.tables[aoa_deg] = table
            self.loaded_files.append(fn)

            tab = np.array(table, dtype=float)
            print(
                f"Loaded {fn}: "
                f"{len(tab)} points, "
                f"Mach range=[{tab[:, 0].min():.3f}, {tab[:, 0].max():.3f}], "
                f"Cd range=[{tab[:, 1].min():.6f}, {tab[:, 1].max():.6f}]"
            )

        self.aoa_grid = sorted(self.tables.keys())
        baseline_aoa = 0.0 if 0.0 in self.tables else self.aoa_grid[0]
        self.base_tab = np.array(self.tables[baseline_aoa], dtype=float)

        print(f"AoA grid loaded: {self.aoa_grid}\n")

    def clear_log(self):
        self.log.clear()

    def log_df(self):
        if not self.log:
            return pd.DataFrame(
                columns=["eval_index", "abs_alpha_deg", "mach", "cd_total", "cd0", "cd_used"]
            )

        df = pd.DataFrame(
            self.log,
            columns=["abs_alpha_deg", "mach", "cd_total", "cd0", "cd_used"]
        )
        df.insert(0, "eval_index", np.arange(len(df)))
        return df

    def cd_base_mach(self, mach_value):
        m = float(mach_value)
        return float(np.interp(m, self.base_tab[:, 0], self.base_tab[:, 1]))

    def cd_from_tables_alpha_mach(self, alpha_rad, mach_value):
        a_deg = abs(float(alpha_rad)) * 180.0 / np.pi
        m = float(mach_value)

        if a_deg <= self.aoa_grid[0]:
            tab = np.array(self.tables[self.aoa_grid[0]], dtype=float)
            return float(np.interp(m, tab[:, 0], tab[:, 1]))

        if a_deg >= self.aoa_grid[-1]:
            tab = np.array(self.tables[self.aoa_grid[-1]], dtype=float)
            return float(np.interp(m, tab[:, 0], tab[:, 1]))

        hi_idx = next(i for i, v in enumerate(self.aoa_grid) if v >= a_deg)
        lo_idx = hi_idx - 1

        a0 = self.aoa_grid[lo_idx]
        a1 = self.aoa_grid[hi_idx]
        t = (a_deg - a0) / (a1 - a0) if a1 != a0 else 0.0

        tab0 = np.array(self.tables[a0], dtype=float)
        tab1 = np.array(self.tables[a1], dtype=float)

        cd0 = float(np.interp(m, tab0[:, 0], tab0[:, 1]))
        cd1 = float(np.interp(m, tab1[:, 0], tab1[:, 1]))

        return (1.0 - t) * cd0 + t * cd1

    def cD(self, alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
        abs_alpha_deg = abs(alpha) * 180.0 / np.pi
        mach = float(mach)

        cd_total = self.cd_from_tables_alpha_mach(alpha, mach)
        cd0 = self.cd_base_mach(mach)
        cd_used = cd_total - cd0

        self.log.append((abs_alpha_deg, mach, cd_total, cd0, cd_used))
        return cd_used

    def make_surface(self):
        def cD_func(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate):
            return self.cD(alpha, beta, mach, reynolds, pitch_rate, yaw_rate, roll_rate)

        return GenericSurface(
            reference_area=self.reference_area,
            reference_length=self.reference_length,
            coefficients={"cD": cD_func},
            center_of_pressure=(0, 0, 0),
            name="AoA-dependent delta drag",
        )


# ============================================================
# WEATHER API
# ============================================================

def fetch_june9_2025_wind_data(latitude, longitude):
    params = {
        "latitude": latitude,
        "longitude": longitude,
        "start_date": WIND_DATE_STR,
        "end_date": WIND_DATE_STR,
        "hourly": "wind_speed_10m,wind_direction_10m",
        "wind_speed_unit": "ms",
        "timezone": "auto",
    }

    url = OPEN_METEO_ARCHIVE_URL + "?" + urllib.parse.urlencode(params)

    with urllib.request.urlopen(url, timeout=30) as response:
        data = json.loads(response.read().decode("utf-8"))

    if "hourly" not in data:
        raise RuntimeError(f"Unexpected API response: {data}")

    hourly = data["hourly"]
    times = hourly.get("time", [])
    speeds = hourly.get("wind_speed_10m", [])
    dirs = hourly.get("wind_direction_10m", [])

    if not (len(times) == len(speeds) == len(dirs)):
        raise RuntimeError("Wind API returned mismatched hourly array lengths.")

    rows = []
    for t, s, d in zip(times, speeds, dirs):
        if s is None or d is None:
            continue
        rows.append({
            "time_utc": t,
            "wind_speed_mps": float(s),
            "wind_direction_from_deg": float(d),
        })

    if not rows:
        raise RuntimeError("No valid wind rows were returned from API.")

    return pd.DataFrame(rows)


def meteorological_to_uv(speed_mps, direction_from_deg):
    """
    Convert meteorological wind direction (direction FROM which wind blows,
    degrees clockwise from north) into east/north components.

    Returns:
        u = eastward component (m/s)
        v = northward component (m/s)
    """
    theta = np.deg2rad(direction_from_deg)

    u = -speed_mps * np.sin(theta)
    v = -speed_mps * np.cos(theta)
    return float(u), float(v)


def select_api_row(wind_df, api_hour):
    if api_hour is None:
        api_hour = 14

    if api_hour < 0 or api_hour >= len(wind_df):
        raise ValueError(f"--api-hour must be between 0 and {len(wind_df) - 1}")

    return wind_df.iloc[int(api_hour)]


# ============================================================
# BUILDERS
# ============================================================

def build_rocket(drag_table, motor_file, use_aoa_drag=False, aoa_model=None):
    rocket = rocketpy.Rocket(
        radius=ROCKET_RADIUS,
        mass=ROCKET_MASS,
        inertia=ROCKET_INERTIA,
        power_off_drag=drag_table,
        power_on_drag=drag_table,
        center_of_mass_without_motor=ROCKET_COM_NO_MOTOR,
        coordinate_system_orientation=ROCKET_COORD_SYS,
    )

    motor = rocketpy.GenericMotor.load_from_eng_file(str(motor_file))
    rocket.add_motor(motor, position=MOTOR_POSITION)

    rocket.add_nose(
        length=NOSE_LENGTH,
        kind="ogive",
        position=0,
        base_radius=ROCKET_RADIUS,
    )

    rocket.add_tail(
        top_radius=TAIL_TOP_RADIUS,
        bottom_radius=TAIL_BOTTOM_RADIUS,
        length=TAIL_LENGTH,
        position=TAIL_POSITION,
    )

    rocket.add_trapezoidal_fins(
        n=FIN_COUNT,
        root_chord=FIN_ROOT_CHORD,
        tip_chord=FIN_TIP_CHORD,
        span=FIN_SPAN,
        position=FIN_POSITION,
    )

    if use_aoa_drag:
        if aoa_model is None:
            raise ValueError("AoA drag requested, but aoa_model is None")
        rocket.add_surfaces(aoa_model.make_surface(), positions=AOA_SURFACE_POSITION)

    return rocket


def show_geometry(rocket):
    rocket.draw()


def build_environment_from_api_row(args, api_row):
    env = rocketpy.Environment(
        latitude=31.02403,
        longitude=-103.66157,
        elevation=198.73
    )

    dt = pd.to_datetime(api_row["time_utc"])
    env.set_date((int(dt.year), int(dt.month), int(dt.day), int(dt.hour)))

    wind_speed = float(api_row["wind_speed_mps"])
    wind_from_deg = float(api_row["wind_direction_from_deg"])

    wind_u, wind_v = meteorological_to_uv(wind_speed, wind_from_deg)

    env.set_atmospheric_model(
        type="custom_atmosphere",
        wind_u=wind_u,
        wind_v=wind_v,
    )

    return env


# ============================================================
# PLOTS
# ============================================================

def plot_aoa_csv_curves(aoa_model):
    plt.figure()

    max_cd = 0.0

    for aoa_deg in aoa_model.aoa_grid:
        tab = np.array(aoa_model.tables[aoa_deg], dtype=float)
        mach_vals = tab[:, 0]
        cd_vals = tab[:, 1]

        plt.plot(mach_vals, cd_vals, marker="o", label=f"{aoa_deg:g} deg")

        if len(cd_vals) > 0:
            max_cd = max(max_cd, float(np.max(cd_vals)))

    plt.xlabel("Mach")
    plt.ylabel("Cd")
    plt.title("AoA CSV Curves: Mach vs Cd")
    plt.xlim(left=0.0)
    plt.ylim(0.0, 1.05 * max_cd if max_cd > 0 else 1.0)
    plt.grid(True)
    plt.legend()
    plt.show()


def plot_aoa_sim_log(aoa_log_df, title="AoA Usage During Simulation"):
    if aoa_log_df.empty:
        print("AoA log is empty, so the AoA drag surface was not called during the simulation.")
        return

    x = aoa_log_df["eval_index"].to_numpy(dtype=float)
    y_cd_used = aoa_log_df["cd_total"].to_numpy(dtype=float)
    y_mach = aoa_log_df["mach"].to_numpy(dtype=float)

    ymax = max(
        float(np.max(y_cd_used)) if len(y_cd_used) else 0.0,
        float(np.max(y_mach)) if len(y_mach) else 0.0
    )
    ymax = 1.05 * ymax if ymax > 0 else 1.0

    plt.figure()
    plt.plot(x, y_cd_used, label="cd_used")
    plt.plot(x, y_mach, label="mach")
    plt.xlabel("AoA surface evaluation index")
    plt.ylabel("Value")
    plt.title(title)
    plt.xlim(left=0.0)
    plt.ylim(0.0, ymax)
    plt.grid(True)
    plt.legend()
    plt.show()


# ============================================================
# METRICS
# ============================================================

def get_peak_angular_metrics(flight):
    t = np.array(flight.time, dtype=float)
    burnout_time = float(flight.rocket.motor.burn_out_time)

    t = t[t <= burnout_time]
    if len(t) == 0:
        raise RuntimeError("No time samples found before burnout.")

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


# ============================================================
# RUN MODES
# ============================================================

def run_preview(args, drag_table):
    rocket = build_rocket(
        drag_table=drag_table,
        motor_file=args.motor_file,
        use_aoa_drag=False,
        aoa_model=None,
    )
    show_geometry(rocket)


def run_single(args, drag_table, wind_df):
    api_row = select_api_row(wind_df, args.api_hour)

    wind_speed = float(api_row["wind_speed_mps"])
    wind_from_deg = float(api_row["wind_direction_from_deg"])
    heading_deg = wind_from_deg

    env = build_environment_from_api_row(args, api_row)

    rocket = build_rocket(
        drag_table=drag_table,
        motor_file=args.motor_file,
        use_aoa_drag=False,
        aoa_model=None,
    )

    flight = rocketpy.Flight(
        rocket=rocket,
        environment=env,
        rail_length=args.rail_length,
        inclination=args.inclination,
        heading=heading_deg,
    )

    print("\nSINGLE FLIGHT RESULTS")
    flight.info()

    metrics = get_peak_angular_metrics(flight)
    for k, v in metrics.items():
        print(f"{k}: {v}")


def run_monte_carlo(args, drag_table, aoa_model, wind_df):
    rng = np.random.default_rng(args.seed)
    results = []
    worst_peak = -np.inf
    worst_aoa_log_df = None

    for k in range(args.runs):
        inclination_run = rng.uniform(args.inclination_min, args.inclination_max)

        row_idx = int(rng.integers(0, len(wind_df)))
        api_row = wind_df.iloc[row_idx]

        wind_speed = float(api_row["wind_speed_mps"])
        wind_from_deg = float(api_row["wind_direction_from_deg"])
        heading_run = wind_from_deg

        try:
            if args.use_aoa_drag and aoa_model is not None:
                aoa_model.clear_log()

            env = build_environment_from_api_row(args, api_row)
            rocket = build_rocket(
                drag_table=drag_table,
                motor_file=args.motor_file,
                use_aoa_drag=args.use_aoa_drag,
                aoa_model=aoa_model,
            )

            flight = rocketpy.Flight(
                rocket=rocket,
                environment=env,
                rail_length=args.rail_length,
                inclination=inclination_run,
                heading=heading_run,
            )

            metrics = get_peak_angular_metrics(flight)

            row = {
                "run": k,
                "time_utc": api_row["time_utc"],
                "surface_wind_speed_mps": wind_speed,
                "wind_direction_from_deg": wind_from_deg,
                "heading_deg_used": heading_run,
                "inclination_deg": inclination_run,
                "use_aoa_drag": args.use_aoa_drag,
                **metrics,
            }
            results.append(row)

            if args.use_aoa_drag and aoa_model is not None:
                if metrics["peak_alpha_mag"] > worst_peak:
                    worst_peak = metrics["peak_alpha_mag"]
                    worst_aoa_log_df = aoa_model.log_df().copy()

            print(
                f"[{k+1}/{args.runs}] completed | "
                f"time={api_row['time_utc']} | "
                f"wind={wind_speed:.3f} m/s from {wind_from_deg:.1f} deg | "
                f"heading={heading_run:.1f} deg | "
                f"incl={inclination_run:.3f} deg | "
                f"peak_alpha_mag={metrics['peak_alpha_mag']:.4f} rad/s^2"
            )

        except Exception as e:
            results.append({
                "run": k,
                "time_utc": api_row["time_utc"],
                "surface_wind_speed_mps": wind_speed,
                "wind_direction_from_deg": wind_from_deg,
                "heading_deg_used": heading_run,
                "inclination_deg": inclination_run,
                "use_aoa_drag": args.use_aoa_drag,
                "error": str(e),
            })
            print(f"[{k+1}/{args.runs}] failed | error: {e}")

    results_df = pd.DataFrame(results)
    results_df.to_csv(args.output_csv, index=False)
    print(f"\nSaved Monte Carlo results to: {args.output_csv}")

    if "peak_alpha_mag" in results_df.columns:
        good = results_df[results_df["peak_alpha_mag"].notna()].copy()
    else:
        good = pd.DataFrame()

    if not good.empty:
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

        plt.figure()
        plt.scatter(good["surface_wind_speed_mps"], good["peak_alpha_mag"])
        plt.xlabel("Surface wind speed (m/s)")
        plt.ylabel("Peak angular acceleration magnitude (rad/s^2)")
        plt.title("Peak angular acceleration vs surface wind speed")
        plt.grid(True)
        plt.show()

    if args.use_aoa_drag and worst_aoa_log_df is not None and not worst_aoa_log_df.empty:
        worst_aoa_log_df.to_csv(args.worst_aoa_log_csv, index=False)
        print(f"Saved worst-case AoA log to: {args.worst_aoa_log_csv}")
        plot_aoa_sim_log(worst_aoa_log_df, title="AoA Drag Usage During Worst-Case Monte Carlo Flight")


# ============================================================
# ARGPARSE
# ============================================================

def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "RocketPy launcher using Deployment_0deg_CdFit.csv as the normal "
            "Mach drag table. Optional AoA-based custom drag is used only in "
            "Monte Carlo when --use-aoa-drag is enabled."
        )
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)

    def add_common_arguments(p):
        p.add_argument(
            "--drag-csv",
            default=str(DEFAULT_BASE_DRAG_CSV),
            help="Base Mach-only drag CSV (default: Deployment_0deg_CdFit.csv)"
        )
        p.add_argument("--motor-file", default=str(DEFAULT_MOTOR_FILE), help=".eng motor file")

        p.add_argument("--latitude", type=float, default=DEFAULT_LATITUDE)
        p.add_argument("--longitude", type=float, default=DEFAULT_LONGITUDE)
        p.add_argument("--elevation", type=float, default=DEFAULT_ELEVATION)

        p.add_argument("--rail-length", type=float, default=5.5)
        p.add_argument("--inclination", type=float, default=89.0)

        p.add_argument(
            "--api-hour",
            type=int,
            default=10,
            help="Hour index 0-23 from June 9 2025 UTC wind data"
        )

    preview = subparsers.add_parser("preview", help="Show rocket geometry only")
    add_common_arguments(preview)

    single = subparsers.add_parser("single", help="Run one flight using June 9 2025 API wind")
    add_common_arguments(single)

    mc = subparsers.add_parser("montecarlo", help="Run Monte Carlo using sampled API wind hours")
    add_common_arguments(mc)

    mc.add_argument("--use-aoa-drag", action="store_true", help="Enable AoA-based delta drag for Monte Carlo only")
    mc.add_argument("--aoa-dir", default=str(DEFAULT_AOA_DIR), help="Directory containing AoA drag CSVs")
    mc.add_argument(
        "--aoa-files",
        nargs="+",
        default=None,
        help="AoA drag CSV filenames; if omitted, auto-discover from --aoa-dir"
    )
    mc.add_argument("--aoa-log-csv", default="aoa_log.csv")
    mc.add_argument("--runs", type=int, default=500)
    mc.add_argument("--seed", type=int, default=12345)
    mc.add_argument("--inclination-min", type=float, default=87.5)
    mc.add_argument("--inclination-max", type=float, default=90.0)
    mc.add_argument("--output-csv", default="monte_carlo_angular_accel_results.csv")
    mc.add_argument("--worst-aoa-log-csv", default="worst_case_aoa_log.csv")

    return parser


# ============================================================
# MAIN
# ============================================================

def main():
    parser = build_parser()
    args = parser.parse_args()

    drag_table = load_base_drag_curve(args.drag_csv)

    aoa_model = None
    if args.mode == "montecarlo" and args.use_aoa_drag:
        aoa_files = args.aoa_files
        if aoa_files is None:
            aoa_files = find_aoa_csv_files(args.aoa_dir)

        aoa_model = AoADragModel(
            aoa_dir=args.aoa_dir,
            aoa_csv_files=aoa_files,
            reference_area=REF_AREA,
            reference_length=REF_LEN,
        )

        plot_aoa_csv_curves(aoa_model)

    wind_df = fetch_june9_2025_wind_data(args.latitude, args.longitude)
    wind_df.to_csv("june9_2025_api_wind.csv", index=False)
    print("Saved API wind table to: june9_2025_api_wind.csv")
    print(wind_df)

    display_rocket = build_rocket(
        drag_table=drag_table,
        motor_file=args.motor_file,
        use_aoa_drag=False,
        aoa_model=None,
    )
    show_geometry(display_rocket)

    if args.mode == "preview":
        return
    elif args.mode == "single":
        run_single(args, drag_table, wind_df)
    elif args.mode == "montecarlo":
        run_monte_carlo(args, drag_table, aoa_model, wind_df)
    else:
        parser.error("Unknown mode")


if __name__ == "__main__":
    main()