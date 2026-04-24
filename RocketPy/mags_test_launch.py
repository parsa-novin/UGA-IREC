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
from airbrake_controller import AirbrakeController


# ============================================================
# DEFAULT PATHS / CONSTANTS
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

DEFAULT_BASE_DRAG_CSV   = BASE_DIR / "Deployment_Fits_Output" / "Deployment_0deg_CdFit.csv"
DEFAULT_MOTOR_FILE      = BASE_DIR / "m2050x.eng"
DEFAULT_AIRBRAKE_CSV_DIR = BASE_DIR / "Deployment_Fits_Output"

ROCKET_RADIUS        = 0.078359
ROCKET_MASS          = 18.7
ROCKET_INERTIA       = (7.15, 7.15, 0.095)
ROCKET_COM_NO_MOTOR  = 1.1633
ROCKET_COORD_SYS     = "nose_to_tail"

NOSE_LENGTH        = 0.7625
TAIL_TOP_RADIUS    = 0.157 / 2
TAIL_BOTTOM_RADIUS = 0.12 / 2
TAIL_LENGTH        = 0.112
TAIL_POSITION      = 2.1467

FIN_COUNT      = 3
FIN_ROOT_CHORD = 0.2032
FIN_TIP_CHORD  = 0.095
FIN_SPAN       = 0.1778
FIN_POSITION   = 1.9445

MOTOR_POSITION           = 2.262
AIRBRAKE_SURFACE_POSITION = 0.762

REF_AREA = np.pi * (ROCKET_RADIUS ** 2)
REF_LEN  = 2 * ROCKET_RADIUS

AIRBRAKE_MAX_ANGLE = 80.0   # degrees — corresponds to deployment_level = 1.0

DEFAULT_LATITUDE  = 34.62906
DEFAULT_LONGITUDE = -86.31311
DEFAULT_ELEVATION = 182.32

OPEN_METEO_ARCHIVE_URL = "https://archive-api.open-meteo.com/v1/archive"
WIND_DATE_STR = "2025-03-28"


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
                    cd_col:   [float(df[cd_col].iloc[0])]
                }),
                df
            ],
            ignore_index=True,
        )

    return df[[mach_col, cd_col]].astype(float).values.tolist()


def load_base_drag_curve(csv_path):
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing base drag CSV: {csv_path}")
    df = pd.read_csv(csv_path)
    return normalize_to_drag_table(df)


# ============================================================
# AIRBRAKE MODEL
# ============================================================

class AirbrakeModel:
    """Models airbrake drag as a function of deployment angle and Mach number.

    Deployment angle maps to RocketPy's deployment_level in [0, 1] via:
        deployment_level = angle_deg / AIRBRAKE_MAX_ANGLE

    Drag contribution from the airbrakes:
        F_drag_extra = effective_cd(level, mach) * q * reference_area

    where effective_cd accounts for both the Cd change between deployment-angle
    CSV tables AND the additional exposed area from the sine transfer function:

        delta_area [m²] = 3982.98097 * sin(angle_deg) / 1e6   (mm² → m²)
        effective_cd    = delta_Cd * (1 + delta_area / reference_area)
        delta_Cd        = Cd_at_angle(mach) - Cd_at_0deg(mach)
    """

    # Transfer-function constant from physical airbrake geometry measurement.
    # A_top [mm²] = _AREA_CONST_MM2 * sin(angle_deg_in_radians)
    _AREA_CONST_MM2 = 3982.98097

    def __init__(self, csv_dir, reference_area=REF_AREA):
        """
        Parameters
        ----------
        csv_dir : str or Path
            Directory containing Deployment_<N>deg_CdFit.csv files.
        reference_area : float
            Rocket cross-sectional reference area [m^2].
        """
        self.csv_dir = Path(csv_dir)
        self.reference_area = reference_area
        self.tables = {}             # {angle_deg: np.ndarray shape (N,2)}
        self.deployment_angles = []  # sorted list of available angles

        self._load_tables()

    def _load_tables(self):
        pattern = re.compile(r"Deployment_(\d+(?:\.\d+)?)deg_CdFit\.csv", re.IGNORECASE)
        found = {}
        for path in self.csv_dir.glob("Deployment_*deg_CdFit.csv"):
            m = pattern.search(path.name)
            if m:
                angle = float(m.group(1))
                df = pd.read_csv(path)
                found[angle] = np.array(normalize_to_drag_table(df), dtype=float)

        if not found:
            raise FileNotFoundError(
                f"No Deployment_<N>deg_CdFit.csv files found in: {self.csv_dir}"
            )

        self.deployment_angles = sorted(found.keys())
        self.tables = found

        print(f"\nAirbrake deployment CSVs loaded from: {self.csv_dir}")
        for ang in self.deployment_angles:
            tab = self.tables[ang]
            print(
                f"  {ang:4g}°: {len(tab)} points, "
                f"Mach=[{tab[:,0].min():.3f}, {tab[:,0].max():.3f}], "
                f"Cd=[{tab[:,1].min():.4f}, {tab[:,1].max():.4f}]"
            )

    # ------------------------------------------------------------------
    # Internal computation
    # ------------------------------------------------------------------

    def _cd_at_angle_mach(self, angle_deg, mach):
        """Interpolate Cd between deployment-angle tables at a given Mach."""
        angles = self.deployment_angles
        angle_deg = float(np.clip(angle_deg, angles[0], angles[-1]))

        if angle_deg <= angles[0]:
            tab = self.tables[angles[0]]
            return float(np.interp(mach, tab[:, 0], tab[:, 1]))

        if angle_deg >= angles[-1]:
            tab = self.tables[angles[-1]]
            return float(np.interp(mach, tab[:, 0], tab[:, 1]))

        hi_idx = next(i for i, v in enumerate(angles) if v >= angle_deg)
        lo_idx = hi_idx - 1

        a0, a1 = angles[lo_idx], angles[hi_idx]
        t = (angle_deg - a0) / (a1 - a0)

        cd0 = float(np.interp(mach, self.tables[a0][:, 0], self.tables[a0][:, 1]))
        cd1 = float(np.interp(mach, self.tables[a1][:, 0], self.tables[a1][:, 1]))
        return (1.0 - t) * cd0 + t * cd1

    def _delta_area(self, angle_deg):
        """Additional exposed area from airbrake geometry [m²].

        Transfer function (from physical measurement):
            A_top [mm²] = 3982.98097 * sin(angle_deg)
        Converted to m²: divide by 1 000 000.
        """
        return self._AREA_CONST_MM2 * np.sin(np.deg2rad(angle_deg)) / 1e6

    # ------------------------------------------------------------------
    # RocketPy interface
    # ------------------------------------------------------------------

    def drag_coefficient_curve(self, deployment_level, mach):
        """Effective Cd for RocketPy AirBrakes surface.

        RocketPy calls this as drag_coefficient_curve(deployment_level, mach).
        deployment_level is in [0, 1] where 1 == AIRBRAKE_MAX_ANGLE degrees.
        """
        angle_deg   = float(deployment_level) * AIRBRAKE_MAX_ANGLE
        cd_deployed = self._cd_at_angle_mach(angle_deg, mach)
        cd_zero     = self._cd_at_angle_mach(0.0, mach)
        delta_cd    = cd_deployed - cd_zero
        delta_area  = self._delta_area(angle_deg)
        return delta_cd * (1.0 + delta_area / self.reference_area)

    def make_controller(self, angle_fn):
        """Return a RocketPy controller function that drives deployment from angle_fn(time).

        angle_fn : callable(float) -> float
            Maps simulation time [s] to deployment angle [degrees, 0..AIRBRAKE_MAX_ANGLE].
        """
        max_angle = AIRBRAKE_MAX_ANGLE

        def _controller(
            time, sampling_rate, state_vector, state_history,
            observed_variables, interactive_objects,  # 6-param RocketPy form
        ):
            air_brakes = interactive_objects
            angle_deg  = float(angle_fn(time))
            angle_deg  = max(0.0, min(max_angle, angle_deg))
            air_brakes.deployment_level = angle_deg / max_angle

        return _controller

    def plot_cd_curves(self):
        """Plot Cd vs Mach for each available deployment angle."""
        plt.figure()
        for ang in self.deployment_angles:
            tab = self.tables[ang]
            plt.plot(tab[:, 0], tab[:, 1], marker="o", label=f"{ang:g}°")
        plt.xlabel("Mach")
        plt.ylabel("Cd")
        plt.title("Airbrake deployment Cd curves")
        plt.xlim(left=0.0)
        plt.grid(True)
        plt.legend()
        plt.show()


# ============================================================
# WEATHER API
# ============================================================

def fetch_june9_2025_wind_data(latitude, longitude):
    params = {
        "latitude":       latitude,
        "longitude":      longitude,
        "start_date":     WIND_DATE_STR,
        "end_date":       WIND_DATE_STR,
        "hourly":         "wind_speed_10m,wind_direction_10m",
        "wind_speed_unit": "ms",
        "timezone":       "auto",
    }

    url = OPEN_METEO_ARCHIVE_URL + "?" + urllib.parse.urlencode(params)

    with urllib.request.urlopen(url, timeout=30) as response:
        data = json.loads(response.read().decode("utf-8"))

    if "hourly" not in data:
        raise RuntimeError(f"Unexpected API response: {data}")

    hourly = data["hourly"]
    times  = hourly.get("time", [])
    speeds = hourly.get("wind_speed_10m", [])
    dirs   = hourly.get("wind_direction_10m", [])

    if not (len(times) == len(speeds) == len(dirs)):
        raise RuntimeError("Wind API returned mismatched hourly array lengths.")

    rows = []
    for t, s, d in zip(times, speeds, dirs):
        if s is None or d is None:
            continue
        rows.append({
            "time_utc":              t,
            "wind_speed_mps":        float(s),
            "wind_direction_from_deg": float(d),
        })

    if not rows:
        raise RuntimeError("No valid wind rows were returned from API.")

    return pd.DataFrame(rows)


def meteorological_to_uv(speed_mps, direction_from_deg):
    """Convert met wind (direction FROM, clockwise from N) to (u, v) components."""
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

def build_rocket(drag_table, motor_file, airbrake_model=None, controller_fn=None,
                 airbrake_sampling_rate=50.0):
    """Build and return a configured RocketPy Rocket.

    Parameters
    ----------
    drag_table : list
        Mach-Cd pairs for base (power-on/off) drag.
    motor_file : str or Path
        Path to .eng motor file.
    airbrake_model : AirbrakeModel or None
        If provided, attaches an airbrake surface to the rocket.
    controller_fn : callable or None
        RocketPy controller_function for the airbrakes.  Can be either
        an AirbrakeController instance (state-machine) or the closure
        returned by AirbrakeModel.make_controller() (constant angle).
        Required when airbrake_model is not None.
    airbrake_sampling_rate : float
        Rate at which the airbrake controller is called [Hz].
    """
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

    if airbrake_model is not None:
        if controller_fn is None:
            raise ValueError("airbrake_model supplied but controller_fn is None")
        rocket.add_air_brakes(
            drag_coefficient_curve=airbrake_model.drag_coefficient_curve,
            controller_function=controller_fn,
            sampling_rate=airbrake_sampling_rate,
            reference_area=airbrake_model.reference_area,
            clamp=True,
            name="AirBrakes",
        )

    return rocket


def build_environment_from_api_row(args, api_row):
    env = rocketpy.Environment(
        latitude=args.latitude,
        longitude=args.longitude,
        elevation=args.elevation,
    )

    dt = pd.to_datetime(api_row["time_utc"])
    env.set_date((int(dt.year), int(dt.month), int(dt.day), int(dt.hour)))

    wind_u, wind_v = meteorological_to_uv(
        float(api_row["wind_speed_mps"]),
        float(api_row["wind_direction_from_deg"]),
    )
    env.set_atmospheric_model(type="custom_atmosphere", wind_u=wind_u, wind_v=wind_v)

    return env


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
        "peak_alpha1":           float(np.max(np.abs(y1))),
        "peak_alpha2":           float(np.max(np.abs(y2))),
        "peak_alpha3":           float(np.max(np.abs(y3))),
        "peak_alpha_mag":        float(alpha_mag[i_mag]),
        "peak_alpha_lat":        float(alpha_lat[i_lat]),
        "time_peak_alpha_mag":   float(t[i_mag]),
        "time_peak_alpha_lat":   float(t[i_lat]),
    }


# ============================================================
# RUN MODES
# ============================================================

def _make_angle_fn(constant_angle):
    """Wrap a constant angle into a time-callable."""
    return lambda _: constant_angle


def run_preview(args, drag_table):
    rocket = build_rocket(drag_table=drag_table, motor_file=args.motor_file)
    rocket.draw()


def run_single(args, drag_table, wind_df, airbrake_model):
    api_row     = select_api_row(wind_df, args.api_hour)
    wind_from_deg = float(api_row["wind_direction_from_deg"])

    env = build_environment_from_api_row(args, api_row)

    if airbrake_model:
        if args.state_machine_controller:
            controller_fn = AirbrakeController(arm_g=args.arm_g, fire_g=args.fire_g)
        else:
            controller_fn = airbrake_model.make_controller(
                _make_angle_fn(args.airbrake_angle)
            )
    else:
        controller_fn = None

    rocket = build_rocket(
        drag_table=drag_table,
        motor_file=args.motor_file,
        airbrake_model=airbrake_model,
        controller_fn=controller_fn,
    )

    flight = rocketpy.Flight(
        rocket=rocket,
        environment=env,
        rail_length=args.rail_length,
        inclination=args.inclination,
        heading=wind_from_deg,
    )

    print("\nSINGLE FLIGHT RESULTS")
    flight.info()
    flight.all_info()

    metrics = get_peak_angular_metrics(flight)
    for k, v in metrics.items():
        print(f"{k}: {v}")


def run_monte_carlo(args, drag_table, wind_df, airbrake_model):
    rng     = np.random.default_rng(args.seed)
    results = []

    randomize_angle = (
        airbrake_model is not None
        and args.airbrake_angle_min is not None
        and args.airbrake_angle_max is not None
    )

    for k in range(args.runs):
        inclination_run = rng.uniform(args.inclination_min, args.inclination_max)

        row_idx = int(rng.integers(0, len(wind_df)))
        api_row = wind_df.iloc[row_idx]

        wind_speed    = float(api_row["wind_speed_mps"])
        wind_from_deg = float(api_row["wind_direction_from_deg"])

        if randomize_angle:
            angle_this_run = float(rng.uniform(args.airbrake_angle_min, args.airbrake_angle_max))
        else:
            angle_this_run = args.airbrake_angle if airbrake_model else 0.0

        try:
            env = build_environment_from_api_row(args, api_row)

            if airbrake_model:
                if args.state_machine_controller:
                    # Fresh instance each run so the state machine starts IDLE.
                    controller_fn = AirbrakeController(
                        arm_g=args.arm_g, fire_g=args.fire_g
                    )
                else:
                    controller_fn = airbrake_model.make_controller(
                        _make_angle_fn(angle_this_run)
                    )
            else:
                controller_fn = None

            rocket = build_rocket(
                drag_table=drag_table,
                motor_file=args.motor_file,
                airbrake_model=airbrake_model,
                controller_fn=controller_fn,
            )

            flight = rocketpy.Flight(
                rocket=rocket,
                environment=env,
                rail_length=args.rail_length,
                inclination=inclination_run,
                heading=wind_from_deg,
            )

            metrics = get_peak_angular_metrics(flight)

            ctrl_label = "state_machine" if args.state_machine_controller else "constant"
            row = {
                "run":                     k,
                "time_utc":                api_row["time_utc"],
                "surface_wind_speed_mps":  wind_speed,
                "wind_direction_from_deg": wind_from_deg,
                "inclination_deg":         inclination_run,
                "airbrake_angle_deg":      angle_this_run,
                "controller":              ctrl_label,
                **metrics,
            }
            results.append(row)

            print(
                f"[{k+1}/{args.runs}] "
                f"time={api_row['time_utc']} | "
                f"wind={wind_speed:.3f} m/s @ {wind_from_deg:.1f}° | "
                f"incl={inclination_run:.3f}° | "
                f"brake={angle_this_run:.1f}° [{ctrl_label}] | "
                f"peak_alpha_mag={metrics['peak_alpha_mag']:.4f} rad/s²"
            )

        except Exception as e:
            results.append({
                "run":                     k,
                "time_utc":                api_row["time_utc"],
                "surface_wind_speed_mps":  wind_speed,
                "wind_direction_from_deg": wind_from_deg,
                "inclination_deg":         inclination_run,
                "airbrake_angle_deg":      angle_this_run,
                "controller":              "state_machine" if args.state_machine_controller else "constant",
                "error":                   str(e),
            })
            print(f"[{k+1}/{args.runs}] failed | error: {e}")

    results_df = pd.DataFrame(results)
    results_df.to_csv(args.output_csv, index=False)
    print(f"\nSaved Monte Carlo results to: {args.output_csv}")

    if "peak_alpha_mag" not in results_df.columns:
        return

    good = results_df[results_df["peak_alpha_mag"].notna()].copy()
    if good.empty:
        return

    worst_row = good.loc[good["peak_alpha_mag"].idxmax()]
    print("\nWORST CASE BY TOTAL ANGULAR ACCELERATION")
    print(worst_row)

    plt.figure()
    plt.hist(good["peak_alpha_mag"], bins=25)
    plt.xlabel("Peak angular acceleration magnitude (rad/s²)")
    plt.ylabel("Count")
    plt.title("Monte Carlo: peak angular acceleration distribution")
    plt.grid(True)
    plt.show()

    plt.figure()
    plt.scatter(good["surface_wind_speed_mps"], good["peak_alpha_mag"])
    plt.xlabel("Surface wind speed (m/s)")
    plt.ylabel("Peak angular acceleration magnitude (rad/s²)")
    plt.title("Peak angular acceleration vs surface wind speed")
    plt.grid(True)
    plt.show()


# ============================================================
# ARGPARSE
# ============================================================

def build_parser():
    parser = argparse.ArgumentParser(
        description="RocketPy rocket flight simulator with optional airbrake drag modeling."
    )

    subparsers = parser.add_subparsers(dest="mode", required=True)

    def add_common_arguments(p):
        p.add_argument(
            "--drag-csv",
            default=str(DEFAULT_BASE_DRAG_CSV),
            help="Base Mach-only drag CSV (default: Deployment_0deg_CdFit.csv). "
                 "When airbrakes are enabled, use the 0° file as the base."
        )
        p.add_argument("--motor-file", default=str(DEFAULT_MOTOR_FILE), help=".eng motor file")

        p.add_argument("--latitude",  type=float, default=DEFAULT_LATITUDE)
        p.add_argument("--longitude", type=float, default=DEFAULT_LONGITUDE)
        p.add_argument("--elevation", type=float, default=DEFAULT_ELEVATION)

        p.add_argument("--rail-length",  type=float, default=5.5)
        p.add_argument("--inclination",  type=float, default=89.0)

        p.add_argument(
            "--api-hour", type=int, default=10,
            help="Hour index 0-23 from wind API data (UTC)"
        )

        # Airbrake options (shared across all modes)
        p.add_argument(
            "--airbrake-angle", type=float, default=0.0,
            help="Constant deployment angle in degrees [0, 80] used when "
                 "--state-machine-controller is NOT set (default: 0)"
        )
        p.add_argument(
            "--airbrake-csv-dir", default=str(DEFAULT_AIRBRAKE_CSV_DIR),
            help="Directory containing Deployment_<N>deg_CdFit.csv files"
        )
        p.add_argument(
            "--no-airbrakes", action="store_true",
            help="Disable airbrake surface entirely (pure base-drag simulation)"
        )
        p.add_argument(
            "--state-machine-controller", action="store_true",
            help="Use the H5 firmware state machine: arm at --arm-g, deploy at --fire-g"
        )
        p.add_argument(
            "--arm-g", type=float, default=5.0,
            help="Acceleration threshold [g] to enter ARMED state (default: 5.0)"
        )
        p.add_argument(
            "--fire-g", type=float, default=3.0,
            help="Acceleration threshold [g] to trigger full deployment (default: 3.0)"
        )

    preview = subparsers.add_parser("preview", help="Show rocket geometry only")
    add_common_arguments(preview)

    single = subparsers.add_parser("single", help="Run one deterministic flight")
    add_common_arguments(single)

    mc = subparsers.add_parser("montecarlo", help="Run Monte Carlo stochastic flights")
    add_common_arguments(mc)
    mc.add_argument("--runs",            type=int,   default=500)
    mc.add_argument("--seed",            type=int,   default=12345)
    mc.add_argument("--inclination-min", type=float, default=87.5)
    mc.add_argument("--inclination-max", type=float, default=90.0)
    mc.add_argument("--output-csv",      default="monte_carlo_angular_accel_results.csv")
    mc.add_argument(
        "--airbrake-angle-min", type=float, default=None,
        help="Min deployment angle for Monte Carlo random sampling (overrides --airbrake-angle)"
    )
    mc.add_argument(
        "--airbrake-angle-max", type=float, default=None,
        help="Max deployment angle for Monte Carlo random sampling (overrides --airbrake-angle)"
    )

    return parser


# ============================================================
# MAIN
# ============================================================

def main():
    parser = build_parser()
    args   = parser.parse_args()

    drag_table = load_base_drag_curve(args.drag_csv)

    # Build airbrake model unless disabled or both poly coefficients are zero
    # and no angle is requested (pure-base-drag shortcut).
    use_airbrakes = not args.no_airbrakes
    airbrake_model = None
    if use_airbrakes:
        airbrake_model = AirbrakeModel(
            csv_dir=args.airbrake_csv_dir,
            reference_area=REF_AREA,
        )
        airbrake_model.plot_cd_curves()

    wind_df = fetch_june9_2025_wind_data(args.latitude, args.longitude)
    wind_df.to_csv("june9_2025_api_wind.csv", index=False)
    print("Saved API wind table to: june9_2025_api_wind.csv")
    print(wind_df)

    # Always show geometry
    display_rocket = build_rocket(drag_table=drag_table, motor_file=args.motor_file)
    display_rocket.draw()

    if args.mode == "preview":
        return
    elif args.mode == "single":
        run_single(args, drag_table, wind_df, airbrake_model)
    elif args.mode == "montecarlo":
        run_monte_carlo(args, drag_table, wind_df, airbrake_model)
    else:
        parser.error("Unknown mode")


if __name__ == "__main__":
    main()
