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
DEFAULT_MOTOR_FILE      = BASE_DIR / "AeroTech_L2200G.eng"
DEFAULT_AIRBRAKE_CSV_DIR = BASE_DIR / "Deployment_Fits_Output"

ROCKET_RADIUS        = 0.078359
ROCKET_MASS          = 18.6
ROCKET_INERTIA       = (6.641, 6.641, 0.09)
ROCKET_COM_NO_MOTOR  = 1.27
ROCKET_COORD_SYS     = "nose_to_tail"

NOSE_LENGTH        = 0.7625
TAIL_TOP_RADIUS    = 0.157 / 2
TAIL_BOTTOM_RADIUS = 0.12 / 2
TAIL_LENGTH        = 0.112
TAIL_POSITION      = 2.205

FIN_COUNT      = 3
FIN_ROOT_CHORD = 0.2032
FIN_TIP_CHORD  = 0.095
FIN_SPAN       = 0.1778
FIN_POSITION   = 1.9445+0.0583

MOTOR_POSITION           = 2.262+0.0583
AIRBRAKE_SURFACE_POSITION = 0.762

DROGUE_CD_S              = 1.16 * np.pi * (0.61 / 2) ** 2   # 0.339 m²
DROGUE_SHOCK_CORD_M      = 1.37
DROGUE_POSITION_FROM_TIP = 1.29

MAIN_CD_S                = 2.59 * np.pi * (2.54 / 2) ** 2   # 13.12 m²
MAIN_SHOCK_CORD_M        = 3.05
MAIN_POSITION_FROM_TIP   = 0.402
MAIN_TRIGGER_AGL_M       = 304.8   # 1000 ft in metres

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
                 airbrake_sampling_rate=50.0, elevation=DEFAULT_ELEVATION):
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

    rocket.add_parachute(
        name="Drogue",
        cd_s=DROGUE_CD_S,
        trigger="apogee",
        sampling_rate=105,
        lag=1,
        noise=(0, 8.3, 0.5),
    )

    main_trigger_asl = elevation + MAIN_TRIGGER_AGL_M

    def _main_trigger(p, h, y):
        # y[5] is z-velocity; negative means descending
        return h <= main_trigger_asl and y[5] < 0

    rocket.add_parachute(
        name="Main",
        cd_s=MAIN_CD_S,
        trigger=_main_trigger,
        sampling_rate=105,
        lag=1,
        noise=(0, 8.3, 0.5),
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

def get_flight_metrics(flight):
    apogee_agl = float(flight.apogee) - float(flight.env.elevation)
    return {
        "apogee_agl_m":      apogee_agl,
        "max_speed_mps":     float(flight.max_speed),
        "max_accel_mps2":    float(flight.max_acceleration),
    }


# ============================================================
# RUN MODES
# ============================================================

def _make_angle_fn(constant_angle):
    """Wrap a constant angle into a time-callable."""
    return lambda _: constant_angle


def run_preview(args, drag_table):
    rocket = build_rocket(drag_table=drag_table, motor_file=args.motor_file, elevation=args.elevation)
    rocket.draw()



# ============================================================
# AIRBRAKE DEBUG LOG
# ============================================================

def debug_flight_airbrakes(flight, airbrake_model, drag_table, deployment_log, output_csv="airbrake_debug.csv"):
    """Print and save a per-timestep airbrake debug table.

    Columns:
        time(s)      simulation time
        alt_agl(m)   altitude above launch point
        vel(m/s)     scalar speed
        mach         Mach number
        level        RocketPy deployment_level [0-1]
        angle(deg)   physical deployment angle
        dArea(m2)    extra exposed area from airbrake geometry [m2]
        totArea(m2)  reference area + dArea
        dCd          effective Cd added by airbrakes (RocketPy surface Cd)
        baseCd       base rocket Cd from drag table at current Mach
        totalCd      baseCd + dCd
    """
    dt_arr = np.array(drag_table, dtype=float)

    COL_W = 12
    headers = [
        "time(s)", "alt_agl(m)", "vel(m/s)", "mach",
        "level", "angle(deg)", "dArea(m2)", "totArea(m2)",
        "dCd", "baseCd", "totalCd",
    ]
    sep = "=" * (COL_W * len(headers) + len(headers) - 1)
    row_fmt = " ".join(f"{{{i}:>{COL_W}}}" for i in range(len(headers)))

    print("\n" + sep)
    print("AIRBRAKE DEBUG LOG")
    print(sep)
    print(row_fmt.format(*headers))
    print("-" * (COL_W * len(headers) + len(headers) - 1))

    rows = []
    prev_level = None

    for t, level in deployment_log:
        try:
            alt  = float(flight.z(t)) - float(flight.env.elevation)
            vel  = float(flight.speed(t))
            mach = float(flight.mach_number(t))
        except Exception:
            continue

        level = float(level)
        angle_deg = level * AIRBRAKE_MAX_ANGLE

        if airbrake_model is not None:
            delta_area = airbrake_model._delta_area(angle_deg)
            delta_cd   = airbrake_model.drag_coefficient_curve(level, mach)
        else:
            delta_area = 0.0
            delta_cd   = 0.0

        total_area = REF_AREA + delta_area
        base_cd    = float(np.interp(mach, dt_arr[:, 0], dt_arr[:, 1]))
        total_cd   = base_cd + delta_cd

        marker = " <-- DEPLOY CHANGE" if (prev_level is not None and level != prev_level) else ""
        prev_level = level

        row_vals = [
            f"{t:.3f}", f"{alt:.2f}", f"{vel:.3f}", f"{mach:.5f}",
            f"{level:.4f}", f"{angle_deg:.2f}", f"{delta_area:.6f}", f"{total_area:.6f}",
            f"{delta_cd:.6f}", f"{base_cd:.6f}", f"{total_cd:.6f}",
        ]
        print(row_fmt.format(*row_vals) + marker)

        rows.append({
            "time_s":        t,
            "alt_agl_m":     alt,
            "velocity_mps":  vel,
            "mach":          mach,
            "level":         level,
            "angle_deg":     angle_deg,
            "delta_area_m2": delta_area,
            "total_area_m2": total_area,
            "delta_cd":      delta_cd,
            "base_cd":       base_cd,
            "total_cd":      total_cd,
        })

    print(sep)

    if rows:
        df = pd.DataFrame(rows)
        deployed = df[df["level"] > 0]
        print("\nSummary:")
        print(f"  Total timesteps logged : {len(df)}")
        print(f"  Timesteps deployed     : {len(deployed)}")
        if not deployed.empty:
            print(f"  First deployment time  : {deployed['time_s'].iloc[0]:.3f} s")
            print(f"  Max angle reached      : {deployed['angle_deg'].max():.2f} deg")
            print(f"  Max delta_area         : {deployed['delta_area_m2'].max():.6f} m2")
            print(f"  Max total_area         : {deployed['total_area_m2'].max():.6f} m2")
            print(f"  Max delta_cd           : {deployed['delta_cd'].max():.6f}")
            print(f"  Max total_cd           : {deployed['total_cd'].max():.6f}")
            print(f"  Alt at first deploy    : {deployed['alt_agl_m'].iloc[0]:.2f} m AGL")
        print(f"  Peak altitude          : {df['alt_agl_m'].max():.2f} m AGL")

        df.to_csv(output_csv, index=False)
        print(f"  Saved debug log to     : {output_csv}")


def run_single(args, drag_table, wind_df, airbrake_model):
    api_row       = select_api_row(wind_df, args.api_hour)
    wind_from_deg = float(api_row["wind_direction_from_deg"])

    env = build_environment_from_api_row(args, api_row)

    deployment_log = []  # [(time, deployment_level), ...]

    if airbrake_model:
        if args.constant_angle:
            _inner_ctrl = airbrake_model.make_controller(
                _make_angle_fn(args.airbrake_angle)
            )
        else:
            _inner_ctrl = AirbrakeController(arm_g=args.arm_g, fire_g=args.fire_g)

        def controller_fn(time, sampling_rate, state_vector, state_history,
                          observed_variables, interactive_objects):
            _inner_ctrl(time, sampling_rate, state_vector, state_history,
                        observed_variables, interactive_objects)
            deployment_log.append((time, interactive_objects.deployment_level))
    else:
        controller_fn = None

    rocket = build_rocket(
        drag_table=drag_table,
        motor_file=args.motor_file,
        airbrake_model=airbrake_model,
        controller_fn=controller_fn,
        elevation=args.elevation,
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
    flight.plots.all()

    debug_flight_airbrakes(
        flight=flight,
        airbrake_model=airbrake_model,
        drag_table=drag_table,
        deployment_log=deployment_log,
        output_csv="airbrake_debug.csv",
    )

    metrics = get_flight_metrics(flight)
    print("\nFLIGHT METRICS")
    print(f"  apogee AGL:   {metrics['apogee_agl_m']:.1f} m")
    print(f"  max speed:    {metrics['max_speed_mps']:.1f} m/s")
    print(f"  max accel:    {metrics['max_accel_mps2']:.1f} m/s²")

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
                if args.constant_angle:
                    controller_fn = airbrake_model.make_controller(
                        _make_angle_fn(angle_this_run)
                    )
                else:
                    # Fresh instance each run so the state machine starts IDLE.
                    controller_fn = AirbrakeController(
                        arm_g=args.arm_g, fire_g=args.fire_g
                    )
            else:
                controller_fn = None

            rocket = build_rocket(
                drag_table=drag_table,
                motor_file=args.motor_file,
                airbrake_model=airbrake_model,
                controller_fn=controller_fn,
                elevation=args.elevation,
            )

            flight = rocketpy.Flight(
                rocket=rocket,
                environment=env,
                rail_length=args.rail_length,
                inclination=inclination_run,
                heading=wind_from_deg,
            )

            metrics = get_flight_metrics(flight)

            ctrl_label = "constant" if args.constant_angle else "state_machine"
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
                f"apogee={metrics['apogee_agl_m']:.1f} m AGL | "
                f"max_v={metrics['max_speed_mps']:.1f} m/s | "
                f"max_a={metrics['max_accel_mps2']:.1f} m/s²"
            )

        except Exception as e:
            results.append({
                "run":                     k,
                "time_utc":                api_row["time_utc"],
                "surface_wind_speed_mps":  wind_speed,
                "wind_direction_from_deg": wind_from_deg,
                "inclination_deg":         inclination_run,
                "airbrake_angle_deg":      angle_this_run,
                "controller":              "constant" if args.constant_angle else "state_machine",
                "error":                   str(e),
            })
            print(f"[{k+1}/{args.runs}] failed | error: {e}")

    results_df = pd.DataFrame(results)
    results_df.to_csv(args.output_csv, index=False)
    print(f"\nSaved Monte Carlo results to: {args.output_csv}")

    metric_columns = ["apogee_agl_m", "max_speed_mps", "max_accel_mps2"]
    if not all(col in results_df.columns for col in metric_columns):
        return

    good = results_df.dropna(subset=metric_columns).copy()
    if good.empty:
        return

    print("\nMAX APOGEE")
    print(good.loc[good["apogee_agl_m"].idxmax()])

    print("\nMAX VELOCITY")
    print(good.loc[good["max_speed_mps"].idxmax()])

    print("\nMAX ACCELERATION")
    print(good.loc[good["max_accel_mps2"].idxmax()])

    plt.figure()
    plt.hist(good["apogee_agl_m"], bins=25)
    plt.xlabel("Apogee AGL (m)")
    plt.ylabel("Count")
    plt.title("Monte Carlo: apogee distribution")
    plt.grid(True)

    plt.figure()
    plt.hist(good["max_speed_mps"], bins=25)
    plt.xlabel("Maximum velocity (m/s)")
    plt.ylabel("Count")
    plt.title("Monte Carlo: maximum velocity distribution")
    plt.grid(True)

    plt.figure()
    plt.hist(good["max_accel_mps2"], bins=25)
    plt.xlabel("Maximum acceleration (m/s²)")
    plt.ylabel("Count")
    plt.title("Monte Carlo: maximum acceleration distribution")
    plt.grid(True)

    plt.figure()
    plt.scatter(good["surface_wind_speed_mps"], good["apogee_agl_m"])
    plt.xlabel("Surface wind speed (m/s)")
    plt.ylabel("Apogee AGL (m)")
    plt.title("Apogee vs surface wind speed")
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
            help="Constant deployment angle in degrees [0, 80] used only with "
                 "--constant-angle (default: 0)"
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
            "--constant-angle", action="store_true",
            help="Use a fixed deployment angle (--airbrake-angle) instead of the "
                 "H5 state-machine controller (default: state machine)"
        )
        p.add_argument(
            "--arm-g", type=float, default=4.0,
            help="Signed vertical acceleration threshold [g] to enter ARMED state. "
                 "Motor ignition drives az to +10 g or more; 4.0 g gives a "
                 "comfortable margin above rail-clearance noise. (default: 4.0)"
        )
        p.add_argument(
            "--fire-g", type=float, default=0.0,
            help="Signed vertical acceleration threshold [g] to trigger deployment. "
                 "At burnout az crosses zero and goes negative; 0.0 deploys the "
                 "instant the net upward acceleration ends. Use a small negative "
                 "value (e.g. -0.5) for hysteresis. (default: 0.0)"
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
    mc.add_argument("--output-csv",      default="monte_carlo_flight_metrics_results.csv")
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
    display_rocket = build_rocket(drag_table=drag_table, motor_file=args.motor_file, elevation=args.elevation)
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
