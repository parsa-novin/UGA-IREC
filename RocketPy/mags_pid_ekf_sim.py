"""
mags_pid_ekf_sim.py  —  copy of mags_test_launch.py, single-run mode replaced
with a pure-Python 1-D PID + EKF closed-loop airbrake simulation.

Mirrors airbrake_closed_loop_sim.m exactly, but uses the actual CSV drag
tables (AirbrakeModel) and the real .eng thrust curve instead of analytical
approximations, so the numbers are more accurate.

Phase 1 — open-loop (lev = 0): RK4 integration finds the natural apogee.
Phase 2 — closed-loop (PID + EKF): targets (natural_apogee − 100 m).

Usage
-----
    python mags_pid_ekf_sim.py                        # all defaults
    python mags_pid_ekf_sim.py --no-airbrakes         # base-drag only

Produces a 9-panel matplotlib figure matching the MATLAB output.
"""

import re
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ============================================================
# CONSTANTS  (mirrored from mags_test_launch.py)
# ============================================================

BASE_DIR = Path(__file__).resolve().parent

DEFAULT_BASE_DRAG_CSV    = BASE_DIR / "Deployment_Fits_Output" / "Deployment_0deg_CdFit.csv"
DEFAULT_MOTOR_FILE       = BASE_DIR / "m2050x_corrected.eng"
DEFAULT_AIRBRAKE_CSV_DIR = BASE_DIR / "Deployment_Fits_Output"

ROCKET_RADIUS  = 0.078359          # body radius [m]
ROCKET_MASS    = 18.6              # dry mass, no motor [kg]
REF_AREA       = np.pi * ROCKET_RADIUS ** 2   # ≈ 0.019293 m²
AIRBRAKE_MAX_ANGLE = 80.0          # degrees at deployment_level = 1.0

DEFAULT_ELEVATION = 182.32         # launch-site MSL elevation [m]

G0 = 9.80665                       # standard gravity [m/s²]

# EKF sensor noise (1-σ)
SIG_BARO = 2.0     # barometer [m]
SIG_IMU  = 1.2     # accelerometer [m/s²]

# PID gains  (same as MATLAB script)
KP, KI, KD = 5e-4, 5e-6, 2e-3

# Simulation timestep
DT = 0.01   # [s]


# ============================================================
# DRAG TABLE HELPERS  (copied from mags_test_launch.py)
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
        raise ValueError(f"Could not infer Mach/Cd columns from: {list(df.columns)}")
    return mach_candidates[0], cd_candidates[0]


def normalize_to_drag_table(df):
    mach_col, cd_col = infer_mach_cd_columns(df)
    df = df.drop_duplicates(subset=mach_col, keep="first")
    df = df.sort_values(mach_col).reset_index(drop=True)
    if float(df[mach_col].iloc[0]) > 0.0:
        df = pd.concat([
            pd.DataFrame({mach_col: [0.0], cd_col: [float(df[cd_col].iloc[0])]}),
            df,
        ], ignore_index=True)
    return df[[mach_col, cd_col]].astype(float).values.tolist()


def load_base_drag_curve(csv_path):
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing base drag CSV: {csv_path}")
    return normalize_to_drag_table(pd.read_csv(csv_path))


# ============================================================
# AIRBRAKE MODEL  (copied from mags_test_launch.py verbatim)
# ============================================================

class AirbrakeModel:
    """Models airbrake drag as a function of deployment angle and Mach number."""

    _AREA_CONST_MM2 = 3982.98097   # A_top [mm²] = const * sin(angle_rad)

    def __init__(self, csv_dir, reference_area=REF_AREA):
        self.csv_dir = Path(csv_dir)
        self.reference_area = reference_area
        self.tables = {}
        self.deployment_angles = []
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
            raise FileNotFoundError(f"No Deployment CSVs found in: {self.csv_dir}")
        self.deployment_angles = sorted(found.keys())
        self.tables = found
        print(f"\nAirbrake CSVs loaded from: {self.csv_dir}")
        for ang in self.deployment_angles:
            tab = self.tables[ang]
            print(f"  {ang:4g}°  {len(tab)} pts  "
                  f"Mach=[{tab[:,0].min():.3f}, {tab[:,0].max():.3f}]  "
                  f"Cd=[{tab[:,1].min():.4f}, {tab[:,1].max():.4f}]")

    def _cd_at_angle_mach(self, angle_deg, mach):
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
        return self._AREA_CONST_MM2 * np.sin(np.deg2rad(angle_deg)) / 1e6

    def drag_coefficient_curve(self, deployment_level, mach):
        """Effective ΔCd added by the airbrake surface at this level and Mach."""
        angle_deg   = float(deployment_level) * AIRBRAKE_MAX_ANGLE
        cd_deployed = self._cd_at_angle_mach(angle_deg, mach)
        cd_zero     = self._cd_at_angle_mach(0.0, mach)
        delta_cd    = cd_deployed - cd_zero
        delta_area  = self._delta_area(angle_deg)
        return delta_cd * (1.0 + delta_area / self.reference_area)


# ============================================================
# MOTOR LOADER  (parses RASP .eng format)
# ============================================================

def load_eng_file(path):
    """Parse a RASP .eng file.

    Returns (t_arr, F_arr, m_prop_kg, m_total_motor_kg).
    Prepends (0, 0) if the curve doesn't start at t = 0.
    """
    path = Path(path)
    t_pts, F_pts = [], []
    m_prop = m_total = None
    with path.open() as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith(";"):
                continue
            parts = line.split()
            if parts[0][0].isalpha():
                # Header: NAME diam_mm len_mm delays m_prop_kg m_total_kg mfr
                m_prop  = float(parts[4])
                m_total = float(parts[5])
            else:
                try:
                    t_pts.append(float(parts[0]))
                    F_pts.append(float(parts[1]))
                except (ValueError, IndexError):
                    pass
    if m_prop is None:
        raise ValueError(f"Could not parse .eng header in: {path}")
    t_arr = np.array(t_pts, dtype=float)
    F_arr = np.array(F_pts, dtype=float)
    if t_arr[0] > 0.0:
        t_arr = np.concatenate([[0.0], t_arr])
        F_arr = np.concatenate([[0.0], F_arr])
    return t_arr, F_arr, m_prop, m_total


# ============================================================
# ISA ATMOSPHERE  (troposphere)
# ============================================================

def _isa(z_agl, elevation=DEFAULT_ELEVATION):
    """Return (density [kg/m³], speed_of_sound [m/s]) at altitude z_agl AGL."""
    z_msl = max(float(z_agl) + elevation, 0.0)
    T     = max(288.15 - 0.0065 * z_msl, 1.0)
    rho   = (101325.0 / (287.05 * 288.15)) * max(1.0 - 0.0065 * z_msl / 288.15, 1e-9) ** (G0 / (287.05 * 0.0065) - 1.0)
    a_snd = np.sqrt(1.4 * 287.05 * T)
    return rho, a_snd


# ============================================================
# AERODYNAMICS
# ============================================================

def total_drag_N(z, v, lev, drag_arr, airbrake_model, elevation=DEFAULT_ELEVATION):
    """Total drag force magnitude [N].  Caller applies sign via np.sign(v)."""
    z = max(float(z), 0.0)
    rho, a_snd = _isa(z, elevation)
    mach     = abs(v) / a_snd
    base_cd  = float(np.interp(mach, drag_arr[:, 0], drag_arr[:, 1]))
    ab_cd    = (airbrake_model.drag_coefficient_curve(lev, mach)
                if (airbrake_model is not None and lev > 1e-6) else 0.0)
    return 0.5 * rho * v * v * (base_cd + ab_cd) * REF_AREA


# ============================================================
# RK4 INTEGRATOR
# ============================================================

def _eom(state, t, lev, thrust_fn, mass_fn, drag_arr, airbrake_model, elevation):
    z, v = state
    Ft   = thrust_fn(t)
    m    = mass_fn(t)
    Fd   = total_drag_N(z, v, lev, drag_arr, airbrake_model, elevation)
    return np.array([v, (Ft - Fd * np.sign(v) - m * G0) / m])


def rk4_step(state, t, dt, lev, thrust_fn, mass_fn, drag_arr, airbrake_model, elevation):
    k1 = _eom(state,           t,        lev, thrust_fn, mass_fn, drag_arr, airbrake_model, elevation)
    k2 = _eom(state + dt/2*k1, t+dt/2,  lev, thrust_fn, mass_fn, drag_arr, airbrake_model, elevation)
    k3 = _eom(state + dt/2*k2, t+dt/2,  lev, thrust_fn, mass_fn, drag_arr, airbrake_model, elevation)
    k4 = _eom(state + dt*k3,   t+dt,    lev, thrust_fn, mass_fn, drag_arr, airbrake_model, elevation)
    out = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    out[0] = max(out[0], 0.0)
    return out


# ============================================================
# EKF
# State  : x = [z (m AGL),  v (m/s upward)]
# Predict: x_{k+1} = A x_k + [0; a_imu*dt]   (IMU drives predict)
# Update : barometer measures z with noise σ_baro
# ============================================================

class EKF1D:
    def __init__(self, dt, sig_baro=SIG_BARO, sig_imu=SIG_IMU):
        self.dt       = dt
        self.sig_baro = sig_baro
        self.sig_imu  = sig_imu
        self.x = np.zeros(2)
        self.P = np.diag([4.0, 0.25])
        self.Q = np.diag([0.5 * (sig_imu * dt**2)**2, (sig_imu * dt)**2])
        self.R = sig_baro**2
        self._A = np.array([[1.0, dt], [0.0, 1.0]])
        self._H = np.array([[1.0, 0.0]])

    def predict(self, a_imu):
        self.x = np.array([
            self.x[0] + self.x[1] * self.dt,
            self.x[1] + a_imu     * self.dt,
        ])
        self.P = self._A @ self.P @ self._A.T + self.Q

    def update_baro(self, z_meas):
        H   = self._H
        y   = float(z_meas - H @ self.x)
        S   = float(H @ self.P @ H.T) + self.R
        K   = (self.P @ H.T) / S          # shape (2,1)
        self.x = self.x + (K * y).flatten()
        self.P = (np.eye(2) - K @ H) @ self.P
        self.x[0] = max(self.x[0], 0.0)   # altitude ≥ 0

    @property
    def z(self):     return self.x[0]
    @property
    def v(self):     return self.x[1]
    @property
    def var_z(self): return float(self.P[0, 0])
    @property
    def var_v(self): return float(self.P[1, 1])


# ============================================================
# APOGEE PREDICTOR
# Numerically integrates the coast EOM from (z0, v0) with deployment
# level lev held constant until v ≤ 0.  Coarse Euler step is accurate
# enough as a PID error signal.
# ============================================================

def predict_apogee(z0, v0, lev, m, drag_arr, airbrake_model,
                   elevation=DEFAULT_ELEVATION, dt_p=0.25, n_max=300):
    z, v = float(z0), float(v0)
    for _ in range(n_max):
        if v <= 0.0:
            break
        Fd = total_drag_N(z, v, lev, drag_arr, airbrake_model, elevation)
        a  = (-Fd - m * G0) / m   # v > 0 → sign(v) = 1
        v += a * dt_p
        z += v * dt_p
    return z


# ============================================================
# PID CONTROLLER
# ============================================================

class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt        = dt
        self._integral = 0.0
        self._prev_err = 0.0

    def step(self, err):
        self._integral += err * self.dt
        deriv           = (err - self._prev_err) / self.dt
        self._prev_err  = err
        return self.Kp * err + self.Ki * self._integral + self.Kd * deriv


# ============================================================
# CORE SIMULATION LOOP
# ============================================================

def _run_loop(t_vec, drag_arr, airbrake_model, thrust_fn, mass_fn,
              t_burn, elevation, ekf=None, pid=None, z_target=None):
    """
    RK4 simulation loop shared by both phases.

    Open-loop  : ekf=None, pid=None  →  lev=0 throughout.
    Closed-loop: pass EKF1D and PID instances and a z_target.

    Returns a dict of NumPy arrays trimmed to the window [launch, touchdown].
    """
    N  = len(t_vec)
    dt = float(t_vec[1] - t_vec[0])

    z_true  = np.zeros(N); v_true  = np.zeros(N)
    z_est   = np.zeros(N); v_est   = np.zeros(N)
    var_z   = np.zeros(N); var_v   = np.zeros(N)
    lev_arr = np.zeros(N); ang_arr = np.zeros(N)
    Fd_arr  = np.zeros(N); Ft_arr  = np.zeros(N)
    Mach_arr= np.zeros(N); mass_arr= np.zeros(N)
    zpred   = np.zeros(N); pid_raw = np.zeros(N)
    accel   = np.zeros(N); pid_err = np.zeros(N)

    state      = np.array([0.0, 0.0])
    ctrl_armed = False
    apo_done   = False
    i_end      = N - 1

    for i, t in enumerate(t_vec):
        lev = lev_arr[i]
        z, v = state
        z_true[i] = z
        v_true[i] = v

        # ── mass, thrust, aero ─────────────────────────────────────────
        m      = mass_fn(t)
        Ft     = thrust_fn(t)
        rho, a_snd = _isa(max(z, 0.0), elevation)
        Mach   = abs(v) / a_snd
        Fd     = total_drag_N(z, v, lev, drag_arr, airbrake_model, elevation)
        a_net  = (Ft - Fd * np.sign(v) - m * G0) / m

        mass_arr[i] = m
        Ft_arr[i]   = Ft
        Fd_arr[i]   = Fd
        Mach_arr[i] = Mach
        accel[i]    = a_net

        # ── EKF predict + update ───────────────────────────────────────
        if ekf is not None:
            a_imu  = a_net + np.random.randn() * ekf.sig_imu
            z_baro = z    + np.random.randn() * ekf.sig_baro
            ekf.predict(a_imu)
            ekf.update_baro(z_baro)

        z_ekf = ekf.z    if ekf is not None else z
        v_ekf = ekf.v    if ekf is not None else v
        z_est[i]  = z_ekf
        v_est[i]  = v_ekf
        var_z[i]  = ekf.var_z if ekf is not None else 0.0
        var_v[i]  = ekf.var_v if ekf is not None else 0.0

        # ── apogee predictor (only after burnout to avoid cost + thrust error) ──
        if t > t_burn - 0.5 and v_ekf > 0.0:
            z_ap = predict_apogee(z_ekf, v_ekf, lev, m,
                                  drag_arr, airbrake_model, elevation)
        else:
            z_ap = z_ekf
        zpred[i] = z_ap

        # ── PID controller ─────────────────────────────────────────────
        new_lev = lev
        if pid is not None and z_target is not None:
            # Arm once: after burnout settling, while ascending
            if not ctrl_armed and not apo_done and t > t_burn + 0.5 and v_ekf > 2.0:
                ctrl_armed = True
            # Disarm at apogee — only after having been armed (avoids v=0 at t=0 locking out)
            if ctrl_armed and v_ekf <= 0.0:
                apo_done   = True
                ctrl_armed = False

            if ctrl_armed:
                err           = z_ap - z_target   # + → overshooting → deploy
                raw           = pid.step(err)
                pid_raw[i]    = raw
                pid_err[i]    = err
                new_lev       = float(np.clip(raw, 0.0, 1.0))

        ang_arr[i] = lev * AIRBRAKE_MAX_ANGLE
        if i + 1 < N:
            lev_arr[i + 1] = new_lev
            ang_arr[i + 1] = new_lev * AIRBRAKE_MAX_ANGLE

        # ── stop at touchdown ──────────────────────────────────────────
        if z <= 0.0 and v < 0.0 and t > 5.0:
            i_end = i
            break

        # ── RK4 step ───────────────────────────────────────────────────
        if i + 1 < N:
            state = rk4_step(state, t, dt, lev_arr[i + 1],
                             thrust_fn, mass_fn, drag_arr, airbrake_model, elevation)

    end = min(i_end + 50, N)
    s   = slice(0, end)
    return {
        "t":       t_vec[s],
        "z":       z_true[s],   "v":       v_true[s],
        "z_est":   z_est[s],    "v_est":   v_est[s],
        "var_z":   var_z[s],    "var_v":   var_v[s],
        "lev":     lev_arr[s],  "ang":     ang_arr[s],
        "Fd":      Fd_arr[s],   "Ft":      Ft_arr[s],
        "Mach":    Mach_arr[s], "mass":    mass_arr[s],
        "zpred":   zpred[s],    "pid_raw": pid_raw[s],
        "accel":   accel[s],    "pid_err": pid_err[s],
    }


# ============================================================
# RUN SINGLE  (replaces the RocketPy-based version in mags_test_launch.py)
# ============================================================

def run_single(args, drag_table, airbrake_model):
    np.random.seed(42)

    # ── motor ────────────────────────────────────────────────────────
    t_thr, F_thr, m_prop, m_total_motor = load_eng_file(args.motor_file)
    t_burn  = float(t_thr[-1])
    m_case  = m_total_motor - m_prop          # motor hardware mass [kg]
    m0      = ROCKET_MASS + m_case + m_prop   # total launch mass   [kg]

    thrust_fn = lambda t: float(np.interp(min(max(t, 0.0), t_burn), t_thr, F_thr))
    mass_fn   = lambda t: m0 - m_prop * min(t, t_burn) / t_burn

    drag_arr  = np.array(drag_table, dtype=float)
    elevation = args.elevation
    t_vec     = np.arange(0.0, 90.0 + DT, DT)

    print(f"\nMotor  : {args.motor_file}")
    print(f"  Total launch mass : {m0:.3f} kg")
    print(f"  Propellant mass   : {m_prop:.3f} kg")
    print(f"  Burnout           : {t_burn:.3f} s")

    # ── Phase 1: open-loop ───────────────────────────────────────────
    print("\nPhase 1 — open-loop (no airbrakes) ...")
    ol = _run_loop(t_vec, drag_arr, airbrake_model,
                   thrust_fn, mass_fn, t_burn, elevation)

    apogee_ol = float(np.max(ol["z"]))
    z_target  = apogee_ol - 100.0
    print(f"  Natural apogee : {apogee_ol:.1f} m AGL  ({apogee_ol * 3.28084:.0f} ft)")
    print(f"  PID target     : {z_target:.1f} m AGL  ({z_target * 3.28084:.0f} ft)")

    # ── Phase 2: closed-loop (PID + EKF) ────────────────────────────
    print("\nPhase 2 — closed-loop (PID + EKF) ...")
    ekf = EKF1D(dt=DT, sig_baro=SIG_BARO, sig_imu=SIG_IMU)
    pid = PID(Kp=KP, Ki=KI, Kd=KD, dt=DT)
    cl  = _run_loop(t_vec, drag_arr, airbrake_model,
                    thrust_fn, mass_fn, t_burn, elevation,
                    ekf=ekf, pid=pid, z_target=z_target)

    apogee_cl = float(np.max(cl["z"]))
    err_m     = apogee_cl - z_target

    print(f"\n  {'='*44}")
    print(f"  Natural apogee (no brakes) : {apogee_ol:8.1f} m  ({apogee_ol*3.28084:.0f} ft)")
    print(f"  PID target (apogee−100 m)  : {z_target:8.1f} m  ({z_target*3.28084:.0f} ft)")
    print(f"  Achieved apogee (PID+EKF)  : {apogee_cl:8.1f} m  ({apogee_cl*3.28084:.0f} ft)")
    print(f"  Error vs target            :   {err_m:+6.1f} m  ({err_m*3.28084:+.1f} ft)")
    print(f"  Max deployment angle       :  {np.max(cl['ang']):5.1f} °")
    print(f"  Max velocity               :  {np.max(cl['v']):5.1f} m/s")
    print(f"  Max Mach                   :  {np.max(cl['Mach']):.3f}")
    print(f"  Max drag force             :  {np.max(cl['Fd']):.0f} N")
    print(f"  Max acceleration           :  {np.max(cl['accel']):.1f} m/s²  ({np.max(cl['accel'])/G0:.2f} g)")
    print(f"  {'='*44}\n")

    _make_plots(ol, cl, apogee_ol, z_target, apogee_cl, t_thr, F_thr, t_burn)
    _make_comparison_plot(ol, cl, apogee_ol, z_target, apogee_cl)


# ============================================================
# COMPARISON FIGURE  (no airbrakes vs airbrakes, 6-panel)
# ============================================================

def _make_comparison_plot(ol, cl, apogee_ol, z_target, apogee_cl):
    """Side-by-side comparison of the open-loop and closed-loop flights."""

    C_ol    = (0.55, 0.55, 0.55)
    C_cl    = (0.12, 0.47, 0.71)
    C_target= (0.84, 0.15, 0.16)
    C_green = (0.17, 0.63, 0.17)
    C_pur   = (0.58, 0.40, 0.74)
    C_teal  = (0.09, 0.75, 0.81)

    # Align time axes: pad the shorter array with its last value so both
    # series cover the same time window on every subplot.
    t_max = max(ol["t"][-1], cl["t"][-1])

    def _pad(run, key, fill=0.0):
        """Return (t_common, values) padded to t_max."""
        t = run["t"]; y = run[key]
        if t[-1] < t_max:
            extra_t = np.array([t_max])
            extra_y = np.array([fill if fill != "last" else y[-1]])
            t = np.concatenate([t, extra_t])
            y = np.concatenate([y, extra_y])
        return t, y

    fig = plt.figure(figsize=(16, 10))
    fig.patch.set_facecolor("white")
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.42, wspace=0.33)

    # ── 1. Altitude ──────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(*_pad(ol, "z", fill="last"), "-",  color=C_ol,  lw=2.0, label="No airbrakes")
    ax1.plot(*_pad(cl, "z", fill="last"), "-",  color=C_cl,  lw=2.0, label="PID + EKF")
    ax1.axhline(apogee_ol, color=C_ol,    ls="--", lw=1.2,
                label=f"Apogee: {apogee_ol:.0f} m")
    ax1.axhline(apogee_cl, color=C_cl,    ls="--", lw=1.2,
                label=f"Achieved: {apogee_cl:.0f} m")
    ax1.axhline(z_target,  color=C_target, ls=":",  lw=1.4,
                label=f"Target: {z_target:.0f} m")
    # annotate apogee difference
    ax1.annotate(
        f"Δ = {apogee_ol - apogee_cl:.0f} m",
        xy=(cl["t"][int(np.argmax(cl["z"]))], apogee_cl),
        xytext=(cl["t"][int(np.argmax(cl["z"]))] + 3, (apogee_ol + apogee_cl) / 2),
        arrowprops=dict(arrowstyle="<->", color="k", lw=1.0),
        fontsize=8, ha="left", va="center",
    )
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Altitude AGL (m)")
    ax1.set_title("Altitude Profile"); ax1.grid(True)
    ax1.legend(fontsize=7, loc="upper right")

    # ── 2. Velocity ──────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(*_pad(ol, "v", fill="last"), "-", color=C_ol, lw=2.0, label="No airbrakes")
    ax2.plot(*_pad(cl, "v", fill="last"), "-", color=C_cl, lw=2.0, label="PID + EKF")
    ax2.axhline(0, color="k", ls=":", lw=0.8)
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity (m/s)")
    ax2.set_title("Vertical Velocity"); ax2.grid(True)
    ax2.legend(fontsize=7)

    # ── 3. Drag Force ────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(*_pad(ol, "Fd"), "-", color=C_ol,  lw=2.0, label="No airbrakes")
    ax3.plot(*_pad(cl, "Fd"), "-", color=C_pur, lw=2.0, label="PID + EKF")
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Drag Force (N)")
    ax3.set_title("Total Drag Force"); ax3.grid(True)
    ax3.legend(fontsize=7)

    # ── 4. Mach Number ───────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(*_pad(ol, "Mach"), "-", color=C_ol,   lw=2.0, label="No airbrakes")
    ax4.plot(*_pad(cl, "Mach"), "-", color=C_teal, lw=2.0, label="PID + EKF")
    ax4.axhline(1.0, color="k", ls="--", lw=1, label="M = 1")
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Mach Number")
    ax4.set_title("Mach Number"); ax4.grid(True)
    ax4.legend(fontsize=7)

    # ── 5. Airbrake Deployment Angle ─────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(*_pad(ol, "ang"), "--", color=C_ol,   lw=1.6, label="No airbrakes (0°)")
    ax5.plot(*_pad(cl, "ang"), "-",  color=C_green, lw=2.0, label="PID + EKF")
    ax5.axhline(AIRBRAKE_MAX_ANGLE, color="k", ls=":", lw=1,
                label=f"Max {AIRBRAKE_MAX_ANGLE:.0f}°")
    ax5.set_ylim(-2, AIRBRAKE_MAX_ANGLE + 10)
    ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Deployment Angle (°)")
    ax5.set_title("Airbrake Deployment Angle"); ax5.grid(True)
    ax5.legend(fontsize=7)

    # ── 6. Apogee Summary Bar Chart ───────────────────────────────────
    ax6 = fig.add_subplot(gs[1, 2])
    labels = ["No Airbrakes", "PID Target", "PID + EKF\nAchieved"]
    values = [apogee_ol, z_target, apogee_cl]
    colors = [C_ol, C_target, C_cl]
    bars   = ax6.bar(labels, values, color=colors, width=0.5,
                     edgecolor="k", linewidth=0.8)
    # value labels on top of each bar
    for bar, val in zip(bars, values):
        ax6.text(bar.get_x() + bar.get_width() / 2,
                 bar.get_height() + 8,
                 f"{val:.0f} m\n({val*3.28084:.0f} ft)",
                 ha="center", va="bottom", fontsize=8, fontweight="bold")
    # error annotation between target and achieved
    x_mid = 2
    ax6.annotate(
        "",
        xy=(x_mid - 0.5, apogee_cl), xytext=(x_mid - 0.5, z_target),
        arrowprops=dict(arrowstyle="<->", color="k", lw=1.2),
    )
    ax6.text(x_mid - 0.55, (apogee_cl + z_target) / 2,
             f"{apogee_cl - z_target:+.1f} m", ha="right", va="center",
             fontsize=8, color="k")
    ax6.set_ylabel("Apogee AGL (m)")
    ax6.set_title("Apogee Summary")
    # set y range with headroom
    y_lo = min(values) * 0.97
    y_hi = max(values) * 1.06
    ax6.set_ylim(y_lo, y_hi)
    ax6.yaxis.set_major_formatter(
        matplotlib.ticker.FuncFormatter(lambda x, _: f"{x:.0f}")
    )
    ax6.grid(True, axis="y")

    fig.suptitle(
        "MAGS Rocket — No Airbrakes vs PID + EKF Airbrake Control\n"
        f"No-brake apogee: {apogee_ol:.0f} m  |  "
        f"Target: {z_target:.0f} m (−100 m)  |  "
        f"Achieved: {apogee_cl:.0f} m  |  "
        f"Error: {apogee_cl - z_target:+.1f} m",
        fontsize=12, fontweight="bold",
    )
    plt.show()


# ============================================================
# PLOTS  (9-panel, matching airbrake_closed_loop_sim.m)
# ============================================================

def _make_plots(ol, cl, apogee_ol, z_target, apogee_cl, t_thr, F_thr, t_burn):

    C_ol    = (0.55, 0.55, 0.55)
    C_cl    = (0.12, 0.47, 0.71)
    C_ekf   = (0.84, 0.15, 0.16)
    C_band  = (0.80, 0.80, 1.00)
    C_green = (0.17, 0.63, 0.17)
    C_pur   = (0.58, 0.40, 0.74)
    C_teal  = (0.09, 0.75, 0.81)

    def shade(ax, t, mu, var):
        sig = np.sqrt(np.abs(var))
        ax.fill_between(t, mu - 2*sig, mu + 2*sig,
                        color=C_band, alpha=0.5, label="EKF ±2σ")

    fig = plt.figure(figsize=(19, 11))
    fig.patch.set_facecolor("white")
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.44, wspace=0.33)

    # ── 1. Altitude ──────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    shade(ax1, cl["t"], cl["z_est"], cl["var_z"])
    ax1.plot(ol["t"], ol["z"],     "--", color=C_ol,  lw=1.4, label="No airbrakes")
    ax1.plot(cl["t"], cl["z"],     "-",  color=C_cl,  lw=1.8, label="Closed-loop true")
    ax1.plot(cl["t"], cl["z_est"], ":",  color=C_ekf, lw=1.4, label="EKF estimate")
    ax1.axhline(z_target,  color=C_ekf, ls="--", lw=1.1,
                label=f"Target {z_target:.0f} m")
    ax1.axhline(apogee_ol, color=C_ol,  ls="--", lw=0.8)
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Altitude AGL (m)")
    ax1.set_title("Altitude"); ax1.grid(True)
    ax1.legend(fontsize=6.5, loc="upper left")

    # ── 2. Velocity ──────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[0, 1])
    shade(ax2, cl["t"], cl["v_est"], cl["var_v"])
    ax2.plot(ol["t"], ol["v"],     "--", color=C_ol,  lw=1.4, label="No airbrakes")
    ax2.plot(cl["t"], cl["v"],     "-",  color=C_cl,  lw=1.8, label="Closed-loop true")
    ax2.plot(cl["t"], cl["v_est"], ":",  color=C_ekf, lw=1.4, label="EKF estimate")
    ax2.axhline(0, color="k", ls=":", lw=0.8)
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity (m/s)")
    ax2.set_title("Vertical Velocity"); ax2.grid(True)
    ax2.legend(fontsize=6.5, loc="best")

    # ── 3. Drag Force ────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[0, 2])
    ax3.plot(cl["t"], cl["Fd"], "-", color=C_pur, lw=1.8)
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Drag Force (N)")
    ax3.set_title("Total Drag Force (closed-loop)"); ax3.grid(True)

    # ── 4. Airbrake Deployment Angle ─────────────────────────────────
    ax4 = fig.add_subplot(gs[1, 0])
    ax4.plot(cl["t"], cl["ang"], "-", color=C_green, lw=1.8)
    ax4.axhline(AIRBRAKE_MAX_ANGLE, color="k", ls=":", lw=1,
                label=f"Max {AIRBRAKE_MAX_ANGLE:.0f}°")
    ax4.set_ylim(-2, AIRBRAKE_MAX_ANGLE + 8)
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Deployment Angle (°)")
    ax4.set_title("Airbrake Deployment Angle"); ax4.grid(True)
    ax4.legend(fontsize=7)

    # ── 5. Mach Number ───────────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(cl["t"], cl["Mach"], "-", color=C_teal, lw=1.8)
    ax5.axhline(1.0, color="k", ls="--", lw=1, label="M = 1")
    ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Mach Number")
    ax5.set_title("Mach Number"); ax5.grid(True)
    ax5.legend(fontsize=7)

    # ── 6. Motor Thrust ──────────────────────────────────────────────
    ax6 = fig.add_subplot(gs[1, 2])
    t_fine = np.linspace(0.0, t_burn, 500)
    F_fine = np.interp(t_fine, t_thr, F_thr)
    ax6.plot(t_fine, F_fine, "k-", lw=1.8, label="Interpolated")
    ax6.plot(t_thr,  F_thr,  "ro", ms=5,   label="Data points")
    ax6.set_xlabel("Time (s)"); ax6.set_ylabel("Thrust (N)")
    ax6.set_title("Motor Thrust  (m2050x_corrected.eng)"); ax6.grid(True)
    ax6.legend(fontsize=7)

    # ── 7. Predicted Apogee vs Target ────────────────────────────────
    ax7 = fig.add_subplot(gs[2, 0])
    ax7.plot(cl["t"], cl["zpred"], "-", color=C_cl, lw=1.8, label="Predicted")
    ax7.axhline(z_target,  color=C_ekf, ls="--", lw=1.2,
                label=f"Target {z_target:.0f} m")
    ax7.axhline(apogee_ol, color=C_ol,  ls="--", lw=0.8,
                label=f"No-brake {apogee_ol:.0f} m")
    i_arm = int(np.argmax(cl["lev"] > 0.01))
    if cl["lev"][i_arm] > 0.01:
        ax7.axvline(cl["t"][i_arm], color="grey", ls=":", lw=1, label="Brakes armed")
    ax7.set_xlabel("Time (s)"); ax7.set_ylabel("Predicted Apogee (m)")
    ax7.set_title("Apogee Prediction  (EKF numerical forward-integration)")
    ax7.grid(True); ax7.legend(fontsize=6.5)

    # ── 8. PID Error & Deployment Level ──────────────────────────────
    ax8  = fig.add_subplot(gs[2, 1])
    ax8r = ax8.twinx()
    ax8.plot( cl["t"], cl["pid_err"], "-",  color=C_cl,    lw=1.4, label="Apogee error (m)")
    ax8.axhline(0, color="k", ls=":", lw=0.8)
    ax8r.plot(cl["t"], cl["lev"],     "-",  color=C_green, lw=1.8, label="Deployment level")
    ax8r.set_ylim(-0.05, 1.15)
    ax8.set_xlabel("Time (s)")
    ax8.set_ylabel("Apogee Error (m)",      color=C_cl)
    ax8r.set_ylabel("Deployment Level [0–1]", color=C_green)
    ax8.tick_params(axis="y", colors=C_cl)
    ax8r.tick_params(axis="y", colors=C_green)
    ax8.set_title("PID: Error Signal & Deployment Level"); ax8.grid(True)
    h1, l1 = ax8.get_legend_handles_labels()
    h2, l2 = ax8r.get_legend_handles_labels()
    ax8.legend(h1 + h2, l1 + l2, fontsize=6.5, loc="best")

    # ── 9. Net Acceleration ──────────────────────────────────────────
    ax9  = fig.add_subplot(gs[2, 2])
    ax9r = ax9.twinx()
    ax9.plot( cl["t"], cl["accel"],       "-",  color=C_cl,  lw=1.8, label="m/s²")
    ax9.axhline(0, color="k", ls=":", lw=0.8)
    ax9r.plot(cl["t"], cl["accel"] / G0,  "--", color=C_ekf, lw=1.4, label="g-force")
    ax9.set_xlabel("Time (s)")
    ax9.set_ylabel("Acceleration (m/s²)", color=C_cl)
    ax9r.set_ylabel("Acceleration (g)",   color=C_ekf)
    ax9.tick_params(axis="y", colors=C_cl)
    ax9r.tick_params(axis="y", colors=C_ekf)
    ax9.set_title("Net Vertical Acceleration"); ax9.grid(True)
    h1, l1 = ax9.get_legend_handles_labels()
    h2, l2 = ax9r.get_legend_handles_labels()
    ax9.legend(h1 + h2, l1 + l2, fontsize=7, loc="best")

    fig.suptitle(
        "MAGS Rocket — Airbrake Closed-Loop Simulation  (PID + EKF)\n"
        f"m2050x_corrected.eng  |  "
        f"No-brake apogee: {apogee_ol:.0f} m  |  "
        f"PID target: {z_target:.0f} m (−100 m)  |  "
        f"Achieved: {apogee_cl:.0f} m",
        fontsize=12, fontweight="bold",
    )
    plt.show()


# ============================================================
# CLI
# ============================================================

def build_parser():
    parser = argparse.ArgumentParser(
        description="MAGS rocket 1-D PID+EKF airbrake simulation (no RocketPy)."
    )
    parser.add_argument("--drag-csv",         default=str(DEFAULT_BASE_DRAG_CSV),
                        help="Base Mach-Cd CSV (default: Deployment_0deg_CdFit.csv)")
    parser.add_argument("--motor-file",       default=str(DEFAULT_MOTOR_FILE),
                        help=".eng motor file (default: m2050x_corrected.eng)")
    parser.add_argument("--airbrake-csv-dir", default=str(DEFAULT_AIRBRAKE_CSV_DIR),
                        help="Directory of Deployment_<N>deg_CdFit.csv files")
    parser.add_argument("--elevation",        type=float, default=DEFAULT_ELEVATION,
                        help="Launch-site elevation above MSL [m]")
    parser.add_argument("--no-airbrakes",     action="store_true",
                        help="Disable airbrake ΔCd (base drag only)")
    return parser


def main():
    args = build_parser().parse_args()

    drag_table = load_base_drag_curve(args.drag_csv)

    airbrake_model = None
    if not args.no_airbrakes:
        airbrake_model = AirbrakeModel(
            csv_dir=args.airbrake_csv_dir,
            reference_area=REF_AREA,
        )

    run_single(args, drag_table, airbrake_model)


if __name__ == "__main__":
    main()
