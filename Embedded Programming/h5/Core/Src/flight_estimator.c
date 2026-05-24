/*
 * flight_estimator.c
 *
 * State estimator for the airbrake control system.
 *
 * Three concerns handled here:
 *
 *  1. Altitude / velocity estimation (2-state Kalman filter)
 *     State: x = [h (m AGL), v (m/s, positive up)]
 *     Barometer is the measurement; a drag-based process model propagates
 *     state between baro packets.
 *
 *  2. Motor impulse integration
 *     During boost the IMU magnitude is used to back-calculate thrust
 *     (F ≈ m * |a_imu| * g0, drag small vs. thrust).  Integrated impulse is
 *     compared to the nominal L2200G value at burnout.  If the deviation
 *     exceeds MOTOR_IMPULSE_DEVIATION_THR the live-integrated value is flagged
 *     and used for mass accounting throughout the flight.
 *
 *  3. Apogee prediction
 *     After burnout, forward-Euler integration of the coast equations
 *     (gravity + Mach-dependent drag) gives a predicted apogee.  The
 *     airbrake_control module reads this every control tick.
 *
 * Coordinate convention
 * ---------------------
 *   Altitude positive up, velocity positive up.
 *   IMU magnitude (all axes) is used — no assumption about board orientation.
 *   During upward coast the magnitude equals F_drag / m (drag opposes motion).
 */

#include "flight_estimator.h"
#include "esc_app.h"
#include "main.h"

#include <math.h>
#include <string.h>

/* ============================================================================
 * Physical constants
 * ========================================================================== */

#define G0_MPS2         9.80665f
#define RHO0_KGM3       1.225f
#define T0_K            288.15f
#define L_KPM           0.0065f     /* troposphere lapse rate [K/m]    */
#define R_AIR           287.05f
#define GAMMA_AIR       1.4f

/* ============================================================================
 * L2200G nominal thrust curve (Aerotech, 75 mm)
 * Time [s] / Thrust [N] breakpoints — trapezoidal total ≈ 5910 N·s
 * ========================================================================== */

#define MOTOR_CURVE_PTS  13U

static const float MOTOR_T_S[MOTOR_CURVE_PTS] __attribute__((unused)) = {
    0.000f, 0.025f, 0.075f, 0.150f, 0.300f, 0.600f,
    1.000f, 1.500f, 2.000f, 2.400f, 2.550f, 2.650f, 2.700f
};
static const float MOTOR_F_N[MOTOR_CURVE_PTS] __attribute__((unused)) = {
    0.0f, 1800.0f, 2700.0f, 2600.0f, 2400.0f, 2300.0f,
    2250.0f, 2200.0f, 2150.0f, 2000.0f, 1200.0f, 300.0f, 0.0f
};

/* ============================================================================
 * Airframe drag table (Mach vs Cd — from OpenRocket / RocketPy model)
 * Matches Mach_bp / Cd_body_bp in init_airbrake_model.m
 * ========================================================================== */

#define CD_TABLE_PTS  11U

static const float CD_MACH_BP[CD_TABLE_PTS] = {
    0.00f, 0.10f, 0.30f, 0.50f, 0.70f, 0.85f,
    0.95f, 1.05f, 1.20f, 1.50f, 2.00f
};
static const float CD_BODY[CD_TABLE_PTS] = {
    0.45f, 0.45f, 0.43f, 0.42f, 0.44f, 0.50f,
    0.65f, 0.75f, 0.62f, 0.52f, 0.48f
};

/* Airbrake ΔCd at full deployment (deploy_level = 1.0), subsonic.
 * Scales as deploy_level² to approximate plate drag behaviour. */
#define CD_AB_FULL_DEPLOY   0.30f

/* ============================================================================
 * KF tuning  (mirrors init_airbrake_model.m observer section)
 * ========================================================================== */

/* Process noise variances */
#define KF_Q_H          0.0001f   /* altitude [m²]       (σ = 0.01 m/s) */
#define KF_Q_V          0.01f     /* velocity [m²/s²]    (σ = 0.1  m/s²) */

/* Baro measurement noise variance  (σ = 0.5 m) */
#define KF_R_BARO       0.25f

/* Initial covariance diagonal */
#define KF_P0_H         100.0f   /* 10² */
#define KF_P0_V         100.0f   /* 10² */

/* Baro ground-level calibration samples */
#define BARO_CAL_SAMPLES  50U

/* Apogee predictor step size and max time */
#define PRED_DT_S        0.05f
#define PRED_MAX_TIME_S  90.0f

/* Descent detection: v < this threshold → past apogee */
#define APOGEE_V_THR_MPS  -2.0f

/* Maximum dt clamped to avoid runaway KF integration on dropped packets */
#define MAX_DT_S          0.10f

/* ============================================================================
 * Module state
 * ========================================================================== */

static volatile EstimatorPhase_t s_phase = EST_IDLE;

/* 2-state KF: x[0]=h, x[1]=v */
static float s_x[2];
static float s_P[2][2];

/* Baro ground reference */
static float     s_h_ground_m = 0.0f;
static uint32_t  s_cal_count  = 0U;
static float     s_cal_sum    = 0.0f;

/* Timing */
static uint32_t  s_last_tick_ms = 0U;

/* Motor integration */
static float s_impulse_live_ns  = 0.0f;
static float s_mass_kg          = ESTIMATOR_M0_KG;
static uint8_t s_use_live_imp   = 0U;

/* Latest apogee prediction */
static float s_apogee_est_m = 0.0f;

/* Deploy level injected by control module */
static float s_inject_deploy = 0.0f;

/* Snapshot for external readers */
static EstimatorState_t s_state_snap;

/* ============================================================================
 * Atmosphere helpers
 * ========================================================================== */

static float Atm_Temperature(float h_m)
{
    float T = T0_K - L_KPM * h_m;
    if (T < 216.65f) T = 216.65f;  /* tropopause floor */
    return T;
}

static float Atm_Density(float h_m)
{
    float T   = Atm_Temperature(h_m);
    float exp = G0_MPS2 / (R_AIR * L_KPM) - 1.0f;  /* ≈ 4.256 */
    return RHO0_KGM3 * powf(T / T0_K, exp);
}

static float Atm_SpeedOfSound(float h_m)
{
    return sqrtf(GAMMA_AIR * R_AIR * Atm_Temperature(h_m));
}

/* ============================================================================
 * Aerodynamic helpers
 * ========================================================================== */

static float CdBodyInterp(float mach)
{
    if (mach <= CD_MACH_BP[0])               return CD_BODY[0];
    if (mach >= CD_MACH_BP[CD_TABLE_PTS-1])  return CD_BODY[CD_TABLE_PTS-1];

    for (uint32_t i = 1U; i < CD_TABLE_PTS; i++)
    {
        if (mach <= CD_MACH_BP[i])
        {
            float t = (mach - CD_MACH_BP[i-1]) / (CD_MACH_BP[i] - CD_MACH_BP[i-1]);
            return CD_BODY[i-1] + t * (CD_BODY[i] - CD_BODY[i-1]);
        }
    }
    return CD_BODY[CD_TABLE_PTS-1];
}

/* Drag acceleration magnitude [m/s²] */
static float DragAccel(float v_mps, float h_m, float deploy_level, float mass_kg)
{
    float rho   = Atm_Density(h_m);
    float spd   = fabsf(v_mps);
    float mach  = spd / Atm_SpeedOfSound(h_m);
    float cd    = CdBodyInterp(mach) + deploy_level * deploy_level * CD_AB_FULL_DEPLOY;
    float q     = 0.5f * rho * spd * spd;
    return (q * cd * ESTIMATOR_REF_AREA_M2) / mass_kg;
}

/* ============================================================================
 * Kalman filter
 * ========================================================================== */

static void KF_Init(float h0)
{
    s_x[0] = h0;
    s_x[1] = 0.0f;

    s_P[0][0] = KF_P0_H;  s_P[0][1] = 0.0f;
    s_P[1][0] = 0.0f;      s_P[1][1] = KF_P0_V;
}

/* Predict step using drag-only process model (coast phase) */
static void KF_Predict(float dt, float deploy_level)
{
    float v = s_x[1];
    float h = s_x[0];

    float a_drag = DragAccel(v, h, deploy_level, s_mass_kg);
    float a      = -G0_MPS2 - (v >= 0.0f ? a_drag : -a_drag);

    s_x[0] += v * dt + 0.5f * a * dt * dt;
    s_x[1] += a * dt;

    /* P = F*P*F' + Q   with F = [[1,dt],[0,1]] */
    float p00 = s_P[0][0] + dt*(s_P[1][0] + s_P[0][1]) + dt*dt*s_P[1][1] + KF_Q_H;
    float p01 = s_P[0][1] + dt * s_P[1][1];
    float p10 = s_P[1][0] + dt * s_P[1][1];
    float p11 = s_P[1][1] + KF_Q_V;

    s_P[0][0] = p00;
    s_P[0][1] = p01;
    s_P[1][0] = p10;
    s_P[1][1] = p11;
}

/* Measurement update with baro altitude */
static void KF_Update(float baro_h_m)
{
    float y     = baro_h_m - s_x[0];
    float S_inv = 1.0f / (s_P[0][0] + KF_R_BARO);
    float K0    = s_P[0][0] * S_inv;
    float K1    = s_P[1][0] * S_inv;

    s_x[0] += K0 * y;
    s_x[1] += K1 * y;

    float p00 = (1.0f - K0) * s_P[0][0];
    float p01 = (1.0f - K0) * s_P[0][1];
    float p10 = s_P[1][0] - K1 * s_P[0][0];
    float p11 = s_P[1][1] - K1 * s_P[0][1];

    s_P[0][0] = p00;
    s_P[0][1] = p01;
    s_P[1][0] = p10;
    s_P[1][1] = p11;
}

/* ============================================================================
 * Apogee predictor
 * ========================================================================== */

static float PredictApogee(float h0, float v0, float deploy_level)
{
    float h = h0;
    float v = v0;

    if (v <= 0.0f) return h;

    for (float t = 0.0f; t < PRED_MAX_TIME_S; t += PRED_DT_S)
    {
        float a_drag = DragAccel(v, h, deploy_level, s_mass_kg);
        float a      = -G0_MPS2 - a_drag;   /* v > 0 during upward coast */
        v += a * PRED_DT_S;
        h += v * PRED_DT_S;
        if (v <= 0.0f) return h;
    }
    return h;
}

/* ============================================================================
 * Motor impulse integration (boost phase only)
 * ========================================================================== */

static void Motor_Integrate(float imu_mag_g, float dt)
{
    float F_est = s_mass_kg * imu_mag_g * G0_MPS2;
    s_impulse_live_ns += F_est * dt;

    float prop_burnt = (s_impulse_live_ns / MOTOR_TOTAL_IMPULSE_NS) * MOTOR_PROP_MASS_KG;
    if (prop_burnt > MOTOR_PROP_MASS_KG) prop_burnt = MOTOR_PROP_MASS_KG;
    s_mass_kg = ESTIMATOR_M0_KG - prop_burnt;
}

static void Motor_FinaliseImpulse(void)
{
    float deviation = fabsf(s_impulse_live_ns - MOTOR_TOTAL_IMPULSE_NS)
                      / MOTOR_TOTAL_IMPULSE_NS;
    s_use_live_imp = (deviation > MOTOR_IMPULSE_DEVIATION_THR) ? 1U : 0U;
    s_mass_kg      = ESTIMATOR_M_DRY_KG;
}

/* ============================================================================
 * Public API
 * ========================================================================== */

void Estimator_Init(void)
{
    s_phase           = EST_IDLE;
    s_h_ground_m      = 0.0f;
    s_cal_count       = 0U;
    s_cal_sum         = 0.0f;
    s_last_tick_ms    = HAL_GetTick();
    s_impulse_live_ns = 0.0f;
    s_mass_kg         = ESTIMATOR_M0_KG;
    s_use_live_imp    = 0U;
    s_apogee_est_m    = 0.0f;
    s_inject_deploy   = 0.0f;

    KF_Init(0.0f);
    memset(&s_state_snap, 0, sizeof(s_state_snap));
}

void Estimator_SetBoost(void)
{
    s_phase           = EST_BOOST;
    s_impulse_live_ns = 0.0f;
    s_mass_kg         = ESTIMATOR_M0_KG;
    KF_Init(s_x[0]);
    s_x[1] = 0.0f;
}

void Estimator_SetCoast(void)
{
    Motor_FinaliseImpulse();
    s_phase = EST_COAST;
}

void Estimator_Task(void)
{
    SensorPacket_t pkt;
    if (!ESC_App_GetLatestSensorPacket(&pkt))
        return;

    uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_last_tick_ms) * 0.001f;
    s_last_tick_ms = now;

    if (dt <= 0.0001f) return;
    if (dt > MAX_DT_S) dt = MAX_DT_S;

    float baro_raw_m = (float)pkt.altitude * 0.01f;
    float ax = pkt.imu16_ax, ay = pkt.imu16_ay, az = pkt.imu16_az;
    float imu_mag_g = sqrtf(ax*ax + ay*ay + az*az);

    switch (s_phase)
    {
        /* ── IDLE: calibrate baro ground reference ──────────────────────── */
        case EST_IDLE:
            s_cal_sum += baro_raw_m;
            s_cal_count++;
            if (s_cal_count >= BARO_CAL_SAMPLES)
            {
                s_h_ground_m = s_cal_sum / (float)s_cal_count;
                s_cal_sum    = 0.0f;
                s_cal_count  = 0U;
            }
            s_x[0] = 0.0f;
            s_x[1] = 0.0f;
            break;

        /* ── BOOST: integrate motor impulse; KF driven by IMU ───────────── */
        case EST_BOOST:
        {
            float h_agl = baro_raw_m - s_h_ground_m;
            Motor_Integrate(imu_mag_g, dt);

            /* Net upward inertial acceleration from IMU magnitude */
            float a_inertial = (imu_mag_g - 1.0f) * G0_MPS2;
            s_x[0] += s_x[1] * dt + 0.5f * a_inertial * dt * dt;
            s_x[1] += a_inertial * dt;

            /* Inflate covariance during burn (higher process noise) */
            s_P[0][0] += KF_Q_H * 10.0f;
            s_P[1][1] += KF_Q_V * 10.0f;

            KF_Update(h_agl);
            break;
        }

        /* ── COAST: KF with drag process model + apogee prediction ─────── */
        case EST_COAST:
        {
            float h_agl = baro_raw_m - s_h_ground_m;

            KF_Predict(dt, s_inject_deploy);
            KF_Update(h_agl);

            s_apogee_est_m = PredictApogee(s_x[0], s_x[1], s_inject_deploy);

            if (s_x[1] < APOGEE_V_THR_MPS)
                s_phase = EST_DESCENT;
            break;
        }

        /* ── DESCENT: keep KF running, brakes already retracted ─────────── */
        case EST_DESCENT:
        {
            float h_agl = baro_raw_m - s_h_ground_m;
            KF_Predict(dt, 0.0f);
            KF_Update(h_agl);
            break;
        }

        default:
            break;
    }

    s_state_snap.altitude_m   = s_x[0];
    s_state_snap.velocity_mps = s_x[1];
    s_state_snap.apogee_est_m = s_apogee_est_m;
    s_state_snap.mass_kg      = s_mass_kg;
    s_state_snap.impulse_ns   = s_impulse_live_ns;
    s_state_snap.impulse_live = s_use_live_imp;
    s_state_snap.phase        = s_phase;
}

void Estimator_GetState(EstimatorState_t *out)
{
    if (out == NULL) return;
    __disable_irq();
    *out = s_state_snap;
    __enable_irq();
}

void Estimator_SetDeployLevel(float level)
{
    s_inject_deploy = level;
}

float Estimator_GetDeployLevel(void)
{
    return s_inject_deploy;
}
