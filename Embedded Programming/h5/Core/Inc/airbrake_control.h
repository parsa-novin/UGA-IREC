#ifndef AIRBRAKE_CONTROL_H
#define AIRBRAKE_CONTROL_H

#include <stdint.h>

/* Default target apogee — 10 000 ft AGL in metres */
#define AIRBRAKE_DEFAULT_TARGET_ALT_M   3048.0f

/* Full airbrake deployment angle [deg] — must match FULL_DEPLOY_ANGLE_DEG */
#define AIRBRAKE_MAX_ANGLE_DEG          70.0f

/* ── Controller tuning (mirrors init_airbrake_model.m) ───────────────── */

/* Proportional gain: full deploy commanded for this many metres of overshoot */
#define CTRL_KP_APOGEE                  (1.0f / 300.0f)

/* Deadband: ignore apogee error below this magnitude [m] */
#define CTRL_DEADBAND_M                 5.0f

/* Control update period [ms] — 20 Hz */
#define CTRL_UPDATE_PERIOD_MS           50U

/* Max rate of change of deploy fraction per control tick [fraction/tick] */
#define CTRL_MAX_DPLEVEL_STEP           0.05f

/* ── API ──────────────────────────────────────────────────────────────── */

void Airbrake_Control_Init(void);
void Airbrake_Control_Start(void);   /* called at burnout */
void Airbrake_Control_Task(void);    /* call every ESC_App_Task() iteration */

/* Set target apogee in feet — accepted at any time via UART */
void Airbrake_Control_SetTargetFt(float feet);

/* Returns current deploy fraction [0..1] */
float Airbrake_Control_GetDeployFraction(void);

#endif /* AIRBRAKE_CONTROL_H */
