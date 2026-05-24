/*
 * airbrake_control.c
 *
 * Proportional apogee controller.
 *
 * After burnout the estimator predicts where the rocket will reach apogee
 * given the current airbrake deployment.  This module converts the predicted-
 * vs-target error into a deployment fraction and commands the airbrakes via
 * Airbrake_SetTargetAngle().
 *
 * Control law (mirrors init_airbrake_model.m Kp_apogee section):
 *
 *   error  = predicted_apogee - target_apogee
 *   level  = Kp * error   (clamped to [0, 1], deadbanded at ±5 m)
 *   angle  = level * MAX_DEPLOY_ANGLE
 *
 * The deploy fraction is forwarded to the estimator so the coast predictor
 * accounts for the actual drag being generated.
 *
 * Rate limiting prevents the actuator from being slammed full-open in one
 * tick, which would exceed the ESC torque budget.
 */

#include "airbrake_control.h"
#include "airbrake_deploy.h"
#include "flight_estimator.h"
#include "main.h"

#include <stdint.h>

/* ============================================================================
 * Module state
 * ========================================================================== */

static float    s_target_m       = AIRBRAKE_DEFAULT_TARGET_ALT_M;
static float    s_deploy_level   = 0.0f;
static uint8_t  s_active         = 0U;
static uint32_t s_last_update_ms = 0U;

/* ============================================================================
 * Public API
 * ========================================================================== */

void Airbrake_Control_Init(void)
{
    s_target_m       = AIRBRAKE_DEFAULT_TARGET_ALT_M;
    s_deploy_level   = 0.0f;
    s_active         = 0U;
    s_last_update_ms = 0U;
}

void Airbrake_Control_Start(void)
{
    s_active         = 1U;
    s_deploy_level   = 0.0f;
    s_last_update_ms = HAL_GetTick();
    Airbrake_StartControl();
    Airbrake_SetTargetAngle(0.0f);
}

void Airbrake_Control_SetTargetFt(float feet)
{
    s_target_m = feet * 0.3048f;
}

float Airbrake_Control_GetDeployFraction(void)
{
    return s_deploy_level;
}

void Airbrake_Control_Task(void)
{
    if (!s_active)
        return;

    uint32_t now = HAL_GetTick();
    if ((now - s_last_update_ms) < CTRL_UPDATE_PERIOD_MS)
        return;
    s_last_update_ms = now;

    EstimatorState_t est;
    Estimator_GetState(&est);

    /* Retract and deactivate on descent */
    if (est.phase == EST_DESCENT)
    {
        s_deploy_level = 0.0f;
        Airbrake_SetTargetAngle(0.0f);
        Estimator_SetDeployLevel(0.0f);
        s_active = 0U;
        return;
    }

    if (est.phase != EST_COAST)
        return;

    float error = est.apogee_est_m - s_target_m;

    /* Deadband — don't hunt near the target */
    if (error > -CTRL_DEADBAND_M && error < CTRL_DEADBAND_M)
    {
        Airbrake_SetTargetAngle(s_deploy_level * AIRBRAKE_MAX_ANGLE_DEG);
        Estimator_SetDeployLevel(s_deploy_level);
        return;
    }

    /* Proportional command — brakes only add drag, never reduce it */
    float cmd = CTRL_KP_APOGEE * error;
    if (cmd < 0.0f) cmd = 0.0f;
    if (cmd > 1.0f) cmd = 1.0f;

    /* Rate limit */
    float delta = cmd - s_deploy_level;
    if (delta >  CTRL_MAX_DPLEVEL_STEP) delta =  CTRL_MAX_DPLEVEL_STEP;
    if (delta < -CTRL_MAX_DPLEVEL_STEP) delta = -CTRL_MAX_DPLEVEL_STEP;
    s_deploy_level += delta;

    Airbrake_SetTargetAngle(s_deploy_level * AIRBRAKE_MAX_ANGLE_DEG);
    Estimator_SetDeployLevel(s_deploy_level);
}
