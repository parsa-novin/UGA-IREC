#include "airbrake_deploy.h"
#include "encoder_app.h"
#include "main.h"
#include "usart.h"

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * SCALE NOTE
 * ----------
 * Encoder_GetPosition_um() returns true micrometres (µm).
 *
 *   1,000 µm = 1.000 mm
 *
 * Therefore:
 *   47.5 mm = 47,500 µm
 *
 * All targets, tolerances, and prints below use µm.
 */

/* Motor-control helpers expected from esc_app.c */
extern void ESC_App_SetPercent(int32_t pct);
extern void ESC_App_StopMotor(void);

typedef enum
{
    DEPLOY_IDLE = 0,
    DEPLOY_START_DELAY,
    DEPLOY_MOVE_FULL,
    DEPLOY_HOLD_FULL,
    DEPLOY_MOVE_60,
    DEPLOY_HOLD_60,
    DEPLOY_MOVE_40,
    DEPLOY_HOLD_40,
    DEPLOY_MOVE_15,
    DEPLOY_HOLD_15,
    DEPLOY_MOVE_ZERO,
    DEPLOY_LOCK_ZERO,
    DEPLOY_DONE,
    DEPLOY_JOG_UP,    /* go to max deployment (70 deg) and hold */
    DEPLOY_JOG_DOWN   /* go to zero position (0 deg) and hold   */
} DeployState_t;

static volatile DeployState_t s_deployState = DEPLOY_IDLE;
static volatile uint32_t s_stateStartTimeMs = 0;

/* ----------------------- user-tunable settings ----------------------- */

/* Internal scale: 1,000 µm = 1 mm */
#define POSITION_UNITS_PER_MM             1000L

/* Transfer-function domain used by the airbrake model */
#define MAX_DISPLACEMENT_MM               40.0f
#define FULL_DEPLOY_ANGLE_DEG             70.0f
#define TRANSFER_SOLVE_ITERS              24U

#define DEPLOY_START_DELAY_MS             200U //THIS

#define DEPLOY_MOVE_PERCENT               50

/* 0.5 mm tolerance => 500 µm */
#define DEPLOY_POSITION_TOL_UNITS         500L

#define DEPLOY_MOVE_TIMEOUT_MS            6000U

#define HOLD_FULL_MS                      3000U
#define HOLD_60_MS                        3000U
#define HOLD_40_MS                        3000U
#define HOLD_15_MS                        3000U
#define LOCK_ZERO_MS                      1500U

/* ------------------------------------------------------------------- */

static void Deploy_Print(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        if (len > (int)sizeof(buf))
            len = (int)sizeof(buf);

        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

static int32_t abs_i32(int32_t x)
{
    return (x < 0) ? -x : x;
}

static float FlapAngle_From_Displacement_mm(float displacement_mm)
{
    if (displacement_mm < 0.0f)
        displacement_mm = 0.0f;

    return (-0.000513f * displacement_mm * displacement_mm * displacement_mm)
         + (0.010615f * displacement_mm * displacement_mm)
         + (2.3199f * displacement_mm)
         - 0.4986f;
}

static void Print_Units_As_mm(const char *label, int32_t pos_units)
{
    int32_t abs_units = abs_i32(pos_units);
    uint32_t mm_whole = (uint32_t)(abs_units / POSITION_UNITS_PER_MM);
    uint32_t mm_frac  = (uint32_t)(abs_units % POSITION_UNITS_PER_MM);

    Deploy_Print("%s%s%lu.%03lu mm (%ld units)\r\n",
                 label,
                 (pos_units < 0) ? "-" : "",
                 (unsigned long)mm_whole,
                 (unsigned long)mm_frac,
                 (long)pos_units);
}

/*
 * Invert the measured displacement->angle transfer function numerically so
 * deploy targets stay aligned with the new airbrake geometry model.
 */
int32_t Airbrake_Angle_To_Position_um(float angle_deg)
{
    float lo_mm;
    float hi_mm;
    uint32_t i;

    if (angle_deg <= 0.0f)
        return 0;

    if (angle_deg >= FULL_DEPLOY_ANGLE_DEG)
        angle_deg = FULL_DEPLOY_ANGLE_DEG;

    lo_mm = 0.0f;
    hi_mm = MAX_DISPLACEMENT_MM;

    for (i = 0U; i < TRANSFER_SOLVE_ITERS; i++)
    {
        float mid_mm = 0.5f * (lo_mm + hi_mm);
        float mid_deg = FlapAngle_From_Displacement_mm(mid_mm);

        if (mid_deg < angle_deg)
            lo_mm = mid_mm;
        else
            hi_mm = mid_mm;
    }

    return (int32_t)((0.5f * (lo_mm + hi_mm)) * (float)POSITION_UNITS_PER_MM + 0.5f);
}

static uint8_t MoveTowardTarget(int32_t target_units)
{
    int32_t pos_units = Encoder_GetPosition_um();
    int32_t err_units = target_units - pos_units;

    if (abs_i32(err_units) <= DEPLOY_POSITION_TOL_UNITS)
    {
        ESC_App_StopMotor();
        return 1U;
    }

    /*
     * Positive command = move "up" / deploy outward
     * Negative command = move back toward 0
     */
    if (err_units > 0)
        ESC_App_SetPercent(+DEPLOY_MOVE_PERCENT);
    else
        ESC_App_SetPercent(-DEPLOY_MOVE_PERCENT/2);

    return 0U;
}

void Airbrake_Deploy_Init(void)
{
    s_deployState = DEPLOY_IDLE;
    s_stateStartTimeMs = 0;
}

void Airbrake_GoToMax(void)
{
    if (s_deployState != DEPLOY_IDLE && s_deployState != DEPLOY_DONE)
        return;
    Deploy_Print("Moving to max deploy (70 deg).\r\n");
    s_deployState      = DEPLOY_JOG_UP;
    s_stateStartTimeMs = HAL_GetTick();
}

void Airbrake_GoToZero(void)
{
    if (s_deployState != DEPLOY_IDLE && s_deployState != DEPLOY_DONE)
        return;
    Deploy_Print("Moving to 0 deg (home).\r\n");
    s_deployState      = DEPLOY_JOG_DOWN;
    s_stateStartTimeMs = HAL_GetTick();
}

void Airbrake_Start_Sequence(void)
{
    if (s_deployState == DEPLOY_IDLE || s_deployState == DEPLOY_DONE)
    {
        int32_t full_units  = Airbrake_Angle_To_Position_um(FULL_DEPLOY_ANGLE_DEG);
        int32_t pos60_units = Airbrake_Angle_To_Position_um(60.0f);
        int32_t pos40_units = Airbrake_Angle_To_Position_um(40.0f);
        int32_t pos15_units = Airbrake_Angle_To_Position_um(15.0f);

        Deploy_Print("\r\n=== Starting Deploy Sequence ===\r\n");
        Deploy_Print("Assuming current position is 0 deg.\r\n");
        Print_Units_As_mm("Full deploy target : ", full_units);
        Print_Units_As_mm("60 deg target      : ", pos60_units);
        Print_Units_As_mm("40 deg target      : ", pos40_units);
        Print_Units_As_mm("15 deg target      : ", pos15_units);
        Deploy_Print("Returning to 0 deg and locking at end.\r\n\r\n");

        s_deployState = DEPLOY_START_DELAY;
        s_stateStartTimeMs = HAL_GetTick();
    }
}

uint8_t Airbrake_Is_Sequence_Active(void)
{
    return (s_deployState != DEPLOY_IDLE && s_deployState != DEPLOY_DONE);
}

void Airbrake_Deploy_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_stateStartTimeMs;

    const int32_t target_full_units = Airbrake_Angle_To_Position_um(FULL_DEPLOY_ANGLE_DEG);
    const int32_t target_60_units   = Airbrake_Angle_To_Position_um(60.0f);
    const int32_t target_40_units   = Airbrake_Angle_To_Position_um(40.0f);
    const int32_t target_15_units   = Airbrake_Angle_To_Position_um(15.0f);
    const int32_t target_zero_units = 0;

    switch (s_deployState)
    {
        case DEPLOY_IDLE:
        case DEPLOY_DONE:
            break;
        case DEPLOY_START_DELAY:
            ESC_App_StopMotor();
            if (elapsed >= DEPLOY_START_DELAY_MS)
            {
                s_deployState = DEPLOY_MOVE_FULL;
                s_stateStartTimeMs = now;
            }
            break;
        case DEPLOY_MOVE_FULL:
            if (MoveTowardTarget(target_full_units))
            {
                Print_Units_As_mm("Reached full deploy target: ", target_full_units);
                s_deployState = DEPLOY_HOLD_FULL;
                s_stateStartTimeMs = now;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout moving to full deploy. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_HOLD_FULL;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_HOLD_FULL:
            ESC_App_StopMotor();
            if (elapsed >= HOLD_FULL_MS)
            {
                Print_Units_As_mm("Moving to 60 deg target: ", target_60_units);
                s_deployState = DEPLOY_MOVE_60;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_MOVE_60:
            if (MoveTowardTarget(target_60_units))
            {
                Print_Units_As_mm("Reached 60 deg target: ", target_60_units);
                s_deployState = DEPLOY_HOLD_60;
                s_stateStartTimeMs = now;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout moving to 60 deg. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_HOLD_60;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_HOLD_60:
            ESC_App_StopMotor();
            if (elapsed >= HOLD_60_MS)
            {
                Print_Units_As_mm("Moving to 40 deg target: ", target_40_units);
                s_deployState = DEPLOY_MOVE_40;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_MOVE_40:
            if (MoveTowardTarget(target_40_units))
            {
                Print_Units_As_mm("Reached 40 deg target: ", target_40_units);
                s_deployState = DEPLOY_HOLD_40;
                s_stateStartTimeMs = now;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout moving to 40 deg. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_HOLD_40;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_HOLD_40:
            ESC_App_StopMotor();
            if (elapsed >= HOLD_40_MS)
            {
                Print_Units_As_mm("Moving to 15 deg target: ", target_15_units);
                s_deployState = DEPLOY_MOVE_15;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_MOVE_15:
            if (MoveTowardTarget(target_15_units))
            {
                Print_Units_As_mm("Reached 15 deg target: ", target_15_units);
                s_deployState = DEPLOY_HOLD_15;
                s_stateStartTimeMs = now;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout moving to 15 deg. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_HOLD_15;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_HOLD_15:
            ESC_App_StopMotor();
            if (elapsed >= HOLD_15_MS)
            {
                Deploy_Print("Returning to 0 deg / 0.000000 mm\r\n");
                s_deployState = DEPLOY_MOVE_ZERO;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_MOVE_ZERO:
            if (MoveTowardTarget(target_zero_units))
            {
                Deploy_Print("Reached 0 deg / 0.000000 mm. Locking.\r\n");
                s_deployState = DEPLOY_LOCK_ZERO;
                s_stateStartTimeMs = now;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout returning to 0 deg. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_LOCK_ZERO;
                s_stateStartTimeMs = now;
            }
            break;

        case DEPLOY_LOCK_ZERO:
            ESC_App_StopMotor();
            if (elapsed >= LOCK_ZERO_MS)
            {
                Deploy_Print("Deploy sequence complete. Locked at 0 deg.\r\n\r\n");
                s_deployState = DEPLOY_DONE;
            }
            break;

        case DEPLOY_JOG_UP:
            if (MoveTowardTarget(target_full_units))
            {
                Print_Units_As_mm("Reached max deploy: ", target_full_units);
                ESC_App_StopMotor();
                s_deployState = DEPLOY_IDLE;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout jogging up. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_IDLE;
            }
            break;

        case DEPLOY_JOG_DOWN:
            if (MoveTowardTarget(target_zero_units))
            {
                Deploy_Print("Reached 0 deg.\r\n");
                ESC_App_StopMotor();
                s_deployState = DEPLOY_IDLE;
            }
            else if (elapsed >= DEPLOY_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Print_Units_As_mm("Timeout jogging down. Current pos = ",
                                  Encoder_GetPosition_um());
                s_deployState = DEPLOY_IDLE;
            }
            break;

        default:
            ESC_App_StopMotor();
            s_deployState = DEPLOY_DONE;
            break;
    }
}
