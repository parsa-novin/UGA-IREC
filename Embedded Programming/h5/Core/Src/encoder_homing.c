#include "encoder_homing.h"
#include "encoder_app.h"
#include "esc_telem.h"
#include "main.h"
#include "usart.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* Motor-control helpers exported from esc_app.c */
extern void ESC_App_SetPercent(int32_t pct);
extern void ESC_App_StopMotor(void);

/* Homing states */
typedef enum {
    HOME_IDLE,
    HOME_MOVE_FORWARD,
    HOME_WAIT_FORWARD,
    HOME_MOVE_BACKWARD,
    HOME_WAIT_BACKWARD,
    HOME_MOVE_TO_CENTER,
    HOME_DONE
} HomingState_t;

/* Homing parameters */
#define HOMING_CURRENT_THRESHOLD_MA  2000U
#define HOMING_SPEED_PERCENT         30
#define HOMING_WAIT_TIME_MS          500U
#define HOMING_SAMPLE_TIME_MS        50U
#define HOMING_MOVE_TIMEOUT_MS       3000U
#define HOMING_CENTER_TOL_UM         30000L

static volatile HomingState_t s_homingState = HOME_IDLE;
static volatile int32_t s_minPosition_um = 0;
static volatile int32_t s_maxPosition_um = 0;
static volatile uint32_t s_stateStartTime = 0;

static void Homing_Print(const char *fmt, ...)
{
    char buf[128];
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

void Encoder_Homing_Init(void)
{
    s_homingState = HOME_IDLE;
    s_minPosition_um = 0;
    s_maxPosition_um = 0;
}

void Encoder_Homing_Start(void)
{
    if (s_homingState == HOME_IDLE || s_homingState == HOME_DONE)
    {
        Homing_Print("\r\n=== Starting Homing Sequence ===\r\n");
        Encoder_Reset();
        s_homingState = HOME_MOVE_FORWARD;
        s_stateStartTime = HAL_GetTick();
    }
}

uint8_t Encoder_Homing_Is_Active(void)
{
    return (s_homingState != HOME_IDLE && s_homingState != HOME_DONE);
}

int32_t Encoder_Homing_Get_Min_um(void)
{
    return s_minPosition_um;
}

int32_t Encoder_Homing_Get_Max_um(void)
{
    return s_maxPosition_um;
}

void Encoder_Homing_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - s_stateStartTime;
    uint32_t current_mA = ESC_Telem_GetCurrent_mA();
    int32_t position_um = Encoder_GetPosition_um();

    switch (s_homingState)
    {
        case HOME_IDLE:
        case HOME_DONE:
            break;

        case HOME_MOVE_FORWARD:
            ESC_App_SetPercent(HOMING_SPEED_PERCENT);

            if (elapsed > HOMING_SAMPLE_TIME_MS && current_mA > HOMING_CURRENT_THRESHOLD_MA)
            {
                s_maxPosition_um = position_um;
                ESC_App_StopMotor();
                Homing_Print("Forward limit hit at %ld um (current: %lu mA)\r\n",
                             (long)s_maxPosition_um, (unsigned long)current_mA);

                s_homingState = HOME_WAIT_FORWARD;
                s_stateStartTime = now;
            }
            else if (elapsed > HOMING_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                s_maxPosition_um = position_um;
                Homing_Print("Forward move timed out at %ld um (current: %lu mA)\r\n",
                             (long)s_maxPosition_um, (unsigned long)current_mA);

                s_homingState = HOME_WAIT_FORWARD;
                s_stateStartTime = now;
            }
            break;

        case HOME_WAIT_FORWARD:
            ESC_App_StopMotor();
            if (elapsed >= HOMING_WAIT_TIME_MS)
            {
                Homing_Print("Moving to backward limit...\r\n");
                s_homingState = HOME_MOVE_BACKWARD;
                s_stateStartTime = now;
            }
            break;

        case HOME_MOVE_BACKWARD:
            ESC_App_SetPercent(-HOMING_SPEED_PERCENT);

            if (elapsed > HOMING_SAMPLE_TIME_MS && current_mA > HOMING_CURRENT_THRESHOLD_MA)
            {
                s_minPosition_um = position_um;
                ESC_App_StopMotor();
                Homing_Print("Backward limit hit at %ld um (current: %lu mA)\r\n",
                             (long)s_minPosition_um, (unsigned long)current_mA);

                s_homingState = HOME_WAIT_BACKWARD;
                s_stateStartTime = now;
            }
            else if (elapsed > HOMING_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                s_minPosition_um = position_um;
                Homing_Print("Backward move timed out at %ld um (current: %lu mA)\r\n",
                             (long)s_minPosition_um, (unsigned long)current_mA);

                s_homingState = HOME_WAIT_BACKWARD;
                s_stateStartTime = now;
            }
            break;

        case HOME_WAIT_BACKWARD:
            ESC_App_StopMotor();
            if (elapsed >= HOMING_WAIT_TIME_MS)
            {
                int32_t center_um = (s_minPosition_um + s_maxPosition_um) / 2;

                Homing_Print("Homing span captured.\r\n");
                Homing_Print("  Min: %ld um\r\n", (long)s_minPosition_um);
                Homing_Print("  Max: %ld um\r\n", (long)s_maxPosition_um);
                Homing_Print("  Range: %ld um\r\n", (long)(s_maxPosition_um - s_minPosition_um));
                Homing_Print("  Center: %ld um\r\n", (long)center_um);
                Homing_Print("Moving to center...\r\n");

                s_homingState = HOME_MOVE_TO_CENTER;
                s_stateStartTime = now;
            }
            break;

        case HOME_MOVE_TO_CENTER:
        {
            int32_t center_um = (s_minPosition_um + s_maxPosition_um) / 2;
            int32_t err_um = center_um - position_um;

            if (abs_i32(err_um) <= HOMING_CENTER_TOL_UM)
            {
                ESC_App_StopMotor();
                Homing_Print("Homing sequence complete.\r\n\r\n");
                s_homingState = HOME_DONE;
            }
            else
            {
                ESC_App_SetPercent((err_um > 0) ? HOMING_SPEED_PERCENT : -HOMING_SPEED_PERCENT);

                if (elapsed > HOMING_MOVE_TIMEOUT_MS)
                {
                    ESC_App_StopMotor();
                    Homing_Print("Center move timed out at %ld um.\r\n\r\n", (long)position_um);
                    s_homingState = HOME_DONE;
                }
            }
            break;
        }
    }
}
