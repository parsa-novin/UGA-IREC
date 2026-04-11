/**
 * @file    flight_trigger.c
 * @brief   Automatic airbrake deployment trigger.
 *
 * Monitors the IMU16 acceleration magnitude from the upstream sensor packet
 * and implements a two-stage arm/fire logic:
 *
 *   IDLE  → ARMED : magnitude > FLIGHT_TRIGGER_ARM_G  (5 G)
 *   ARMED → FIRED : magnitude < FLIGHT_TRIGGER_FIRE_G (3 G)
 *
 * When FIRED, SD_Logger_Start() and Airbrake_Start_Sequence() are called
 * exactly once.  The trigger then remains in the FIRED state for the rest
 * of the power cycle.
 */

#include "flight_trigger.h"
#include "esc_app.h"
#include "airbrake_deploy.h"
#include "encoder_homing.h"
#include "sd_logger.h"
#include "usart.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/* =========================================================================
 * Module state
 * ========================================================================= */

static volatile FlightTriggerState_t s_state = FLIGHT_TRIGGER_IDLE;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static void Trigger_Print(const char *fmt, ...)
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

static float Accel_Magnitude(float ax, float ay, float az)
{
    return sqrtf(ax * ax + ay * ay + az * az);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void Flight_Trigger_Init(void)
{
    s_state = FLIGHT_TRIGGER_IDLE;
}

void Flight_Trigger_Task(void)
{
    /* Once fired the trigger is locked out for the rest of the power cycle. */
    if (s_state == FLIGHT_TRIGGER_FIRED)
        return;

    SensorPacket_t pkt;
    if (!ESC_App_GetLatestSensorPacket(&pkt))
        return; /* no upstream data yet */

    float mag_g = Accel_Magnitude(pkt.imu16_ax, pkt.imu16_ay, pkt.imu16_az);

    switch (s_state)
    {
        case FLIGHT_TRIGGER_IDLE:
            if (mag_g > FLIGHT_TRIGGER_ARM_G)
            {
                s_state = FLIGHT_TRIGGER_ARMED;
                Trigger_Print("[TRIGGER] ARMED — IMU16 |a| = %.2f g (threshold %.1f g)\r\n",
                              (double)mag_g, (double)FLIGHT_TRIGGER_ARM_G);
            }
            break;

        case FLIGHT_TRIGGER_ARMED:
            if (mag_g < FLIGHT_TRIGGER_FIRE_G)
            {
                /* Only fire if no conflicting operation is running */
                if (!Encoder_Homing_Is_Active() && !Airbrake_Is_Sequence_Active())
                {
                    s_state = FLIGHT_TRIGGER_FIRED;
                    Trigger_Print("[TRIGGER] FIRED  — IMU16 |a| = %.2f g (threshold %.1f g). Starting deploy.\r\n",
                                  (double)mag_g, (double)FLIGHT_TRIGGER_FIRE_G);
                    SD_Logger_Start();
                    Airbrake_Start_Sequence();
                }
            }
            break;

        default:
            break;
    }
}

FlightTriggerState_t Flight_Trigger_GetState(void)
{
    return s_state;
}
