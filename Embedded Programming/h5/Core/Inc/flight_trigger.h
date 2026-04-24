#ifndef FLIGHT_TRIGGER_H
#define FLIGHT_TRIGGER_H

#include <stdint.h>

/**
 * @brief  Two-stage automatic deployment trigger based on IMU16 acceleration
 *         magnitude.
 *
 * Stage 1 — ARM:   magnitude exceeds FLIGHT_TRIGGER_ARM_G (5 G).
 *                  Indicates motor burn is underway.
 *
 * Stage 2 — FIRE:  after armed, magnitude drops below FLIGHT_TRIGGER_FIRE_G
 *                  (3 G).  Indicates burnout; starts the SD logger and the
 *                  airbrake deployment sequence.
 *
 * Once fired the trigger locks out so the sequence is only started once per
 * power cycle.  Ground commands (H, 67, UP, DOWN) are unaffected.
 */

#define FLIGHT_TRIGGER_ARM_G   5.0f   /* arm threshold  — motor burning   */
#define FLIGHT_TRIGGER_FIRE_G  3.0f   /* fire threshold — burnout detected */

typedef enum
{
    FLIGHT_TRIGGER_IDLE  = 0, /* waiting to arm                */
    FLIGHT_TRIGGER_ARMED,     /* above arm threshold, watching */
    FLIGHT_TRIGGER_FIRED,     /* sequence started, locked out  */
} FlightTriggerState_t;

void                 Flight_Trigger_Init(void);
void                 Flight_Trigger_Task(void);
FlightTriggerState_t Flight_Trigger_GetState(void);

#endif /* FLIGHT_TRIGGER_H */
