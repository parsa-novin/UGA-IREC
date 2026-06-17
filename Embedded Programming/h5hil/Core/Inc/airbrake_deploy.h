#ifndef AIRBRAKE_DEPLOY_H
#define AIRBRAKE_DEPLOY_H

#include <stdint.h>

/* Initialize the airbrake deployment system */
void Airbrake_Deploy_Init(void);

/* Task to be called in main loop */
void Airbrake_Deploy_Task(void);

/* Start the fixed test sequence (ground testing only) */
void Airbrake_Start_Sequence(void);

/* Enter continuous control mode — called by airbrake_control at burnout */
void Airbrake_StartControl(void);

/* Set the angle the control loop should track [0..70 deg].
 * Has effect only while in DEPLOY_CONTROL state. */
void Airbrake_SetTargetAngle(float angle_deg);

/* Check if any deployment activity is active */
uint8_t Airbrake_Is_Sequence_Active(void);

/* Convert angle to position in micrometers */
int32_t Airbrake_Angle_To_Position_um(float angle_deg);

/* Go directly to max deployment (70 deg) or zero, without the full sequence */
void Airbrake_GoToMax(void);
void Airbrake_GoToZero(void);

#endif /* AIRBRAKE_DEPLOY_H */
