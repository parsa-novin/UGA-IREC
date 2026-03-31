#ifndef AIRBRAKE_DEPLOY_H
#define AIRBRAKE_DEPLOY_H

#include <stdint.h>

/* Initialize the airbrake deployment system */
void Airbrake_Deploy_Init(void);

/* Task to be called in main loop */
void Airbrake_Deploy_Task(void);

/* Start the deployment sequence */
void Airbrake_Start_Sequence(void);

/* Check if sequence is active */
uint8_t Airbrake_Is_Sequence_Active(void);

/* Convert angle to position in micrometers */
int32_t Airbrake_Angle_To_Position_um(float angle_deg);

#endif /* AIRBRAKE_DEPLOY_H */
