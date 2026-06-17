#ifndef ENCODER_HOMING_H
#define ENCODER_HOMING_H

#include <stdint.h>

/* Initialize homing system */
void Encoder_Homing_Init(void);

/* Start homing sequence */
void Encoder_Homing_Start(void);

/* Task to be called in main loop */
void Encoder_Homing_Task(void);

/* Check if homing is active */
uint8_t Encoder_Homing_Is_Active(void);

/* Get min position (um) */
int32_t Encoder_Homing_Get_Min_um(void);

/* Get max position (um) */
int32_t Encoder_Homing_Get_Max_um(void);

#endif /* ENCODER_HOMING_H */
