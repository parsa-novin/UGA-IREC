#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include <stdint.h>

/* Initialize ESC telemetry reception */
void ESC_Telem_Init(void);

/* Task to be called periodically */
void ESC_Telem_Task(void);

/* UART RX complete callback - call from HAL_UART_RxCpltCallback */
void ESC_Telem_RxCpltCallback(void);

/* Getters for telemetry data */
uint8_t  ESC_Telem_GetTemp_C(void);
uint32_t ESC_Telem_GetVoltage_mV(void);
uint32_t ESC_Telem_GetCurrent_mA(void);
uint32_t ESC_Telem_GetConsumption_mAh(void);
uint32_t ESC_Telem_GetERPM(void);
uint8_t  ESC_Telem_IsValid(void);
uint32_t ESC_Telem_GetCRCErrors(void);

#endif /* ESC_TELEM_H */
