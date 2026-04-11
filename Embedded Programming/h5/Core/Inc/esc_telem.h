#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include <stdint.h>

/* Initialize ESC telemetry reception */
void ESC_Telem_Init(void);

/* Task to be called periodically */
void ESC_Telem_Task(void);

/* UART5 IRQ handler — call directly from UART5_IRQHandler */
void ESC_Telem_Uart5IrqHandler(void);

/* Getters for telemetry data */
uint8_t  ESC_Telem_GetTemp_C(void);
uint32_t ESC_Telem_GetVoltage_mV(void);
uint32_t ESC_Telem_GetCurrent_mA(void);
uint32_t ESC_Telem_GetConsumption_mAh(void);
uint32_t ESC_Telem_GetERPM(void);
uint8_t  ESC_Telem_IsValid(void);
uint32_t ESC_Telem_GetCRCErrors(void);
uint32_t ESC_Telem_GetPacketsOk(void);

/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */

#endif /* ESC_TELEM_H */
