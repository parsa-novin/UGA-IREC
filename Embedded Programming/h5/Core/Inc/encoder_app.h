#pragma once

#include <stdint.h>

/**
 * @brief  Initialize the encoder. Call once after GPIO is configured.
 */
void    Encoder_App_Init(void);

/**
 * @brief  Poll Hall pins and handle debug printing.
 *         Call as fast as possible from the main loop — NO delays between calls.
 */
void    Encoder_App_Task(void);

/**
 * @brief  Returns raw Hall step count.
 */
int32_t Encoder_GetCount(void);

/**
 * @brief  Returns position in micrometres. Positive = forward, negative = reverse.
 */
int32_t Encoder_GetPosition_um(void);

/**
 * @brief  Resets count and position to zero.
 */
void    Encoder_Reset(void);

/**
 * @brief  Stub — kept so stm32h5xx_it.c compiles without changes.
 *         Polling handles everything now; EXTI is not used.
 */
void    Encoder_EXTI_Callback(uint16_t GPIO_Pin);
