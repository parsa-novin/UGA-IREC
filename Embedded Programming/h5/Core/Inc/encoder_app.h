#pragma once

#include <stdint.h>

/**
 * @brief  Initialize the encoder. Call once after GPIO is configured.
 */
void    Encoder_App_Init(void);

/**
 * @brief  Handle periodic debug printing from the main loop.
 */
void    Encoder_App_Task(void);

/**
 * @brief  Poll Hall pins once. Intended to be called from TIM4 ISR.
 */
void    Encoder_TimerPollCallback(void);

/**
 * @brief  Returns raw Hall step count.
 */
int32_t Encoder_GetCount(void);

/**
 * @brief  Returns position in micrometres. Positive = forward, negative = reverse.
 */
int32_t Encoder_GetPosition_um(void);

/**
 * @brief  Sets the current physical position as the new zero reference.
 */
void    Encoder_Reset(void);

/**
 * @brief  Edge-specific EXTI hooks used on STM32H5.
 */
void    Encoder_EXTI_Callback(uint16_t GPIO_Pin);
void    Encoder_EXTI_Rising_Callback(uint16_t GPIO_Pin);
void    Encoder_EXTI_Falling_Callback(uint16_t GPIO_Pin);
