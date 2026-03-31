/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h5xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32h5xx_it.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
#include "encoder_app.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void NMI_Handler(void)
{
  while (1) {}
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI6_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

void EXTI7_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}

void UART5_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart5);
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    Encoder_EXTI_Callback(GPIO_Pin);
}
/* USER CODE END 1 */
