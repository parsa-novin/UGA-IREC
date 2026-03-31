/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "usart.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* UART4 init function */
void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) { Error_Handler(); }
}

void MX_UART5_Init(void)
{
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) { Error_Handler(); }
}

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) { Error_Handler(); }
}

void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) { Error_Handler(); }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) { Error_Handler(); }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  if(uartHandle->Instance==UART4)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }

    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==UART5)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }

    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_UART5;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  }
  else if(uartHandle->Instance==USART1)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if(uartHandle->Instance==USART2)
  {
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{
  if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  }
  else if(uartHandle->Instance==UART5)
  {
    __HAL_RCC_UART5_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15|GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  }
  else if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  }
  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */
