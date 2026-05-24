/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* ADV7280 INTR: active-low interrupt output from the video decoder (PA8) */
#define ADV_INTR_Pin        GPIO_PIN_8
#define ADV_INTR_GPIO_Port  GPIOA

/* ADV7280 PWRDWN: active-low power-down control (PC14).
 * Drive HIGH for normal operation; pull LOW only to power-down the chip. */
#define ADV_PWRDWN_Pin        GPIO_PIN_14
#define ADV_PWRDWN_GPIO_Port  GPIOC

/* External camera multiplexer select (PC3).
 * LOW = camera 0 selected (default); HIGH = camera 1 selected. */
#define CAM_MUX_SEL_Pin        GPIO_PIN_3
#define CAM_MUX_SEL_GPIO_Port  GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
