/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* none */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t prom[8];              /* PROM words 0..7 */
  uint32_t D1;                   /* raw pressure ADC */
  uint32_t D2;                   /* raw temperature ADC */

  int32_t temperature_centiC;    /* 0.01 degC */
  int32_t pressure_centi_mbar;   /* 0.01 mbar */

  float temperature_degC;
  float pressure_mbar;

  uint8_t prom_loaded;
  uint8_t data_valid;
} MS5607_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MS5607_CS_PORT              GPIOB
#define MS5607_CS_PIN               GPIO_PIN_0

#define MS5607_CMD_RESET            0x1E
#define MS5607_CMD_ADC_READ         0x00
#define MS5607_CMD_CONV_D1_4096     0x48
#define MS5607_CMD_CONV_D2_4096     0x58

#define MS5607_PROM_ADDR_0          0xA0
#define MS5607_PROM_ADDR_1          0xA2
#define MS5607_PROM_ADDR_2          0xA4
#define MS5607_PROM_ADDR_3          0xA6
#define MS5607_PROM_ADDR_4          0xA8
#define MS5607_PROM_ADDR_5          0xAA
#define MS5607_PROM_ADDR_6          0xAC
#define MS5607_PROM_ADDR_7          0xAE

#define MS5607_RESET_DELAY_MS       3U
#define MS5607_CONV_DELAY_MS        10U
#define MS5607_SPI_TIMEOUT_MS       100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* none */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static MS5607_Data_t ms5607;

/* Watch these in ST-LINK Live Expressions */
volatile uint8_t g_rx0 = 0;
volatile uint8_t g_rx1 = 0;
volatile uint8_t g_rx2 = 0;
volatile uint8_t g_rx3 = 0;
volatile uint16_t g_prom[8] = {0};
volatile uint32_t g_D1 = 0;
volatile uint32_t g_D2 = 0;
volatile int32_t  g_temp_centiC = 0;
volatile int32_t  g_press_centi_mbar = 0;
volatile float    g_temp_degC = 0.0f;
volatile float    g_press_mbar = 0.0f;
volatile uint8_t  g_prom_loaded = 0;
volatile uint8_t  g_data_valid = 0;
volatile uint32_t g_loop_count = 0;
volatile uint32_t g_last_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MS5607_CS_Low(void);
static void MS5607_CS_High(void);
static HAL_StatusTypeDef MS5607_WriteCommand(uint8_t cmd);
static HAL_StatusTypeDef MS5607_Reset(void);
static HAL_StatusTypeDef MS5607_ReadPROMWord(uint8_t prom_cmd, uint16_t *value);
static HAL_StatusTypeDef MS5607_ReadAllPROM(MS5607_Data_t *dev);
static HAL_StatusTypeDef MS5607_ReadADC(uint32_t *value);
static HAL_StatusTypeDef MS5607_StartConversionAndRead(uint8_t conv_cmd, uint32_t *value);
static HAL_StatusTypeDef MS5607_ReadRaw(MS5607_Data_t *dev);
static void MS5607_Compensate(MS5607_Data_t *dev);
static void MS5607_UpdateDebugGlobals(const MS5607_Data_t *dev);
static void Application_Init(void);
static void Application_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MS5607_CS_Low(void)
{
  HAL_GPIO_WritePin(MS5607_CS_PORT, MS5607_CS_PIN, GPIO_PIN_RESET);
}

static void MS5607_CS_High(void)
{
  HAL_GPIO_WritePin(MS5607_CS_PORT, MS5607_CS_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef MS5607_WriteCommand(uint8_t cmd)
{
  HAL_StatusTypeDef status;

  MS5607_CS_Low();
  status = HAL_SPI_Transmit(&hspi1, &cmd, 1, MS5607_SPI_TIMEOUT_MS);
  MS5607_CS_High();

  return status;
}

static HAL_StatusTypeDef MS5607_Reset(void)
{
  HAL_StatusTypeDef status;

  status = MS5607_WriteCommand(MS5607_CMD_RESET);
  HAL_Delay(MS5607_RESET_DELAY_MS);

  return status;
}

static HAL_StatusTypeDef MS5607_ReadPROMWord(uint8_t prom_cmd, uint16_t *value)
{
  HAL_StatusTypeDef status;
  uint8_t tx[3] = {prom_cmd, 0x00, 0x00};
  uint8_t rx[3] = {0x00, 0x00, 0x00};

  if (value == NULL)
  {
    return HAL_ERROR;
  }

  MS5607_CS_Low();
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, MS5607_SPI_TIMEOUT_MS);
  MS5607_CS_High();

  if (status != HAL_OK)
  {
    return status;
  }

  *value = ((uint16_t)rx[1] << 8) | rx[2];
  return HAL_OK;
}

static HAL_StatusTypeDef MS5607_ReadAllPROM(MS5607_Data_t *dev)
{
  static const uint8_t prom_cmds[8] =
  {
    MS5607_PROM_ADDR_0,
    MS5607_PROM_ADDR_1,
    MS5607_PROM_ADDR_2,
    MS5607_PROM_ADDR_3,
    MS5607_PROM_ADDR_4,
    MS5607_PROM_ADDR_5,
    MS5607_PROM_ADDR_6,
    MS5607_PROM_ADDR_7
  };

  uint8_t i;
  HAL_StatusTypeDef status;

  if (dev == NULL)
  {
    return HAL_ERROR;
  }

  for (i = 0; i < 8; i++)
  {
    status = MS5607_ReadPROMWord(prom_cmds[i], &dev->prom[i]);
    if (status != HAL_OK)
    {
      dev->prom_loaded = 0;
      return status;
    }
  }

  dev->prom_loaded = 1;
  return HAL_OK;
}

static HAL_StatusTypeDef MS5607_ReadADC(uint32_t *value)
{
  HAL_StatusTypeDef status;
  uint8_t tx[4] = {MS5607_CMD_ADC_READ, 0x00, 0x00, 0x00};
  uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

  if (value == NULL)
  {
    return HAL_ERROR;
  }

  MS5607_CS_Low();
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, MS5607_SPI_TIMEOUT_MS);
  MS5607_CS_High();

  if (status != HAL_OK)
  {
    return status;
  }

  *value = ((uint32_t)rx[1] << 16) |
           ((uint32_t)rx[2] << 8)  |
           ((uint32_t)rx[3]);

  return HAL_OK;
}

static HAL_StatusTypeDef MS5607_StartConversionAndRead(uint8_t conv_cmd, uint32_t *value)
{
  HAL_StatusTypeDef status;

  status = MS5607_WriteCommand(conv_cmd);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(MS5607_CONV_DELAY_MS);

  status = MS5607_ReadADC(value);
  return status;
}

static HAL_StatusTypeDef MS5607_ReadRaw(MS5607_Data_t *dev)
{
  HAL_StatusTypeDef status;

  if (dev == NULL)
  {
    return HAL_ERROR;
  }

  status = MS5607_StartConversionAndRead(MS5607_CMD_CONV_D1_4096, &dev->D1);
  if (status != HAL_OK)
  {
    dev->data_valid = 0;
    return status;
  }

  status = MS5607_StartConversionAndRead(MS5607_CMD_CONV_D2_4096, &dev->D2);
  if (status != HAL_OK)
  {
    dev->data_valid = 0;
    return status;
  }

  if ((dev->D1 == 0U) || (dev->D2 == 0U))
  {
    dev->data_valid = 0;
    return HAL_ERROR;
  }

  dev->data_valid = 1;
  return HAL_OK;
}

static void MS5607_Compensate(MS5607_Data_t *dev)
{
  int32_t dT;
  int32_t TEMP;
  int32_t T2 = 0;
  int32_t temp_minus_2000;
  int32_t temp_plus_1500;

  int64_t OFF;
  int64_t SENS;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;
  int64_t pressure;

  if ((dev == NULL) || (dev->prom_loaded == 0U) || (dev->data_valid == 0U))
  {
    return;
  }

  /*
   * Datasheet coefficient mapping:
   * prom[1] = C1
   * prom[2] = C2
   * prom[3] = C3
   * prom[4] = C4
   * prom[5] = C5
   * prom[6] = C6
   */

  dT   = (int32_t)dev->D2 - ((int32_t)dev->prom[5] << 8);
  TEMP = 2000 + (int32_t)(((int64_t)dT * (int64_t)dev->prom[6]) >> 23);

  OFF  = ((int64_t)dev->prom[2] << 17) + (((int64_t)dev->prom[4] * (int64_t)dT) >> 6);
  SENS = ((int64_t)dev->prom[1] << 16) + (((int64_t)dev->prom[3] * (int64_t)dT) >> 7);

  if (TEMP < 2000)
  {
    T2 = (int32_t)(((int64_t)dT * (int64_t)dT) >> 31);

    temp_minus_2000 = TEMP - 2000;
    OFF2  = (61LL * (int64_t)temp_minus_2000 * (int64_t)temp_minus_2000) >> 4;
    SENS2 = 2LL * (int64_t)temp_minus_2000 * (int64_t)temp_minus_2000;

    if (TEMP < -1500)
    {
      temp_plus_1500 = TEMP + 1500;
      OFF2  += 15LL * (int64_t)temp_plus_1500 * (int64_t)temp_plus_1500;
      SENS2 += 8LL  * (int64_t)temp_plus_1500 * (int64_t)temp_plus_1500;
    }
  }

  TEMP -= T2;
  OFF  -= OFF2;
  SENS -= SENS2;

  pressure = ((((int64_t)dev->D1 * SENS) >> 21) - OFF) >> 15;

  dev->temperature_centiC  = TEMP;
  dev->pressure_centi_mbar = (int32_t)pressure;
  dev->temperature_degC    = ((float)TEMP) / 100.0f;
  dev->pressure_mbar       = ((float)pressure) / 100.0f;
}

static void MS5607_UpdateDebugGlobals(const MS5607_Data_t *dev)
{
  uint8_t i;

  if (dev == NULL)
  {
    return;
  }

  for (i = 0; i < 8; i++)
  {
    g_prom[i] = dev->prom[i];
  }

  g_D1 = dev->D1;
  g_D2 = dev->D2;
  g_temp_centiC = dev->temperature_centiC;
  g_press_centi_mbar = dev->pressure_centi_mbar;
  g_temp_degC = dev->temperature_degC;
  g_press_mbar = dev->pressure_mbar;
  g_prom_loaded = dev->prom_loaded;
  g_data_valid = dev->data_valid;
}

static void Application_Init(void)
{
  uint8_t i;

  for (i = 0; i < 8; i++)
  {
    ms5607.prom[i] = 0;
  }

  ms5607.D1 = 0;
  ms5607.D2 = 0;
  ms5607.temperature_centiC = 0;
  ms5607.pressure_centi_mbar = 0;
  ms5607.temperature_degC = 0.0f;
  ms5607.pressure_mbar = 0.0f;
  ms5607.prom_loaded = 0;
  ms5607.data_valid = 0;

  MS5607_CS_High();
  HAL_Delay(20);

  g_last_status = (uint32_t)MS5607_Reset();
  if (g_last_status != HAL_OK)
  {
    MS5607_UpdateDebugGlobals(&ms5607);
    return;
  }

  g_last_status = (uint32_t)MS5607_ReadAllPROM(&ms5607);
  MS5607_UpdateDebugGlobals(&ms5607);
}

static void Application_Task(void)
{
  HAL_StatusTypeDef status;

  status = MS5607_ReadRaw(&ms5607);
  g_last_status = (uint32_t)status;

  if (status == HAL_OK)
  {
    MS5607_Compensate(&ms5607);
  }

  MS5607_UpdateDebugGlobals(&ms5607);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Application_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){
    Application_Task();
    g_loop_count++;
    HAL_Delay(50);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
