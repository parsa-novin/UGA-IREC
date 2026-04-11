/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — RunCam SPI command controller
  *
  * SPI2 slave interface (8-bit frames, mode 0, MSB first):
  *   'R' (0x52) — Start recording
  *   'S' (0x53) — Stop  recording
  *   'O' (0x4F) — Power on  (simulate power button)
  *   'Z' (0x5A) — Power off (simulate power button)
  *   'C' (0x43) — Read current from INA219; reply is a 4-byte IEEE-754
  *                float (milliamps, big-endian) sent in the next SPI
  *                transaction initiated by the master.
  *
  * Two-phase 'C' response protocol
  * ---------------------------------
  * SPI is full-duplex but the STM32 is a slave and cannot initiate transfers.
  * When 'C' is received the STM32 reads the INA219 over I2C, converts the
  * result to a 4-byte float (mA, big-endian), and pre-loads it into the SPI
  * TX buffer via HAL_SPI_Transmit_IT.  The master must then clock exactly 4
  * bytes with NSS asserted to receive the value.  The master should allow at
  * least 2 ms between sending 'C' and starting the read transaction to give
  * the STM32 time to complete the I2C read (~1 ms at 400 kHz).
  *
  * INA219 wiring
  * -------------
  * I2C1 (PB6 = SCL, PB7 = SDA), address 0x45 (A1=VS, A0=GND).
  * See ina219.h for shunt resistor and max-current configuration.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "runcam_device.h"
#include "ina219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_CMD_START_RECORDING  ((uint8_t)'R')   /* 0x52 */
#define SPI_CMD_STOP_RECORDING   ((uint8_t)'S')   /* 0x53 */
#define SPI_CMD_POWER_ON         ((uint8_t)'O')   /* 0x4F */
#define SPI_CMD_POWER_OFF        ((uint8_t)'Z')   /* 0x5A */
#define SPI_CMD_READ_CURRENT     ((uint8_t)'C')   /* 0x43 */

/** Milliseconds to allow the camera UART to become ready after power-up. */
#define RUNCAM_BOOT_DELAY_MS     (2000U)

/** Attempts to call RunCam_GetDeviceInfo before declaring the camera absent. */
#define RUNCAM_INIT_RETRIES      (5U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi2;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
static RunCam_HandleTypeDef hrc;
static INA219_HandleTypeDef hina;

/*
 * Single-byte receive buffer for SPI commands.
 * volatile: prevents the compiler optimising away the read in the main loop.
 * File-scope: lifetime extends beyond the HAL callback that writes it.
 */
static volatile uint8_t spi_rx_byte = 0U;

/*
 * Flag written by HAL_SPI_RxCpltCallback (ISR context) and cleared by the
 * main loop (thread context).  uint8_t stores/loads are atomic on Cortex-M,
 * so no critical section is required for this single-producer/single-consumer
 * pattern.
 */
static volatile uint8_t spi_cmd_pending = 0U;

/*
 * Four-byte TX buffer holding the most recent current reading as an
 * IEEE-754 single-precision float in big-endian byte order.
 *
 * Written by the main loop (RunCam_DispatchSpiCommand), read by the SPI
 * peripheral when HAL_SPI_Transmit_IT fires.  Declared at file scope so its
 * lifetime covers the entire async transmit window.
 *
 * A union is used to reinterpret the float bit pattern as bytes without
 * violating strict aliasing rules.
 */
static union
{
    float   f;
    uint8_t b[4];
} spi_current_tx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
static RunCam_StatusTypeDef RunCam_Init(void);
static void SPI_StartReceive(void);
static void RunCam_DispatchSpiCommand(uint8_t cmd);
static void SPI_SendCurrentReading(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Initialise the RunCam handle and confirm the camera is reachable
  *         and in video mode.
  *
  * Blocks for up to (RUNCAM_BOOT_DELAY_MS + RUNCAM_INIT_RETRIES x 200 ms).
  * If the camera is not present at boot the function returns an error code;
  * the caller proceeds and the 'O' SPI command can retry later.
  *
  * @retval RUNCAM_OK on success, error code otherwise.
  */
static RunCam_StatusTypeDef RunCam_Init(void)
{
    uint8_t              retries = RUNCAM_INIT_RETRIES;
    RunCam_StatusTypeDef status;

    RunCam_AttachUart(&hrc, &huart4, 300U);

    /* Allow the camera UART to settle after power-up */
    HAL_Delay(RUNCAM_BOOT_DELAY_MS);

    /* Probe until device info is returned or retries exhausted */
    do
    {
        status = RunCam_GetDeviceInfo(&hrc);
        if (status == RUNCAM_OK)
        {
            break;
        }
        HAL_Delay(200U);
    }
    while (retries-- > 0U);

    if (status != RUNCAM_OK)
    {
        return status;
    }

    /*
     * Defensive stop: if a prior reset left the camera mid-recording this
     * cleanly closes that clip before we do anything else.  Return value is
     * intentionally ignored — the camera may not be recording.
     */
    (void)RunCam_StopRecording(&hrc);
    HAL_Delay(200U);

    /* Guarantee video mode before accepting SPI commands */
    return RunCam_EnsureVideoMode(&hrc);
}

/**
  * @brief  Re-arm the non-blocking SPI receive for the next command byte.
  *
  * Must be called once before entering the main loop and once more at the
  * end of every HAL_SPI_RxCpltCallback so consecutive commands are not lost.
  */
static void SPI_StartReceive(void)
{
    /*
     * HAL_SPI_Receive_IT takes a plain uint8_t *; cast away volatile here.
     * Correctness is ensured by spi_cmd_pending: the buffer is only read in
     * the main loop after the flag is set, and only re-armed after it is
     * cleared.
     */
    (void)HAL_SPI_Receive_IT(&hspi2, (uint8_t *)&spi_rx_byte, 1U);
}

/**
  * @brief  Read the INA219 current register over I2C and pre-load the result
  *         into the SPI TX buffer as a big-endian IEEE-754 float (mA).
  *
  * The SPI master must initiate a separate 4-byte read transaction after
  * sending 'C' to collect the response.  Allow ~2 ms between the 'C' byte
  * and the follow-up read to ensure the I2C transaction has completed.
  *
  * If the INA219 read fails, 0.0f (four zero bytes) is transmitted so the
  * master always receives exactly 4 bytes with a predictable failure value.
  */
static void SPI_SendCurrentReading(void)
{
    float current_mA = 0.0f;

    /* Read current — failure leaves current_mA = 0.0f */
    (void)INA219_ReadCurrent_mA(&hina, &current_mA);

    /*
     * Store as big-endian so the master can read MSB first without needing
     * to byte-swap on little-endian hosts.
     *
     * The union reinterprets the float bit pattern without strict-aliasing UB.
     */
    spi_current_tx.f = current_mA;

    /* ARM the SPI TX.  HAL will clock out the 4 bytes on the next master
     * transaction.  Return value ignored — if this fails the master will
     * receive 0xFF bytes (MISO line idle high), which is distinct from a
     * valid float and can be detected as an error on the master side.      */
    (void)HAL_SPI_Transmit_IT(&hspi2, spi_current_tx.b, sizeof(spi_current_tx.b));
}

/**
  * @brief  Decode and execute one SPI command byte.
  *
  * RunCam API return values are cast to void — there is no channel to report
  * errors back to the SPI master in this design, except for 'C' which
  * transmits its result in a follow-up SPI transaction.
  *
  * @param  cmd  Raw byte received over SPI.
  */
static void RunCam_DispatchSpiCommand(uint8_t cmd)
{
    switch (cmd)
    {
        case SPI_CMD_START_RECORDING:
            (void)RunCam_StartRecording(&hrc);
            break;

        case SPI_CMD_STOP_RECORDING:
            (void)RunCam_StopRecording(&hrc);
            break;

        case SPI_CMD_POWER_ON:
            /*
             * Simulate the physical power button to turn the camera on, then
             * wait for boot and re-run init so the handle reflects the fresh
             * feature set reported after power-up.
             */
            (void)RunCam_PowerButton(&hrc);
            HAL_Delay(RUNCAM_BOOT_DELAY_MS);
            (void)RunCam_Init();
            break;

        case SPI_CMD_POWER_OFF:
            /*
             * Stop any active recording first so the SD-card file is not
             * corrupted, then simulate the power button to shut down.
             */
            (void)RunCam_StopRecording(&hrc);
            HAL_Delay(500U);
            (void)RunCam_PowerButton(&hrc);
            break;

        case SPI_CMD_READ_CURRENT:
            /*
             * Read INA219 over I2C and pre-load the 4-byte float result into
             * the SPI TX buffer.  The master must clock 4 more bytes to read
             * the value — see file header for the two-phase protocol.
             */
            SPI_SendCurrentReading();
            break;

        default:
            /* Unknown / noise byte — silently ignore */
            break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration -------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration -------------------------------------------------------*/

    /* Reset of all peripherals, initialises the Flash interface and SysTick */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialise all configured peripherals */
    MX_GPIO_Init();
    MX_UART4_Init();
    MX_SPI2_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */

    /*
     * Initialise the INA219 current monitor.  If this fails (device absent
     * or I2C bus fault) we proceed anyway; 'C' commands will return 0.0f
     * until the device becomes available.
     */
    (void)INA219_Init(&hina, &hi2c1);

    /*
     * Bring the RunCam up.  If the camera is absent at boot (e.g. powered
     * separately) we carry on — the 'O' SPI command will retry init.
     */
    (void)RunCam_Init();

    /* Arm the first SPI byte receive before entering the loop */
    SPI_StartReceive();

    /* USER CODE END 2 */

    /* Infinite loop -----------------------------------------------------------*/
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        if (spi_cmd_pending != 0U)
        {
            uint8_t cmd     = (uint8_t)spi_rx_byte; /* snapshot before clearing */
            spi_cmd_pending = 0U;

            RunCam_DispatchSpiCommand(cmd);
        }
    }
    /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  SPI receive-complete callback — called from ISR context by HAL.
  *
  * ISR work is kept to an absolute minimum: set the pending flag and re-arm
  * the peripheral.  All RunCam UART and I2C traffic executes in the main loop.
  *
  * @param  hspi  SPI handle that triggered the callback.
  */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        spi_cmd_pending = 1U;

        /* Re-arm immediately so the next command byte is never missed */
        SPI_StartReceive();
    }
}

/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* AXI clock gating */
    RCC->CKGAENR = 0xE003FFFFU;

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = 64U;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 4U;
    RCC_OscInitStruct.PLL.PLLN            = 8U;
    RCC_OscInitStruct.PLL.PLLP            = 2U;
    RCC_OscInitStruct.PLL.PLLQ            = 2U;
    RCC_OscInitStruct.PLL.PLLR            = 2U;
    RCC_OscInitStruct.PLL.PLLRGE          = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL       = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN        = 0U;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType       = RCC_CLOCKTYPE_HCLK    | RCC_CLOCKTYPE_SYSCLK
                                      | RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2
                                      | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource    = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKDivider   = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider   = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider  = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider  = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider  = RCC_APB2_DIV1;
    RCC_ClkInitStruct.APB4CLKDivider  = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialisation Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */

    hi2c1.Instance              = I2C1;

    /*
     * Timing for 400 kHz (Fast Mode) with PCLK1 = 16 MHz (HSI, APB1 ÷1).
     * Value computed by STM32CubeMX for these clock settings:
     *   PRESC=0, SCLDEL=3, SDADEL=0, SCLH=9, SCLL=19  → 0x00300F13
     * Verify with CubeMX if the APB1 clock changes.
     */
    hi2c1.Init.Timing           = 0x00300F13U;
    hi2c1.Init.OwnAddress1      = 0U;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0U;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enable the analogue I2C filter (improves noise immunity on long traces) */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Digital filter: 0 = disabled (analogue filter is sufficient here) */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0U) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief SPI2 Initialisation Function
  * @retval None
  */
static void MX_SPI2_Init(void)
{
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */

    hspi2.Instance                        = SPI2;
    hspi2.Init.Mode                       = SPI_MODE_SLAVE;
    hspi2.Init.Direction                  = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize                   = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity                = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase                   = SPI_PHASE_1EDGE;
    hspi2.Init.NSS                        = SPI_NSS_HARD_INPUT;
    hspi2.Init.FirstBit                   = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode                     = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation             = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial              = 0x0U;
    hspi2.Init.NSSPMode                   = SPI_NSS_PULSE_DISABLE;
    hspi2.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi2.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

/**
  * @brief ADC1 Initialisation Function
  *
  * Configured for single-channel differential measurement on INN5/INP5
  * (PB0/PB1).  Not used by the current firmware but left initialised so
  * the peripheral is ready if needed.
  *
  * @retval None
  */
static void MX_ADC1_Init(void)
{
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_HandleTypeDef  hadc1  = {0};
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    hadc1.Instance                      = ADC1;
    hadc1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution               = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode             = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait         = DISABLE;
    hadc1.Init.ContinuousConvMode       = DISABLE;
    hadc1.Init.NbrOfConversion          = 1U;
    hadc1.Init.DiscontinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun                  = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode         = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel                = ADC_CHANNEL_5;
    sConfig.Rank                   = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime           = ADC_SAMPLETIME_1CYCLE_5;
    sConfig.SingleDiff             = ADC_DIFFERENTIAL_ENDED;
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;
    sConfig.Offset                 = 0U;
    sConfig.OffsetSignedSaturation = DISABLE;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief UART4 Initialisation Function
  * @retval None
  */
static void MX_UART4_Init(void)
{
    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */

    huart4.Instance                    = UART4;
    huart4.Init.BaudRate               = 115200U;
    huart4.Init.WordLength             = UART_WORDLENGTH_8B;
    huart4.Init.StopBits               = UART_STOPBITS_1;
    huart4.Init.Parity                 = UART_PARITY_NONE;
    huart4.Init.Mode                   = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */
}

/**
  * @brief GPIO Initialisation Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* MPU Configuration */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    /*
     * Region 0: deny all access across the full 4 GB address space.
     * Higher-numbered regions added later carve out legitimate memory.
     */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x0U;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87U;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  Called on HAL error.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file;
    (void)line;
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
