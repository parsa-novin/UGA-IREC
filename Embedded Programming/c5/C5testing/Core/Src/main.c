/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "barometer.h"
#include "imu.h"
#include "magnetometer.h"
#include "bmi.h"
#include "temperature.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief  Binary telemetry packet transmitted over USART1.
 *
 * Layout (75 bytes, little-endian, packed):
 *   [0x00] header    : uint16_t     — sync word 0xAA55 for receiver framing
 *   [0x02] altitude  : int32_t      — cm above reference (barometer)
 *   [0x06] pressure  : int32_t      — Pa (same ADC cycle as altitude, no extra cost)
 *   [0x0A] imu16_ax  : float        — ±16g IMU  X accel (g)
 *   [0x0E] imu16_ay  : float
 *   [0x12] imu16_az  : float
 *   [0x16] imu4_ax   : float        — ±4g  IMU  X accel (g)
 *   [0x1A] imu4_ay   : float
 *   [0x1E] imu4_az   : float
 *   [0x22] mag_x     : float        — magnetometer X (Gauss)
 *   [0x26] mag_y     : float
 *   [0x2A] mag_z     : float
 *   [0x2E] bmi_ax    : float        — backup IMU X accel (g)
 *   [0x32] bmi_ay    : float
 *   [0x36] bmi_az    : float
 *   [0x3A] bmi_gx    : float        — backup IMU X gyro (dps)
 *   [0x3E] bmi_gy    : float
 *   [0x42] bmi_gz    : float
 *   [0x46] ext_temp  : float        — TMP127-Q1 temperature (°C)
 *   [0x4A] checksum  : uint8_t      — XOR of bytes [0x02]..[0x49]
 *
 * Total: 75 bytes.
 * At 460800 baud (8N1, 10 bits/byte) one packet takes ~1.63 ms to transmit.
 */
typedef struct __attribute__((packed)) {
    uint16_t header;      /* always 0xAA55                     */
    int32_t  altitude;    /* cm                                */
    int32_t pressure;
    float    imu16_ax;    /* g                                 */
    float    imu16_ay;
    float    imu16_az;
    float    imu4_ax;     /* g                                 */
    float    imu4_ay;
    float    imu4_az;
    float    mag_x;       /* Gauss                             */
    float    mag_y;
    float    mag_z;
    float    bmi_ax;      /* g                                 */
    float    bmi_ay;
    float    bmi_az;
    float    bmi_gx;      /* dps                               */
    float    bmi_gy;
    float    bmi_gz;
    float    ext_temp;    /* °C                                */
    uint8_t  checksum;    /* XOR of all bytes after header     */
} SensorPacket_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PKT_HEADER      0xAA55u
#define UART_TX_TIMEOUT 10u     /* ms — 75 bytes @ 460800 baud = 1.63 ms        */

/*
 * MMC5983MA runs in continuous-measurement mode at 200 Hz ODR (configured in
 * MMC_init).  MMC_getMag() is a plain 8-byte SPI burst read with no trigger
 * command and no blocking status poll.
 *
 * MMC_DIVIDER gates the read to every Nth loop iteration.  The last-known
 * mag values are held in the packet struct and retransmitted on skipped loops.
 *
 * Timing budget (12 MHz SYSCLK, SPI ÷8 = 1.5 MHz, UART 460800):
 *   Barometer_calculate()  : 2 × HAL_Delay(1) + SPI reads  ≈ 2.1 ms
 *   getAltitude/Pressure   : cached returns (calculate=false) ≈ 0.0 ms
 *   IMU ×2 burst reads     : 2 × 8 bytes @ 1.5 MHz         ≈ 0.05 ms
 *   BMI270 burst read      : 14 bytes @ 1.5 MHz             ≈ 0.08 ms
 *   TMP127 transaction     : 4 bytes @ 1.5 MHz              ≈ 0.02 ms
 *   MMC burst read (÷4)    : 8 bytes amortised              ≈ 0.005 ms
 *   UART TX 75 bytes       : 75 × 10 bits / 460800          ≈ 1.63 ms
 *   ─────────────────────────────────────────────────────────────────
 *   Total                                                   ≈ 3.88 ms
 *   Loop rate                                               ≈ 258 Hz
 *   Effective mag rate     : 258 / MMC_DIVIDER              ≈  64 Hz
 */
#define MMC_DIVIDER     4u      /* mag updated at ~loop_rate/4 ≈ 64 Hz         */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
IMU_Handle imu_16g;  /* high-range IMU  — CS on GPIOA GPIO_PIN_15 */
IMU_Handle imu_4g;   /* low-range  IMU  — CS on GPIOB GPIO_PIN_4  */

MMC_Handle mag;      /* MMC5983MA magnetometer */
BMI_Handle bmi;      /* BMI270 backup IMU      */
TMP127_Handle tmp;   /* TMP127-Q1 temperature  */

SensorPacket_t pkt;
static uint32_t mmc_tick = 0u;   /* counts main-loop iterations for MMC divider */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Compute XOR checksum over the payload region of the packet.
 *         Covers every byte between the header and the checksum field.
 */
static uint8_t packet_checksum(const SensorPacket_t *p)
{
    const uint8_t *start = (const uint8_t *)p + sizeof(p->header);
    const uint8_t *end   = (const uint8_t *)&p->checksum;
    uint8_t xor = 0u;
    for (const uint8_t *b = start; b < end; b++) {
        xor ^= *b;
    }
    return xor;
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

  Barometer_init();
  IMU_init(&imu_16g, GPIOA, GPIO_PIN_15, ACCEL_FS_16G);
  IMU_init(&imu_4g,  GPIOB, GPIO_PIN_4,  ACCEL_FS_4G);

  /* Magnetometer — update CS pin to match schematic */
  mag.cs_port = GPIOA;
  mag.cs_pin  = GPIO_PIN_4;
  MMC_init(&mag);

  /* Backup IMU (BMI270) — update CS pin to match schematic */
  BMI_init(&bmi, GPIOB, GPIO_PIN_9, BMI_ACC_FS_8G, BMI_GYR_FS_2000DPS);

  /* External temperature sensor (TMP127-Q1) — update CS pin to match schematic */
  TMP127_init(&tmp, GPIOA, GPIO_PIN_8);

  /* Pre-fill the header — it never changes */
  pkt.header = PKT_HEADER;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ── Read all sensors ──────────────────────────────────────────────── */

	  pkt.altitude = Barometer_getAltitude(true);
	  pkt.pressure = Barometer_getPressure(false);

	  float ax, ay, az;

	  IMU_getAccel(&imu_16g, &ax, &ay, &az);
	  pkt.imu16_ax = ax;
	  pkt.imu16_ay = ay;
	  pkt.imu16_az = az;

	  IMU_getAccel(&imu_4g, &ax, &ay, &az);
	  pkt.imu4_ax = ax;
	  pkt.imu4_ay = ay;
	  pkt.imu4_az = az;

	  float mx, my, mz;
	  if (++mmc_tick >= MMC_DIVIDER) {
	      mmc_tick = 0u;
	      MMC_getMag(&mag, &mx, &my, &mz);
	      pkt.mag_x = mx;
	      pkt.mag_y = my;
	      pkt.mag_z = mz;
	  }

	  float gx, gy, gz;
	  BMI_getAllMotion(&bmi, &ax, &ay, &az, &gx, &gy, &gz);
	  pkt.bmi_ax = ax;  pkt.bmi_ay = ay;  pkt.bmi_az = az;
	  pkt.bmi_gx = gx;  pkt.bmi_gy = gy;  pkt.bmi_gz = gz;

	  pkt.ext_temp = TMP127_getTemp(&tmp);

      /* ── Seal and transmit ─────────────────────────────────────────────── */
      pkt.checksum = packet_checksum(&pkt);

      HAL_UART_Transmit(&huart1, (uint8_t *)&pkt, sizeof(pkt), UART_TX_TIMEOUT);
      HAL_Delay(20);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
