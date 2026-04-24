/*
 * barometer.c
 *
 *  Created on: 5 oct. 2018
 *      Author: alex
 */

#include "barometer.h"
#include "stm32c0xx_hal.h"
#include <math.h>

#define SPI_TIMEOUT 50 //in ms

#define MS5611_EN  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define MS5611_DIS HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define CMD_RESET    0x1E
#define CMD_PROM_C1  0xA2
#define CMD_PROM_C2  0xA4
#define CMD_PROM_C3  0xA6
#define CMD_PROM_C4  0xA8
#define CMD_PROM_C5  0xAA
#define CMD_PROM_C6  0xAC

#define PRESSURE_OSR_256  0x40
#define PRESSURE_OSR_512  0x42
#define PRESSURE_OSR_1024 0x44
#define PRESSURE_OSR_2048 0x46
#define PRESSURE_OSR_4096 0x48

#define TEMP_OSR_256  0x50
#define TEMP_OSR_512  0x52
#define TEMP_OSR_1024 0x54
#define TEMP_OSR_2048 0x56
#define TEMP_OSR_4096 0x58

#define CONVERSION_OSR_256  1
#define CONVERSION_OSR_512  2
#define CONVERSION_OSR_1024 3
#define CONVERSION_OSR_2048 5
#define CONVERSION_OSR_4096 10

static uint16_t prom[6];
extern SPI_HandleTypeDef hspi1;

// min OSR by default
static uint8_t  pressAddr = PRESSURE_OSR_256;
static uint8_t  tempAddr  = TEMP_OSR_256;
static uint32_t convDelay = CONVERSION_OSR_256;

static int32_t temperature;
static int32_t pressure;
static float   altitude;

static void     ms5611_init(void);
static void     ms5611_write(uint8_t data);
static uint16_t ms5611_read16bits(uint8_t reg);
static uint32_t ms5611_read24bits(uint8_t reg);
static uint32_t ms5611_readRawTemp(void);
static uint32_t ms5611_readRawPressure(void);

/* ---------------------------------------------------------------------------
 * spi_transfer — uses HAL_SPI_TransmitReceive so the peripheral state machine
 * is managed entirely by HAL.  The manual __HAL_SPI_ENABLE / register-poll
 * approach was causing MISO to stay high due to re-enabling the peripheral
 * mid-transaction.
 * --------------------------------------------------------------------------*/
static HAL_StatusTypeDef spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint8_t dummy_rx[len];
    uint8_t dummy_tx[len];

    /* If caller doesn't care about RX, point at a scratch buffer */
    if (rx == NULL)
    {
        rx = dummy_rx;
    }
    /* If caller doesn't supply TX, send zeros */
    if (tx == NULL)
    {
        for (uint16_t i = 0; i < len; i++) dummy_tx[i] = 0x00;
        tx = dummy_tx;
    }

    return HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, SPI_TIMEOUT);
}

/* ---------------------------------------------------------------------------
 * ms5611_write — assert CS, send one command byte, deassert CS.
 * --------------------------------------------------------------------------*/
static void ms5611_write(uint8_t data)
{
    MS5611_EN;
    spi_transfer(&data, NULL, 1);
    MS5611_DIS;
}

/* ---------------------------------------------------------------------------
 * ms5611_read16bits — send register address + 2 dummy bytes, return 16-bit
 * value from bytes [1:2] of the response.
 * --------------------------------------------------------------------------*/
static uint16_t ms5611_read16bits(uint8_t reg)
{
    uint8_t tx[3] = {reg, 0x00, 0x00};
    uint8_t rx[3] = {0x00, 0x00, 0x00};

    MS5611_EN;
    spi_transfer(tx, rx, 3);
    MS5611_DIS;

    return ((uint16_t)rx[1] << 8) | (uint16_t)rx[2];
}

/* ---------------------------------------------------------------------------
 * ms5611_read24bits — send register address + 3 dummy bytes, return 24-bit
 * ADC value from bytes [1:3] of the response.
 * --------------------------------------------------------------------------*/
static uint32_t ms5611_read24bits(uint8_t reg)
{
    uint8_t tx[4] = {reg, 0x00, 0x00, 0x00};
    uint8_t rx[4] = {0x00, 0x00, 0x00, 0x00};

    MS5611_EN;
    spi_transfer(tx, rx, 4);
    MS5611_DIS;

    return ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];
}

/* ---------------------------------------------------------------------------
 * ms5611_init — reset the sensor and read the 6 factory calibration words
 * from PROM.
 * --------------------------------------------------------------------------*/
static void ms5611_init(void)
{
    MS5611_DIS;
    HAL_Delay(10);

    ms5611_write(CMD_RESET);
    HAL_Delay(10);

    prom[0] = ms5611_read16bits(CMD_PROM_C1);
    prom[1] = ms5611_read16bits(CMD_PROM_C2);
    prom[2] = ms5611_read16bits(CMD_PROM_C3);
    prom[3] = ms5611_read16bits(CMD_PROM_C4);
    prom[4] = ms5611_read16bits(CMD_PROM_C5);
    prom[5] = ms5611_read16bits(CMD_PROM_C6);
}

static uint32_t ms5611_readRawTemp(void)
{
    ms5611_write(tempAddr);
    HAL_Delay(convDelay);
    return ms5611_read24bits(0x00);
}

static uint32_t ms5611_readRawPressure(void)
{
    ms5611_write(pressAddr);
    HAL_Delay(convDelay);
    return ms5611_read24bits(0x00);
}

/* ===========================================================================
 * Public API
 * ==========================================================================*/

void Barometer_init(void)
{
    ms5611_init();
}

void Barometer_setOSR(OSR osr)
{
    switch (osr)
    {
        default:
        case OSR_256:
            pressAddr = PRESSURE_OSR_256;
            tempAddr  = TEMP_OSR_256;
            convDelay = CONVERSION_OSR_256;
            break;
        case OSR_512:
            pressAddr = PRESSURE_OSR_512;
            tempAddr  = TEMP_OSR_512;
            convDelay = CONVERSION_OSR_512;
            break;
        case OSR_1024:
            pressAddr = PRESSURE_OSR_1024;
            tempAddr  = TEMP_OSR_1024;
            convDelay = CONVERSION_OSR_1024;
            break;
        case OSR_2048:
            pressAddr = PRESSURE_OSR_2048;
            tempAddr  = TEMP_OSR_2048;
            convDelay = CONVERSION_OSR_2048;
            break;
        case OSR_4096:
            pressAddr = PRESSURE_OSR_4096;
            tempAddr  = TEMP_OSR_4096;
            convDelay = CONVERSION_OSR_4096;
            break;
    }
}

int32_t Barometer_getTemp(bool calculate)
{
    if (calculate) Barometer_calculate();
    return temperature;
}

int32_t Barometer_getPressure(bool calculate)
{
    if (calculate) Barometer_calculate();
    return pressure;
}

float Barometer_getAltitude(bool calculate)
{
    if (calculate) Barometer_calculate();
    return altitude;
}

void Barometer_calculate(void)
{
    int32_t dT;
    int64_t TEMP, OFF, SENS, P;
    uint32_t D1, D2;
    float press, r, c;

    D1 = ms5611_readRawPressure();
    D2 = ms5611_readRawTemp();

    dT   = (int32_t)D2 - ((int32_t)prom[4] * 256);
    TEMP = 2000 + ((int64_t)dT * prom[5]) / 8388608;
    OFF  = (int64_t)prom[1] * 65536 + ((int64_t)prom[3] * dT) / 128;
    SENS = (int64_t)prom[0] * 32768 + ((int64_t)prom[2] * dT) / 256;

    if (TEMP < 2000)    // second order temperature compensation
    {
        int64_t T2      = ((int64_t)dT * dT) >> 31;
        int64_t Aux_64  = (TEMP - 2000) * (TEMP - 2000);
        int64_t OFF2    = (5 * Aux_64) >> 1;
        int64_t SENS2   = (5 * Aux_64) >> 2;
        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    P = (D1 * SENS / 2097152 - OFF) / 32768;

    temperature = (int32_t)TEMP;
    pressure    = (int32_t)P;

    press    = (float)pressure;
    r        = press / 101325.0f;
    c        = 1.0f / 5.255f;
    altitude = (1.0f - powf(r, c)) * 44330.77f;
}
