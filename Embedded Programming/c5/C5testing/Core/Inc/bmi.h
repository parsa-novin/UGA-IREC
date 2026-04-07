/*
 * bmi.h
 *
 * Driver for the Bosch BMI270 6-axis IMU (backup IMU) over 4-wire SPI.
 * CHIP_ID: 0x24.
 *
 * SPI protocol note: reads require one dummy byte to be discarded after
 * the address byte — this is handled transparently by the driver.
 *
 * Initialization note: the BMI270 requires a CSB toggle after power-up to
 * select SPI mode (done automatically in BMI_init via a dummy CHIP_ID read).
 */

#ifndef BMI_H_
#define BMI_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32c0xx_hal.h"

/**
 * @brief Accelerometer full-scale range.
 * Matches ACC_RANGE.acc_range field values.
 */
typedef enum {
    BMI_ACC_FS_2G  = 0x00,  /* ±2g,  16384 LSB/g */
    BMI_ACC_FS_4G  = 0x01,  /* ±4g,   8192 LSB/g */
    BMI_ACC_FS_8G  = 0x02,  /* ±8g,   4096 LSB/g (default) */
    BMI_ACC_FS_16G = 0x03,  /* ±16g,  2048 LSB/g */
} BMI_AccelFS;

/**
 * @brief Gyroscope full-scale range.
 * Matches GYR_RANGE.gyr_range field values.
 */
typedef enum {
    BMI_GYR_FS_2000DPS = 0x00, /* ±2000 dps, 16.384 LSB/dps (default) */
    BMI_GYR_FS_1000DPS = 0x01, /* ±1000 dps, 32.768 LSB/dps */
    BMI_GYR_FS_500DPS  = 0x02, /* ±500 dps,  65.536 LSB/dps */
    BMI_GYR_FS_250DPS  = 0x03, /* ±250 dps, 131.072 LSB/dps */
    BMI_GYR_FS_125DPS  = 0x04, /* ±125 dps, 262.144 LSB/dps */
} BMI_GyroFS;

/**
 * @brief Per-instance state for one BMI270.
 */
typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
    BMI_AccelFS   acc_fs;
    BMI_GyroFS    gyr_fs;
    float         acc_lsb_per_g;   /* LSB/g  for current range */
    float         gyr_lsb_per_dps; /* LSB/dps for current range */
} BMI_Handle;

/**
 * @brief Initialise the BMI270: switch to SPI, soft-reset, enable acc + gyro.
 *
 * @param bmi      Pointer to an uninitialised BMI_Handle.
 * @param cs_port  GPIO port for chip-select.
 * @param cs_pin   GPIO pin  for chip-select.
 * @param acc_fs   Accelerometer full-scale range.
 * @param gyr_fs   Gyroscope full-scale range.
 * @return true if CHIP_ID matched (0x24), false otherwise.
 */
extern bool  BMI_init(BMI_Handle *bmi, GPIO_TypeDef *cs_port,
                      uint16_t cs_pin, BMI_AccelFS acc_fs, BMI_GyroFS gyr_fs);

/**
 * @brief Read calibrated acceleration on all three axes.
 *
 * @param bmi  Initialised BMI_Handle.
 * @param ax   Output: X acceleration in g.
 * @param ay   Output: Y acceleration in g.
 * @param az   Output: Z acceleration in g.
 */
extern void  BMI_getAccel(BMI_Handle *bmi, float *ax, float *ay, float *az);

/**
 * @brief Read calibrated angular rate on all three axes.
 *
 * @param bmi  Initialised BMI_Handle.
 * @param gx   Output: X angular rate in dps.
 * @param gy   Output: Y angular rate in dps.
 * @param gz   Output: Z angular rate in dps.
 */
extern void  BMI_getGyro(BMI_Handle *bmi, float *gx, float *gy, float *gz);

/**
 * @brief Read all six motion axes in one burst (12 bytes).
 *
 * @param bmi  Initialised BMI_Handle.
 * @param ax, ay, az  Acceleration in g.
 * @param gx, gy, gz  Angular rate in dps.
 */
extern void  BMI_getAllMotion(BMI_Handle *bmi,
                              float *ax, float *ay, float *az,
                              float *gx, float *gy, float *gz);

/**
 * @brief Read die temperature.
 *
 * @param bmi  Initialised BMI_Handle.
 * @return Temperature in degrees Celsius (resolution: 1/512 °C).
 */
extern float BMI_getTemp(BMI_Handle *bmi);

#endif /* BMI_H_ */
