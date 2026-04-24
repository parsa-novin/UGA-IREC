/*
 * imu.h
 *
 * Driver for the InvenSense IIM-42352 3-axis accelerometer over SPI.
 * Two IMU instances with independent CS pins are supported via IMU_Handle.
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32c0xx_hal.h"

/**
 * @brief Accelerometer full-scale range selection.
 *        Matches ACCEL_FS_SEL field in ACCEL_CONFIG0 (bits 7:5).
 */
typedef enum {
    ACCEL_FS_16G = 0,  /* ±16g, 2048 LSB/g  (default) */
    ACCEL_FS_8G  = 1,  /* ±8g,  4096 LSB/g  */
    ACCEL_FS_4G  = 2,  /* ±4g,  8192 LSB/g  */
    ACCEL_FS_2G  = 3,  /* ±2g,  16384 LSB/g */
} AccelFS;

/**
 * @brief Per-instance state for one IIM-42352.
 *        Initialise with IMU_init() before using any other function.
 */
typedef struct {
    GPIO_TypeDef *cs_port;  /* GPIO port for chip-select  */
    uint16_t      cs_pin;   /* GPIO pin  for chip-select  */
    AccelFS       fs;       /* Current full-scale setting */
    float         lsb_per_g; /* Sensitivity in LSB/g      */
} IMU_Handle;

/**
 * @brief Initialise the IMU: soft-reset, select Low-Noise mode, apply FS range.
 *
 * @param imu      Pointer to an uninitialised IMU_Handle.
 * @param cs_port  GPIO port connected to the IMU CS pin.
 * @param cs_pin   GPIO pin  connected to the IMU CS pin.
 * @param fs       Full-scale range to apply immediately after reset.
 * @return true if WHO_AM_I matched (0x6D), false otherwise.
 */
extern bool  IMU_init(IMU_Handle *imu, GPIO_TypeDef *cs_port,
                      uint16_t cs_pin, AccelFS fs);

/**
 * @brief Change the accelerometer full-scale range at runtime.
 *
 * @param imu  Initialised IMU_Handle.
 * @param fs   New full-scale selection (ACCEL_FS_2G … ACCEL_FS_16G).
 */
extern void  IMU_setFullScale(IMU_Handle *imu, AccelFS fs);

/**
 * @brief Read calibrated acceleration on all three axes.
 *
 * @param imu  Initialised IMU_Handle.
 * @param ax   Output: X acceleration in g.
 * @param ay   Output: Y acceleration in g.
 * @param az   Output: Z acceleration in g.
 */
extern void  IMU_getAccel(IMU_Handle *imu, float *ax, float *ay, float *az);

/**
 * @brief Read die temperature.
 *
 * @param imu  Initialised IMU_Handle.
 * @return Temperature in degrees Celsius.
 */
extern float IMU_getTemp(IMU_Handle *imu);

/**
 * @brief Read the raw 16-bit accelerometer counts (no scaling).
 *
 * @param imu  Initialised IMU_Handle.
 * @param rx   Output: raw X counts.
 * @param ry   Output: raw Y counts.
 * @param rz   Output: raw Z counts.
 */
extern void  IMU_getRawAccel(IMU_Handle *imu,
                             int16_t *rx, int16_t *ry, int16_t *rz);

#endif /* IMU_H_ */
