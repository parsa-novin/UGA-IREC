/*
 * magnetometer.h
 *
 * Driver for the MEMSIC MMC5983MA 3-axis magnetometer over 4-wire SPI.
 * Full-scale range: ±8 Gauss, 18-bit resolution, 16384 counts/G.
 */

#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32c0xx_hal.h"

/**
 * @brief MMC5983MA instance handle — one per physical device.
 */
typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
} MMC_Handle;

/**
 * @brief Initialise the MMC5983MA: perform SET, verify Product ID (0x30).
 *
 * @param mag      Pointer to an MMC_Handle with cs_port/cs_pin filled in.
 * @return true if Product ID matched, false otherwise.
 */
extern bool  MMC_init(MMC_Handle *mag);

/**
 * @brief Trigger one magnetic measurement and read all three axes.
 *
 * Blocks until the Meas_M_Done status bit is set (~8 ms at BW=00).
 *
 * @param mag  Initialised MMC_Handle.
 * @param x    Output: X-axis field in Gauss.
 * @param y    Output: Y-axis field in Gauss.
 * @param z    Output: Z-axis field in Gauss.
 */
extern void  MMC_getMag(MMC_Handle *mag, float *x, float *y, float *z);

/**
 * @brief Read the on-chip temperature sensor.
 *
 * Range: -75 °C to +125 °C, ~0.8 °C/count.
 *
 * @param mag  Initialised MMC_Handle.
 * @return Temperature in degrees Celsius.
 */
extern float MMC_getTemp(MMC_Handle *mag);

/**
 * @brief Issue a SET pulse to restore sensor magnetisation.
 *
 * Call after exposure to fields > 10 Gauss.
 *
 * @param mag  Initialised MMC_Handle.
 */
extern void  MMC_set(MMC_Handle *mag);

/**
 * @brief Issue a RESET pulse (opposite polarity to SET).
 *
 * @param mag  Initialised MMC_Handle.
 */
extern void  MMC_reset(MMC_Handle *mag);

#endif /* MAGNETOMETER_H_ */
