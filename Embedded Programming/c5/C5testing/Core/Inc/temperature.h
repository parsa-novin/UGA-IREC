/*
 * temperature.h
 *
 * Driver for the Texas Instruments TMP127-Q1 14-bit SPI temperature sensor.
 * Resolution: 0.03125 °C/LSB.  Range: −55 °C to +175 °C.
 *
 * SPI protocol note: the TMP127-Q1 uses a 3-wire SPI with a single
 * bidirectional SIO pin.  In the standard 4-wire STM32 wiring, connect
 * SIO → MISO only (read-only configuration); MOSI is not used.
 * Each CS-asserted transaction is exactly 32 clocks:
 *   • First 16 clocks  — device drives SIO with the 16-bit temperature word.
 *   • Second 16 clocks — controller drives SIO with the configuration word
 *                        (0x0000 = continuous conversion, 0x00FF = shutdown).
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32c0xx_hal.h"

/**
 * @brief Per-instance state for one TMP127-Q1.
 */
typedef struct {
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
} TMP127_Handle;

/**
 * @brief Initialise the TMP127-Q1 and place it in continuous conversion mode.
 *
 * Performs one 32-bit SPI transaction to write 0x0000 to the configuration
 * register.  The temperature value returned by this first transaction may
 * be erroneous; discard it.  Wait at least 270 ms before calling
 * TMP127_getTemp() for the first valid reading.
 *
 * @param tmp      Pointer to an uninitialised TMP127_Handle.
 * @param cs_port  GPIO port for chip-select.
 * @param cs_pin   GPIO pin  for chip-select.
 */
extern void  TMP127_init(TMP127_Handle *tmp,
                         GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief Read the current temperature.
 *
 * Issues one 32-bit SPI transaction and writes 0x0000 to the configuration
 * register to keep the device in continuous conversion mode.
 *
 * @param tmp  Initialised TMP127_Handle.
 * @return Temperature in degrees Celsius (14-bit, 0.03125 °C resolution).
 */
extern float TMP127_getTemp(TMP127_Handle *tmp);

#endif /* TEMPERATURE_H_ */
