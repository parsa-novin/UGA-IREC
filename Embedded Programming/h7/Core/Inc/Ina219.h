/**
  ******************************************************************************
  * @file    ina219.h
  * @brief   INA219 bi-directional current/power monitor driver.
  *
  * Device:  Texas Instruments INA219AxD
  * Bus:     I2C (up to 400 kHz fast mode)
  * Address: 0x45 (A1=1, A0=0 — pin A1 tied to VS, A0 to GND)
  *
  * This driver configures the INA219 for continuous shunt+bus voltage
  * measurement and returns current in milliamps (mA) as a float.
  *
  * Shunt resistor value is set by INA219_SHUNT_OHMS below.  Adjust this
  * to match the physical component on your board before building.
  *
  * Calibration register derivation
  * --------------------------------
  *   Current_LSB  = Max_Current / 2^15
  *                = INA219_MAX_CURRENT_A / 32768
  *
  *   Cal          = trunc(0.04096 / (Current_LSB × R_shunt))
  *
  * With the defaults below (100 mA max, 0.1 Ω shunt):
  *   Current_LSB  = 0.1 / 32768  ≈ 3.0518e-6 A/LSB  (≈ 3.052 µA/LSB)
  *   Cal          = trunc(0.04096 / (3.0518e-6 × 0.1))
  *                = trunc(134,186.6) = 4096  — rounds to a clean power of 2
  *
  * Register 0x04 (Current) read value × Current_LSB = current in amps.
  ******************************************************************************
  */

#ifndef INA219_H
#define INA219_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* -------------------------------------------------------------------------- */
/* User-configurable parameters — adjust to match your hardware               */
/* -------------------------------------------------------------------------- */

/** 7-bit I2C address of the INA219 (A1=VS, A0=GND → 0x45). */
#define INA219_I2C_ADDR        (0x45U)

/** Shunt resistor value in ohms. */
#define INA219_SHUNT_OHMS      (0.05f)

/** Maximum expected current in amps (sets Current_LSB granularity). */
#define INA219_MAX_CURRENT_A   (2.5f)

/** I2C timeout in milliseconds for each transaction. */
#define INA219_I2C_TIMEOUT_MS  (10U)

/* -------------------------------------------------------------------------- */
/* INA219 register addresses                                                  */
/* -------------------------------------------------------------------------- */

#define INA219_REG_CONFIG      (0x00U)  /**< Configuration register           */
#define INA219_REG_SHUNT_V     (0x01U)  /**< Shunt voltage register           */
#define INA219_REG_BUS_V       (0x02U)  /**< Bus voltage register             */
#define INA219_REG_POWER       (0x03U)  /**< Power register                   */
#define INA219_REG_CURRENT     (0x04U)  /**< Current register (requires Cal)  */
#define INA219_REG_CALIBRATION (0x05U)  /**< Calibration register             */

/* -------------------------------------------------------------------------- */
/* Configuration register value                                               */
/*                                                                            */
/* Bits [15:13] = 011  — reserved, must write 0; bus/shunt both enabled       */
/* Bit  [13]    = 1    — reset bit (leave 0 for normal operation)             */
/* Bits [12:11] = BRNG = 1  — bus voltage range 32 V                         */
/* Bits [10:9]  = PGA  = 00 — shunt PGA ÷1 (±40 mV range, suits ≤100 mA)    */
/* Bits [8:7]   = BADC = 0011 — bus ADC 12-bit / 532 µs                      */
/* Bits [6:3]   = SADC = 0011 — shunt ADC 12-bit / 532 µs                    */
/* Bits [2:0]   = MODE = 111  — continuous shunt + bus                        */
/*                                                                            */
/* Encoded:  0 0 1 1  1 0 0 0  0 0 0 1  1111  = 0x381F                        */
/* -------------------------------------------------------------------------- */
#define INA219_CONFIG_VALUE    (0x381FU)

/* -------------------------------------------------------------------------- */
/* Return status                                                               */
/* -------------------------------------------------------------------------- */

typedef enum
{
    INA219_OK        = 0,  /**< Operation completed successfully              */
    INA219_ERROR     = 1,  /**< I2C transaction error                         */
    INA219_BAD_PARAM = 2,  /**< NULL pointer or invalid argument              */
} INA219_StatusTypeDef;

/* -------------------------------------------------------------------------- */
/* Handle                                                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    I2C_HandleTypeDef *hi2c;         /**< I2C peripheral handle               */
    float              currentLsb_A; /**< Current LSB in amps, computed once  */
    uint16_t           calValue;     /**< Calibration register value          */
} INA219_HandleTypeDef;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Initialise the INA219: compute calibration, write config and
  *         calibration registers.
  *
  * @param  hina  Pointer to an INA219_HandleTypeDef.
  * @param  hi2c  Pointer to an initialised I2C_HandleTypeDef.
  * @retval INA219_OK on success, INA219_ERROR on I2C failure.
  */
INA219_StatusTypeDef INA219_Init(INA219_HandleTypeDef *hina,
                                  I2C_HandleTypeDef    *hi2c);

/**
  * @brief  Read the current register and convert to milliamps.
  *
  * The INA219 must have been initialised (calibration register written)
  * before calling this function, otherwise the current register reads zero.
  *
  * @param  hina        Pointer to an initialised INA219_HandleTypeDef.
  * @param  current_mA  Output: current in milliamps (signed, negative = reverse flow).
  * @retval INA219_OK on success, error code otherwise.
  */
INA219_StatusTypeDef INA219_ReadCurrent_mA(INA219_HandleTypeDef *hina,
                                            float                *current_mA);

#ifdef __cplusplus
}
#endif

#endif /* INA219_H */
