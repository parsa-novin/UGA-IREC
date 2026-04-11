/**
  ******************************************************************************
  * @file    sd_logger.h
  * @brief   SD card + UART CSV logger for the airbrake controller.
  *
  * Log columns (written in this order, comma-separated):
  *
  *   Timestamp_ms        — milliseconds since SD_Logger_Start() was called
  *   Position_units      — actuator position in internal encoder units
  *                         (1,000,000 units = 1 mm of travel)
  *   ESC_Valid           — 1 if ESC telemetry is fresh, 0 if stale/absent
  *   ESC_Temp_C          — ESC temperature in degrees C
  *   ESC_Voltage_mV      — bus voltage in millivolts
  *   ESC_Current_mA      — phase current in milliamps
  *   ESC_Consumption_mAh — accumulated consumption in milliamp-hours
  *   ESC_eRPM            — electrical RPM × 100 (raw AM32 field × 100)
  *   Altitude            — barometric altitude (raw upstream int32 field)
  *   Pressure            — raw pressure field from upstream board
  *   IMU16_Ax            — IMU16 accelerometer X  (g)
  *   IMU16_Ay            — IMU16 accelerometer Y  (g)
  *   IMU16_Az            — IMU16 accelerometer Z  (g)
  *   IMU4_Ax             — IMU4  accelerometer X  (g)
  *   IMU4_Ay             — IMU4  accelerometer Y  (g)
  *   IMU4_Az             — IMU4  accelerometer Z  (g)
  *   Mag_X               — magnetometer X
  *   Mag_Y               — magnetometer Y
  *   Mag_Z               — magnetometer Z
  *   BMI_Ax              — BMI accelerometer X    (g)
  *   BMI_Ay              — BMI accelerometer Y    (g)
  *   BMI_Az              — BMI accelerometer Z    (g)
  *   BMI_Gx              — BMI gyroscope X        (deg/s)
  *   BMI_Gy              — BMI gyroscope Y        (deg/s)
  *   BMI_Gz              — BMI gyroscope Z        (deg/s)
  *   Ext_Temp            — external temperature from upstream board
  ******************************************************************************
  */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Configuration — change these to suit your application
 * ------------------------------------------------------------------------- */

/** Log filename on the SD card */
#define SD_LOG_FILENAME     "airbrake_log.csv"

/** Target log rate in Hz (one row per 1000/SD_LOG_RATE_HZ milliseconds) */
#define SD_LOG_RATE_HZ      100U

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
  * @brief  Initialise SDMMC hardware and mount the FatFS volume.
  *
  * Safe to call with no card inserted — returns 0 and logs a message but
  * does not hang.  Called automatically from ESC_App_Init().
  *
  * @retval 1 if the SD card was mounted successfully, 0 otherwise.
  */
uint8_t SD_Logger_Init(void);

/**
  * @brief  Open a new log file, write the CSV header, and begin logging.
  *
  * If the SD card is unavailable, data is still printed to UART4 so it can
  * be captured by a terminal or the Python logging tool.
  */
void SD_Logger_Start(void);

/**
  * @brief  Flush and close the log file.  Safe to call when not logging.
  */
void SD_Logger_Stop(void);

/**
  * @brief  Returns 1 while logging is active.
  */
uint8_t SD_Logger_Is_Active(void);

/**
  * @brief  Rate-limited task — call from the main loop every iteration.
  *
  * Internally rate-limited to SD_LOG_RATE_HZ.  When a new row is due,
  * collects data from all application modules and writes one CSV line to
  * both the SD file (if available) and UART4.
  */
void SD_Logger_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_LOGGER_H */
