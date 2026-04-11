/**
  ******************************************************************************
  * @file    sd_logger.c
  * @brief   SD card + UART CSV data logger for the airbrake controller.
  *
  * Data flow
  * ---------
  * SD_Logger_Task() is called from the main loop every iteration.  It is
  * rate-limited internally; when a row is due it snapshots data from:
  *
  *   - Encoder_GetPosition_um()       — actuator position
  *   - ESC_Telem_Get*()               — ESC telemetry (temp, V, I, mAh, eRPM)
  *   - ESC_App_GetLatestSensorPacket()— upstream board sensors (IMU, baro, mag)
  *
  * Each row is written identically to both the SD file and UART4, so a
  * terminal or the Python logging tool receives the same CSV as the card.
  *
  * CSV columns — see sd_logger.h for the full column description.
  *
  * Timestamp precision
  * -------------------
  * HAL_GetTick() returns a uint32_t millisecond counter.  Storing the elapsed
  * time as a float loses sub-second precision after ~4194 s (70 min) because
  * a 32-bit float only has 24 bits of mantissa.  Instead we store the raw
  * millisecond count as an integer and format it without floating-point so
  * precision is maintained for the full uint32_t range (~49 days).
  *
  * Sync policy
  * -----------
  * f_sync() is called every SYNC_EVERY_N_ROWS rows.  At 50 Hz that is
  * every 100 ms, which is a reasonable balance between card wear and
  * data safety in the event of a power loss.
  ******************************************************************************
  */

#include "sd_logger.h"
#include "encoder_app.h"
#include "esc_app.h"
#include "esc_telem.h"
#include "main.h"
#include "sdmmc.h"
#include "usart.h"
#include "app_filex.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* =========================================================================
 * Configuration
 * ========================================================================= */

/** Period between log rows in milliseconds */
#define LOG_PERIOD_MS       (1000U / SD_LOG_RATE_HZ)

/** Call f_sync() after this many rows (limits card write amplification) */
#define SYNC_EVERY_N_ROWS   5U

/* =========================================================================
 * CSV header — must match the snprintf format string in SD_Logger_Task()
 *              exactly, column for column.
 * ========================================================================= */

static const char k_csvHeader[] =
    "Timestamp_ms,"
    "Position_units,"
    "ESC_Valid,"
    "ESC_Temp_C,"
    "ESC_Voltage_mV,"
    "ESC_Current_mA,"
    "ESC_Consumption_mAh,"
    "ESC_eRPM,"
    "Altitude,"
    "Pressure,"
    "IMU16_Ax,"
    "IMU16_Ay,"
    "IMU16_Az,"
    "IMU4_Ax,"
    "IMU4_Ay,"
    "IMU4_Az,"
    "Mag_X,"
    "Mag_Y,"
    "Mag_Z,"
    "BMI_Ax,"
    "BMI_Ay,"
    "BMI_Az,"
    "BMI_Gx,"
    "BMI_Gy,"
    "BMI_Gz,"
    "Ext_Temp"
    "\r\n";

/* =========================================================================
 * Module state
 * ========================================================================= */

static volatile uint8_t s_isLogging    = 0U;
static uint32_t         s_lastLogTime  = 0U;
static uint32_t         s_logStartTime = 0U;
static uint8_t          s_syncCounter  = 0U;
static uint8_t          s_sdMounted    = 0U;
static uint8_t          s_fileOpen     = 0U;
static uint8_t          s_filexReady   = 0U;

#define FILEX_MEDIA_BUFFER_SIZE 4096U

static FX_MEDIA s_fxMedia;
static FX_FILE  s_fxFile;
static UCHAR    s_fxMediaBuffer[FILEX_MEDIA_BUFFER_SIZE];

/* =========================================================================
 * Private helpers
 * ========================================================================= */

/**
  * @brief  Printf-style write to UART4 (debug / upstream link).
  */
static void Logger_Print(const char *fmt, ...)
{
    char    buf[320];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        if (len > (int)sizeof(buf))
            len = (int)sizeof(buf);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/**
  * @brief  Write a string to the SD file (if mounted) AND to UART4.
  *
  * This single helper guarantees that the SD card and the UART stream
  * always contain identical content without duplicating the format string.
  */
static void Logger_Write(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);

    /* UART4 output — always */
    HAL_UART_Transmit(&huart2, (const uint8_t *)str, len, HAL_MAX_DELAY);

    if (s_sdMounted && s_fileOpen)
    {
        UINT status = fx_file_write(&s_fxFile, (VOID *)str, len);
        if (status != FX_SUCCESS)
        {
            s_fileOpen  = 0U;
            s_sdMounted = 0U;
            Logger_Print("FileX write failed (%u). Logging to UART only.\r\n",
                         (unsigned)status);
            return;
        }

        s_syncCounter++;
        if (s_syncCounter >= SYNC_EVERY_N_ROWS)
        {
            (void)fx_media_flush(&s_fxMedia);
            s_syncCounter = 0U;
        }
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

uint8_t SD_Logger_Init(void)
{
    /*
     * MX_SDMMC1_SD_Init() is guarded internally against double-init so this
     * call is safe even if main() already called it during peripheral setup.
     */
    MX_SDMMC1_SD_Init();

    if (!SDMMC_IsHwReady())
    {
        Logger_Print("SD card not detected. Logging to UART only.\r\n");
        return 0U;
    }

    if (!s_filexReady)
    {
        UINT fx_init = MX_FileX_Init();
        if (fx_init != FX_SUCCESS)
        {
            Logger_Print("FileX init failed (%u). Logging to UART only.\r\n",
                         (unsigned)fx_init);
            return 0U;
        }
        s_filexReady = 1U;
    }

    if (s_sdMounted)
        return 1U;

    UINT status = fx_media_open(&s_fxMedia,
                                "SDCARD",
                                fx_stm32_sd_driver,
                                FX_NULL,
                                s_fxMediaBuffer,
                                sizeof(s_fxMediaBuffer));
    if (status != FX_SUCCESS)
    {
        s_sdMounted = 0U;
        Logger_Print("SD mount failed (FileX status %u). Logging to UART only.\r\n",
                     (unsigned)status);
        return 0U;
    }

    s_sdMounted = 1U;
    Logger_Print("SD card mounted.\r\n");
    return 1U;
}

void SD_Logger_Start(void)
{
    if (!s_sdMounted)
    {
        /* Re-attempt mount: card may have been inserted after boot */
        if (!SD_Logger_Init())
            Logger_Print("SD unavailable — logging to UART only.\r\n");
    }

    if (s_sdMounted)
    {
        UINT status;

        /* Start each session with a fresh file. */
        (void)fx_file_delete(&s_fxMedia, (CHAR *)SD_LOG_FILENAME);

        status = fx_file_create(&s_fxMedia, (CHAR *)SD_LOG_FILENAME);
        if ((status != FX_SUCCESS) && (status != FX_ALREADY_CREATED))
        {
            Logger_Print("FileX create failed (%u) — logging to UART only.\r\n",
                         (unsigned)status);
            s_sdMounted = 0U;
        }
        else
        {
            status = fx_file_open(&s_fxMedia, &s_fxFile, (CHAR *)SD_LOG_FILENAME, FX_OPEN_FOR_WRITE);
            if (status != FX_SUCCESS)
            {
                Logger_Print("FileX open failed (%u) — logging to UART only.\r\n",
                             (unsigned)status);
                s_sdMounted = 0U;
                s_fileOpen  = 0U;
            }
            else
            {
                (void)fx_file_seek(&s_fxFile, 0U);
                s_fileOpen = 1U;
            }
        }
    }

    s_syncCounter  = 0U;
    s_isLogging    = 1U;
    s_logStartTime = HAL_GetTick();
    s_lastLogTime  = s_logStartTime;

    Logger_Print("\r\n=== Logging Started (%s) ===\r\n", SD_LOG_FILENAME);

    /* Write the CSV header to both SD and UART */
    Logger_Write(k_csvHeader);
}

void SD_Logger_Stop(void)
{
    if (!s_isLogging)
        return;

    s_isLogging = 0U;

    if (s_sdMounted && s_fileOpen)
    {
        (void)fx_media_flush(&s_fxMedia);
        (void)fx_file_close(&s_fxFile);
        s_fileOpen = 0U;
        Logger_Print("SD file closed.\r\n");
    }

    Logger_Print("=== Logging Stopped ===\r\n");
}

uint8_t SD_Logger_Is_Active(void)
{
    return s_isLogging;
}

void SD_Logger_Task(void)
{
    if (!s_isLogging)
        return;

    uint32_t now = HAL_GetTick();
    if ((now - s_lastLogTime) < LOG_PERIOD_MS)
        return;

    s_lastLogTime = now;

    /* -----------------------------------------------------------------------
     * Snapshot all data sources
     * --------------------------------------------------------------------- */

    /* Elapsed time — kept as integer milliseconds to preserve precision */
    uint32_t elapsed_ms = now - s_logStartTime;

    /* Encoder */
    int32_t position_units = Encoder_GetPosition_um();

    /* ESC telemetry */
    uint8_t  esc_valid       = ESC_Telem_IsValid();
    uint8_t  esc_temp_c      = ESC_Telem_GetTemp_C();
    uint32_t esc_voltage_mV  = ESC_Telem_GetVoltage_mV();
    uint32_t esc_current_mA  = ESC_Telem_GetCurrent_mA();
    uint32_t esc_cons_mAh    = ESC_Telem_GetConsumption_mAh();
    uint32_t esc_erpm        = ESC_Telem_GetERPM();

    /* Upstream sensor packet — read via ESC_App_GetLatestSensorPacket().
     * If no valid packet has been received yet, all sensor fields default
     * to 0 / 0.0f so the CSV row is still well-formed.                    */
    SensorPacket_t sensors;
    uint8_t        sensors_valid = ESC_App_GetLatestSensorPacket(&sensors);

    int32_t altitude = sensors_valid ? sensors.altitude  : 0;
    int32_t pressure = sensors_valid ? sensors.pressure  : 0;

    float imu16_ax = sensors_valid ? sensors.imu16_ax : 0.0f;
    float imu16_ay = sensors_valid ? sensors.imu16_ay : 0.0f;
    float imu16_az = sensors_valid ? sensors.imu16_az : 0.0f;

    float imu4_ax  = sensors_valid ? sensors.imu4_ax  : 0.0f;
    float imu4_ay  = sensors_valid ? sensors.imu4_ay  : 0.0f;
    float imu4_az  = sensors_valid ? sensors.imu4_az  : 0.0f;

    float mag_x    = sensors_valid ? sensors.mag_x    : 0.0f;
    float mag_y    = sensors_valid ? sensors.mag_y    : 0.0f;
    float mag_z    = sensors_valid ? sensors.mag_z    : 0.0f;

    float bmi_ax   = sensors_valid ? sensors.bmi_ax   : 0.0f;
    float bmi_ay   = sensors_valid ? sensors.bmi_ay   : 0.0f;
    float bmi_az   = sensors_valid ? sensors.bmi_az   : 0.0f;
    float bmi_gx   = sensors_valid ? sensors.bmi_gx   : 0.0f;
    float bmi_gy   = sensors_valid ? sensors.bmi_gy   : 0.0f;
    float bmi_gz   = sensors_valid ? sensors.bmi_gz   : 0.0f;

    float ext_temp = sensors_valid ? sensors.ext_temp : 0.0f;

    /* -----------------------------------------------------------------------
     * Format one CSV row
     *
     * Column order must match k_csvHeader exactly.
     *
     * Floating-point fields use %.4f (4 decimal places) throughout.
     * Integer fields use their natural format specifiers.
     * No trailing comma; \r\n terminates the line.
     * --------------------------------------------------------------------- */
    char line[512];
    snprintf(line, sizeof(line),
             "%lu,"      /* Timestamp_ms         */
             "%ld,"      /* Position_units        */
             "%u,"       /* ESC_Valid             */
             "%u,"       /* ESC_Temp_C            */
             "%lu,"      /* ESC_Voltage_mV        */
             "%lu,"      /* ESC_Current_mA        */
             "%lu,"      /* ESC_Consumption_mAh   */
             "%lu,"      /* ESC_eRPM              */
             "%ld,"      /* Altitude              */
             "%ld,"      /* Pressure              */
             "%.4f,"     /* IMU16_Ax              */
             "%.4f,"     /* IMU16_Ay              */
             "%.4f,"     /* IMU16_Az              */
             "%.4f,"     /* IMU4_Ax               */
             "%.4f,"     /* IMU4_Ay               */
             "%.4f,"     /* IMU4_Az               */
             "%.4f,"     /* Mag_X                 */
             "%.4f,"     /* Mag_Y                 */
             "%.4f,"     /* Mag_Z                 */
             "%.4f,"     /* BMI_Ax                */
             "%.4f,"     /* BMI_Ay                */
             "%.4f,"     /* BMI_Az                */
             "%.4f,"     /* BMI_Gx                */
             "%.4f,"     /* BMI_Gy                */
             "%.4f,"     /* BMI_Gz                */
             "%.4f"      /* Ext_Temp  (no trailing comma) */
             "\r\n",
             (unsigned long)elapsed_ms,
             (long)position_units,
             (unsigned int)esc_valid,
             (unsigned int)esc_temp_c,
             (unsigned long)esc_voltage_mV,
             (unsigned long)esc_current_mA,
             (unsigned long)esc_cons_mAh,
             (unsigned long)esc_erpm,
             (long)altitude,
             (long)pressure,
             (double)imu16_ax, (double)imu16_ay, (double)imu16_az,
             (double)imu4_ax,  (double)imu4_ay,  (double)imu4_az,
             (double)mag_x,    (double)mag_y,    (double)mag_z,
             (double)bmi_ax,   (double)bmi_ay,   (double)bmi_az,
             (double)bmi_gx,   (double)bmi_gy,   (double)bmi_gz,
             (double)ext_temp);

    Logger_Write(line);
}
