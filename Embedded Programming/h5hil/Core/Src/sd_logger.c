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
  *   - ESC_App_GetCamCurrent_mA()     — camera circuit current (H7 INA219)
  *   - ESC_App_GetBattVoltage_mV()    — battery voltage (H5 ADC on PB0)
  *
  * Each row is written identically to both the SD file and UART2, so a
  * terminal or the Python logging tool receives the same CSV as the card.
  *
  * Unified CSV format
  * ------------------
  * Column order exactly matches the ground-station Python logger so that a
  * single analysis script works with files from either source without any
  * column removal or renaming.  See sd_logger.h for the full column list.
  *
  * Timestamp
  * ---------
  * epoch_time_s is an absolute Unix timestamp (seconds since 1970-01-01 UTC)
  * derived from the sync sent by the ground station on connection.  Before
  * the epoch is synchronised the column contains elapsed seconds since
  * SD_Logger_Start() was called (relative time, still monotonic and useful).
  *
  * Sync policy
  * -----------
  * f_sync() is called every SYNC_EVERY_N_ROWS rows.  At 100 Hz that is
  * every 50 ms, balancing card wear against data safety.
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

#include <math.h>
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
 * CSV header — column order must match the ground-station Python CSV exactly.
 * ========================================================================= */

static const char k_csvHeader[] =
    "epoch_time_s,"
    "altitude_m,"
    "pressure_kpa,"
    "imu16_ax_g,"
    "imu16_ay_g,"
    "imu16_az_g,"
    "imu16_accel_mag_g,"
    "imu4_ax_g,"
    "imu4_ay_g,"
    "imu4_az_g,"
    "imu4_accel_mag_g,"
    "mag_x_gauss,"
    "mag_y_gauss,"
    "mag_z_gauss,"
    "mag_mag_gauss,"
    "bmi_ax_g,"
    "bmi_ay_g,"
    "bmi_az_g,"
    "bmi_accel_mag_g,"
    "bmi_gx_dps,"
    "bmi_gy_dps,"
    "bmi_gz_dps,"
    "bmi_gyro_mag_dps,"
    "ext_temp_c,"
    "esc_valid,"
    "esc_temp_c,"
    "esc_voltage_v,"
    "esc_current_a,"
    "esc_consumption_mah,"
    "esc_erpm,"
    "encoder_position_mm,"
    "flap_angle_deg,"
    "cam_current_ma,"
    "batt_voltage_v"
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

/* Epoch synchronisation state */
static uint32_t         s_epochSec     = 0U;   /* Unix seconds at sync point  */
static uint32_t         s_epochTickMs  = 0U;   /* HAL_GetTick() at sync point */
static uint8_t          s_epochValid   = 0U;   /* 1 after SD_Logger_SetEpoch  */

#define FILEX_MEDIA_BUFFER_SIZE 4096U

static FX_MEDIA s_fxMedia;
static FX_FILE  s_fxFile;
static UCHAR    s_fxMediaBuffer[FILEX_MEDIA_BUFFER_SIZE];

/* =========================================================================
 * Private helpers
 * ========================================================================= */

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

static void Logger_Write(const char *str)
{
    uint16_t len = (uint16_t)strlen(str);

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
            UINT flush_status = fx_media_flush(&s_fxMedia);
            s_syncCounter = 0U;
            if (flush_status != FX_SUCCESS)
            {
                s_fileOpen  = 0U;
                s_sdMounted = 0U;
                Logger_Print("SD flush failed (%u). Logging to UART only.\r\n",
                             (unsigned)flush_status);
            }
        }
    }
}

/* Compute vector magnitude (avoids dependency on libm sqrt for small vectors) */
static float Vec3Mag(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

/* Flap angle from encoder position in mm — same polynomial as ground station */
static float DisplacementMm_to_AngleDeg(float mm)
{
    float angle = -0.000513f * (mm * mm * mm)
                + 0.010615f * (mm * mm)
                +  2.3199f  * mm
                -  0.4986f;
    if (angle < 0.0f)  angle = 0.0f;
    if (angle > 70.0f) angle = 70.0f;
    return angle;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

uint8_t SD_Logger_Init(void)
{
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
        if (!SD_Logger_Init())
            Logger_Print("SD unavailable — logging to UART only.\r\n");
    }

    if (s_sdMounted)
    {
        UINT status;

        /* Close any previously open handle before deleting — fx_file_delete
         * fails on an open file, which then causes fx_file_open to fail and
         * disables SD writing for the rest of the session.               */
        if (s_fileOpen)
        {
            (void)fx_media_flush(&s_fxMedia);
            (void)fx_file_close(&s_fxFile);
            s_fileOpen = 0U;
        }

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
    Logger_Write(k_csvHeader);
}

void SD_Logger_Stop(void)
{
    if (!s_isLogging)
        return;

    s_isLogging = 0U;

    if (s_sdMounted && s_fileOpen)
    {
        UINT flush_status = fx_media_flush(&s_fxMedia);
        if (flush_status != FX_SUCCESS)
            Logger_Print("SD flush on stop failed (%u).\r\n", (unsigned)flush_status);

        UINT close_status = fx_file_close(&s_fxFile);
        if (close_status != FX_SUCCESS)
            Logger_Print("SD file close failed (%u).\r\n", (unsigned)close_status);

        s_fileOpen = 0U;
        Logger_Print("SD file closed.\r\n");
    }

    Logger_Print("=== Logging Stopped ===\r\n");
}

uint8_t SD_Logger_Is_Active(void)
{
    return s_isLogging;
}

void SD_Logger_SetEpoch(uint32_t unix_seconds)
{
    s_epochSec    = unix_seconds;
    s_epochTickMs = HAL_GetTick();
    s_epochValid  = 1U;
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
     * Timestamp — absolute epoch if synced, else relative elapsed seconds
     * --------------------------------------------------------------------- */
    float epoch_time_s;
    if (s_epochValid)
    {
        uint32_t elapsed_since_sync = now - s_epochTickMs;
        epoch_time_s = (float)s_epochSec + (float)elapsed_since_sync * 0.001f;
    }
    else
    {
        uint32_t elapsed_ms = now - s_logStartTime;
        epoch_time_s = (float)elapsed_ms * 0.001f;
    }

    /* -----------------------------------------------------------------------
     * Snapshot all data sources
     * --------------------------------------------------------------------- */

    int32_t position_um = Encoder_GetPosition_um();
    float   position_mm = (float)position_um * 0.001f;

    uint8_t  esc_valid       = ESC_Telem_IsValid();
    uint8_t  esc_temp_c      = ESC_Telem_GetTemp_C();
    uint32_t esc_voltage_mV  = ESC_Telem_GetVoltage_mV();
    uint32_t esc_current_mA  = ESC_Telem_GetCurrent_mA();
    uint32_t esc_cons_mAh    = ESC_Telem_GetConsumption_mAh();
    uint32_t esc_erpm        = ESC_Telem_GetERPM();

    float    cam_current_mA  = ESC_App_GetCamCurrent_mA();
    float    batt_voltage_v  = (float)ESC_App_GetBattVoltage_mV() * 0.001f;

    SensorPacket_t sensors;
    uint8_t        sensors_valid = ESC_App_GetLatestSensorPacket(&sensors);

    float altitude_m    = sensors_valid ? (float)sensors.altitude  : 0.0f;
    float pressure_kpa  = sensors_valid ? (float)sensors.pressure * 0.001f : 0.0f;

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

    /* Derived magnitudes */
    float imu16_mag   = Vec3Mag(imu16_ax, imu16_ay, imu16_az);
    float imu4_mag    = Vec3Mag(imu4_ax,  imu4_ay,  imu4_az);
    float mag_mag     = Vec3Mag(mag_x,    mag_y,    mag_z);
    float bmi_accel   = Vec3Mag(bmi_ax,   bmi_ay,   bmi_az);
    float bmi_gyro    = Vec3Mag(bmi_gx,   bmi_gy,   bmi_gz);
    float flap_angle  = DisplacementMm_to_AngleDeg(position_mm > 0.0f ? position_mm : 0.0f);

    /* -----------------------------------------------------------------------
     * Format one CSV row — column order must match k_csvHeader exactly.
     * --------------------------------------------------------------------- */
    char line[640];
    snprintf(line, sizeof(line),
             "%.3f,"    /* epoch_time_s          */
             "%.4f,"    /* altitude_m            */
             "%.4f,"    /* pressure_kpa          */
             "%.4f,"    /* imu16_ax_g            */
             "%.4f,"    /* imu16_ay_g            */
             "%.4f,"    /* imu16_az_g            */
             "%.4f,"    /* imu16_accel_mag_g     */
             "%.4f,"    /* imu4_ax_g             */
             "%.4f,"    /* imu4_ay_g             */
             "%.4f,"    /* imu4_az_g             */
             "%.4f,"    /* imu4_accel_mag_g      */
             "%.4f,"    /* mag_x_gauss           */
             "%.4f,"    /* mag_y_gauss           */
             "%.4f,"    /* mag_z_gauss           */
             "%.4f,"    /* mag_mag_gauss         */
             "%.4f,"    /* bmi_ax_g              */
             "%.4f,"    /* bmi_ay_g              */
             "%.4f,"    /* bmi_az_g              */
             "%.4f,"    /* bmi_accel_mag_g       */
             "%.4f,"    /* bmi_gx_dps            */
             "%.4f,"    /* bmi_gy_dps            */
             "%.4f,"    /* bmi_gz_dps            */
             "%.4f,"    /* bmi_gyro_mag_dps      */
             "%.4f,"    /* ext_temp_c            */
             "%u,"      /* esc_valid             */
             "%u,"      /* esc_temp_c            */
             "%.3f,"    /* esc_voltage_v         */
             "%.3f,"    /* esc_current_a         */
             "%lu,"     /* esc_consumption_mah   */
             "%lu,"     /* esc_erpm              */
             "%.4f,"    /* encoder_position_mm   */
             "%.3f,"    /* flap_angle_deg        */
             "%.3f,"    /* cam_current_ma        */
             "%.3f"     /* batt_voltage_v  (no trailing comma) */
             "\r\n",
             (double)epoch_time_s,
             (double)altitude_m,
             (double)pressure_kpa,
             (double)imu16_ax, (double)imu16_ay, (double)imu16_az, (double)imu16_mag,
             (double)imu4_ax,  (double)imu4_ay,  (double)imu4_az,  (double)imu4_mag,
             (double)mag_x,    (double)mag_y,    (double)mag_z,    (double)mag_mag,
             (double)bmi_ax,   (double)bmi_ay,   (double)bmi_az,   (double)bmi_accel,
             (double)bmi_gx,   (double)bmi_gy,   (double)bmi_gz,   (double)bmi_gyro,
             (double)ext_temp,
             (unsigned int)esc_valid,
             (unsigned int)esc_temp_c,
             (double)((float)esc_voltage_mV * 0.001f),
             (double)((float)esc_current_mA * 0.001f),
             (unsigned long)esc_cons_mAh,
             (unsigned long)esc_erpm,
             (double)position_mm,
             (double)flap_angle,
             (double)cam_current_mA,
             (double)batt_voltage_v);

    Logger_Write(line);
}
