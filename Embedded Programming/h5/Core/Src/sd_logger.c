#include "sd_logger.h"
#include "airbrake_config.h"
#include "encoder_app.h"
#include "esc_telem.h"
#include "main.h"
#include "sdmmc.h"
#include "usart.h"
#include "fatfs.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Log rate: one line every LOG_PERIOD_MS milliseconds */
#define LOG_PERIOD_MS  (1000U / SD_LOG_RATE_HZ)

static volatile uint8_t s_isLogging    = 0;
static uint32_t         s_lastLogTime  = 0;
static uint32_t         s_logStartTime = 0;

#ifdef FATFS_MIDDLEWARE_ENABLED
static FATFS   s_fs;
static FIL     s_logFile;
static uint8_t s_sdMounted = 0;
#endif

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static void Logger_Print(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0)
    {
        if (len > (int)sizeof(buf)) len = (int)sizeof(buf);
        HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/*
 * SD_Logger_Init
 *
 * Initialises the SDMMC hardware (if not already done) and attempts to mount
 * the FAT filesystem.  Safe to call with no card inserted — returns 0 and
 * prints a message but does not hang or call Error_Handler().
 *
 * Called from ESC_App_Init() after the ESC arm hold, so the PWM neutral
 * signal is already stable before any SD card delay occurs.
 */
uint8_t SD_Logger_Init(void)
{
    /*
     * Initialise SDMMC hardware here rather than in main() so that a missing
     * or slow-to-respond card does not delay the PWM neutral output.
     * MX_SDMMC1_SD_Init() is safe to call multiple times — HAL checks the
     * peripheral state and skips re-init if already done.
     */
    MX_SDMMC1_SD_Init();

    if (!SDMMC_IsHwReady())
    {
        Logger_Print("SD card not detected (HAL_SD_Init failed).\r\n");
        Logger_Print("Logging to UART only.\r\n");
        return 0;
    }

#ifdef FATFS_MIDDLEWARE_ENABLED
    FRESULT res = f_mount(&s_fs, SDPath, 1);
    if (res != FR_OK)
    {
        s_sdMounted = 0;
        Logger_Print("SD mount failed (FatFS error %d). Logging to UART only.\r\n", res);
        return 0;
    }

    s_sdMounted = 1;
    Logger_Print("SD card mounted.\r\n");
    return 1;
#else
    Logger_Print("SD logging disabled (FatFS middleware not enabled).\r\n");
    Logger_Print("To enable: add FatFS middleware in STM32CubeMX and define "
                 "FATFS_MIDDLEWARE_ENABLED.\r\n");
    return 0;
#endif
}

void SD_Logger_Start(void)
{
#ifdef FATFS_MIDDLEWARE_ENABLED
    /* Re-attempt mount if not already mounted */
    if (!s_sdMounted)
    {
        if (!SD_Logger_Init())
        {
            Logger_Print("SD unavailable — logging to UART only.\r\n");
        }
    }

    if (s_sdMounted)
    {
        FRESULT res = f_open(&s_logFile, SD_LOG_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);
        if (res != FR_OK)
        {
            Logger_Print("SD file open failed (error %d) — logging to UART only.\r\n", res);
            s_sdMounted = 0;
        }
        else
        {
            const char *header =
                "Time_s,Position_um,Temp_C,Voltage_mV,Current_mA,"
                "Consumption_mAh,eRPM\r\n";
            UINT written;
            f_write(&s_logFile, header, strlen(header), &written);
            f_sync(&s_logFile);
            Logger_Print("SD logging started: %s\r\n", SD_LOG_FILENAME);
        }
    }
#endif

    Logger_Print("\r\n=== Logging Started ===\r\n");
    Logger_Print("Time_s,Position_um,Temp_C,Voltage_mV,Current_mA,"
                 "Consumption_mAh,eRPM\r\n");

    s_isLogging    = 1;
    s_logStartTime = HAL_GetTick();
    s_lastLogTime  = s_logStartTime;
}

void SD_Logger_Stop(void)
{
    if (!s_isLogging)
        return;

#ifdef FATFS_MIDDLEWARE_ENABLED
    if (s_sdMounted)
    {
        f_close(&s_logFile);
        Logger_Print("SD file closed.\r\n");
    }
#endif

    Logger_Print("=== Logging Stopped ===\r\n");
    s_isLogging = 0;
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

    float    time_s          = (float)(now - s_logStartTime) / 1000.0f;
    int32_t  position_um     = Encoder_GetPosition_um();
    uint8_t  temp_c          = ESC_Telem_GetTemp_C();
    uint32_t voltage_mV      = ESC_Telem_GetVoltage_mV();
    uint32_t current_mA      = ESC_Telem_GetCurrent_mA();
    uint32_t consumption_mAh = ESC_Telem_GetConsumption_mAh();
    uint32_t erpm            = ESC_Telem_GetERPM();

    char line[128];
    snprintf(line, sizeof(line),
             "%.3f,%ld,%u,%lu,%lu,%lu,%lu\r\n",
             time_s,
             (long)position_um,
             temp_c,
             (unsigned long)voltage_mV,
             (unsigned long)current_mA,
             (unsigned long)consumption_mAh,
             (unsigned long)erpm);

#ifdef FATFS_MIDDLEWARE_ENABLED
    if (s_sdMounted)
    {
        UINT written;
        f_write(&s_logFile, line, strlen(line), &written);

        static uint8_t sync_counter = 0;
        if (++sync_counter >= 5)
        {
            f_sync(&s_logFile);
            sync_counter = 0;
        }
    }
#endif

    Logger_Print("%s", line);
}
