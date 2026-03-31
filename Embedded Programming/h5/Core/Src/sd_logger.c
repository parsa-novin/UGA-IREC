#include "sd_logger.h"
#include "airbrake_config.h"
#include "encoder_app.h"
#include "esc_telem.h"
#include "main.h"
#include "usart.h"
#include "fatfs.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* Calculate log period from configured rate */
#define LOG_PERIOD_MS  (1000U / SD_LOG_RATE_HZ)

static volatile uint8_t s_isLogging = 0;
static uint32_t s_lastLogTime = 0;
static uint32_t s_logStartTime = 0;

#ifdef FATFS_MIDDLEWARE_ENABLED
static FATFS s_fs;
static FIL s_logFile;
static volatile uint8_t s_sdMounted = 0;
#endif

/* For printing log data */
static void Logger_Print(const char *fmt, ...)
{
    char buf[256];
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

uint8_t SD_Logger_Init(void)
{
#ifdef FATFS_MIDDLEWARE_ENABLED
    /* Mount SD card */
    FRESULT res = f_mount(&s_fs, SDPath, 1);

    if (res != FR_OK)
    {
        s_sdMounted = 0;
        Logger_Print("SD mount failed (error %d)\r\n", res);
        return 0;
    }

    s_sdMounted = 1;
    Logger_Print("SD card mounted successfully.\r\n");
    return 1;
#else
    Logger_Print("SD logging disabled (FatFS middleware not enabled)\r\n");
    Logger_Print("Data will be logged to UART only.\r\n");
    Logger_Print("To enable SD: Add FatFS middleware in STM32CubeMX\r\n");
    return 0;
#endif
}

void SD_Logger_Start(void)
{
#ifdef FATFS_MIDDLEWARE_ENABLED
    if (!s_sdMounted)
    {
        /* Try to mount again */
        if (!SD_Logger_Init())
        {
            Logger_Print("SD card not available - logging to UART only\r\n");
        }
    }

    if (s_sdMounted)
    {
        /* Create/open log file */
        FRESULT res = f_open(&s_logFile, SD_LOG_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);

        if (res != FR_OK)
        {
            Logger_Print("SD file open failed (error %d) - logging to UART only\r\n", res);
            s_sdMounted = 0;
        }
        else
        {
            /* Write CSV header */
            const char *header = "Time_s,Position_um,Temp_C,Voltage_mV,Current_mA,Consumption_mAh,eRPM\r\n";
            UINT bytesWritten;
            f_write(&s_logFile, header, strlen(header), &bytesWritten);
            f_sync(&s_logFile);
            Logger_Print("SD logging started to %s\r\n", SD_LOG_FILENAME);
        }
    }
#endif

    /* Print header to UART as well */
    Logger_Print("\r\n=== Logging Started ===\r\n");
    Logger_Print("Time_s,Position_um,Temp_C,Voltage_mV,Current_mA,Consumption_mAh,eRPM\r\n");

    s_isLogging = 1;
    s_logStartTime = HAL_GetTick();
    s_lastLogTime = s_logStartTime;
}

void SD_Logger_Stop(void)
{
    if (s_isLogging)
    {
#ifdef FATFS_MIDDLEWARE_ENABLED
        if (s_sdMounted)
        {
            f_close(&s_logFile);
            Logger_Print("\r\nSD logging stopped.\r\n");
        }
#endif
        Logger_Print("\r\n=== Logging Stopped ===\r\n");
        s_isLogging = 0;
    }
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

    /* Check if it's time to log */
    if ((now - s_lastLogTime) < LOG_PERIOD_MS)
        return;

    s_lastLogTime = now;

    /* Calculate elapsed time in seconds */
    uint32_t elapsed_ms = now - s_logStartTime;
    float time_s = (float)elapsed_ms / 1000.0f;

    /* Get encoder position */
    int32_t position_um = Encoder_GetPosition_um();

    /* Get telemetry data */
    uint8_t temp_c = ESC_Telem_GetTemp_C();
    uint32_t voltage_mV = ESC_Telem_GetVoltage_mV();
    uint32_t current_mA = ESC_Telem_GetCurrent_mA();
    uint32_t consumption_mAh = ESC_Telem_GetConsumption_mAh();
    uint32_t erpm = ESC_Telem_GetERPM();

    /* Format log line */
    char logLine[128];
    snprintf(logLine, sizeof(logLine),
             "%.3f,%ld,%u,%lu,%lu,%lu,%lu\r\n",
             time_s,
             (long)position_um,
             temp_c,
             (unsigned long)voltage_mV,
             (unsigned long)current_mA,
             (unsigned long)consumption_mAh,
             (unsigned long)erpm);

#ifdef FATFS_MIDDLEWARE_ENABLED
    /* Write to SD card if available */
    if (s_sdMounted)
    {
        UINT bytesWritten;
        f_write(&s_logFile, logLine, strlen(logLine), &bytesWritten);

        /* Sync periodically (every 5 samples) */
        static uint8_t sync_counter = 0;
        if (++sync_counter >= 5)
        {
            f_sync(&s_logFile);
            sync_counter = 0;
        }
    }
#endif

    /* Always write to UART */
    Logger_Print("%s", logLine);
}
