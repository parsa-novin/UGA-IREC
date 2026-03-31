#include "esc_app.h"

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "esc_telem.h"
#include "command_sequence.h"
#include "sd_logger.h"
#include "airbrake_deploy.h"
#include "encoder_homing.h"
#include "encoder_app.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t usart1RxByte;
static uint8_t usart2RxByte;
static uint8_t uart4RxByte;

static char rxLine1[64];
static char rxLine2[64];
static volatile uint32_t rxIndex1 = 0;
static volatile uint32_t rxIndex2 = 0;

/* PWM limits */
static const int32_t PWM_MIN_US = 1000;
static const int32_t PWM_MAX_US = 2000;

/* Neutral trim */
static int32_t neutralUs = 1500;
static int32_t currentPulseUs = 1500;

static const int32_t NEUTRAL_MIN_US = 1300;
static const int32_t NEUTRAL_MAX_US = 1700;

/* Small forward/reverse creep for F/B commands */
static const int32_t CREEP_OFFSET_US = 80;

/* ESC arming sequence */
static const uint32_t ARM_HIGH_MS    = 2000;
static const uint32_t ARM_LOW_MS     = 2000;
static const uint32_t ARM_NEUTRAL_MS = 1500;

static void StartUartRxITs(void)
{
    HAL_UART_Receive_IT(&huart1, &usart1RxByte, 1);
    HAL_UART_Receive_IT(&huart2, &usart2RxByte, 1);
    HAL_UART_Receive_IT(&huart4, &uart4RxByte, 1);
}

static void App_Print(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
        return;

    if (len > (int)sizeof(buf))
        len = (int)sizeof(buf);

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

static int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t map_i32(int32_t x, int32_t in_min, int32_t in_max,
                       int32_t out_min, int32_t out_max)
{
    return (int32_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

static bool is_signed_integer(const char *s)
{
    if (s == NULL || *s == '\0')
        return false;

    if (*s == '+' || *s == '-')
    {
        s++;
        if (*s == '\0')
            return false;
    }

    while (*s)
    {
        if (!isdigit((unsigned char)*s))
            return false;
        s++;
    }

    return true;
}

static void trim_whitespace(char *s)
{
    char *start = s;
    while (*start && isspace((unsigned char)*start))
        start++;

    if (start != s)
        memmove(s, start, strlen(start) + 1);

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1]))
    {
        s[len - 1] = '\0';
        len--;
    }
}

static void strtoupper_inplace(char *s)
{
    while (*s)
    {
        *s = (char)toupper((unsigned char)*s);
        s++;
    }
}

static void PWM_SetMicroseconds(int32_t pulseUs)
{
    pulseUs = clamp_i32(pulseUs, PWM_MIN_US, PWM_MAX_US);
    currentPulseUs = pulseUs;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)pulseUs);
}

static void ApplyOutput(void)
{
    PWM_SetMicroseconds(currentPulseUs);
}

/* ------------------------------------------------------------------------- */
/* Exported helpers for sequence modules                                     */
/* ------------------------------------------------------------------------- */
void ESC_App_SetPulseUs(int32_t pulseUs)
{
    currentPulseUs = clamp_i32(pulseUs, PWM_MIN_US, PWM_MAX_US);
    ApplyOutput();
}

void ESC_App_StopMotor(void)
{
    currentPulseUs = neutralUs;
    ApplyOutput();
}

void ESC_App_SetPercent(int32_t pct)
{
    pct = clamp_i32(pct, -100, 100);

    if (pct >= 0)
        currentPulseUs = map_i32(pct, 0, 100, neutralUs, PWM_MAX_US);
    else
        currentPulseUs = map_i32(pct, -100, 0, PWM_MIN_US, neutralUs);

    ApplyOutput();
}

int32_t ESC_App_GetNeutralUs(void)
{
    return neutralUs;
}

static void HoldPulse(int32_t pulseUs, uint32_t durationMs, const char *label)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t lastDot = 0;

    App_Print("%s", label);
    PWM_SetMicroseconds(pulseUs);

    while ((HAL_GetTick() - t0) < durationMs)
    {
        PWM_SetMicroseconds(pulseUs);
        HAL_Delay(20);

        if ((HAL_GetTick() - t0) - lastDot >= 500U)
        {
            App_Print(".");
            lastDot += 500U;
        }
    }

    App_Print("\r\n");
}

static void HandleCommand(char *cmd)
{
    trim_whitespace(cmd);

    if (cmd[0] == '\0')
        return;

    char upperCmd[64];
    strncpy(upperCmd, cmd, sizeof(upperCmd) - 1);
    upperCmd[sizeof(upperCmd) - 1] = '\0';
    strtoupper_inplace(upperCmd);

    if (strncmp(upperCmd, "NEUTRAL", 7) == 0)
    {
        char *valStr = cmd + 7;
        while (*valStr == ' ')
            valStr++;

        if (is_signed_integer(valStr))
        {
            int32_t requested = (int32_t)strtol(valStr, NULL, 10);
            neutralUs = clamp_i32(requested, NEUTRAL_MIN_US, NEUTRAL_MAX_US);
            currentPulseUs = neutralUs;

            App_Print("NEUTRAL set to %ld us\r\n", (long)neutralUs);
            ApplyOutput();
            return;
        }
    }

    if (strcmp(upperCmd, "O") == 0)
    {
        Encoder_Reset();
        App_Print("Encoder position set to 0\r\n");
        return;
    }

    if (upperCmd[0] == 'N')
    {
        char *valStr = cmd + 1;
        while (*valStr == ' ')
            valStr++;

        if (is_signed_integer(valStr))
        {
            int32_t pulse = clamp_i32((int32_t)strtol(valStr, NULL, 10),
                                      PWM_MIN_US, PWM_MAX_US);
            currentPulseUs = pulse;
            App_Print("PULSE %ld us\r\n", (long)pulse);
            ApplyOutput();
            return;
        }
    }

    if (strcmp(upperCmd, "F") == 0)
    {
        currentPulseUs = neutralUs + CREEP_OFFSET_US;
        App_Print("CMD: slow forward (%ld us)\r\n", (long)currentPulseUs);
        ApplyOutput();
        return;
    }

    if (strcmp(upperCmd, "B") == 0)
    {
        currentPulseUs = neutralUs - CREEP_OFFSET_US;
        App_Print("CMD: slow reverse (%ld us)\r\n", (long)currentPulseUs);
        ApplyOutput();
        return;
    }

    if (strcmp(upperCmd, "S") == 0 || strcmp(upperCmd, "STOP") == 0)
    {
        currentPulseUs = neutralUs;
        App_Print("CMD: stop (neutral=%ld us)\r\n", (long)neutralUs);
        ApplyOutput();
        return;
    }

    if (is_signed_integer(upperCmd))
    {
        int32_t pct = clamp_i32((int32_t)strtol(upperCmd, NULL, 10), -100, 100);

        if (pct >= 0)
            currentPulseUs = map_i32(pct, 0, 100, neutralUs, PWM_MAX_US);
        else
            currentPulseUs = map_i32(pct, -100, 0, PWM_MIN_US, neutralUs);

        App_Print("CMD: %ld%% -> %ld us\r\n", (long)pct, (long)currentPulseUs);
        ApplyOutput();
        return;
    }

    App_Print("Unknown command\r\n");
    App_Print("  F            = slow forward\r\n");
    App_Print("  B            = slow reverse\r\n");
    App_Print("  S / STOP     = stop at current neutral\r\n");
    App_Print("  -100..100    = percentage throttle\r\n");
    App_Print("  N1500        = direct pulse width in us\r\n");
    App_Print("  NEUTRAL <us> = set neutral trim\r\n");
    App_Print("  67           = start deployment sequence + logging\r\n");
    App_Print("  H            = home encoder (find min/max)\r\n");
}

static void ProcessConsoleByte(char c, char *buf, volatile uint32_t *idx)
{
    /* Check if deployment or homing is active - ignore input during sequence */
    if (Airbrake_Is_Sequence_Active() || Encoder_Homing_Is_Active())
    {
        return;
    }

    /* Feed character to sequence detector */
    Command_Sequence_Process_Char(c);

    if (c == '\r' || c == '\n')
    {
        if (*idx > 0U)
        {
            buf[*idx] = '\0';
            HandleCommand(buf);
            *idx = 0U;
        }
    }
    else
    {
        if (*idx < 63U)
        {
            buf[(*idx)++] = c;
        }
        else
        {
            *idx = 0U;
        }
    }
}

void ESC_App_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    StartUartRxITs();
    ESC_Telem_Init();

    App_Print("\r\n");
    App_Print("AM32 bidirectional ESC controller\r\n");
    App_Print("=================================\r\n");
    App_Print("PB1 = TIM3_CH4 PWM\r\n");
    App_Print("PA9 = USART1_TX\r\n");
    App_Print("PA10 = USART1_RX\r\n");
    App_Print("PA2 = USART2_TX\r\n");
    App_Print("PA3 = USART2_RX\r\n");
    App_Print("PB15 = UART5_RX telemetry\r\n");
    App_Print("Starting neutral: %ld us\r\n\r\n", (long)neutralUs);

    HoldPulse(PWM_MAX_US, ARM_HIGH_MS,    "Arming: HIGH pulse ");
    HoldPulse(PWM_MIN_US, ARM_LOW_MS,     "Arming: LOW  pulse ");
    HoldPulse(neutralUs,  ARM_NEUTRAL_MS, "Arming: NEUTRAL    ");

    currentPulseUs = neutralUs;
    ApplyOutput();

    App_Print("\r\nESC armed and ready.\r\n");
    App_Print("USART1 and USART2 now share console TX/RX.\r\n");
    App_Print("Commands on either: F  B  S  -100..100  N1500  NEUTRAL <us>\r\n");
    App_Print("Special: 67 = start deployment + logging, H = home encoder\r\n\r\n");

    Command_Sequence_Init();
    Airbrake_Deploy_Init();
    Encoder_Homing_Init();

    if (SD_Logger_Init())
    {
        App_Print("SD card mounted successfully.\r\n");
    }
    else
    {
        App_Print("SD card mount failed - logging disabled.\r\n");
    }
}

void ESC_App_Task(void)
{
    /* Keep all background/state-machine tasks serviced every loop */
    ESC_Telem_Task();
    Encoder_Homing_Task();
    Airbrake_Deploy_Task();
    SD_Logger_Task();

    if (Command_Sequence_Check_Deploy_Trigger())
    {
        Command_Sequence_Clear_Deploy_Trigger();

        App_Print("\r\n*** DEPLOYMENT SEQUENCE INITIATED ***\r\n");
        App_Print("Starting SD logging at 50 Hz...\r\n\r\n");

        SD_Logger_Start();
        Airbrake_Start_Sequence();
    }

    if (Command_Sequence_Check_Homing_Trigger())
    {
        Command_Sequence_Clear_Homing_Trigger();

        App_Print("\r\n*** HOMING SEQUENCE INITIATED ***\r\n");
        Encoder_Homing_Start();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        HAL_UART_Transmit(&huart2, &uart4RxByte, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, &uart4RxByte, 1, HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart4, &uart4RxByte, 1);
        return;
    }

    if (huart->Instance == UART5)
    {
        ESC_Telem_RxCpltCallback();
        return;
    }

    if (huart->Instance == USART2)
    {
        ProcessConsoleByte((char)usart2RxByte, rxLine2, &rxIndex2);
        HAL_UART_Receive_IT(&huart2, &usart2RxByte, 1);
        return;
    }

    if (huart->Instance == USART1)
    {
        ProcessConsoleByte((char)usart1RxByte, rxLine1, &rxIndex1);
        HAL_UART_Receive_IT(&huart1, &usart1RxByte, 1);
        return;
    }
}
