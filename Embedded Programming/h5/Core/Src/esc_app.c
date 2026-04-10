/*
 * esc_app.c
 *
 * AM32 bidirectional ESC controller application layer.
 *
 * Arming strategy:
 *   AM32 in bidirectional mode arms on a stable neutral (1500 µs) signal.
 *   PWM is started immediately at neutral on init, then held there for
 *   ESC_ARM_HOLD_MS to give the ESC time to power on, boot its firmware,
 *   and recognise the neutral signal before any motion commands are sent.
 *
 * Pin assignments (for reference):
 *   PB1      TIM3_CH4   PWM signal to ESC
 *   PA9/PA10 USART1     Console TX/RX (primary)
 *   PA2/PA3  USART2     Console TX/RX (secondary / debug)
 *   PA0/PA1  UART4      Passthrough / external console
 *   PB5/PB6  UART5      ESC serial telemetry RX (AM32 auto-telem)
 */

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

/* =========================================================================
 * Constants
 * ========================================================================= */

#define PWM_MIN_US          1000
#define PWM_MAX_US          2000
#define PWM_NEUTRAL_US      1500

#define NEUTRAL_MIN_US      1300
#define NEUTRAL_MAX_US      1700

#define CREEP_OFFSET_US     80

/*
 * Time to hold neutral PWM after startup before allowing motion commands.
 * Covers the ESC's full boot sequence + AM32 neutral-detection window.
 */
#define ESC_ARM_HOLD_MS     5000U

#define RX_BUF_LEN          64U

/* =========================================================================
 * Static state
 * ========================================================================= */

static uint8_t s_rx1Byte;
static uint8_t s_rx2Byte;
static uint8_t s_rx4Byte;

static char              s_rxLine1[RX_BUF_LEN];
static char              s_rxLine2[RX_BUF_LEN];
static volatile uint32_t s_rxIdx1 = 0U;
static volatile uint32_t s_rxIdx2 = 0U;

static int32_t s_neutralUs      = PWM_NEUTRAL_US;
static int32_t s_currentPulseUs = PWM_NEUTRAL_US;

/* =========================================================================
 * Private helpers — arithmetic
 * ========================================================================= */

static int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t map_i32(int32_t x,
                       int32_t in_min,  int32_t in_max,
                       int32_t out_min, int32_t out_max)
{
    return (int32_t)((x - in_min) * (out_max - out_min)
                     / (in_max - in_min) + out_min);
}

static bool is_signed_integer(const char *s)
{
    if (!s || *s == '\0') return false;
    if (*s == '+' || *s == '-') { s++; if (*s == '\0') return false; }
    while (*s) { if (!isdigit((unsigned char)*s)) return false; s++; }
    return true;
}

static void trim_whitespace(char *s)
{
    char *start = s;
    while (*start && isspace((unsigned char)*start)) start++;
    if (start != s) memmove(s, start, strlen(start) + 1);

    size_t len = strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1]))
        s[--len] = '\0';
}

static void str_to_upper(char *s)
{
    while (*s) { *s = (char)toupper((unsigned char)*s); s++; }
}

/* =========================================================================
 * Private helpers — output
 * ========================================================================= */

static void App_Print(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > (int)sizeof(buf)) len = (int)sizeof(buf);

    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart4, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
}

/* =========================================================================
 * Private helpers — PWM
 * ========================================================================= */

static void PWM_Set(int32_t pulseUs)
{
    pulseUs = clamp_i32(pulseUs, PWM_MIN_US, PWM_MAX_US);
    s_currentPulseUs = pulseUs;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)pulseUs);
}

/* =========================================================================
 * Private helpers — UART RX
 * ========================================================================= */

static void StartUartRxITs(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx1Byte, 1);
    HAL_UART_Receive_IT(&huart2, &s_rx2Byte, 1);
    HAL_UART_Receive_IT(&huart4, &s_rx4Byte, 1);
}

/* =========================================================================
 * Private helpers — command dispatch
 * ========================================================================= */

/*
 * HandleCommand — dispatches a complete newline-terminated command line.
 *
 * This is the primary dispatch path when using the Python console tool,
 * which always terminates commands with \n.  Every command is handled here,
 * including H (homing) and 67 (deploy) which are also detectable mid-stream
 * via Command_Sequence_Process_Char().
 *
 * Handling H and 67 here directly means the sequences start immediately
 * when the line is received, without waiting for the next ESC_App_Task()
 * iteration to poll the trigger flags.
 */
static void HandleCommand(char *cmd)
{
    trim_whitespace(cmd);
    if (cmd[0] == '\0') return;

    char upper[RX_BUF_LEN];
    strncpy(upper, cmd, sizeof(upper) - 1);
    upper[sizeof(upper) - 1] = '\0';
    str_to_upper(upper);

    /* ── H — start homing sequence ──────────────────────────────────────── */
    if (strcmp(upper, "H") == 0)
    {
        if (Airbrake_Is_Sequence_Active())
        {
            App_Print("Cannot home: deployment sequence is active.\r\n");
            return;
        }
        if (Encoder_Homing_Is_Active())
        {
            App_Print("Homing already in progress.\r\n");
            return;
        }
        App_Print("\r\n*** HOMING SEQUENCE INITIATED ***\r\n");
        Encoder_Homing_Start();
        return;
    }

    /* ── 67 — start deployment sequence ─────────────────────────────────── */
    if (strcmp(upper, "67") == 0)
    {
        if (Encoder_Homing_Is_Active())
        {
            App_Print("Cannot deploy: homing sequence is active.\r\n");
            return;
        }
        if (Airbrake_Is_Sequence_Active())
        {
            App_Print("Deployment already in progress.\r\n");
            return;
        }
        App_Print("\r\n*** DEPLOYMENT SEQUENCE INITIATED ***\r\n");
        SD_Logger_Start();
        Airbrake_Start_Sequence();
        return;
    }

    /* ── All other commands: block during active sequences ──────────────── */
    if (Airbrake_Is_Sequence_Active() || Encoder_Homing_Is_Active())
    {
        App_Print("Sequence active — motion commands ignored.\r\n");
        return;
    }

    /* ── NEUTRAL <us> ───────────────────────────────────────────────────── */
    if (strncmp(upper, "NEUTRAL", 7) == 0)
    {
        const char *val = cmd + 7;
        while (*val == ' ') val++;
        if (is_signed_integer(val))
        {
            s_neutralUs = clamp_i32((int32_t)strtol(val, NULL, 10),
                                    NEUTRAL_MIN_US, NEUTRAL_MAX_US);
            App_Print("NEUTRAL set to %ld us\r\n", (long)s_neutralUs);
            PWM_Set(s_neutralUs);
            return;
        }
    }

    /* ── O — zero encoder ───────────────────────────────────────────────── */
    if (strcmp(upper, "O") == 0)
    {
        Encoder_Reset();
        App_Print("Encoder position set to 0\r\n");
        return;
    }

    /* ── N<us> — direct pulse width ─────────────────────────────────────── */
    if (upper[0] == 'N' && is_signed_integer(upper + 1))
    {
        int32_t pulse = clamp_i32((int32_t)strtol(cmd + 1, NULL, 10),
                                  PWM_MIN_US, PWM_MAX_US);
        App_Print("PULSE %ld us\r\n", (long)pulse);
        PWM_Set(pulse);
        return;
    }

    /* ── F — slow forward jog ───────────────────────────────────────────── */
    if (strcmp(upper, "F") == 0)
    {
        int32_t pulse = s_neutralUs + CREEP_OFFSET_US;
        App_Print("JOG forward (%ld us)\r\n", (long)pulse);
        PWM_Set(pulse);
        return;
    }

    /* ── B — slow reverse jog ───────────────────────────────────────────── */
    if (strcmp(upper, "B") == 0)
    {
        int32_t pulse = s_neutralUs - CREEP_OFFSET_US;
        App_Print("JOG reverse (%ld us)\r\n", (long)pulse);
        PWM_Set(pulse);
        return;
    }

    /* ── S / STOP ───────────────────────────────────────────────────────── */
    if (strcmp(upper, "S") == 0 || strcmp(upper, "STOP") == 0)
    {
        App_Print("STOP (neutral = %ld us)\r\n", (long)s_neutralUs);
        PWM_Set(s_neutralUs);
        return;
    }

    /* ── -100..100 — percentage throttle ────────────────────────────────── */
    if (is_signed_integer(upper))
    {
        int32_t pct = clamp_i32((int32_t)strtol(upper, NULL, 10), -100, 100);
        int32_t pulse = (pct >= 0)
            ? map_i32(pct,  0, 100, s_neutralUs, PWM_MAX_US)
            : map_i32(pct, -100, 0, PWM_MIN_US,  s_neutralUs);
        App_Print("THROTTLE %ld%% -> %ld us\r\n", (long)pct, (long)pulse);
        PWM_Set(pulse);
        return;
    }

    /* ── Unknown ────────────────────────────────────────────────────────── */
    App_Print("Unknown command. Available commands:\r\n");
    App_Print("  F              slow forward jog\r\n");
    App_Print("  B              slow reverse jog\r\n");
    App_Print("  S / STOP       stop motor\r\n");
    App_Print("  -100..100      percentage throttle\r\n");
    App_Print("  N<us>          direct pulse width (e.g. N1500)\r\n");
    App_Print("  NEUTRAL <us>   set neutral trim (%d-%d us)\r\n",
              NEUTRAL_MIN_US, NEUTRAL_MAX_US);
    App_Print("  O              zero encoder position\r\n");
    App_Print("  67             start deployment sequence + SD logging\r\n");
    App_Print("  H              start encoder homing sequence\r\n");
}

/*
 * ProcessConsoleByte — accumulates characters and dispatches on CR/LF.
 *
 * Each character is also fed to Command_Sequence_Process_Char() so that
 * H and 67 work even without a newline terminator (e.g. direct terminal).
 * When using the Python tool, the \n it appends will trigger HandleCommand()
 * which dispatches immediately — the sequence trigger flags are a fallback.
 */
static void ProcessConsoleByte(char c, char *buf, volatile uint32_t *idx)
{
    /* Feed every character to the mid-stream sequence detector */
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
        if (*idx < (RX_BUF_LEN - 1U))
            buf[(*idx)++] = c;
        else
            *idx = 0U;
    }
}

/* =========================================================================
 * Public API — motor control (called by sequence modules)
 * ========================================================================= */

void ESC_App_SetPulseUs(int32_t pulseUs)
{
    PWM_Set(pulseUs);
}

void ESC_App_StopMotor(void)
{
    PWM_Set(s_neutralUs);
}

void ESC_App_SetPercent(int32_t pct)
{
    pct = clamp_i32(pct, -100, 100);
    int32_t pulse = (pct >= 0)
        ? map_i32(pct,  0, 100, s_neutralUs, PWM_MAX_US)
        : map_i32(pct, -100, 0, PWM_MIN_US,  s_neutralUs);
    PWM_Set(pulse);
}

int32_t ESC_App_GetNeutralUs(void)
{
    return s_neutralUs;
}

/* =========================================================================
 * Public API — init / task
 * ========================================================================= */

void ESC_App_Init(void)
{
    /* ── Step 1: PWM neutral immediately ────────────────────────────────── */
    s_currentPulseUs = PWM_NEUTRAL_US;
    s_neutralUs      = PWM_NEUTRAL_US;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    PWM_Set(PWM_NEUTRAL_US);

    /* ── Step 2: UARTs + telemetry ──────────────────────────────────────── */
    StartUartRxITs();
    ESC_Telem_Init();

    /* ── Step 3: Banner ─────────────────────────────────────────────────── */
    App_Print("\r\n");
    App_Print("=========================================\r\n");
    App_Print("  AM32 Bidirectional ESC Controller\r\n");
    App_Print("=========================================\r\n");
    App_Print("PWM    : PB1 (TIM3_CH4), neutral = %ld us\r\n", (long)s_neutralUs);
    App_Print("Console: USART1 (PA9/PA10), USART2 (PA2/PA3)\r\n");
    App_Print("Telem  : UART5 RX (PB5)\r\n\r\n");

    /* ── Step 4: ESC arm hold ───────────────────────────────────────────── */
    App_Print("Waiting for ESC to power on and arm (%lu s)...",
              (unsigned long)(ESC_ARM_HOLD_MS / 1000U));

    uint32_t arm_start   = HAL_GetTick();
    uint32_t last_dot_at = 0U;

    while ((HAL_GetTick() - arm_start) < ESC_ARM_HOLD_MS)
    {
        uint32_t elapsed = HAL_GetTick() - arm_start;
        if (elapsed - last_dot_at >= 1000U)
        {
            App_Print(".");
            last_dot_at += 1000U;
        }
        HAL_Delay(50U);
    }
    App_Print("\r\nESC armed and ready.\r\n\r\n");

    /* ── Step 5: Application modules ────────────────────────────────────── */
    Command_Sequence_Init();
    Airbrake_Deploy_Init();
    Encoder_Homing_Init();
    Encoder_App_Init();

    if (SD_Logger_Init())
        App_Print("SD card mounted.\r\n");
    else
        App_Print("SD card not available — logging to UART only.\r\n");

    App_Print("\r\nCommands: F  B  S  -100..100  N<us>  NEUTRAL <us>  O  67  H\r\n\r\n");
}

void ESC_App_Task(void)
{
    /* Service all background state machines every loop iteration */
    ESC_Telem_Task();
    Encoder_App_Task();
    Encoder_Homing_Task();
    Airbrake_Deploy_Task();
    SD_Logger_Task();

    /*
     * Check mid-stream trigger flags (set when H or 67 arrive without \n).
     * When the Python tool sends a full line, HandleCommand() has already
     * acted and these flags will be clear — the checks below are a no-op.
     */
    if (Command_Sequence_Check_Deploy_Trigger())
    {
        Command_Sequence_Clear_Deploy_Trigger();
        if (!Encoder_Homing_Is_Active() && !Airbrake_Is_Sequence_Active())
        {
            App_Print("\r\n*** DEPLOYMENT SEQUENCE INITIATED ***\r\n");
            SD_Logger_Start();
            Airbrake_Start_Sequence();
        }
    }

    if (Command_Sequence_Check_Homing_Trigger())
    {
        Command_Sequence_Clear_Homing_Trigger();
        if (!Airbrake_Is_Sequence_Active() && !Encoder_Homing_Is_Active())
        {
            App_Print("\r\n*** HOMING SEQUENCE INITIATED ***\r\n");
            Encoder_Homing_Start();
        }
    }
}

/* =========================================================================
 * UART interrupt callbacks
 * ========================================================================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        HAL_UART_Transmit(&huart1, &s_rx4Byte, 1, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart2, &s_rx4Byte, 1, HAL_MAX_DELAY);
        HAL_UART_Receive_IT(&huart4, &s_rx4Byte, 1);
        return;
    }

    if (huart->Instance == UART5)
    {
        ESC_Telem_RxCpltCallback();
        return;
    }

    if (huart->Instance == USART1)
    {
        HAL_UART_Transmit(&huart1, &s_rx1Byte, 1, HAL_MAX_DELAY);
        ProcessConsoleByte((char)s_rx1Byte, s_rxLine1, &s_rxIdx1);
        HAL_UART_Receive_IT(&huart1, &s_rx1Byte, 1);
        return;
    }

    if (huart->Instance == USART2)
    {
        ProcessConsoleByte((char)s_rx2Byte, s_rxLine2, &s_rxIdx2);
        HAL_UART_Receive_IT(&huart2, &s_rx2Byte, 1);
        return;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        HAL_UART_Receive_IT(&huart1, &s_rx1Byte, 1);
    else if (huart->Instance == USART2)
        HAL_UART_Receive_IT(&huart2, &s_rx2Byte, 1);
    else if (huart->Instance == UART4)
        HAL_UART_Receive_IT(&huart4, &s_rx4Byte, 1);
    else if (huart->Instance == UART5)
        ESC_Telem_Init();
}
