/*
 * encoder_homing.c
 *
 * Encoder homing sequence for the airbrake linear actuator.
 *
 * Overview
 * --------
 * Homing drives the actuator to both mechanical end-stops using motor
 * current (from ESC telemetry on UART5) as the stall detector, then
 * returns to the retracted (zero) position and zeros the encoder there.
 *
 * After homing completes:
 *   - Encoder position 0 = fully retracted end-stop.
 *   - s_rangeUnits = total travel in internal position units.
 *
 * Sequence
 * --------
 *   1. HOME_MOVE_RETRACT  — drive toward retracted end-stop.
 *   2. HOME_WAIT_RETRACT  — motor off, let current fall back to idle.
 *   3. HOME_ZERO          — zero the encoder (retracted end = 0).
 *   4. HOME_MOVE_EXTEND   — drive toward extended end-stop.
 *   5. HOME_WAIT_EXTEND   — motor off, record position as s_rangeUnits.
 *   6. HOME_MOVE_TO_ZERO  — return to position 0.
 *   7. HOME_DONE
 *
 * Stall detection — rolling average
 * ----------------------------------
 * The previous approach (threshold on instantaneous current) triggered on
 * start-up inrush and commutation spikes.  The replacement maintains a
 * circular buffer of the last CURRENT_AVG_SAMPLES telemetry readings and
 * computes a rolling average each task call.
 *
 * A stall is declared when:
 *   1. The rolling average exceeds HOMING_STALL_AVG_MA, AND
 *   2. That condition has been sustained for HOMING_STALL_CONFIRM_MS.
 *
 * The buffer covers CURRENT_AVG_WINDOW_MS (250 ms) at AM32's ~100 Hz
 * telemetry rate, which is approximately 25 samples.  The buffer is sized
 * at 32 slots to stay safely above that.  Duplicate samples between
 * telemetry packets are harmless — they contribute the same value and
 * the average remains correct.
 *
 * The inrush ignore window (HOMING_CURRENT_IGNORE_MS) prevents the motor
 * start-up spike from pre-loading the buffer with high values.  During
 * that window no samples are collected and the average is 0.
 *
 * Diagnostic output
 * -----------------
 * When a stall candidate is first detected and when it is confirmed or
 * cleared, the average and instantaneous current are printed to the
 * console.  This lets you tune HOMING_STALL_AVG_MA from the serial log
 * without needing an oscilloscope.
 */

#include "encoder_homing.h"
#include "encoder_app.h"
#include "esc_telem.h"
#include "main.h"
#include "usart.h"

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

/* =========================================================================
 * External motor-control helpers (defined in esc_app.c)
 * ========================================================================= */
extern void ESC_App_SetPercent(int32_t pct);
extern void ESC_App_StopMotor(void);

/* =========================================================================
 * Tuning parameters
 * ========================================================================= */

/* Motor speed during homing — low enough to stall cleanly without slamming */
#define HOMING_SPEED_PCT            20

/*
 * Rolling-average stall threshold (mA).
 * Set this above the motor's free-running current at HOMING_SPEED_PCT but
 * below its locked-rotor current.  Because transient spikes are averaged
 * out, you can set this lower than you would for an instantaneous threshold
 * without false triggers.  Start at 1800 mA and lower if genuine stalls
 * are not detected; raise if free-running motion trips the detector.
 */
#define HOMING_STALL_AVG_MA         10000U

/*
 * Rolling average window — number of samples in the circular buffer.
 * At ~100 Hz telemetry, 32 samples ≈ 320 ms of history.
 * Only samples collected after the inrush ignore window count, so the
 * effective window at the start of a move is shorter until the buffer fills.
 */
#define CURRENT_AVG_SAMPLES         32U

/*
 * Inrush ignore window (ms).
 * No samples are collected during this period after a move starts.
 * Prevents the start-up current spike from poisoning the rolling average.
 */
#define HOMING_CURRENT_IGNORE_MS    200U

/*
 * Stall confirmation window (ms).
 * The rolling average must remain above HOMING_STALL_AVG_MA continuously
 * for this long before a stall is declared.  Guards against a brief burst
 * of high-current samples pushing the average above the threshold.
 */
#define HOMING_STALL_CONFIRM_MS     150U

/* Settle time after hitting an end-stop before starting the next move (ms) */
#define HOMING_SETTLE_MS            800U

/* Overall per-move timeout (ms) */
#define HOMING_MOVE_TIMEOUT_MS      8000U

/* Return-to-zero tolerance in internal position units (~0.05 mm) */
#define HOMING_ZERO_TOL_UNITS       50000L

/* =========================================================================
 * Rolling current average — circular buffer
 * ========================================================================= */

static uint32_t s_avgBuf[CURRENT_AVG_SAMPLES];
static uint8_t  s_avgHead  = 0;
static uint8_t  s_avgCount = 0;
static uint64_t s_avgSum   = 0;   /* uint64 avoids overflow at ~65 A × 32 samples */

/* Discard all samples — called when entering a move state */
static void CurrentAvg_Reset(void)
{
    memset(s_avgBuf, 0, sizeof(s_avgBuf));
    s_avgHead  = 0;
    s_avgCount = 0;
    s_avgSum   = 0;
}

/* Add one reading; oldest sample is evicted when the buffer is full */
static void CurrentAvg_Push(uint32_t current_mA)
{
    if (s_avgCount == CURRENT_AVG_SAMPLES)
    {
        /* Buffer full — subtract the value about to be overwritten */
        s_avgSum -= s_avgBuf[s_avgHead];
    }
    else
    {
        s_avgCount++;
    }

    s_avgBuf[s_avgHead] = current_mA;
    s_avgSum           += current_mA;
    s_avgHead           = (uint8_t)((s_avgHead + 1U) % CURRENT_AVG_SAMPLES);
}

/* Return the rolling average, or 0 if the buffer is empty */
static uint32_t CurrentAvg_Get(void)
{
    if (s_avgCount == 0U) return 0U;
    return (uint32_t)(s_avgSum / (uint64_t)s_avgCount);
}

/* =========================================================================
 * State machine
 * ========================================================================= */

typedef enum
{
    HOME_IDLE,
    HOME_MOVE_RETRACT,
    HOME_WAIT_RETRACT,
    HOME_ZERO,
    HOME_MOVE_EXTEND,
    HOME_WAIT_EXTEND,
    HOME_MOVE_TO_ZERO,
    HOME_DONE
} HomingState_t;

static HomingState_t s_state         = HOME_IDLE;
static uint32_t      s_stateEnterMs  = 0;
static uint32_t      s_stallOnsetMs  = 0;
static uint8_t       s_stallTracking = 0;
static int32_t       s_rangeUnits    = 0;

/* =========================================================================
 * Private helpers
 * ========================================================================= */

static void Homing_Print(const char *fmt, ...)
{
    char buf[160];
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

static int32_t abs_i32(int32_t x) { return (x < 0) ? -x : x; }

/* Reset all per-state tracking and the current average on every transition */
static void EnterState(HomingState_t next)
{
    s_state         = next;
    s_stateEnterMs  = HAL_GetTick();
    s_stallTracking = 0;
    s_stallOnsetMs  = 0;
    CurrentAvg_Reset();
}

/*
 * StallDetected — call once per Encoder_Homing_Task() iteration.
 *
 * Phase 1 (elapsed < HOMING_CURRENT_IGNORE_MS):
 *   Returns 0 without sampling.  The buffer stays at 0 so the average
 *   cannot accidentally cross the threshold from inrush values.
 *
 * Phase 2 (elapsed >= HOMING_CURRENT_IGNORE_MS):
 *   Pushes the latest ESC_Telem_GetCurrent_mA() reading into the buffer.
 *   Evaluates the rolling average against HOMING_STALL_AVG_MA.
 *   When the average first crosses the threshold, starts a confirmation
 *   timer.  Returns 1 only after the average has been above threshold
 *   continuously for HOMING_STALL_CONFIRM_MS.
 *   If the average drops below threshold, the confirmation resets.
 */
static uint8_t StallDetected(uint32_t now)
{
    /* Phase 1: inrush ignore */
    if ((now - s_stateEnterMs) < HOMING_CURRENT_IGNORE_MS)
        return 0U;

    /* Phase 2: sample and evaluate */
    uint32_t instant_mA = ESC_Telem_GetCurrent_mA();
    CurrentAvg_Push(instant_mA);
    uint32_t avg_mA = CurrentAvg_Get();

    if (avg_mA > HOMING_STALL_AVG_MA)
    {
        if (!s_stallTracking)
        {
            /* Average just crossed the threshold — start confirmation clock */
            s_stallTracking = 1;
            s_stallOnsetMs  = now;
            Homing_Print("Stall candidate: avg=%lu mA (limit=%u mA) "
                         "— confirming...\r\n",
                         (unsigned long)avg_mA, HOMING_STALL_AVG_MA);
        }
        else if ((now - s_stallOnsetMs) >= HOMING_STALL_CONFIRM_MS)
        {
            /* Sustained — genuine stall */
            Homing_Print("Stall confirmed: avg=%lu mA, instant=%lu mA\r\n",
                         (unsigned long)avg_mA, (unsigned long)instant_mA);
            return 1U;
        }
    }
    else
    {
        if (s_stallTracking)
        {
            /* Average fell back — was a transient, not a stall */
            Homing_Print("Stall candidate cleared: avg=%lu mA "
                         "dropped below limit.\r\n",
                         (unsigned long)avg_mA);
        }
        s_stallTracking = 0;
    }

    return 0U;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void Encoder_Homing_Init(void)
{
    ESC_App_StopMotor();
    CurrentAvg_Reset();
    s_state      = HOME_IDLE;
    s_rangeUnits = 0;
}

void Encoder_Homing_Start(void)
{
    if (s_state == HOME_IDLE || s_state == HOME_DONE)
    {
        Homing_Print("\r\n=== Homing Sequence Started ===\r\n");
        Homing_Print("Stall detection: %u-sample rolling average > %u mA "
                     "for %u ms\r\n",
                     CURRENT_AVG_SAMPLES,
                     HOMING_STALL_AVG_MA,
                     HOMING_STALL_CONFIRM_MS);
        Homing_Print("Inrush ignore window: %u ms\r\n\r\n",
                     HOMING_CURRENT_IGNORE_MS);
        Homing_Print("Step 1/4: Moving to retracted end-stop...\r\n");
        ESC_App_StopMotor();
        EnterState(HOME_MOVE_RETRACT);
    }
}

uint8_t Encoder_Homing_Is_Active(void)
{
    return (s_state != HOME_IDLE && s_state != HOME_DONE);
}

int32_t Encoder_Homing_Get_Range_Units(void)
{
    return s_rangeUnits;
}

void Encoder_Homing_Task(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - s_stateEnterMs;
    int32_t  pos     = Encoder_GetPosition_um();

    switch (s_state)
    {
        case HOME_IDLE:
        case HOME_DONE:
            break;

        /* ------------------------------------------------------------------ */
        case HOME_MOVE_RETRACT:
        {
            ESC_App_SetPercent(-HOMING_SPEED_PCT);

            if (StallDetected(now) || elapsed >= HOMING_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                if (elapsed >= HOMING_MOVE_TIMEOUT_MS)
                    Homing_Print("Retract timed out — using current position.\r\n");
                else
                    Homing_Print("Retracted end-stop found.\r\n");
                Homing_Print("Step 2/4: Settling...\r\n");
                EnterState(HOME_WAIT_RETRACT);
            }
            break;
        }

        /* ------------------------------------------------------------------ */
        case HOME_WAIT_RETRACT:
            ESC_App_StopMotor();
            if (elapsed >= HOMING_SETTLE_MS)
                EnterState(HOME_ZERO);
            break;

        /* ------------------------------------------------------------------ */
        case HOME_ZERO:
            Encoder_Reset();
            Homing_Print("Encoder zeroed at retracted end-stop. Position = 0.\r\n");
            Homing_Print("Step 3/4: Moving to extended end-stop...\r\n");
            EnterState(HOME_MOVE_EXTEND);
            break;

        /* ------------------------------------------------------------------ */
        case HOME_MOVE_EXTEND:
        {
            ESC_App_SetPercent(+HOMING_SPEED_PCT);

            if (StallDetected(now) || elapsed >= HOMING_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                s_rangeUnits = Encoder_GetPosition_um();

                if (elapsed >= HOMING_MOVE_TIMEOUT_MS)
                    Homing_Print("Extend timed out — using current position.\r\n");
                else
                    Homing_Print("Extended end-stop found.\r\n");

                Homing_Print("Full travel = %ld units (%.3f mm)\r\n",
                             (long)s_rangeUnits,
                             (double)s_rangeUnits / 1000000.0);
                Homing_Print("Step 4/4: Settling at extended end-stop...\r\n");
                EnterState(HOME_WAIT_EXTEND);
            }
            break;
        }

        /* ------------------------------------------------------------------ */
        case HOME_WAIT_EXTEND:
            ESC_App_StopMotor();
            if (elapsed >= HOMING_SETTLE_MS)
            {
                Homing_Print("Returning to retracted position (0)...\r\n");
                EnterState(HOME_MOVE_TO_ZERO);
            }
            break;

        /* ------------------------------------------------------------------ */
        case HOME_MOVE_TO_ZERO:
        {
            int32_t err = -pos;   /* target is position 0 */

            if (abs_i32(err) <= HOMING_ZERO_TOL_UNITS)
            {
                ESC_App_StopMotor();
                Homing_Print("=== Homing Complete ===\r\n");
                Homing_Print("  Retracted : 0 units (0.000 mm)\r\n");
                Homing_Print("  Extended  : %ld units (%.3f mm)\r\n",
                             (long)s_rangeUnits,
                             (double)s_rangeUnits / 1000000.0);
                Homing_Print("  Final pos : %ld units\r\n\r\n", (long)pos);
                EnterState(HOME_DONE);
            }
            else if (elapsed >= HOMING_MOVE_TIMEOUT_MS)
            {
                ESC_App_StopMotor();
                Homing_Print("Return-to-zero timed out at %ld units.\r\n\r\n",
                             (long)pos);
                EnterState(HOME_DONE);
            }
            else
            {
                ESC_App_SetPercent((err > 0) ? +HOMING_SPEED_PCT : -HOMING_SPEED_PCT);
            }
            break;
        }

        default:
            ESC_App_StopMotor();
            EnterState(HOME_DONE);
            break;
    }
}
