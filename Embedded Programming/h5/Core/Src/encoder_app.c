#include "encoder_app.h"

#include "main.h"
#include "tim.h"
#include "usart.h"

#include <stdint.h>
/* -------------------------------------------------------------------------
 * Pin mapping
 * ------------------------------------------------------------------------- */
#define HALL_A_PORT      GPIOA
#define HALL_A_PIN       GPIO_PIN_7   /* bit 2 in Hall state byte */
#define HALL_A_BIT       2U

#define HALL_B_PORT      GPIOA
#define HALL_B_PIN       GPIO_PIN_6   /* bit 1 in Hall state byte */
#define HALL_B_BIT       1U

#define HALL_C_PORT      GPIOA
#define HALL_C_PIN       GPIO_PIN_4   /* bit 0 in Hall state byte */
#define HALL_C_BIT       0U

/* -------------------------------------------------------------------------
 * Mechanical constants - adjust to match your motor/leadscrew
 *   COUNTS_PER_REV : Hall steps per full mechanical revolution
 *                    = 6 * number_of_pole_pairs
 *   UM_PER_REV     : Linear travel per revolution in micrometres
 *                    1000 um = 1 mm, 1000000 um = 1 m
 *   ENCODER_DIR_SIGN: +1 or -1, flip if position counts backwards
 * ------------------------------------------------------------------------- */
#define COUNTS_PER_REV    42
#define UM_PER_REV        1000L
#define ENCODER_DIR_SIGN  1

/* Debug print cadence */
#define DEBUG_PRINT_MS    200U

/* -------------------------------------------------------------------------
 * Hall commutation sequence (Gray code order for forward rotation)
 *
 *   Index : 0     1     2     3     4     5
 *   State : 001   101   100   110   010   011
 *
 * Forward step  -> index increases mod 6
 * Reverse step  -> index decreases mod 6
 * ------------------------------------------------------------------------- */
static const uint8_t k_hallSequence[6] = {
    0b001, 0b101, 0b100, 0b110, 0b010, 0b011
};

/* -------------------------------------------------------------------------
 * State
 * s_count and s_hallState are written from EXTI ISR context and read from
 * the main loop. Declared volatile; int32_t/uint8_t writes on Cortex-M33
 * are naturally atomic for aligned accesses, so no critical section needed.
 * ------------------------------------------------------------------------- */
static int32_t  s_count              = 0;
static uint8_t  s_hallState          = 0;
static uint32_t s_pollHits           = 0;
static uint32_t s_invalidTransitions = 0;
static uint32_t s_lastPrintMs        = 0;

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

static void Encoder_Print(const char *fmt, ...)
{
    (void)fmt;
}

static uint8_t ReadHallState(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(HALL_A_PORT, HALL_A_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t b = (HAL_GPIO_ReadPin(HALL_B_PORT, HALL_B_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t c = (HAL_GPIO_ReadPin(HALL_C_PORT, HALL_C_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    return (uint8_t)((a << HALL_A_BIT) | (b << HALL_B_BIT) | (c << HALL_C_BIT));
}

static int8_t HallIndex(uint8_t state)
{
    for (int8_t i = 0; i < 6; i++)
    {
        if (k_hallSequence[i] == state)
            return i;
    }
    return -1;
}

static int8_t HallStepDelta(uint8_t prev, uint8_t curr)
{
    int8_t ip = HallIndex(prev);
    int8_t ic = HallIndex(curr);

    if (ip < 0 || ic < 0 || prev == curr)
        return 0;

    if (((ip + 1) % 6) == ic)
        return +1;
    if (((ip + 5) % 6) == ic)
        return -1;

    return 0; /* skipped step - missed a transition */
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

void Encoder_App_Init(void)
{
    s_count              = 0;
    s_invalidTransitions = 0;
    s_pollHits           = 0;
    s_hallState          = ReadHallState();
    s_lastPrintMs        = HAL_GetTick();

    Encoder_Print(
        "Encoder init | hall=%u%u%u (0x%X) | seq_idx=%d\r\n",
        (s_hallState >> HALL_A_BIT) & 1U,
        (s_hallState >> HALL_B_BIT) & 1U,
        (s_hallState >> HALL_C_BIT) & 1U,
        s_hallState,
        (int)HallIndex(s_hallState)
    );
}

void Encoder_Reset(void)
{
    s_count = 0;
}

int32_t Encoder_GetCount(void)
{
    return s_count;
}

int32_t Encoder_GetPosition_um(void)
{
    return (int32_t)(((int64_t)s_count * UM_PER_REV) / COUNTS_PER_REV);
}

void Encoder_TimerPollCallback(void)
{
    uint8_t curr = ReadHallState();
    uint8_t prev = s_hallState;

    if (curr != prev)
    {
        s_hallState = curr;
        s_pollHits++;

        int8_t delta = HallStepDelta(prev, curr);
        if (delta != 0)
            s_count += (ENCODER_DIR_SIGN * delta);
        else
            s_invalidTransitions++;
    }
}

void Encoder_App_Task(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_lastPrintMs) < DEBUG_PRINT_MS)
        return;
    s_lastPrintMs = now;

    int32_t count = s_count;
    int64_t um    = ((int64_t)count * UM_PER_REV) / COUNTS_PER_REV;
    char    sign  = (um < 0) ? '-' : '+';
    if (um < 0)
        um = -um;

    uint32_t mm_whole = (uint32_t)(um / 1000);
    uint32_t mm_frac  = (uint32_t)(um % 1000);

    Encoder_Print(
        "enc hall=%u%u%u raw=0x%X cnt=%ld pos=%c%lu.%03lumm exti=%lu bad=%lu\r\n",
        (s_hallState >> HALL_A_BIT) & 1U,
        (s_hallState >> HALL_B_BIT) & 1U,
        (s_hallState >> HALL_C_BIT) & 1U,
        s_hallState,
        (long)count,
        sign,
        (unsigned long)mm_whole,
        (unsigned long)mm_frac,
        (unsigned long)s_pollHits,
        (unsigned long)s_invalidTransitions
    );
}

void Encoder_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

void Encoder_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

void Encoder_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
        Encoder_TimerPollCallback();
}
