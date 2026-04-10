/*
 * esc_telem.c
 *
 * AM32 / BLHeli32 serial telemetry decoder.
 *
 * Packet format — 10 bytes at 115200 8N1:
 *   [0]      temp_c        uint8    degrees C
 *   [1..2]   voltage       uint16   big-endian, unit = 10 mV
 *   [3..4]   current       uint16   big-endian, unit = 10 mA
 *   [5..6]   consumption   uint16   big-endian, unit = 1 mAh
 *   [7..8]   erpm          uint16   big-endian, × 100 = actual eRPM
 *   [9]      crc           uint8    XOR of bytes [0..8]
 *
 * Architecture
 * ------------
 * All byte accumulation, CRC checking, and decoding happen inside
 * ESC_Telem_RxCpltCallback() in ISR context.  This prevents the race
 * condition where blocking HAL_UART_Transmit calls in the main loop
 * delay ESC_Telem_Task() long enough for the ISR to overwrite the buffer
 * mid-read, producing a permanently stuck current value.
 *
 * Buffer policy
 * -------------
 * The buffer is cleared (s_bufIdx reset to 0) after EVERY decode attempt,
 * whether the CRC passed or failed.  This trades re-sync speed (the old
 * shift-left approach could re-sync in 1 byte after a CRC failure) for
 * simplicity and safety: there are no stale bytes left in the buffer, so
 * a bad packet can never contaminate the next decode attempt.
 *
 * At 115200 baud a 10-byte packet arrives every ~870 µs.  Discarding one
 * misaligned packet and waiting for the next clean one adds at most one
 * packet period of latency (~870 µs), which is completely acceptable for
 * a 100 Hz telemetry stream used only for stall detection.
 *
 * Atomicity
 * ---------
 * Published values (s_current_mA etc.) are volatile uint32_t written as
 * single naturally-aligned stores in the ISR.  On Cortex-M33 these are
 * atomic — the main loop can read them without a critical section.
 */

#include "esc_telem.h"
#include "main.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>

/* =========================================================================
 * Constants
 * ========================================================================= */

#define TELEM_PACKET_LEN    10U
#define TELEM_UART          huart5
#define TELEM_STALE_MS      500U

/* =========================================================================
 * ISR-side accumulation state — only touched inside the ISR
 * ========================================================================= */

static uint8_t s_rxByte             = 0;
static uint8_t s_buf[TELEM_PACKET_LEN];
static uint8_t s_bufIdx             = 0;   /* bytes received, 0..TELEM_PACKET_LEN */

/* =========================================================================
 * Published telemetry — written in ISR, read in main loop
 * ========================================================================= */

static volatile uint8_t  s_temp_c      = 0;
static volatile uint32_t s_voltage_mV  = 0;
static volatile uint32_t s_current_mA  = 0;
static volatile uint32_t s_consumption = 0;
static volatile uint32_t s_erpm        = 0;
static volatile uint8_t  s_valid       = 0;
static volatile uint32_t s_lastValidMs = 0;

static volatile uint32_t s_crcErrors   = 0;
static volatile uint32_t s_packetsOk   = 0;

/* =========================================================================
 * ISR helpers
 * ========================================================================= */

static uint8_t CalcCRC(const uint8_t *buf)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < (TELEM_PACKET_LEN - 1U); i++)
        crc ^= buf[i];
    return crc;
}

/*
 * ProcessBuffer — called when s_bufIdx == TELEM_PACKET_LEN.
 *
 * Checks CRC.  On pass, publishes decoded values.
 * On pass OR fail, resets s_bufIdx to 0 — the buffer is always cleared
 * after every read so no stale bytes carry over to the next attempt.
 */
static void ProcessBuffer(void)
{
    if (CalcCRC(s_buf) == s_buf[TELEM_PACKET_LEN - 1U])
    {
        s_temp_c      = s_buf[0];
        s_voltage_mV  = (uint32_t)(((uint16_t)s_buf[1] << 8) | s_buf[2]) * 10UL;
        s_current_mA  = (uint32_t)(((uint16_t)s_buf[3] << 8) | s_buf[4]) * 10UL;
        s_consumption = (uint32_t)(((uint16_t)s_buf[5] << 8) | s_buf[6]);
        s_erpm        = (uint32_t)(((uint16_t)s_buf[7] << 8) | s_buf[8]) * 100UL;
        s_valid       = 1;
        s_lastValidMs = HAL_GetTick();
        s_packetsOk++;
    }
    else
    {
        s_crcErrors++;
    }

    /* Always clear the buffer after every read attempt */
    memset(s_buf, 0, sizeof(s_buf));
    s_bufIdx = 0;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void ESC_Telem_Init(void)
{
    memset(s_buf, 0, sizeof(s_buf));
    s_bufIdx      = 0;
    s_crcErrors   = 0;
    s_packetsOk   = 0;
    s_valid       = 0;
    s_lastValidMs = 0;
    s_temp_c      = 0;
    s_voltage_mV  = 0;
    s_current_mA  = 0;
    s_consumption = 0;
    s_erpm        = 0;

    HAL_UART_Receive_IT(&TELEM_UART, &s_rxByte, 1U);
}

/*
 * ESC_Telem_RxCpltCallback — ISR context (called from HAL_UART_RxCpltCallback).
 *
 * Appends the received byte, decodes when the buffer is full, re-arms.
 */
void ESC_Telem_RxCpltCallback(void)
{
    if (s_bufIdx < TELEM_PACKET_LEN)
    {
        s_buf[s_bufIdx] = s_rxByte;
        s_bufIdx++;
    }

    if (s_bufIdx == TELEM_PACKET_LEN)
    {
        ProcessBuffer();   /* always resets s_bufIdx to 0 */
    }

    HAL_UART_Receive_IT(&TELEM_UART, &s_rxByte, 1U);
}

/*
 * ESC_Telem_Task — main-loop context.
 * Only handles validity expiry; all decode work is done in the ISR.
 */
void ESC_Telem_Task(void)
{
    if (s_valid && (HAL_GetTick() - s_lastValidMs) > TELEM_STALE_MS)
        s_valid = 0;
}

/* =========================================================================
 * Accessors
 * ========================================================================= */

uint8_t  ESC_Telem_GetTemp_C(void)          { return s_temp_c;      }
uint32_t ESC_Telem_GetVoltage_mV(void)      { return s_voltage_mV;  }
uint32_t ESC_Telem_GetCurrent_mA(void)      { return s_current_mA;  }
uint32_t ESC_Telem_GetConsumption_mAh(void) { return s_consumption; }
uint32_t ESC_Telem_GetERPM(void)            { return s_erpm;        }
uint8_t  ESC_Telem_IsValid(void)            { return s_valid;       }
uint32_t ESC_Telem_GetCRCErrors(void)       { return s_crcErrors;   }
uint32_t ESC_Telem_GetPacketsOk(void)       { return s_packetsOk;   }
