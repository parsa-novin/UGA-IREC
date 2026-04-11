/*
 * esc_telem.c
 *
 * AM32 / BLHeli32 serial telemetry decoder.
 *
 * Packet format — 10 bytes at 115200 8N1:
 *   [0]      temp_c        uint8    degrees C
 *   [1..2]   voltage       uint16   big-endian, unit = 10 mV  (centivolts)
 *   [3..4]   current       uint16   big-endian, unit = 10 mA  (centiamps)
 *   [5..6]   consumption   uint16   big-endian, unit = 1 mAh
 *   [7..8]   erpm          uint16   big-endian, × 100 = actual eRPM
 *   [9]      crc           uint8    CRC8/DVB-S2 (poly 0xD5) of bytes [0..8]
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
#define TELEM_VOLT_SCALE_MV_X10      10UL
#define TELEM_VOLT_SCALE_MV_X100     100UL
#define TELEM_CURRENT_SCALE_MA       10UL
#define TELEM_ERPM_SCALE             100UL
#define TELEM_VOLT_MIN_PLAUSIBLE_MV  3000UL
#define TELEM_VOLT_MAX_PLAUSIBLE_MV 60000UL

/* =========================================================================
 * ISR-side accumulation state — only touched inside the ISR
 * ========================================================================= */

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

/*
 * CRC8/DVB-S2 lookup table — polynomial 0xD5, initial value 0x00.
 * Matches AM32 firmware's get_crc8() / crc8tab[].
 */
static const uint8_t s_crc8_table[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
    0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
    0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
    0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
    0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
    0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
    0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
    0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
    0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
    0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
    0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
    0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
};

static uint8_t CalcCRC(const uint8_t *buf)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < (TELEM_PACKET_LEN - 1U); i++)
        crc = s_crc8_table[crc ^ buf[i]];
    return crc;
}

/*
 * ProcessBuffer — called when s_bufIdx == TELEM_PACKET_LEN.
 *
 * Checks CRC.  On pass, publishes decoded values.
 * On pass OR fail, resets s_bufIdx to 0 — the buffer is always cleared
 * after every read so no stale bytes carry over to the next attempt.
 */
static uint32_t ScaleVoltage_mV(uint16_t rawVoltage)
{
    /*
     * Different ESC firmware variants report voltage in either 10 mV or
     * 100 mV units. Auto-select the scale that lands in a sane battery range.
     */
    uint32_t mv_x10 = (uint32_t)rawVoltage * TELEM_VOLT_SCALE_MV_X10;
    uint32_t mv_x100 = (uint32_t)rawVoltage * TELEM_VOLT_SCALE_MV_X100;

    if ((mv_x10 < TELEM_VOLT_MIN_PLAUSIBLE_MV) &&
        (mv_x100 <= TELEM_VOLT_MAX_PLAUSIBLE_MV))
    {
        return mv_x100;
    }

    return mv_x10;
}

static void PublishFrame(const uint8_t *frame)
{
    s_temp_c      = frame[0];
    s_voltage_mV  = ScaleVoltage_mV((uint16_t)(((uint16_t)frame[1] << 8) | frame[2]));
    s_current_mA  = (uint32_t)(((uint16_t)frame[3] << 8) | frame[4]) * TELEM_CURRENT_SCALE_MA;
    s_consumption = (uint32_t)(((uint16_t)frame[5] << 8) | frame[6]);
    s_erpm        = (uint32_t)(((uint16_t)frame[7] << 8) | frame[8]) * TELEM_ERPM_SCALE;
    s_valid       = 1;
    s_lastValidMs = HAL_GetTick();
    s_packetsOk++;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void ESC_Telem_Init(void)
{
    memset(s_buf, 0, sizeof(s_buf));
    s_bufIdx = 0;

    /* USER CODE BEGIN ESC_Telem_Init */
    /* RXNEIE is set permanently by MX_UART5_Init — no HAL IT re-arm needed.
     * Just clear any stale error flags so the ISR starts clean. */
    __HAL_UART_CLEAR_FLAG(&TELEM_UART, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                       UART_CLEAR_FEF  | UART_CLEAR_PEF  | UART_CLEAR_IDLEF);

    /* Paranoid check: re-enable RXNEIE/IDLEIE if something cleared them */
    if (!(TELEM_UART.Instance->CR1 & USART_CR1_RXNEIE_RXFNEIE))
    {
        SET_BIT(TELEM_UART.Instance->CR1, USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE);
        SET_BIT(TELEM_UART.Instance->CR3, USART_CR3_EIE);
    }
    /* USER CODE END ESC_Telem_Init */
}

/*
 * ESC_Telem_Uart5IrqHandler — called directly from UART5_IRQHandler.
 *
 * Reads all available bytes from RDR in one ISR invocation (drains the
 * RXNE flag in a loop), decodes when the buffer is full, and handles
 * error flags without re-arming.  RXNEIE stays permanently set — no
 * per-byte HAL_UART_Receive_IT call, no HAL state-machine overhead.
 *
 * On ORE or FE the buffer is reset; the sliding-window resync is only
 * used for normal CRC failures (which occur on initial alignment).
 */
void ESC_Telem_Uart5IrqHandler(void)
{
    uint32_t isr = TELEM_UART.Instance->ISR;

    /* Drain every byte already in RDR before handling errors */
    while ((isr & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t byte = (uint8_t)(TELEM_UART.Instance->RDR & 0xFFU);

        if (s_bufIdx < TELEM_PACKET_LEN)
        {
            s_buf[s_bufIdx++] = byte;
        }
        else
        {
            s_bufIdx    = 0;
            s_buf[0]    = byte;
            s_bufIdx    = 1U;
        }

        if (s_bufIdx >= TELEM_PACKET_LEN)
        {
            if (CalcCRC(s_buf) == s_buf[TELEM_PACKET_LEN - 1U])
            {
                PublishFrame(s_buf);
                s_bufIdx = 0;
            }
            else
            {
                /* CRC failed — discard entire buffer and wait for the next
                 * clean 10-byte window.  No stale bytes carry over. */
                s_crcErrors++;
                s_bufIdx = 0;
            }
        }

        isr = TELEM_UART.Instance->ISR;
    }

    /* IDLE: line went quiet after the last byte — this marks the end of a
     * packet.  If the buffer isn't empty and hasn't reached a full packet
     * we're misaligned; flush so the next packet starts clean. */
    if ((isr & USART_ISR_IDLE) != 0U)
    {
        if (s_bufIdx != 0U)
            s_bufIdx = 0;
        __HAL_UART_CLEAR_FLAG(&TELEM_UART, UART_CLEAR_IDLEF);
    }

    /* Handle error flags — reset buffer on framing/overrun so the next
     * clean packet locks immediately rather than after 10 misaligned bytes */
    if ((isr & USART_ISR_ORE) != 0U)
    {
        s_bufIdx = 0;
        __HAL_UART_CLEAR_FLAG(&TELEM_UART, UART_CLEAR_OREF);
    }
    if ((isr & USART_ISR_FE) != 0U)
    {
        s_bufIdx = 0;
        __HAL_UART_CLEAR_FLAG(&TELEM_UART, UART_CLEAR_FEF);
    }
    if ((isr & USART_ISR_NE) != 0U)
    {
        __HAL_UART_CLEAR_FLAG(&TELEM_UART, UART_CLEAR_NEF);
    }
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

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

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
