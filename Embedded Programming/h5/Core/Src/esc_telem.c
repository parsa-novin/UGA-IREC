#include "esc_telem.h"

#include "main.h"
#include "usart.h"

#include <stdint.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * AM32 / BLHeli32 serial telemetry — 10-byte auto packet at 115200 8N1
 *
 * Byte layout:
 *   [0]      temp_c          uint8   degrees C
 *   [1..2]   voltage         uint16  big-endian, unit = 10 mV  (÷100 → V)
 *   [3..4]   current         uint16  big-endian, unit = 10 mA  (÷100 → A)
 *   [5..6]   consumption     uint16  big-endian, unit = 1 mAh
 *   [7..8]   erpm            uint16  big-endian, ×100 = actual eRPM
 *   [9]      crc             XOR of bytes [0..8]
 * ------------------------------------------------------------------------- */

#define TELEM_PACKET_LEN   10U

/* UART handle used for telemetry receive — UART5 */
#define TELEM_UART         huart5

static volatile uint8_t  s_rxBuf[TELEM_PACKET_LEN];
static volatile uint8_t  s_rxReady   = 0;
static volatile uint32_t s_crcErrors = 0;

static uint8_t  s_temp_c        = 0;
static uint32_t s_voltage_mV    = 0;
static uint32_t s_current_mA    = 0;
static uint32_t s_consumption   = 0;
static uint32_t s_erpm          = 0;
static uint8_t  s_valid         = 0;

static uint8_t CalcCRC(const volatile uint8_t *buf)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < (TELEM_PACKET_LEN - 1U); i++)
        crc ^= buf[i];
    return crc;
}

static void DecodePacket(const volatile uint8_t *buf)
{
    if (CalcCRC(buf) != buf[TELEM_PACKET_LEN - 1U])
    {
        s_crcErrors++;
        return;
    }

    s_temp_c      = buf[0];
    s_voltage_mV  = (uint32_t)(((uint16_t)buf[1] << 8) | buf[2]) * 10UL;
    s_current_mA  = (uint32_t)(((uint16_t)buf[3] << 8) | buf[4]) * 10UL;
    s_consumption = (uint32_t)(((uint16_t)buf[5] << 8) | buf[6]);
    s_erpm        = (uint32_t)(((uint16_t)buf[7] << 8) | buf[8]) * 100UL;
    s_valid       = 1;
}

void ESC_Telem_RxCpltCallback(void)
{
    s_rxReady = 1;
    HAL_UART_Receive_IT(&TELEM_UART, (uint8_t *)s_rxBuf, TELEM_PACKET_LEN);
}

void ESC_Telem_Init(void)
{
    s_rxReady   = 0;
    s_crcErrors = 0;
    s_valid     = 0;
    memset((void *)s_rxBuf, 0, sizeof(s_rxBuf));

    HAL_UART_Receive_IT(&TELEM_UART, (uint8_t *)s_rxBuf, TELEM_PACKET_LEN);
}

void ESC_Telem_Task(void)
{
    if (!s_rxReady)
        return;

    s_rxReady = 0;
    DecodePacket(s_rxBuf);
}

uint8_t  ESC_Telem_GetTemp_C(void)          { return s_temp_c;      }
uint32_t ESC_Telem_GetVoltage_mV(void)      { return s_voltage_mV;  }
uint32_t ESC_Telem_GetCurrent_mA(void)      { return s_current_mA;  }
uint32_t ESC_Telem_GetConsumption_mAh(void) { return s_consumption; }
uint32_t ESC_Telem_GetERPM(void)            { return s_erpm;        }
uint8_t  ESC_Telem_IsValid(void)            { return s_valid;       }
uint32_t ESC_Telem_GetCRCErrors(void)       { return s_crcErrors;   }
