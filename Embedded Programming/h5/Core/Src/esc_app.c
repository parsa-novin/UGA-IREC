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
 *   PA9/PA10 USART1     Aggregate packet TX / optional command RX
 *   PA2/PA3  USART2     Aggregate packet TX / optional command RX
 *   PA0/PA1  UART4      RS422 upstream packet RX / debug TX
 *   PB5/PB6  UART5      ESC serial telemetry RX (AM32 auto-telem)
 *   PB10/12/14/15 SPI2  Camera control (master)
 *
 * Camera SPI2 commands (sent as single ASCII bytes to the RunCam STM32H7):
 *   'R' — start recording
 *   'S' — stop  recording
 *   'O' — power on
 *   'Z' — power off
 *   'C' — request current reading (follow-up: read 4-byte IEEE-754 float, mA)
 *   'E' — query echo buffer byte count (follow-up: read 1 byte = pending count)
 *   'G' — fetch next echo byte (follow-up: read 1 byte = echo byte or 0x00)
 *
 * Echo passthrough ('E'/'G' poll loop):
 *   H7 queues ASCII status lines ("CAM:...\n") in a ring buffer after each
 *   RunCam UART event.  This file polls those lines every CAM_ECHO_POLL_PERIOD_MS
 *   and forwards each complete line verbatim to huart1 and huart2 so the
 *   ground station's serial reader can pick them up alongside the telemetry stream.
 */

#include "esc_app.h"
#include "hil_config.h"

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "esc_telem.h"
#include "command_sequence.h"
#include "sd_logger.h"
#include "airbrake_deploy.h"
#include "airbrake_control.h"
#include "flight_estimator.h"
#include "encoder_homing.h"
#include "encoder_app.h"
#include "flight_trigger.h"

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

#define PWM_MIN_US              1000
#define PWM_MAX_US              2000
#define PWM_NEUTRAL_US          1500

#define NEUTRAL_MIN_US          1300
#define NEUTRAL_MAX_US          1700

#define CREEP_OFFSET_US         80

/*
 * Time to hold neutral PWM after startup before allowing motion commands.
 * Covers the ESC's full boot sequence + AM32 neutral-detection window.
 */
#define ESC_ARM_HOLD_MS         5000U

#define RX_BUF_LEN              64U
#define UPSTREAM_HEADER         0xAA55u
#define AGGREGATE_HEADER        0xA55Au
#define PACKET_TX_TIMEOUT       20U
#define UART_POLL_BURST_MAX     64U
#define TELEMETRY_TX_PERIOD_MS  200U  /* 5 Hz */
#define FALLBACK_TX_PERIOD_MS   200U
#define UART4_DEBUG_TX_PERIOD_MS 500U
/* Camera SPI2 constants */
#define SPI_TX_TIMEOUT_MS       50U
#define CAMERA_CURRENT_DELAY_MS 3U

#define SPI_CMD_CAMERA_START    ((uint8_t)'R')
#define SPI_CMD_CAMERA_STOP     ((uint8_t)'S')
#define SPI_CMD_CAMERA_ON       ((uint8_t)'O')
#define SPI_CMD_CAMERA_OFF      ((uint8_t)'Z')
#define SPI_CMD_CAMERA_CURR     ((uint8_t)'C')
#define SPI_CMD_CAM_ECHO_AVAIL  ((uint8_t)'E')   /* query pending echo byte count  */
#define SPI_CMD_CAM_ECHO_GET    ((uint8_t)'G')   /* fetch one byte from echo buffer */

/* Echo poll period and per-cycle byte cap */
#define CAM_ECHO_POLL_PERIOD_MS     100U
#define CAM_ECHO_MAX_BYTES_PER_POLL 48U
#define CAM_CMD_RESPONSE_TIMEOUT_MS 500U

/* 10 Hz poll period for camera current and battery voltage */
#define CAM_CURR_POLL_PERIOD_MS     100U
#define BATT_VOLT_POLL_PERIOD_MS    100U

/*
 * SensorPacket_t is defined in esc_app.h so it is visible to sd_logger.c
 * and any other consumer.  The _Static_assert lives there too.
 * AggregatePacket_t is internal to this file only.
 */
typedef struct __attribute__((packed))
{
    uint16_t       header;
    SensorPacket_t upstream;
    uint8_t        esc_valid;
    uint8_t        esc_temp_c;
    uint32_t       esc_voltage_mV;
    uint32_t       esc_current_mA;
    uint32_t       esc_consumption_mAh;
    uint32_t       esc_erpm;
    int32_t        encoder_position_um;
    float          cam_current_mA;
    uint32_t       batt_voltage_mV;
    uint8_t        checksum;
} AggregatePacket_t;

/* =========================================================================
 * Static state
 * ========================================================================= */

static char              s_rxLine1[RX_BUF_LEN];
static char              s_rxLine2[RX_BUF_LEN];
static volatile uint32_t s_rxIdx1 = 0U;
static volatile uint32_t s_rxIdx2 = 0U;

static int32_t s_neutralUs      = PWM_NEUTRAL_US;
static int32_t s_currentPulseUs = PWM_NEUTRAL_US;

static uint8_t                 s_upstreamRxBuf[UPSTREAM_PACKET_LEN];
static volatile uint32_t       s_upstreamRxIdx         = 0U;
static volatile SensorPacket_t s_upstreamPacket;
static volatile uint8_t        s_upstreamPacketValid   = 0U;
static volatile uint32_t       s_upstreamPacketVersion = 0U;
static uint32_t                s_lastForwardedVersion  = 0U;
static uint32_t                s_lastTelemetryTxMs     = 0U;
static uint32_t                s_lastFallbackTxMs      = 0U;
static uint32_t                s_lastUart4DebugTxMs    = 0U;
static uint32_t                s_lastCamEchoPollMs     = 0U;
static bool                    s_camCmdPending         = false;
static uint32_t                s_camCmdDeadlineMs      = 0U;
static char                    s_camCmdExpectedLine[24];
static char                    s_camCmdTimeoutText[40];
static float                   s_camCurrentMa          = 0.0f;
static uint32_t                s_battVoltMv            = 0U;
static uint32_t                s_lastCamCurrPollMs     = 0U;
static uint32_t                s_lastBattVoltPollMs    = 0U;
static uint8_t                 s_headerFirstByte       = 0U;
static volatile uint32_t       s_uart4RxCount          = 0U;
static volatile uint32_t       s_uart4ErrorCount       = 0U;
static volatile uint32_t       s_uart4OreCount         = 0U;
static volatile uint32_t       s_uart4FeCount          = 0U;
static volatile uint32_t       s_uart4NeCount          = 0U;
static volatile uint32_t       s_uart4PeCount          = 0U;
static volatile uint32_t       s_uart4RtofCount        = 0U;
static volatile uint32_t       s_uart4CmfCount         = 0U;
static volatile uint32_t       s_uart4EobfCount        = 0U;
static volatile uint32_t       s_uart4LastIsr          = 0U;
static volatile uint32_t       s_uart4LastErrorCode    = 0U;
static volatile uint32_t       s_uart4LastCr1          = 0U;
static volatile uint32_t       s_uart4LastCr2          = 0U;
static volatile uint32_t       s_uart4LastCr3          = 0U;
static uint8_t                 s_uart4Preview[16];
static uint32_t                s_uart4PreviewCount     = 0U;
static uint16_t                s_lastCandidateHeader   = 0U;
static uint8_t                 s_lastCandidateChecksum = 0U;
static uint8_t                 s_lastComputedChecksum  = 0U;
static uint8_t                 s_lastCandidateValid    = 0U;

static void Upstream_EnableIrqRx(void);
static void Upstream_EnsureIrqRxEnabled(void);
static void Upstream_PollRx(void);
static void __attribute__((unused)) Uart4_DebugPrintTask(void);
static void Camera_PollEchoTask(void);
static void Camera_CheckCommandTimeoutTask(void);
static void Camera_PollCurrentTask(void);
static void Battery_ADC_Init(void);
static void Battery_PollVoltageTask(void);
static void ProcessConsoleByte(char c, char *buf, volatile uint32_t *idx);
#ifdef HIL_MODE
static void HIL_ProcessPacket(const uint8_t *buf);
static void HIL_PollRx(void);
#endif

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


static uint8_t PacketChecksum(const uint8_t *start, const uint8_t *end)
{
    uint8_t xorv = 0U;

    while (start < end)
    {
        xorv ^= *start;
        start++;
    }

    return xorv;
}

static uint8_t SensorPacket_IsValid(const SensorPacket_t *packet)
{
    const uint8_t *raw = (const uint8_t *)packet;

    if ((packet->header != UPSTREAM_HEADER) && (packet->header != (uint16_t)0x55AAu))
        return 0U;

    return (uint8_t)(PacketChecksum(raw + sizeof(packet->header),
                                    raw + sizeof(*packet) - sizeof(packet->checksum))
                     == packet->checksum);
}


/* =========================================================================
 * Private helpers — camera SPI2
 * ========================================================================= */

/**
  * @brief  Transmit a single command byte to the camera over SPI2.
  * @param  cmd  One of SPI_CMD_CAMERA_*.
  * @retval true on success, false on HAL error or timeout.
  */
static bool Camera_SendSpiByte(uint8_t cmd)
{
    return (HAL_SPI_Transmit(&hspi2, &cmd, 1U, SPI_TX_TIMEOUT_MS) == HAL_OK);
}

static void Camera_ForwardText(const char *text)
{
    uint16_t len;

    if (text == NULL)
        return;

    len = (uint16_t)strlen(text);
    HAL_UART_Transmit(&huart1, (uint8_t *)text, len, PACKET_TX_TIMEOUT);
    HAL_UART_Transmit(&huart2, (uint8_t *)text, len, PACKET_TX_TIMEOUT);
}

static void Camera_BeginPendingCommand(const char *expected_line,
                                       const char *timeout_text)
{
    if ((expected_line == NULL) || (timeout_text == NULL))
        return;

    strncpy(s_camCmdExpectedLine, expected_line, sizeof(s_camCmdExpectedLine) - 1U);
    s_camCmdExpectedLine[sizeof(s_camCmdExpectedLine) - 1U] = '\0';

    strncpy(s_camCmdTimeoutText, timeout_text, sizeof(s_camCmdTimeoutText) - 1U);
    s_camCmdTimeoutText[sizeof(s_camCmdTimeoutText) - 1U] = '\0';

    s_camCmdDeadlineMs = HAL_GetTick() + CAM_CMD_RESPONSE_TIMEOUT_MS;
    s_camCmdPending = true;
}

static void Camera_ObserveEchoLine(const char *line)
{
    if (!s_camCmdPending || (line == NULL))
        return;

    if (strncmp(line, s_camCmdExpectedLine, strlen(s_camCmdExpectedLine)) == 0)
    {
        s_camCmdPending = false;
    }
}

static void Camera_CheckCommandTimeoutTask(void)
{
    if (!s_camCmdPending)
        return;

    if ((int32_t)(HAL_GetTick() - s_camCmdDeadlineMs) >= 0)
    {
        Camera_ForwardText(s_camCmdTimeoutText);
        s_camCmdPending = false;
    }
}

/**
  * @brief  Send 'C' to the camera and read back the 4-byte current float.
  *
  * Two-phase protocol:
  *   1. Transmit 'C' — camera reads INA219 and loads its TX FIFO.
  *   2. Wait CAMERA_CURRENT_DELAY_MS for the I2C read to complete.
  *   3. Clock 4 dummy bytes out to receive the IEEE-754 float (mA).
  *
  * The float is in the camera's native (little-endian) byte order.
  * Both this board and the RunCam STM32 are Cortex-M little-endian,
  * so no byte-swap is needed.
  *
  * @param  current_mA  Output: current in milliamps.
  * @retval true on success, false on any SPI error or null pointer.
  */
static bool Camera_ReadCurrent_mA(float *current_mA)
{
    uint8_t cmd         = SPI_CMD_CAMERA_CURR;
    uint8_t tx_dummy[4] = {0U, 0U, 0U, 0U};
    uint8_t rx_bytes[4] = {0U, 0U, 0U, 0U};
    float   value;

    if (current_mA == NULL)
        return false;

    /* Phase 1: send the command byte */
    if (HAL_SPI_Transmit(&hspi2, &cmd, 1U, SPI_TX_TIMEOUT_MS) != HAL_OK)
        return false;

    /* Phase 2: allow the camera time to read the INA219 over I2C */
    HAL_Delay(CAMERA_CURRENT_DELAY_MS);

    /* Phase 3: clock out 4 dummy bytes to receive the float response */
    if (HAL_SPI_TransmitReceive(&hspi2,
                                 tx_dummy, rx_bytes,
                                 4U, SPI_TX_TIMEOUT_MS) != HAL_OK)
        return false;

    memcpy(&value, rx_bytes, sizeof(value));
    *current_mA = value;
    return true;
}

/**
  * @brief  Poll the H7 echo buffer and forward any complete CAM:...\n lines
  *         to both downstream UARTs for pickup by the ground station.
  *
  * Two-phase SPI protocol per transaction:
  *   1. Send command byte ('E' or 'G') in its own SPI transaction.
  *   2. Wait 1 ms for H7 to process the command and pre-load its MISO response.
  *   3. Clock one dummy byte while receiving the response byte.
  *
  * Lines are accumulated across poll cycles in a static buffer so that a line
  * split across two poll windows is reassembled correctly.
  */
static void Camera_PollEchoTask(void)
{
    static char     line_buf[128];
    static uint32_t line_len = 0U;

    uint32_t now = HAL_GetTick();
    if ((now - s_lastCamEchoPollMs) < CAM_ECHO_POLL_PERIOD_MS)
        return;
    s_lastCamEchoPollMs = now;

    /* ── Phase 1: query pending byte count ──────────────────────────────── */
    uint8_t cmd   = SPI_CMD_CAM_ECHO_AVAIL;
    uint8_t dummy = 0x00U;
    uint8_t avail = 0U;

    if (HAL_SPI_Transmit(&hspi2, &cmd, 1U, SPI_TX_TIMEOUT_MS) != HAL_OK)
        return;

    HAL_Delay(1U); /* H7 processes 'E' and arms its 1-byte MISO response */

    if (HAL_SPI_TransmitReceive(&hspi2, &dummy, &avail, 1U, SPI_TX_TIMEOUT_MS) != HAL_OK)
        return;

    if (avail == 0U)
        return;

    if (avail > CAM_ECHO_MAX_BYTES_PER_POLL)
        avail = CAM_ECHO_MAX_BYTES_PER_POLL;

    /* ── Phase 2: fetch bytes one at a time, forward complete lines ─────── */
    cmd = SPI_CMD_CAM_ECHO_GET;

    for (uint8_t i = 0U; i < avail; i++)
    {
        uint8_t b = 0x00U;

        if (HAL_SPI_Transmit(&hspi2, &cmd, 1U, SPI_TX_TIMEOUT_MS) != HAL_OK)
            break;

        HAL_Delay(1U); /* H7 processes 'G' and arms the next echo byte */

        if (HAL_SPI_TransmitReceive(&hspi2, &dummy, &b, 1U, SPI_TX_TIMEOUT_MS) != HAL_OK)
            break;

        /* H7 returns 0x00 when the buffer empties earlier than expected */
        if ((b == 0x00U) && (line_len == 0U))
            continue;

        if (line_len < (sizeof(line_buf) - 1U))
            line_buf[line_len++] = (char)b;

        if (b == (uint8_t)'\n')
        {
            line_buf[line_len] = '\0';
            Camera_ObserveEchoLine(line_buf);

            /* Complete line received — forward verbatim to both downstream UARTs. */
            HAL_UART_Transmit(&huart1, (uint8_t *)line_buf, (uint16_t)line_len, PACKET_TX_TIMEOUT);
            HAL_UART_Transmit(&huart2, (uint8_t *)line_buf, (uint16_t)line_len, PACKET_TX_TIMEOUT);
            line_len = 0U;
        }
    }
}

/* =========================================================================
 * Private helpers — upstream packet
 * ========================================================================= */

static void Upstream_ProcessByte(uint8_t byte)
{
    if (s_upstreamRxIdx == 0U)
    {
        /* Accept either header byte order: 55 AA or AA 55 */
        if ((byte != 0x55U) && (byte != 0xAAU))
            return;

        s_uart4PreviewCount = 0U;
        s_headerFirstByte = byte;
    }
    else if (s_upstreamRxIdx == 1U)
    {
        uint8_t expected = (s_headerFirstByte == 0x55U) ? 0xAAU : 0x55U;
        if (byte != expected)
        {
            if ((byte == 0x55U) || (byte == 0xAAU))
            {
                s_upstreamRxIdx    = 1U;
                s_upstreamRxBuf[0] = byte;
                s_headerFirstByte  = byte;
                }
            else
            {
                s_upstreamRxIdx = 0U;
            }
            return;
        }
    }

    s_upstreamRxBuf[s_upstreamRxIdx++] = byte;

    if (s_uart4PreviewCount < sizeof(s_uart4Preview))
    {
        s_uart4Preview[s_uart4PreviewCount++] = byte;
    }

    if (s_upstreamRxIdx >= UPSTREAM_PACKET_LEN)
    {
        SensorPacket_t candidate;
        const uint8_t *raw = (const uint8_t *)&candidate;

        memcpy(&candidate, s_upstreamRxBuf, sizeof(candidate));
        s_lastCandidateHeader   = candidate.header;
        s_lastCandidateChecksum = candidate.checksum;
        s_lastComputedChecksum  = PacketChecksum(raw + sizeof(candidate.header),
                                                 raw + sizeof(candidate) - sizeof(candidate.checksum));
        s_lastCandidateValid    = (uint8_t)(s_lastComputedChecksum == s_lastCandidateChecksum);

        if (SensorPacket_IsValid(&candidate))
        {
            /* Normalize header representation regardless of wire byte order. */
            candidate.header        = UPSTREAM_HEADER;
            s_upstreamPacket        = candidate;
            s_upstreamPacketValid   = 1U;
            s_upstreamPacketVersion++;
        }
        else
        {
            (void)0;
        }

        s_upstreamRxIdx = 0U;
    }
}

static void Upstream_EnableIrqRx(void)
{
    s_upstreamRxIdx   = 0U;
    s_headerFirstByte = 0U;
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF |
                                  UART_CLEAR_PEF | UART_CLEAR_IDLEF | UART_CLEAR_RTOF |
                                  UART_CLEAR_CMF | USART_ICR_EOBCF);
    CLEAR_BIT(huart4.Instance->CR3, USART_CR3_RXFTIE);
    CLEAR_BIT(huart4.Instance->CR1, USART_CR1_RTOIE | USART_CR1_EOBIE | USART_CR1_CMIE | USART_CR1_MME | USART_CR1_PEIE);
    CLEAR_BIT(huart4.Instance->CR2, USART_CR2_RTOEN | USART_CR2_ADDM7);
    SET_BIT(huart4.Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
    SET_BIT(huart4.Instance->CR3, USART_CR3_EIE);
}

static void Upstream_EnsureIrqRxEnabled(void)
{
    uint32_t cr1 = huart4.Instance->CR1;
    uint32_t cr2 = huart4.Instance->CR2;
    uint32_t cr3 = huart4.Instance->CR3;

    if (((cr1 & (USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE))
          != (USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE)) ||
        ((cr2 & (USART_CR2_RTOEN | USART_CR2_ADDM7)) != 0U) ||
        ((cr3 & USART_CR3_EIE) == 0U) ||
        ((cr3 & USART_CR3_RXFTIE) != 0U))
    {
        Upstream_EnableIrqRx();
    }
}

static void Upstream_PollRx(void)
{
    uint32_t guard = 0U;

    while ((__HAL_UART_GET_FLAG(&huart4, USART_ISR_RXNE_RXFNE)) != 0U)
    {
        uint8_t byte = (uint8_t)(huart4.Instance->RDR & 0xFFU);
        s_uart4RxCount++;
        Upstream_ProcessByte(byte);

        if (++guard >= UART_POLL_BURST_MAX)
            break;
    }
}

static void __attribute__((unused)) Uart4_DebugPrintTask(void)
{
    char     line[512];
    char     preview[64];
    int      len;
    int      previewLen = 0;
    uint32_t now = HAL_GetTick();
    uint32_t cr1 = huart4.Instance->CR1;
    uint32_t cr2 = huart4.Instance->CR2;
    uint32_t cr3 = huart4.Instance->CR3;
    uint32_t isr = huart4.Instance->ISR;
    uint32_t i;

    if ((now - s_lastUart4DebugTxMs) < UART4_DEBUG_TX_PERIOD_MS)
        return;

    s_lastUart4DebugTxMs = now;

    for (i = 0U; (i < s_uart4PreviewCount) && (i < 16U); i++)
    {
        int wrote = snprintf(&preview[previewLen], sizeof(preview) - (size_t)previewLen,
                             "%02X%s", s_uart4Preview[i], ((i + 1U) < s_uart4PreviewCount) ? " " : "");
        if ((wrote <= 0) || ((size_t)wrote >= (sizeof(preview) - (size_t)previewLen)))
            break;
        previewLen += wrote;
    }

    len = snprintf(line, sizeof(line),
                   "UART4 rx=%lu idx=%lu valid=%lu ver=%lu last_ok=%u hdr=0x%04X rx_ck=0x%02X calc_ck=0x%02X err=%lu ore=%lu fe=%lu ne=%lu pe=%lu cr1=0x%08lX cr2=0x%08lX cr3=0x%08lX isr=0x%08lX raw=%s\r\n",
                   (unsigned long)s_uart4RxCount,
                   (unsigned long)s_upstreamRxIdx,
                   (unsigned long)s_upstreamPacketValid,
                   (unsigned long)s_upstreamPacketVersion,
                   (unsigned int)s_lastCandidateValid,
                   (unsigned int)s_lastCandidateHeader,
                   (unsigned int)s_lastCandidateChecksum,
                   (unsigned int)s_lastComputedChecksum,
                   (unsigned long)s_uart4ErrorCount,
                   (unsigned long)s_uart4OreCount,
                   (unsigned long)s_uart4FeCount,
                   (unsigned long)s_uart4NeCount,
                   (unsigned long)s_uart4PeCount,
                   (unsigned long)cr1,
                   (unsigned long)cr2,
                   (unsigned long)cr3,
                   (unsigned long)isr,
                   (previewLen > 0) ? preview : "--");

    if (len <= 0)
        return;

    if (len >= (int)sizeof(line))
        len = (int)sizeof(line) - 1;

    HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, PACKET_TX_TIMEOUT);
}

#ifdef HIL_MODE
/* =========================================================================
 * HIL receive path — parse 100-byte aggregate packets from USART2
 *
 * The PC running hil_gui.py transmits the same AggregatePacket_t format
 * the H5 normally outputs.  We strip the outer aggregate wrapper, validate
 * both checksums, and push the embedded SensorPacket_t into the normal
 * upstream buffer so the entire flight-control stack runs unchanged.
 *
 * Any byte on USART2 that is NOT the 0xA5 / 0x5A header start byte is
 * forwarded to the ASCII command handler (ProcessConsoleByte) so that
 * "H", "67", "UP", "DOWN", "ALTxxxxx", etc. still work from the same port.
 * ========================================================================= */

static void HIL_ProcessPacket(const uint8_t *buf)
{
    /* Verify aggregate header (little-endian 0xA55A → bytes 0xA5, 0x5A) */
    uint16_t hdr;
    memcpy(&hdr, buf, sizeof(hdr));
    if (hdr != AGGREGATE_HEADER)
        return;

    /* Validate aggregate checksum: XOR of bytes [2 .. sizeof-2] inclusive */
    uint8_t cs = PacketChecksum(buf + sizeof(uint16_t),
                                buf + sizeof(AggregatePacket_t) - sizeof(uint8_t));
    if (cs != buf[sizeof(AggregatePacket_t) - 1U])
        return;

    /* Extract the embedded SensorPacket_t that starts at byte offset 2 */
    SensorPacket_t candidate;
    memcpy(&candidate, buf + sizeof(uint16_t), sizeof(SensorPacket_t));

    /* Validate the inner upstream checksum */
    if (!SensorPacket_IsValid(&candidate))
        return;

    /* Normalise header byte order and publish to the shared buffer */
    candidate.header = UPSTREAM_HEADER;
    __disable_irq();
    s_upstreamPacket        = candidate;
    s_upstreamPacketValid   = 1U;
    s_upstreamPacketVersion++;
    __enable_irq();
}

static void HIL_PollRx(void)
{
    static uint8_t  s_hilBuf[sizeof(AggregatePacket_t)];
    static uint32_t s_hilIdx = 0U;
    uint32_t        guard    = 0U;

    while ((__HAL_UART_GET_FLAG(&huart2, USART_ISR_RXNE_RXFNE) != 0U)
           && (guard < UART_POLL_BURST_MAX))
    {
        uint8_t byte = (uint8_t)(huart2.Instance->RDR & 0xFFU);
        guard++;

        /* Clear any line errors so the FIFO does not stall */
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(&huart2);
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))  __HAL_UART_CLEAR_FEFLAG(&huart2);
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE))  __HAL_UART_CLEAR_NEFLAG(&huart2);

        if (s_hilIdx == 0U)
        {
            /* 0xA5 is the first byte of AGGREGATE_HEADER = 0xA55A (little-endian) */
            if (byte == 0xA5U)
            {
                s_hilBuf[s_hilIdx++] = byte;
            }
            else
            {
                /* ASCII command character — route to console handler */
                ProcessConsoleByte((char)byte, s_rxLine2, &s_rxIdx2);
            }
        }
        else if (s_hilIdx == 1U)
        {
            /* Second byte of the header must be 0x5A */
            if (byte == 0x5AU)
            {
                s_hilBuf[s_hilIdx++] = byte;
            }
            else
            {
                /* Not a valid frame start — replay first byte as ASCII */
                ProcessConsoleByte((char)s_hilBuf[0], s_rxLine2, &s_rxIdx2);
                s_hilIdx = 0U;
                if (byte == 0xA5U)
                    s_hilBuf[s_hilIdx++] = byte;
                else
                    ProcessConsoleByte((char)byte, s_rxLine2, &s_rxIdx2);
            }
        }
        else
        {
            s_hilBuf[s_hilIdx++] = byte;
            if (s_hilIdx >= (uint32_t)sizeof(AggregatePacket_t))
            {
                HIL_ProcessPacket(s_hilBuf);
                s_hilIdx = 0U;
            }
        }
    }
}
#endif /* HIL_MODE */

/**
  * @brief  Initialise ADC1 for single-ended polling on channel 9 (PB0).
  *
  * Uses direct register access because the HAL ADC driver is not included in
  * this project (HAL_ADC_MODULE_ENABLED is not defined).  The GPIOB clock is
  * already enabled by other peripheral init functions.  ADC clock source is
  * the asynchronous kernel clock (CKMODE=00) divided by 4 via the CCR PRESC
  * field of ADC12_COMMON.
  *
  * Voltage recovery (empirically calibrated):
  *   Vbat_mV = ADC_count × 12140 / 4096
  *
  * The theoretical 68 kΩ / 33 kΩ divider constant (10100) under-reads by
  * factor 7.44/6.19; 12140 corrects for the actual board resistor values.
  * (2S3P LiIon max 8.4 V, OPA340NA unity-gain buffer)
  */
static void Battery_ADC_Init(void)
{
    /* Enable ADC1/ADC2 clock (shared RCC bit on H5) */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
    (void)RCC->AHB2ENR;  /* read-back barrier so the clock is active before use */

    /* PB0 → analog input, no pull (GPIOB clock already enabled by SPI/TIM init) */
    GPIOB->MODER  |=  (3UL << (0U * 2U));  /* bits 1:0 = 11 → Analog */
    GPIOB->PUPDR  &= ~(3UL << (0U * 2U));  /* bits 1:0 = 00 → no pull */

    /* Async clock prescaler ÷4 in the ADC12 common register (PRESC = 0b0010) */
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_PRESC) | (2UL << ADC_CCR_PRESC_Pos);

    /* Exit deep power-down mode, then enable internal voltage regulator */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |=  ADC_CR_ADVREGEN;
    HAL_Delay(1U);  /* regulator needs ≥ 20 µs; 1 ms tick is the minimum delay */

    /* Single-ended calibration (ADEN must be 0 during calibration) */
    ADC1->CR &= ~ADC_CR_ADCALDIF;          /* single-ended mode */
    ADC1->CR |=  ADC_CR_ADCAL;             /* start calibration  */
    while (ADC1->CR & ADC_CR_ADCAL) {}     /* wait for ADCAL=0   */

    /* Enable ADC and wait for hardware ready flag */
    ADC1->ISR  = ADC_ISR_ADRDY;            /* clear any stale ready flag */
    ADC1->CR  |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}

    /* CFGR: 12-bit (RES=00), overwrite-on-overrun, software trigger, single conv */
    ADC1->CFGR = ADC_CFGR_OVRMOD;

    /* SMPR1 SMP9 = 111 → 640.5 ADC clock cycles (maximum, best accuracy) */
    ADC1->SMPR1 = ADC_SMPR1_SMP9;

    /* SQR1: sequence length = 1 (L=0000), first conversion = channel 9 */
    ADC1->SQR1 = (9UL << ADC_SQR1_SQ1_Pos);
}

/**
  * @brief  Rate-limited task: read camera circuit current from H7 at 10 Hz.
  */
static void Camera_PollCurrentTask(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_lastCamCurrPollMs) < CAM_CURR_POLL_PERIOD_MS)
        return;
    s_lastCamCurrPollMs = now;

    float val;
    if (Camera_ReadCurrent_mA(&val))
        s_camCurrentMa = val;
}

/**
  * @brief  Rate-limited task: poll battery voltage from ADC1 at 10 Hz.
  *
  * Triggers a single software conversion on channel 9 (PB0) and converts
  * the raw 12-bit count to millivolts using an empirically calibrated constant.
  * Theoretical 68k/33k divider gives × 10100 / 4096; board measures 6.19 V at
  * 7.44 V true, so the constant is scaled by 7.44/6.19 → × 12140 / 4096.
  */
static void Battery_PollVoltageTask(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_lastBattVoltPollMs) < BATT_VOLT_POLL_PERIOD_MS)
        return;
    s_lastBattVoltPollMs = now;

    /* Start a single software-triggered conversion */
    ADC1->ISR  = ADC_ISR_EOC;          /* clear any previous end-of-conversion flag */
    ADC1->CR  |= ADC_CR_ADSTART;

    /* Poll for EOC with a 2 ms timeout */
    uint32_t deadline = HAL_GetTick() + 2U;
    while (!(ADC1->ISR & ADC_ISR_EOC))
    {
        if (HAL_GetTick() >= deadline)
            return;  /* timeout — discard this sample, try again next period */
    }

    /* Reading DR clears EOC automatically */
    uint32_t raw = ADC1->DR;

    /* Empirical calibration: 10100 × (7.44/6.19) ≈ 12140 */
    s_battVoltMv = (raw * 12140UL) / 4096UL;
}

static void FillAggregatePacket(AggregatePacket_t *packet, const SensorPacket_t *upstream)
{
    packet->header              = AGGREGATE_HEADER;
    packet->upstream            = *upstream;
    packet->esc_valid           = ESC_Telem_IsValid();
    packet->esc_temp_c          = ESC_Telem_GetTemp_C();
    packet->esc_voltage_mV      = ESC_Telem_GetVoltage_mV();
    packet->esc_current_mA      = ESC_Telem_GetCurrent_mA();
    packet->esc_consumption_mAh = ESC_Telem_GetConsumption_mAh();
    packet->esc_erpm            = ESC_Telem_GetERPM();
    packet->encoder_position_um = Encoder_GetPosition_um();
    packet->cam_current_mA      = s_camCurrentMa;
    packet->batt_voltage_mV     = s_battVoltMv;
    packet->checksum            = PacketChecksum(((const uint8_t *)packet) + sizeof(packet->header),
                                                ((const uint8_t *)packet) + sizeof(*packet) - sizeof(packet->checksum));
}

static void ForwardAggregatePacketIfReady(void)
{
    AggregatePacket_t packet;
    SensorPacket_t    upstream;
    uint32_t          now = HAL_GetTick();

    if (!s_upstreamPacketValid)
        return;

    if ((now - s_lastTelemetryTxMs) < TELEMETRY_TX_PERIOD_MS)
        return;

    if (s_upstreamPacketVersion == s_lastForwardedVersion)
        return;

    __disable_irq();
    upstream = s_upstreamPacket;
    __enable_irq();

    FillAggregatePacket(&packet, &upstream);
    HAL_UART_Transmit(&huart1, (uint8_t *)&packet, sizeof(packet), PACKET_TX_TIMEOUT);
#ifndef HIL_MODE
    /* In HIL mode USART2 RX carries binary HIL packets — don't pollute it
     * with outbound binary frames.  USART1 still forwards to any ground station. */
    HAL_UART_Transmit(&huart2, (uint8_t *)&packet, sizeof(packet), PACKET_TX_TIMEOUT);
#endif

    s_lastForwardedVersion = s_upstreamPacketVersion;
    s_lastTelemetryTxMs    = now;
}

static void ForwardFallbackPacketToUsart2IfNeeded(void)
{
    AggregatePacket_t packet;
    uint32_t          now = HAL_GetTick();

    if (s_upstreamPacketValid)
        return;

    if ((now - s_lastFallbackTxMs) < FALLBACK_TX_PERIOD_MS)
        return;

    memset(&packet, 0, sizeof(packet));
    packet.header   = AGGREGATE_HEADER;
    packet.checksum = PacketChecksum(((const uint8_t *)&packet) + sizeof(packet.header),
                                     ((const uint8_t *)&packet) + sizeof(packet) - sizeof(packet.checksum));

    HAL_UART_Transmit(&huart1, (uint8_t *)&packet, sizeof(packet), PACKET_TX_TIMEOUT);
#ifndef HIL_MODE
    HAL_UART_Transmit(&huart2, (uint8_t *)&packet, sizeof(packet), PACKET_TX_TIMEOUT);
#endif
    s_lastFallbackTxMs = now;
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

/* Forward declaration: used by polling helper below. */
static void ProcessConsoleByte(char c, char *buf, volatile uint32_t *idx);

/* =========================================================================
 * Private helpers — UART polling RX
 * ========================================================================= */

static void PollUartInputs(void)
{
    uint8_t  byte;
    uint32_t i;

    /* Optional console commands on USART1 */
    for (i = 0U; i < UART_POLL_BURST_MAX; i++)
    {
        if (!__HAL_UART_GET_FLAG(&huart1, USART_ISR_RXNE_RXFNE))
            break;

        byte = (uint8_t)(huart1.Instance->RDR & 0xFFU);

        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
            __HAL_UART_CLEAR_OREFLAG(&huart1);
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE))
            __HAL_UART_CLEAR_FEFLAG(&huart1);
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE))
            __HAL_UART_CLEAR_NEFLAG(&huart1);
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_PE))
            __HAL_UART_CLEAR_PEFLAG(&huart1);

        ProcessConsoleByte((char)byte, s_rxLine1, &s_rxIdx1);
    }

#ifndef HIL_MODE
    /* Optional console commands on USART2.
     * In HIL mode USART2 bytes are consumed by HIL_PollRx() which
     * already routes non-packet bytes to ProcessConsoleByte. */
    for (i = 0U; i < UART_POLL_BURST_MAX; i++)
    {
        if (!__HAL_UART_GET_FLAG(&huart2, USART_ISR_RXNE_RXFNE))
            break;

        byte = (uint8_t)(huart2.Instance->RDR & 0xFFU);

        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
            __HAL_UART_CLEAR_OREFLAG(&huart2);
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))
            __HAL_UART_CLEAR_FEFLAG(&huart2);
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE))
            __HAL_UART_CLEAR_NEFLAG(&huart2);
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE))
            __HAL_UART_CLEAR_PEFLAG(&huart2);

        ProcessConsoleByte((char)byte, s_rxLine2, &s_rxIdx2);
    }
#endif /* !HIL_MODE */
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
            return;
        if (Encoder_Homing_Is_Active())
            return;
        Encoder_Homing_Start();
        return;
    }

    /* ── 67 — start deployment sequence ─────────────────────────────────── */
    if (strcmp(upper, "67") == 0)
    {
        if (Encoder_Homing_Is_Active())
            return;
        if (Airbrake_Is_Sequence_Active())
            return;
        SD_Logger_Start();
        Airbrake_Start_Sequence();
        return;
    }

    /* ── UP — go to max deployment (70 deg) ────────────────────────────── */
    if (strcmp(upper, "UP") == 0)
    {
        if (Encoder_Homing_Is_Active())
            return;
        if (Airbrake_Is_Sequence_Active())
            return;
        Airbrake_GoToMax();
        return;
    }

    /* ── DOWN — go to zero position (0 deg) ────────────────────────────── */
    if (strcmp(upper, "DOWN") == 0)
    {
        if (Encoder_Homing_Is_Active())
            return;
        if (Airbrake_Is_Sequence_Active())
            return;
        Airbrake_GoToZero();
        return;
    }

    /* ── Camera commands ─────────────────────────────────────────────────── */

    /*
     * Helper: if the SPI byte fails to reach the H7, emit an immediate timeout
     * line on both downstream UARTs so the ground station isn't left waiting
     * for an echo that will never arrive.  If the SPI succeeds the H7 echoes
     * its own CMD + result lines via the normal poll loop.
     */
#define CAM_SPI_SEND_OR_TMO(cmd_byte, echo_cmd_str)                         \
    do {                                                                      \
        static const char _tmo[] = echo_cmd_str "CAM:RC:TMO\n";             \
        if (Camera_SendSpiByte(cmd_byte))                                    \
        {                                                                     \
            Camera_BeginPendingCommand(echo_cmd_str, _tmo);                  \
        }                                                                     \
        else                                                                  \
        {                                                                     \
            Camera_ForwardText(_tmo);                                         \
        }                                                                     \
    } while (0)

    if (strcmp(upper, "CAMSTART") == 0)
    {
        CAM_SPI_SEND_OR_TMO(SPI_CMD_CAMERA_START, "CAM:CMD:REC\n");
        return;
    }

    if (strcmp(upper, "CAMSTOP") == 0)
    {
        CAM_SPI_SEND_OR_TMO(SPI_CMD_CAMERA_STOP, "CAM:CMD:STP\n");
        return;
    }

    if (strcmp(upper, "CAMON") == 0)
    {
        CAM_SPI_SEND_OR_TMO(SPI_CMD_CAMERA_ON, "CAM:CMD:ON\n");
        return;
    }

    if (strcmp(upper, "CAMOFF") == 0)
    {
        CAM_SPI_SEND_OR_TMO(SPI_CMD_CAMERA_OFF, "CAM:CMD:OFF\n");
        return;
    }

#undef CAM_SPI_SEND_OR_TMO

    if (strcmp(upper, "CAMCURR") == 0)
    {
        float camera_current_mA = 0.0f;
        static const char tmo[] = "CAM:CMD:CUR\nCAM:RC:TMO\n";

        if (!Camera_ReadCurrent_mA(&camera_current_mA))
        {
            Camera_ForwardText(tmo);
        }
        else
        {
            Camera_BeginPendingCommand("CAM:CMD:CUR\n", tmo);
        }
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
            PWM_Set(s_neutralUs);
            return;
        }
    }

    /* ── O — zero encoder ───────────────────────────────────────────────── */
    if (strcmp(upper, "O") == 0)
    {
        Encoder_Reset();
        return;
    }

    /* ── N<us> — direct pulse width ─────────────────────────────────────── */
    if (upper[0] == 'N' && is_signed_integer(upper + 1))
    {
        int32_t pulse = clamp_i32((int32_t)strtol(cmd + 1, NULL, 10),
                                  PWM_MIN_US, PWM_MAX_US);
        PWM_Set(pulse);
        return;
    }

    /* ── F — slow forward jog ───────────────────────────────────────────── */
    if (strcmp(upper, "F") == 0)
    {
        PWM_Set(s_neutralUs + CREEP_OFFSET_US);
        return;
    }

    /* ── B — slow reverse jog ───────────────────────────────────────────── */
    if (strcmp(upper, "B") == 0)
    {
        PWM_Set(s_neutralUs - CREEP_OFFSET_US);
        return;
    }

    /* ── S / STOP ───────────────────────────────────────────────────────── */
    if (strcmp(upper, "S") == 0 || strcmp(upper, "STOP") == 0)
    {
        PWM_Set(s_neutralUs);
        return;
    }

    /* ── RESET — software reset via NVIC ───────────────────────────────────── */
    if (strcmp(upper, "RESET") == 0)
    {
        NVIC_SystemReset();
        return;
    }

    /* ── ALT<feet> — set target apogee altitude for airbrake controller ─────
     *   Example: "ALT10000" targets 10 000 ft (default).
     *   Accepted before launch or during coast.                          */
    if (strncmp(upper, "ALT", 3) == 0 && is_signed_integer(upper + 3))
    {
        float feet = (float)strtol(upper + 3, NULL, 10);
        Airbrake_Control_SetTargetFt(feet);
        return;
    }

    /* ── EPOCH<unix_seconds> — synchronise SD logger epoch clock ───────────
     *   Ground station sends "EPOCH1716000000\n" on connect.
     *   strtoul returns 0 on empty/invalid input, which the >0 guard rejects. */
    if (strncmp(upper, "EPOCH", 5) == 0)
    {
        uint32_t unix_sec = (uint32_t)strtoul(upper + 5, NULL, 10);
        if (unix_sec > 0U)
            SD_Logger_SetEpoch(unix_sec);
        return;
    }

    /* ── -100..100 — percentage throttle ────────────────────────────────── */
    if (is_signed_integer(upper))
    {
        int32_t pct = clamp_i32((int32_t)strtol(upper, NULL, 10), -100, 100);
        int32_t pulse = (pct >= 0)
            ? map_i32(pct,  0, 100, s_neutralUs, PWM_MAX_US)
            : map_i32(pct, -100, 0, PWM_MIN_US,  s_neutralUs);
        PWM_Set(pulse);
        return;
    }
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
    /* Single-letter operator commands execute immediately (no newline needed). */
    if (*idx == 0U)
    {
        char uc = (char)toupper((unsigned char)c);

        if (uc == 'F' || uc == 'B' || uc == 'S' || uc == 'O' || uc == 'H')
        {
            char cmd[2] = {uc, '\0'};
            HandleCommand(cmd);
            return;
        }
    }

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
 * Public API — sensor data
 * ========================================================================= */

/**
  * @brief  Copy the most recent validated upstream sensor packet.
  *
  * Disables interrupts briefly to prevent a torn read of the volatile
  * s_upstreamPacket field, which is written in IRQ context by
  * Upstream_ProcessByte().
  *
  * @param  out  Caller-supplied buffer; must not be NULL.
  * @retval 1 if at least one valid packet has been received, 0 otherwise.
  */
uint8_t ESC_App_GetLatestSensorPacket(SensorPacket_t *out)
{
    if (out == NULL)
        return 0U;

    __disable_irq();
    uint8_t valid = s_upstreamPacketValid;
    if (valid)
        *out = s_upstreamPacket;   /* interrupt-locked struct copy */
    __enable_irq();

    return valid;
}

float ESC_App_GetCamCurrent_mA(void)
{
    return s_camCurrentMa;
}

uint32_t ESC_App_GetBattVoltage_mV(void)
{
    return s_battVoltMv;
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
#ifndef HIL_MODE
    /* Normal operation: receive raw upstream packets from C5 board over UART4 */
    Upstream_EnableIrqRx();
#endif
    /* In HIL mode UART4 is left idle; HIL_PollRx() reads USART2 instead. */
    ESC_Telem_Init();

    /* ── Step 3: ESC arm hold ───────────────────────────────────────────── */
    HAL_Delay(ESC_ARM_HOLD_MS);

    /* ── Step 4: Application modules ────────────────────────────────────── */
    Command_Sequence_Init();
    Airbrake_Deploy_Init();
    Encoder_Homing_Init();
    Encoder_App_Init();
    HAL_TIM_Base_Start_IT(&htim4);
    Battery_ADC_Init();
    SD_Logger_Init();
    Flight_Trigger_Init();
    Estimator_Init();
    Airbrake_Control_Init();
}

void ESC_App_Task(void)
{
    /* Service all background state machines every loop iteration */
#ifdef HIL_MODE
    /* HIL mode: parse aggregate packets arriving on USART2 from the PC tool.
     * ASCII command bytes that arrive interleaved are forwarded to the
     * console handler inside HIL_PollRx(), so commands still work. */
    HIL_PollRx();
#else
    Upstream_EnsureIrqRxEnabled();
    Upstream_PollRx();
#endif
    PollUartInputs();
    ESC_Telem_Task();
    Encoder_App_Task();
    Encoder_Homing_Task();
    Estimator_Task();
    Airbrake_Control_Task();
    Airbrake_Deploy_Task();
    SD_Logger_Task();
    Flight_Trigger_Task();
    ForwardAggregatePacketIfReady();
    ForwardFallbackPacketToUsart2IfNeeded();
    Camera_PollEchoTask();
    Camera_CheckCommandTimeoutTask();
    Camera_PollCurrentTask();
    Battery_PollVoltageTask();

    if (Command_Sequence_Check_Deploy_Trigger())
    {
        Command_Sequence_Clear_Deploy_Trigger();
        if (!Encoder_Homing_Is_Active() && !Airbrake_Is_Sequence_Active())
        {
            SD_Logger_Start();
            Airbrake_Start_Sequence();
        }
    }

    if (Command_Sequence_Check_Homing_Trigger())
    {
        Command_Sequence_Clear_Homing_Trigger();
        if (!Airbrake_Is_Sequence_Active() && !Encoder_Homing_Is_Active())
            Encoder_Homing_Start();
    }

#ifdef HIL_MODE
    /* ── HIL status line at 10 Hz ──────────────────────────────────────────
     * Emitted on USART2 so hil_gui.py can track the estimated flight state
     * and the commanded airbrake angle.  The GUI parses "angle=X.Xdeg" to
     * update the commanded-angle plot and to feed the live RocketPy sim.
     *
     * Format (ASCII, newline terminated):
     *   [HIL] t=<ms> alt=<m>m vel=<mps>mps deploy=<0-1> angle=<deg>deg
     * -------------------------------------------------------------------- */
    {
        static uint32_t s_hilStatusTxMs = 0U;
        uint32_t hil_now = HAL_GetTick();
        if ((hil_now - s_hilStatusTxMs) >= 100U)
        {
            s_hilStatusTxMs = hil_now;
            EstimatorState_t est;
            Estimator_GetState(&est);
            float hil_deploy = Airbrake_Control_GetDeployFraction();
            float hil_angle  = hil_deploy * AIRBRAKE_MAX_ANGLE_DEG;
            char  hil_line[128];
            int   hil_n = snprintf(hil_line, sizeof(hil_line),
                "[HIL] t=%lu alt=%.0fm vel=%.1fmps deploy=%.3f angle=%.1fdeg\r\n",
                (unsigned long)hil_now,
                (double)est.altitude_m,
                (double)est.velocity_mps,
                (double)hil_deploy,
                (double)hil_angle);
            if (hil_n > 0 && hil_n < (int)sizeof(hil_line))
                HAL_UART_Transmit(&huart2, (uint8_t *)hil_line, (uint16_t)hil_n, 20U);
        }
    }
#endif /* HIL_MODE */
}

/* =========================================================================
 * UART interrupt callbacks
 * ========================================================================= */

void ESC_App_Uart4IrqHandler(void)
{
#ifdef HIL_MODE
    /* UART4 is not used in HIL mode (RXNEIE is never enabled).
     * If the IRQ somehow fires, clear all pending flags and bail. */
    huart4.Instance->ICR = 0xFFFFFFFFU;
    return;
#endif

    uint32_t isr = huart4.Instance->ISR;

    s_uart4LastIsr = isr;
    s_uart4LastCr1 = huart4.Instance->CR1;
    s_uart4LastCr2 = huart4.Instance->CR2;
    s_uart4LastCr3 = huart4.Instance->CR3;
    s_uart4LastErrorCode = 0U;

    while ((__HAL_UART_GET_FLAG(&huart4, USART_ISR_RXNE_RXFNE)) != 0U)
    {
        uint8_t byte = (uint8_t)(huart4.Instance->RDR & 0xFFU);
        s_uart4RxCount++;
        Upstream_ProcessByte(byte);
    }

    if ((isr & USART_ISR_ORE) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4OreCount++;
        s_uart4LastErrorCode |= HAL_UART_ERROR_ORE;
        s_upstreamRxIdx = 0U;
        s_headerFirstByte = 0U;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF);
    }

    if ((isr & USART_ISR_FE) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4FeCount++;
        s_uart4LastErrorCode |= HAL_UART_ERROR_FE;
        s_upstreamRxIdx = 0U;
        s_headerFirstByte = 0U;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_FEF);
    }

    if ((isr & USART_ISR_NE) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4NeCount++;
        s_uart4LastErrorCode |= HAL_UART_ERROR_NE;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_NEF);
    }

    if ((isr & USART_ISR_PE) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4PeCount++;
        s_uart4LastErrorCode |= HAL_UART_ERROR_PE;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_PEF);
    }

    if ((isr & USART_ISR_RTOF) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4RtofCount++;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_RTOF);
    }

    if ((isr & USART_ISR_CMF) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4CmfCount++;
        __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_CMF);
    }

    if ((isr & USART_ISR_EOBF) != 0U)
    {
        s_uart4ErrorCount++;
        s_uart4EobfCount++;
        huart4.Instance->ICR = USART_ICR_EOBCF;
    }
}

