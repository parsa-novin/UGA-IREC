/*
 * temperature.c
 *
 * Driver for the Texas Instruments TMP127-Q1 14-bit SPI temperature sensor.
 *
 * SPI protocol (datasheet §8.5.2):
 *   - CPOL=0, CPHA=0 (Mode 0) — also compatible with Mode 3
 *   - 32-bit transaction per CS assertion, MSB first
 *   - Bits [31:16] received: 14-bit temperature in [15:2], always 11b in [1:0]
 *   - Bits [15:0]  sent:     configuration register
 *       0x0000 → continuous conversion mode (normal operation)
 *       0x00FF → shutdown mode
 *
 * Temperature formula:
 *   raw16 = received 16-bit word
 *   T(°C) = (int16_t)(raw16) >> 2   × 0.03125
 *   (right-shift by 2 discards the two fixed '11b' LSBs; signed shift
 *    preserves the sign bit for negative temperatures)
 */

#include "temperature.h"

/* ── Timing ─────────────────────────────────────────────────────────────── */
#define SPI_TIMEOUT_MS  50u

/* ── Configuration words ─────────────────────────────────────────────────── */
#define CFG_CONTINUOUS  0x0000u   /* continuous conversion mode */

/* ── External SPI handle (defined in spi.c / CubeMX) ───────────────────── */
extern SPI_HandleTypeDef hspi1;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static inline void cs_assert(TMP127_Handle *tmp)
{
    HAL_GPIO_WritePin(tmp->cs_port, tmp->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(TMP127_Handle *tmp)
{
    HAL_GPIO_WritePin(tmp->cs_port, tmp->cs_pin, GPIO_PIN_SET);
}

/*
 * Execute one 32-bit SPI frame.
 * tx_cfg: 16-bit configuration word written in the second half of the frame.
 * Returns the 16-bit temperature/device-ID word received in the first half.
 */
static uint16_t tmp127_transfer(TMP127_Handle *tmp, uint16_t tx_cfg)
{
    uint8_t tx[4] = {
        0x00u,                            /* first 16 bits — don't care (device drives SIO) */
        0x00u,
        (uint8_t)(tx_cfg >> 8u),          /* configuration MSB */
        (uint8_t)(tx_cfg & 0xFFu)         /* configuration LSB */
    };
    uint8_t rx[4] = { 0u, 0u, 0u, 0u };

    cs_assert(tmp);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4u, SPI_TIMEOUT_MS);
    cs_deassert(tmp);

    return (uint16_t)(((uint16_t)rx[0] << 8u) | rx[1]);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void TMP127_init(TMP127_Handle *tmp, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    tmp->cs_port = cs_port;
    tmp->cs_pin  = cs_pin;

    cs_deassert(tmp);
    HAL_Delay(1u);

    /* Write continuous conversion mode; discard the (possibly erroneous) result */
    tmp127_transfer(tmp, CFG_CONTINUOUS);

    /* Datasheet §8.5.2.2: wait one full conversion period before first valid read */
    HAL_Delay(300u);   /* > 270 ms max conversion period */
}

float TMP127_getTemp(TMP127_Handle *tmp)
{
    /* Read temperature; keep device in continuous conversion mode */
    uint16_t word = tmp127_transfer(tmp, CFG_CONTINUOUS);

    /* Bits [15:2] = 14-bit two's-complement temperature, bits [1:0] always 11b */
    int16_t raw = (int16_t)word >> 2;   /* arithmetic right-shift preserves sign */

    return (float)raw * 0.03125f;
}
