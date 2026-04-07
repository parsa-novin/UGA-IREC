/*
 * magnetometer.c
 *
 * Driver for the MEMSIC MMC5983MA 3-axis magnetometer over 4-wire SPI.
 *
 * SPI protocol:
 *   - MSB first, CPOL=0 / CPHA=0 (Mode 0)
 *   - First byte: [R/W | A6 | A5 | A4 | A3 | A2 | A1 | A0]
 *     R/W=1 → read,  R/W=0 → write
 *   - Reads: one transaction, address byte + N data bytes (no dummy byte)
 *
 * 18-bit output reconstruction (7 burst bytes, registers 0x00–0x06):
 *   X = Xout0[7:0]<<10 | Xout1[7:0]<<2 | Xyzout2[7:6]
 *   Y = Yout0[7:0]<<10 | Yout1[7:0]<<2 | Xyzout2[5:4]
 *   Z = Zout0[7:0]<<10 | Zout1[7:0]<<2 | Xyzout2[3:2]
 *
 * Null field = 131072 (2^17).  Sensitivity = 16384 counts/Gauss.
 */

#include "magnetometer.h"

/* ── Timing ─────────────────────────────────────────────────────────────── */
#define SPI_TIMEOUT_MS  50u

/* ── Register addresses ─────────────────────────────────────────────────── */
#define REG_XOUT0       0x00u   /* X-axis MSBs [17:10]                       */
#define REG_TOUT        0x07u   /* Temperature output (1 byte)               */
#define REG_STATUS1     0x08u   /* Status: bit1=Meas_T_Done, bit0=Meas_M_Done*/
#define REG_CTRL0       0x09u   /* Internal control 0                        */
#define REG_CTRL1       0x0Au   /* Internal control 1 (BW, SW_RST)          */
#define REG_PRODUCT_ID  0x2Fu   /* Product ID: 0x30                          */

/* ── Register bit masks ─────────────────────────────────────────────────── */
#define CTRL0_TM_M      0x01u   /* Trigger one magnetic measurement          */
#define CTRL0_TM_T      0x02u   /* Trigger one temperature measurement       */
#define CTRL0_DO_RESET  0x08u   /* Issue RESET pulse                         */
#define CTRL0_DO_SET    0x10u   /* Issue SET pulse                           */
#define STATUS_DONE_M   0x01u   /* Meas_M_Done bit                           */
#define STATUS_DONE_T   0x02u   /* Meas_T_Done bit                           */
#define PRODUCT_ID_VAL  0x30u

/* ── Magnetometer output constants ──────────────────────────────────────── */
#define MMC_NULL_FIELD  131072.0f   /* Zero-gauss offset (2^17)              */
#define MMC_SENS        16384.0f    /* Counts per Gauss                      */

/* ── SPI flags ───────────────────────────────────────────────────────────── */
#define SPI_READ   0x80u
#define SPI_WRITE  0x00u

/* ── External SPI handle (defined in spi.c / CubeMX) ───────────────────── */
extern SPI_HandleTypeDef hspi1;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static inline void cs_assert(MMC_Handle *mag)
{
    HAL_GPIO_WritePin(mag->cs_port, mag->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(MMC_Handle *mag)
{
    HAL_GPIO_WritePin(mag->cs_port, mag->cs_pin, GPIO_PIN_SET);
}

static void mmc_write_reg(MMC_Handle *mag, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { (uint8_t)(SPI_WRITE | reg), data };
    uint8_t rx[2];
    cs_assert(mag);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, SPI_TIMEOUT_MS);
    cs_deassert(mag);
}

static uint8_t mmc_read_reg(MMC_Handle *mag, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(SPI_READ | reg), 0x00u };
    uint8_t rx[2] = { 0x00u, 0x00u };
    cs_assert(mag);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, SPI_TIMEOUT_MS);
    cs_deassert(mag);
    return rx[1];
}

/* Burst-read len bytes starting at reg into buf. */
static void mmc_read_burst(MMC_Handle *mag, uint8_t reg,
                            uint8_t *buf, uint8_t len)
{
    uint8_t tx[1u + len];
    uint8_t rx[1u + len];
    tx[0] = (uint8_t)(SPI_READ | reg);
    for (uint8_t i = 1u; i <= len; i++) tx[i] = 0x00u;

    cs_assert(mag);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(1u + len), SPI_TIMEOUT_MS);
    cs_deassert(mag);

    for (uint8_t i = 0u; i < len; i++) buf[i] = rx[1u + i];
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void MMC_set(MMC_Handle *mag)
{
    mmc_write_reg(mag, REG_CTRL0, CTRL0_DO_SET);
    HAL_Delay(1u);   /* SET pulse completes in <1 ms */
}

void MMC_reset(MMC_Handle *mag)
{
    mmc_write_reg(mag, REG_CTRL0, CTRL0_DO_RESET);
    HAL_Delay(1u);
}

bool MMC_init(MMC_Handle *mag)
{
    cs_deassert(mag);
    HAL_Delay(10u);

    /* Issue a SET pulse to initialise the sensor bridge */
    MMC_set(mag);

    /* Verify Product ID */
    return (mmc_read_reg(mag, REG_PRODUCT_ID) == PRODUCT_ID_VAL);
}

void MMC_getMag(MMC_Handle *mag, float *x, float *y, float *z)
{
    /* Trigger one measurement */
    mmc_write_reg(mag, REG_CTRL0, CTRL0_TM_M);

    /* Poll until Meas_M_Done (~8 ms at BW=00) */
    while (!(mmc_read_reg(mag, REG_STATUS1) & STATUS_DONE_M)) {
        HAL_Delay(1u);
    }

    /* Burst-read 7 bytes: Xout0–Xout1, Yout0–Yout1, Zout0–Zout1, Xyzout2 */
    uint8_t buf[7];
    mmc_read_burst(mag, REG_XOUT0, buf, 7u);

    /* Reconstruct 18-bit unsigned counts for each axis */
    uint32_t xc = ((uint32_t)buf[0] << 10u) | ((uint32_t)buf[1] << 2u) | ((buf[6] >> 6u) & 0x03u);
    uint32_t yc = ((uint32_t)buf[2] << 10u) | ((uint32_t)buf[3] << 2u) | ((buf[6] >> 4u) & 0x03u);
    uint32_t zc = ((uint32_t)buf[4] << 10u) | ((uint32_t)buf[5] << 2u) | ((buf[6] >> 2u) & 0x03u);

    *x = ((float)xc - MMC_NULL_FIELD) / MMC_SENS;
    *y = ((float)yc - MMC_NULL_FIELD) / MMC_SENS;
    *z = ((float)zc - MMC_NULL_FIELD) / MMC_SENS;
}

float MMC_getTemp(MMC_Handle *mag)
{
    /* Trigger one temperature measurement */
    mmc_write_reg(mag, REG_CTRL0, CTRL0_TM_T);

    /* Poll until Meas_T_Done */
    while (!(mmc_read_reg(mag, REG_STATUS1) & STATUS_DONE_T)) {
        HAL_Delay(1u);
    }

    uint8_t raw = mmc_read_reg(mag, REG_TOUT);

    /* T(°C) = counts × 0.8 − 75   (range: 0 → −75 °C, 250 → +125 °C) */
    return (float)raw * 0.8f - 75.0f;
}
