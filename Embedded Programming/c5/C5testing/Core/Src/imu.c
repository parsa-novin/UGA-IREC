/*
 * imu.c
 *
 * Driver for the InvenSense IIM-42352 3-axis accelerometer over 4-wire SPI.
 *
 * SPI protocol (from datasheet §9.6):
 *   - MSB first, data latched on rising edge (CPOL=0, CPHA=1 — Mode 0/3)
 *   - First byte: [R/W | A6 | A5 | A4 | A3 | A2 | A1 | A0]
 *     R/W=1 → read,  R/W=0 → write
 *   - Max SCLK: 24 MHz
 *
 * Multiple register banks are selected via REG_BANK_SEL (0x76).
 * All registers used here are in Bank 0 (default after reset).
 */

#include "imu.h"
#include <math.h>

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define SPI_TIMEOUT_MS   50u

/* ── Register addresses (Bank 0 unless noted) ────────────────────────────── */
#define REG_DEVICE_CONFIG    0x11u   /* soft reset                           */
#define REG_INT_CONFIG       0x14u
#define REG_TEMP_DATA1       0x1Du   /* temperature MSB                      */
#define REG_TEMP_DATA0       0x1Eu   /* temperature LSB                      */
#define REG_ACCEL_DATA_X1    0x1Fu   /* accel X MSB – base of 6-byte block   */
#define REG_ACCEL_DATA_X0    0x20u
#define REG_ACCEL_DATA_Y1    0x21u
#define REG_ACCEL_DATA_Y0    0x22u
#define REG_ACCEL_DATA_Z1    0x23u
#define REG_ACCEL_DATA_Z0    0x24u
#define REG_INT_STATUS       0x2Du
#define REG_INTF_CONFIG0     0x4Cu
#define REG_INTF_CONFIG1     0x4Du
#define REG_PWR_MGMT0        0x4Eu   /* accelerometer power mode             */
#define REG_ACCEL_CONFIG0    0x50u   /* FS select + ODR                      */
#define REG_WHO_AM_I         0x75u   /* always reads 0x6D                    */
#define REG_BANK_SEL         0x76u   /* register bank selection              */

/* ── Register bit definitions ────────────────────────────────────────────── */
#define SOFT_RESET_BIT       0x01u   /* DEVICE_CONFIG bit 0                  */
#define ACCEL_MODE_LN        0x03u   /* PWR_MGMT0 bits 1:0 = Low-Noise mode  */
#define WHO_AM_I_VALUE       0x6Du

/* ACCEL_CONFIG0: FS in bits 7:5, ODR in bits 3:0 */
#define ACCEL_ODR_1KHZ       0x06u   /* 1 kHz – LN mode default              */

/* SPI R/W flag */
#define SPI_READ             0x80u
#define SPI_WRITE            0x00u

/* ── External SPI handle (defined in spi.c / CubeMX) ────────────────────── */
extern SPI_HandleTypeDef hspi1;

/* ── Sensitivity table (LSB/g) indexed by AccelFS enum ──────────────────── */
static const float k_lsb_per_g[4] = {
    2048.0f,   /* ACCEL_FS_16G */
    4096.0f,   /* ACCEL_FS_8G  */
    8192.0f,   /* ACCEL_FS_4G  */
    16384.0f,  /* ACCEL_FS_2G  */
};

/* ── Private helpers ─────────────────────────────────────────────────────── */

static inline void cs_assert(IMU_Handle *imu)
{
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(IMU_Handle *imu)
{
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
}

/* Raw SPI transfer: tx and rx must both be len bytes long. */
static void spi_transfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint8_t dummy_rx[len];
    uint8_t dummy_tx[len];

    if (rx == NULL) rx = dummy_rx;
    if (tx == NULL) {
        for (uint16_t i = 0; i < len; i++) dummy_tx[i] = 0x00u;
        tx = dummy_tx;
    }
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, SPI_TIMEOUT_MS);
}

/* Write one byte to a register. */
static void imu_write_reg(IMU_Handle *imu, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { (uint8_t)(SPI_WRITE | reg), data };
    cs_assert(imu);
    spi_transfer(tx, NULL, 2);
    cs_deassert(imu);
}

/* Read one byte from a register. */
static uint8_t imu_read_reg(IMU_Handle *imu, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(SPI_READ | reg), 0x00u };
    uint8_t rx[2] = { 0x00u, 0x00u };
    cs_assert(imu);
    spi_transfer(tx, rx, 2);
    cs_deassert(imu);
    return rx[1];
}

/* Burst-read len bytes starting at reg into buf. */
static void imu_read_burst(IMU_Handle *imu, uint8_t reg,
                           uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + len];
    uint8_t rx[1 + len];
    tx[0] = (uint8_t)(SPI_READ | reg);
    for (uint8_t i = 1; i <= len; i++) tx[i] = 0x00u;

    cs_assert(imu);
    spi_transfer(tx, rx, 1 + len);
    cs_deassert(imu);

    for (uint8_t i = 0; i < len; i++) buf[i] = rx[1 + i];
}

/* Switch register bank (0–4). Normally only Bank 0 is needed here. */
static void imu_select_bank(IMU_Handle *imu, uint8_t bank)
{
    imu_write_reg(imu, REG_BANK_SEL, bank & 0x07u);
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool IMU_init(IMU_Handle *imu, GPIO_TypeDef *cs_port,
              uint16_t cs_pin, AccelFS fs)
{
    imu->cs_port = cs_port;
    imu->cs_pin  = cs_pin;

    /* De-assert CS before touching the bus */
    cs_deassert(imu);
    HAL_Delay(10u);

    /* Soft reset – bit 0 of DEVICE_CONFIG */
    imu_write_reg(imu, REG_DEVICE_CONFIG, SOFT_RESET_BIT);
    HAL_Delay(2u);   /* datasheet: wait ≥1 ms after reset */

    /* Verify device identity */
    if (imu_read_reg(imu, REG_WHO_AM_I) != WHO_AM_I_VALUE) {
        return false;
    }

    /* Set full-scale range and ODR (1 kHz, LN mode) */
    IMU_setFullScale(imu, fs);

    /* Enable accelerometer in Low-Noise mode.
     * After transitioning from OFF, wait ≥200 µs before any register write. */
    imu_write_reg(imu, REG_PWR_MGMT0, ACCEL_MODE_LN);
    HAL_Delay(1u);   /* >200 µs */

    return true;
}

void IMU_setFullScale(IMU_Handle *imu, AccelFS fs)
{
    imu->fs         = fs;
    imu->lsb_per_g  = k_lsb_per_g[fs];

    /* ACCEL_CONFIG0: bits 7:5 = FS_SEL, bits 3:0 = ODR */
    uint8_t cfg = (uint8_t)(((uint8_t)fs << 5u) | ACCEL_ODR_1KHZ);
    imu_write_reg(imu, REG_ACCEL_CONFIG0, cfg);
}

void IMU_getRawAccel(IMU_Handle *imu,
                     int16_t *rx, int16_t *ry, int16_t *rz)
{
    uint8_t buf[6];
    imu_read_burst(imu, REG_ACCEL_DATA_X1, buf, 6u);

    *rx = (int16_t)(((uint16_t)buf[0] << 8u) | buf[1]);
    *ry = (int16_t)(((uint16_t)buf[2] << 8u) | buf[3]);
    *rz = (int16_t)(((uint16_t)buf[4] << 8u) | buf[5]);
}

void IMU_getAccel(IMU_Handle *imu, float *ax, float *ay, float *az)
{
    int16_t rx, ry, rz;
    IMU_getRawAccel(imu, &rx, &ry, &rz);
    float scale = 1.0f / imu->lsb_per_g;
    *ax = (float)rx * scale;
    *ay = (float)ry * scale;
    *az = (float)rz * scale;
}

float IMU_getTemp(IMU_Handle *imu)
{
    uint8_t buf[2];
    imu_read_burst(imu, REG_TEMP_DATA1, buf, 2u);
    int16_t raw = (int16_t)(((uint16_t)buf[0] << 8u) | buf[1]);
    /* Formula from datasheet §14.6: T(°C) = raw/132.48 + 25 */
    return ((float)raw / 132.48f) + 25.0f;
}
