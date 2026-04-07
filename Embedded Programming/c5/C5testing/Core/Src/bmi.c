/*
 * bmi.c
 *
 * Bare-metal STM32 HAL driver for the Bosch BMI270 6-axis IMU (backup IMU).
 *
 * SPI protocol (datasheet §6.4):
 *   - MSB first, CPOL=0 / CPHA=0 (Mode 0) — also compatible with Mode 3
 *   - R/W bit = bit 7 of the address byte (1=read, 0=write)
 *   - Reads: ONE dummy byte must be discarded immediately after the address
 *     byte.  Single-register read = 3 bytes total (addr, dummy, data).
 *   - Writes: address byte followed immediately by data, no dummy byte.
 *
 * SPI mode selection (datasheet §6.1):
 *   The BMI270 powers up in I2C mode.  A CS toggle (low → high) switches it
 *   to SPI mode for the current power cycle.  BMI_init() performs this via a
 *   short CS pulse before the first real transaction.
 *
 * Config blob upload (datasheet §3.2 / Bosch Application Note):
 *   The internal feature microcontroller REQUIRES the config file to be
 *   loaded before enabling the accelerometer or gyroscope.  Without it,
 *   INTERNAL_STATUS[3:0] stays 0x00 and sensor data may be unreliable.
 *   The blob is embedded below (source: bmi270_maximum_fifo.c, Bosch
 *   Sensortec, BSD-3-Clause) and uploaded automatically inside BMI_init().
 *
 * Upload sequence:
 *   1. Disable adv_power_save (PWR_CONF = 0x00), wait > 450 µs
 *   2. INIT_CTRL (0x59) = 0x00  — start config load
 *   3. For each 8-byte chunk N:
 *        Write half-word address N/2 to INIT_ADDR_0/1 (0x5B/0x5C)
 *        Burst-write 8 bytes to INIT_DATA (0x5E)
 *   4. INIT_CTRL (0x59) = 0x01  — end config load
 *   5. Wait 20 ms, then check INTERNAL_STATUS (0x21) bits[3:0] == 0x01
 */

#include "bmi.h"

/* ── Timing ──────────────────────────────────────────────────────────────── */
#define SPI_TIMEOUT_MS   50u
#define BMI_CFG_CHUNK    8u     /* bytes per config upload packet              */

/* ── Register addresses ──────────────────────────────────────────────────── */
#define REG_CHIP_ID          0x00u
#define REG_INTERNAL_STATUS  0x21u
#define REG_TEMP_LSB         0x22u   /* TEMP_0 (LSB), TEMP_1 (MSB) at 0x23    */
#define REG_ACC_X_LSB        0x0Cu   /* base of 6-byte accel block             */
#define REG_GYR_X_LSB        0x12u   /* base of 6-byte gyro  block             */
#define REG_INIT_CTRL        0x59u
#define REG_INIT_ADDR_0      0x5Bu   /* low  byte of 12-bit half-word address  */
#define REG_INIT_ADDR_1      0x5Cu   /* high byte of 12-bit half-word address  */
#define REG_INIT_DATA        0x5Eu
#define REG_ACC_CONF         0x40u
#define REG_ACC_RANGE        0x41u
#define REG_GYR_CONF         0x42u
#define REG_GYR_RANGE        0x43u
#define REG_PWR_CONF         0x7Cu
#define REG_PWR_CTRL         0x7Du
#define REG_CMD              0x7Eu

/* ── Register values ─────────────────────────────────────────────────────── */
#define CHIP_ID_VAL          0x24u
#define CMD_SOFT_RESET       0xB6u
#define INTERNAL_STATUS_OK   0x01u

/* ACC_CONF: perf_mode=1, bwp=010 (normal), odr=0x08 (100 Hz) → 0xA8 */
#define ACC_CONF_VAL         0xA8u
/* GYR_CONF: filter_perf=1, noise_perf=0, bwp=10 (normal), odr=0x09 (200 Hz) → 0xA9 */
#define GYR_CONF_VAL         0xA9u
/* PWR_CTRL: temp_en | gyr_en | acc_en */
#define PWR_CTRL_ALL_EN      0x0Eu

/* ── Sensitivity look-up tables ──────────────────────────────────────────── */
/* LSB/g  for  2g / 4g / 8g / 16g  (ACC_RANGE 0x00…0x03) */
static const float k_acc_lsb[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
/* LSB/dps for 2000 / 1000 / 500 / 250 / 125 dps (GYR_RANGE 0x00…0x04) */
static const float k_gyr_lsb[5] = { 16.384f, 32.768f, 65.536f, 131.072f, 262.144f };

/* ── Internal SPI buffer (sized for the largest burst: 1+1+12 = 14 bytes) ─ */
#define BMI_BUF_MAX  32u

/* ── Config blob (Bosch Sensortec, BSD-3-Clause, bmi270_maximum_fifo.c) ─── */
static const uint8_t s_bmi270_cfg[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x1a, 0x00, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
    0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e,
    0x90, 0x32, 0x21, 0x2e, 0x59, 0xf5, 0x10, 0x30, 0x21, 0x2e, 0x6a, 0xf5, 0x1a, 0x24, 0x22, 0x00,
    0x80, 0x2e, 0x3b, 0x00, 0xc8, 0x2e, 0x44, 0x47, 0x22, 0x00, 0x37, 0x00, 0xa4, 0x00, 0xff, 0x0f,
    0xd1, 0x00, 0x07, 0xad, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1, 0x80, 0x2e, 0x00, 0xc1,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x24, 0xfc, 0xf5, 0x80, 0x30, 0x40, 0x42, 0x50, 0x50,
    0x00, 0x30, 0x12, 0x24, 0xeb, 0x00, 0x03, 0x30, 0x00, 0x2e, 0xc1, 0x86, 0x5a, 0x0e, 0xfb, 0x2f,
    0x21, 0x2e, 0xfc, 0xf5, 0x13, 0x24, 0x63, 0xf5, 0xe0, 0x3c, 0x48, 0x00, 0x22, 0x30, 0xf7, 0x80,
    0xc2, 0x42, 0xe1, 0x7f, 0x3a, 0x25, 0xfc, 0x86, 0xf0, 0x7f, 0x41, 0x33, 0x98, 0x2e, 0xc2, 0xc4,
    0xd6, 0x6f, 0xf1, 0x30, 0xf1, 0x08, 0xc4, 0x6f, 0x11, 0x24, 0xff, 0x03, 0x12, 0x24, 0x00, 0xfc,
    0x61, 0x09, 0xa2, 0x08, 0x36, 0xbe, 0x2a, 0xb9, 0x13, 0x24, 0x38, 0x00, 0x64, 0xbb, 0xd1, 0xbe,
    0x94, 0x0a, 0x71, 0x08, 0xd5, 0x42, 0x21, 0xbd, 0x91, 0xbc, 0xd2, 0x42, 0xc1, 0x42, 0x00, 0xb2,
    0xfe, 0x82, 0x05, 0x2f, 0x50, 0x30, 0x21, 0x2e, 0x21, 0xf2, 0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e,
    0xf0, 0x6f, 0x02, 0x30, 0x02, 0x42, 0x20, 0x26, 0xe0, 0x6f, 0x02, 0x31, 0x03, 0x40, 0x9a, 0x0a,
    0x02, 0x42, 0xf0, 0x37, 0x05, 0x2e, 0x5e, 0xf7, 0x10, 0x08, 0x12, 0x24, 0x1e, 0xf2, 0x80, 0x42,
    0x83, 0x84, 0xf1, 0x7f, 0x0a, 0x25, 0x13, 0x30, 0x83, 0x42, 0x3b, 0x82, 0xf0, 0x6f, 0x00, 0x2e,
    0x00, 0x2e, 0xd0, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x00, 0x2e, 0x12, 0x40, 0x52, 0x42, 0x3e, 0x84,
    0x00, 0x40, 0x40, 0x42, 0x7e, 0x82, 0xe1, 0x7f, 0xf2, 0x7f, 0x98, 0x2e, 0x6a, 0xd6, 0x21, 0x30,
    0x23, 0x2e, 0x61, 0xf5, 0xeb, 0x2c, 0xe1, 0x6f
};

/* ── External SPI handle (defined in spi.c / CubeMX) ───────────────────── */
extern SPI_HandleTypeDef hspi1;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static inline void cs_assert(BMI_Handle *bmi)
{
    HAL_GPIO_WritePin(bmi->cs_port, bmi->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_deassert(BMI_Handle *bmi)
{
    HAL_GPIO_WritePin(bmi->cs_port, bmi->cs_pin, GPIO_PIN_SET);
}

/* Single-register read.  Returns rx[2]; rx[1] is the required dummy byte. */
static uint8_t bmi_read_reg(BMI_Handle *bmi, uint8_t reg)
{
    uint8_t tx[3] = { reg | 0x80u, 0x00u, 0x00u };
    uint8_t rx[3] = { 0x00u, 0x00u, 0x00u };
    cs_assert(bmi);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3u, SPI_TIMEOUT_MS);
    cs_deassert(bmi);
    return rx[2];
}

/* Single-register write.  No dummy byte needed. */
static void bmi_write_reg(BMI_Handle *bmi, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7Fu, val };
    uint8_t rx[2];
    cs_assert(bmi);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, SPI_TIMEOUT_MS);
    cs_deassert(bmi);
}

/*
 * Burst read len bytes from contiguous registers starting at reg.
 * Total transfer = len + 2 bytes (addr + dummy + data).
 * Caller must ensure len + 2 <= BMI_BUF_MAX.
 */
static void bmi_read_burst(BMI_Handle *bmi, uint8_t reg,
                            uint8_t *buf, uint8_t len)
{
    uint8_t tx[BMI_BUF_MAX] = {0};
    uint8_t rx[BMI_BUF_MAX] = {0};
    tx[0] = reg | 0x80u;
    cs_assert(bmi);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 2u), SPI_TIMEOUT_MS);
    cs_deassert(bmi);
    /* rx[0]=addr echo, rx[1]=dummy, rx[2..len+1]=data */
    for (uint8_t i = 0u; i < len; i++) {
        buf[i] = rx[i + 2u];
    }
}

/*
 * Burst write len bytes to contiguous registers starting at reg.
 * Caller must ensure len + 1 <= BMI_BUF_MAX.
 */
static void bmi_write_burst(BMI_Handle *bmi, uint8_t reg,
                             const uint8_t *data, uint8_t len)
{
    uint8_t tx[BMI_BUF_MAX];
    uint8_t rx[BMI_BUF_MAX];
    tx[0] = reg & 0x7Fu;
    for (uint8_t i = 0u; i < len; i++) {
        tx[i + 1u] = data[i];
    }
    cs_assert(bmi);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 1u), SPI_TIMEOUT_MS);
    cs_deassert(bmi);
}

/*
 * Upload the config blob to INIT_DATA in BMI_CFG_CHUNK-byte packets.
 *
 * INIT_ADDR_0 (0x5B) and INIT_ADDR_1 (0x5C) hold the 12-bit half-word
 * address of the next packet:
 *   INIT_ADDR_0 bits[3:0] = hw_index[3:0]
 *   INIT_ADDR_1 bits[7:0] = hw_index[11:4]
 * hw_index advances by (chunk_bytes / 2) half-words per packet.
 */
static void bmi_upload_config(BMI_Handle *bmi)
{
    const uint16_t total = (uint16_t)sizeof(s_bmi270_cfg);
    uint16_t hw_index = 0u;

    for (uint16_t offset = 0u; offset < total; offset += BMI_CFG_CHUNK) {
        uint8_t chunk = (uint8_t)(((offset + BMI_CFG_CHUNK) <= total)
                                  ? BMI_CFG_CHUNK
                                  : (uint8_t)(total - offset));

        /* Write INIT_ADDR_0 and INIT_ADDR_1 as a 2-byte burst to 0x5B */
        uint8_t addr_bytes[2] = {
            (uint8_t)(hw_index & 0x0Fu),          /* INIT_ADDR_0: index[3:0]  */
            (uint8_t)((hw_index >> 4u) & 0xFFu)   /* INIT_ADDR_1: index[11:4] */
        };
        bmi_write_burst(bmi, REG_INIT_ADDR_0, addr_bytes, 2u);

        /* Write chunk bytes to INIT_DATA (auto-increments internally) */
        bmi_write_burst(bmi, REG_INIT_DATA, &s_bmi270_cfg[offset], chunk);

        hw_index += (uint16_t)(chunk / 2u);
    }
}

/* ── Public API ──────────────────────────────────────────────────────────── */

bool BMI_init(BMI_Handle *bmi,
              GPIO_TypeDef *cs_port, uint16_t cs_pin,
              BMI_AccelFS acc_fs, BMI_GyroFS gyr_fs)
{
    bmi->cs_port = cs_port;
    bmi->cs_pin  = cs_pin;

    /* Pre-assert CS high, then pulse low briefly to select SPI mode (§6.1) */
    cs_deassert(bmi);
    HAL_Delay(5u);
    cs_assert(bmi);
    HAL_Delay(1u);
    cs_deassert(bmi);
    HAL_Delay(1u);

    /* Soft reset */
    bmi_write_reg(bmi, REG_CMD, CMD_SOFT_RESET);
    HAL_Delay(100u);   /* power-on + reset sequence */

    /* Re-enter SPI mode after reset (device reverts to I2C on power-cycle) */
    cs_assert(bmi);
    HAL_Delay(1u);
    cs_deassert(bmi);
    HAL_Delay(2u);

    /* Verify chip ID */
    uint8_t id = bmi_read_reg(bmi, REG_CHIP_ID);
    if (id != CHIP_ID_VAL) {
        return false;
    }

    /* ── Config blob upload sequence ── */

    /* 1. Disable advanced power save (mandatory before config upload) */
    bmi_write_reg(bmi, REG_PWR_CONF, 0x00u);
    HAL_Delay(1u);    /* > 450 µs */

    /* 2. Assert INIT_CTRL = 0 */
    bmi_write_reg(bmi, REG_INIT_CTRL, 0x00u);

    /* 3. Upload blob in 8-byte chunks */
    bmi_upload_config(bmi);

    /* 4. Assert INIT_CTRL = 1 */
    bmi_write_reg(bmi, REG_INIT_CTRL, 0x01u);
    HAL_Delay(20u);

    /* 5. Check INTERNAL_STATUS[3:0] == 0x01 (init_ok) */
    uint8_t status = bmi_read_reg(bmi, REG_INTERNAL_STATUS);
    if ((status & 0x0Fu) != INTERNAL_STATUS_OK) {
        return false;
    }

    /* ── Sensor configuration ── */
    bmi->acc_fs        = acc_fs;
    bmi->acc_lsb_per_g = k_acc_lsb[(uint8_t)acc_fs & 0x03u];
    bmi_write_reg(bmi, REG_ACC_CONF,  ACC_CONF_VAL);
    bmi_write_reg(bmi, REG_ACC_RANGE, (uint8_t)acc_fs);

    bmi->gyr_fs          = gyr_fs;
    bmi->gyr_lsb_per_dps = k_gyr_lsb[(uint8_t)gyr_fs & 0x07u];
    bmi_write_reg(bmi, REG_GYR_CONF,  GYR_CONF_VAL);
    bmi_write_reg(bmi, REG_GYR_RANGE, (uint8_t)gyr_fs);

    /* Enable accelerometer, gyroscope, and temperature */
    bmi_write_reg(bmi, REG_PWR_CTRL, PWR_CTRL_ALL_EN);
    HAL_Delay(50u);

    return true;
}

void BMI_getAccel(BMI_Handle *bmi, float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    bmi_read_burst(bmi, REG_ACC_X_LSB, buf, 6u);

    int16_t rx = (int16_t)(((uint16_t)buf[1] << 8u) | buf[0]);
    int16_t ry = (int16_t)(((uint16_t)buf[3] << 8u) | buf[2]);
    int16_t rz = (int16_t)(((uint16_t)buf[5] << 8u) | buf[4]);

    *ax = (float)rx / bmi->acc_lsb_per_g;
    *ay = (float)ry / bmi->acc_lsb_per_g;
    *az = (float)rz / bmi->acc_lsb_per_g;
}

void BMI_getGyro(BMI_Handle *bmi, float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    bmi_read_burst(bmi, REG_GYR_X_LSB, buf, 6u);

    int16_t rx = (int16_t)(((uint16_t)buf[1] << 8u) | buf[0]);
    int16_t ry = (int16_t)(((uint16_t)buf[3] << 8u) | buf[2]);
    int16_t rz = (int16_t)(((uint16_t)buf[5] << 8u) | buf[4]);

    *gx = (float)rx / bmi->gyr_lsb_per_dps;
    *gy = (float)ry / bmi->gyr_lsb_per_dps;
    *gz = (float)rz / bmi->gyr_lsb_per_dps;
}

void BMI_getAllMotion(BMI_Handle *bmi,
                     float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz)
{
    /*
     * ACC and GYR registers are contiguous: 0x0C–0x17 (12 bytes total).
     * Read them in one burst to minimise CS toggling and sampling skew.
     */
    uint8_t buf[12];
    bmi_read_burst(bmi, REG_ACC_X_LSB, buf, 12u);

    int16_t arx = (int16_t)(((uint16_t)buf[1]  << 8u) | buf[0]);
    int16_t ary = (int16_t)(((uint16_t)buf[3]  << 8u) | buf[2]);
    int16_t arz = (int16_t)(((uint16_t)buf[5]  << 8u) | buf[4]);
    int16_t grx = (int16_t)(((uint16_t)buf[7]  << 8u) | buf[6]);
    int16_t gry = (int16_t)(((uint16_t)buf[9]  << 8u) | buf[8]);
    int16_t grz = (int16_t)(((uint16_t)buf[11] << 8u) | buf[10]);

    *ax = (float)arx / bmi->acc_lsb_per_g;
    *ay = (float)ary / bmi->acc_lsb_per_g;
    *az = (float)arz / bmi->acc_lsb_per_g;
    *gx = (float)grx / bmi->gyr_lsb_per_dps;
    *gy = (float)gry / bmi->gyr_lsb_per_dps;
    *gz = (float)grz / bmi->gyr_lsb_per_dps;
}

float BMI_getTemp(BMI_Handle *bmi)
{
    /*
     * TEMP_LSB (0x22) and TEMP_MSB (0x23): 16-bit signed, little-endian.
     * T(°C) = raw / 512.0 + 23.0   (datasheet §4.13)
     */
    uint8_t buf[2];
    bmi_read_burst(bmi, REG_TEMP_LSB, buf, 2u);

    int16_t raw = (int16_t)(((uint16_t)buf[1] << 8u) | buf[0]);
    return (float)raw / 512.0f + 23.0f;
}
