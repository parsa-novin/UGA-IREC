/**
  ******************************************************************************
  * @file    esc_app.h
  * @brief   AM32 bidirectional ESC controller — public interface.
  ******************************************************************************
  */

#ifndef ESC_APP_H
#define ESC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =========================================================================
 * Upstream sensor packet type
 *
 * Defined here (rather than only inside esc_app.c) so that sd_logger.c
 * and any other consumer can snapshot the struct without duplicating the
 * layout.  The packed attribute and field order must match the wire format
 * sent by the upstream sensor board exactly.
 * ========================================================================= */

#define UPSTREAM_PACKET_LEN  75U

typedef struct __attribute__((packed))
{
    uint16_t header;       /**< 0xAA55 — upstream frame sync word            */
    int32_t  altitude;     /**< Barometric altitude (raw int32 from board)   */
    int32_t  pressure;     /**< Raw pressure value from upstream board       */
    float    imu16_ax;     /**< IMU16 accelerometer X (g)                    */
    float    imu16_ay;     /**< IMU16 accelerometer Y (g)                    */
    float    imu16_az;     /**< IMU16 accelerometer Z (g)                    */
    float    imu4_ax;      /**< IMU4  accelerometer X (g)                    */
    float    imu4_ay;      /**< IMU4  accelerometer Y (g)                    */
    float    imu4_az;      /**< IMU4  accelerometer Z (g)                    */
    float    mag_x;        /**< Magnetometer X                               */
    float    mag_y;        /**< Magnetometer Y                               */
    float    mag_z;        /**< Magnetometer Z                               */
    float    bmi_ax;       /**< BMI accelerometer X (g)                      */
    float    bmi_ay;       /**< BMI accelerometer Y (g)                      */
    float    bmi_az;       /**< BMI accelerometer Z (g)                      */
    float    bmi_gx;       /**< BMI gyroscope X (deg/s)                      */
    float    bmi_gy;       /**< BMI gyroscope Y (deg/s)                      */
    float    bmi_gz;       /**< BMI gyroscope Z (deg/s)                      */
    float    ext_temp;     /**< External temperature from upstream board     */
    uint8_t  checksum;     /**< XOR of all bytes after the header            */
} SensorPacket_t;

/* Compile-time check — wire format is exactly UPSTREAM_PACKET_LEN bytes */
_Static_assert(sizeof(SensorPacket_t) == UPSTREAM_PACKET_LEN,
               "SensorPacket_t size must match upstream wire format");

/* =========================================================================
 * Public API — init / task
 * ========================================================================= */

/**
  * @brief  Initialise all application sub-modules, arm the ESC, start UARTs.
  *         Blocks for ESC_ARM_HOLD_MS milliseconds on first call.
  */
void ESC_App_Init(void);

/**
  * @brief  Main-loop task — service all sub-module state machines.
  */
void ESC_App_Task(void);

/**
  * @brief  Dedicated UART4 IRQ handler for upstream packet RX.
  *         Uses direct register reads instead of HAL byte-by-byte RX.
  */
void ESC_App_Uart4IrqHandler(void);

/* =========================================================================
 * Public API — motor control (called by sequence and homing modules)
 * ========================================================================= */

void    ESC_App_SetPulseUs(int32_t pulseUs);
void    ESC_App_StopMotor(void);
void    ESC_App_SetPercent(int32_t pct);
int32_t ESC_App_GetNeutralUs(void);

/* =========================================================================
 * Public API — sensor data
 * ========================================================================= */

/**
  * @brief  Copy the most recent validated upstream sensor packet.
  *
  * Thread-safe: briefly disables interrupts to prevent a torn read of the
  * volatile s_upstreamPacket buffer.
  *
  * @param  out  Destination buffer; must not be NULL.
  * @retval 1 if a valid packet has been received at least once, 0 otherwise.
  */
uint8_t ESC_App_GetLatestSensorPacket(SensorPacket_t *out);

/**
  * @brief  Return the most recently polled camera circuit current in milliamps.
  *         Updated at ~10 Hz by Camera_PollCurrentTask().
  *         Returns 0.0f until the first successful SPI read.
  */
float ESC_App_GetCamCurrent_mA(void);

/**
  * @brief  Return the most recently measured battery voltage in millivolts.
  *         Updated at ~10 Hz by Battery_PollVoltageTask().
  *         Returns 0 until the ADC has been read at least once.
  */
uint32_t ESC_App_GetBattVoltage_mV(void);

#ifdef __cplusplus
}
#endif

#endif /* ESC_APP_H */
