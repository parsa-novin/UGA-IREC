#ifndef __RUNCAM_DEVICE_H__
#define __RUNCAM_DEVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stddef.h>

/*
 * RunCam Device Protocol
 * Header byte: 0xCC
 * CRC8: DVB-S2 polynomial 0xD5, initial value 0x00
 */

#define RUNCAM_HEADER                           0xCCU
#define RUNCAM_DEFAULT_TIMEOUT_MS               100U

/* Command IDs */
#define RUNCAM_CMD_GET_DEVICE_INFO              0x00U
#define RUNCAM_CMD_CAMERA_CONTROL               0x01U
#define RUNCAM_CMD_5KEY_PRESS                   0x02U
#define RUNCAM_CMD_5KEY_RELEASE                 0x03U
#define RUNCAM_CMD_5KEY_CONNECTION              0x04U
#define RUNCAM_CMD_REQUEST_FC_ATTITUDE          0x50U

/* Camera control action IDs */
#define RUNCAM_ACTION_WIFI_BTN                  0x00U
#define RUNCAM_ACTION_POWER_BTN                 0x01U
#define RUNCAM_ACTION_CHANGE_MODE               0x02U
#define RUNCAM_ACTION_START_RECORDING           0x03U
#define RUNCAM_ACTION_STOP_RECORDING            0x04U

/* 5-key OSD press actions */
#define RUNCAM_5KEY_SET                         0x01U
#define RUNCAM_5KEY_LEFT                        0x02U
#define RUNCAM_5KEY_RIGHT                       0x03U
#define RUNCAM_5KEY_UP                          0x04U
#define RUNCAM_5KEY_DOWN                        0x05U

/* 5-key connection actions */
#define RUNCAM_5KEY_OPEN                        0x01U
#define RUNCAM_5KEY_CLOSE                       0x02U

/* Feature flags returned by GET_DEVICE_INFO */
#define RUNCAM_FEATURE_SIMULATE_POWER_BUTTON    (1U << 0)
#define RUNCAM_FEATURE_SIMULATE_WIFI_BUTTON     (1U << 1)
#define RUNCAM_FEATURE_CHANGE_MODE              (1U << 2)
#define RUNCAM_FEATURE_SIMULATE_5_KEY_OSD       (1U << 3)
#define RUNCAM_FEATURE_SETTINGS_ACCESS          (1U << 4)
#define RUNCAM_FEATURE_DISPLAYPORT              (1U << 5)
#define RUNCAM_FEATURE_START_RECORDING          (1U << 6)
#define RUNCAM_FEATURE_STOP_RECORDING           (1U << 7)
#define RUNCAM_FEATURE_FC_ATTITUDE              (1U << 9)

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t protocolVersion;
    uint16_t features;
    uint32_t timeoutMs;
} RunCam_HandleTypeDef;

typedef enum
{
    RUNCAM_OK = 0,
    RUNCAM_ERROR,
    RUNCAM_TIMEOUT,
    RUNCAM_BAD_PARAM,
    RUNCAM_BAD_CRC,
    RUNCAM_BAD_HEADER,
    RUNCAM_UNSUPPORTED
} RunCam_StatusTypeDef;

typedef struct
{
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} RunCam_FcAttitudeTypeDef;

void RunCam_AttachUart(RunCam_HandleTypeDef *hrc,
                       UART_HandleTypeDef *huart,
                       uint32_t timeoutMs);

uint8_t RunCam_Crc8(const uint8_t *data, uint16_t len);
void RunCam_FlushRx(RunCam_HandleTypeDef *hrc);
uint8_t RunCam_HasFeature(const RunCam_HandleTypeDef *hrc, uint16_t featureMask);

RunCam_StatusTypeDef RunCam_GetDeviceInfo(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_CameraControl(RunCam_HandleTypeDef *hrc, uint8_t action);

RunCam_StatusTypeDef RunCam_WiFiButton(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_PowerButton(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_ChangeMode(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_StartRecording(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_StopRecording(RunCam_HandleTypeDef *hrc);

RunCam_StatusTypeDef RunCam_5KeyOpenConnection(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_5KeyCloseConnection(RunCam_HandleTypeDef *hrc);
RunCam_StatusTypeDef RunCam_5KeyPress(RunCam_HandleTypeDef *hrc, uint8_t key);
RunCam_StatusTypeDef RunCam_5KeyRelease(RunCam_HandleTypeDef *hrc);

RunCam_StatusTypeDef RunCam_RequestFcAttitude(RunCam_HandleTypeDef *hrc,
                                               RunCam_FcAttitudeTypeDef *attitude);

RunCam_StatusTypeDef RunCam_EnsureVideoMode(RunCam_HandleTypeDef *hrc);

/* Generic raw helpers if you want to extend this later */
RunCam_StatusTypeDef RunCam_SendPacket(RunCam_HandleTypeDef *hrc,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payloadLen);

RunCam_StatusTypeDef RunCam_SendPacketAndRead(RunCam_HandleTypeDef *hrc,
                                              uint8_t cmd,
                                              const uint8_t *payload,
                                              uint16_t payloadLen,
                                              uint8_t *rx,
                                              uint16_t rxLen);

#ifdef __cplusplus
}
#endif

#endif /* __RUNCAM_DEVICE_H__ */
