#include "runcam_device.h"
#include <string.h>

static RunCam_StatusTypeDef RunCam_HalToStatus(HAL_StatusTypeDef halStatus)
{
    switch (halStatus)
    {
        case HAL_OK:
            return RUNCAM_OK;
        case HAL_TIMEOUT:
            return RUNCAM_TIMEOUT;
        default:
            return RUNCAM_ERROR;
    }
}

static RunCam_StatusTypeDef RunCam_VerifyResponse(const uint8_t *rx, uint16_t rxLen)
{
    if ((rx == NULL) || (rxLen < 2U))
    {
        return RUNCAM_BAD_PARAM;
    }

    if (rx[0] != RUNCAM_HEADER)
    {
        return RUNCAM_BAD_HEADER;
    }

    if (RunCam_Crc8(rx, (uint16_t)(rxLen - 1U)) != rx[rxLen - 1U])
    {
        return RUNCAM_BAD_CRC;
    }

    return RUNCAM_OK;
}

void RunCam_AttachUart(RunCam_HandleTypeDef *hrc,
                       UART_HandleTypeDef *huart,
                       uint32_t timeoutMs)
{
    if (hrc == NULL)
    {
        return;
    }

    hrc->huart = huart;
    hrc->protocolVersion = 0U;
    hrc->features = 0U;
    hrc->timeoutMs = (timeoutMs == 0U) ? RUNCAM_DEFAULT_TIMEOUT_MS : timeoutMs;
}

uint8_t RunCam_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;

    if (data == NULL)
    {
        return crc;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)((crc << 1U) ^ 0xD5U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

void RunCam_FlushRx(RunCam_HandleTypeDef *hrc)
{
    uint8_t dummy;

    if ((hrc == NULL) || (hrc->huart == NULL))
    {
        return;
    }

    while (HAL_UART_Receive(hrc->huart, &dummy, 1U, 1U) == HAL_OK)
    {
        /* discard stale bytes */
    }
}

uint8_t RunCam_HasFeature(const RunCam_HandleTypeDef *hrc, uint16_t featureMask)
{
    if (hrc == NULL)
    {
        return 0U;
    }

    return ((hrc->features & featureMask) == featureMask) ? 1U : 0U;
}

// Add to runcam_device.c
// Cycles ChangeMode until the camera LED indicates video mode,
// but since we have no LED feedback over UART on Split 4,
// we use a known-state approach: send ChangeMode twice to
// guarantee we land on video (Photo->Video->Photo means 1 press
// if in photo, 0 if already in video — so query first).
RunCam_StatusTypeDef RunCam_EnsureVideoMode(RunCam_HandleTypeDef *hrc)
{
    if (hrc == NULL)
    {
        return RUNCAM_BAD_PARAM;
    }

    // Re-query device info to get fresh feature state
    RunCam_StatusTypeDef status = RunCam_GetDeviceInfo(hrc);
    if (status != RUNCAM_OK)
    {
        return status;
    }

    if (!RunCam_HasFeature(hrc, RUNCAM_FEATURE_START_RECORDING))
    {
        // Camera doesn't support recording — already in wrong mode
        // or feature not advertised yet. Try a mode change.
        RunCam_ChangeMode(hrc);
        HAL_Delay(1500);

        // Re-query to see if recording feature appeared
        status = RunCam_GetDeviceInfo(hrc);
        if (status != RUNCAM_OK)
        {
            return status;
        }
    }

    return RunCam_HasFeature(hrc, RUNCAM_FEATURE_START_RECORDING)
               ? RUNCAM_OK
               : RUNCAM_UNSUPPORTED;
}

RunCam_StatusTypeDef RunCam_SendPacket(RunCam_HandleTypeDef *hrc,
                                       uint8_t cmd,
                                       const uint8_t *payload,
                                       uint16_t payloadLen)
{
    uint8_t tx[80];
    uint16_t txLen;
    HAL_StatusTypeDef halStatus;

    if ((hrc == NULL) || (hrc->huart == NULL))
    {
        return RUNCAM_BAD_PARAM;
    }

    if (payloadLen > (uint16_t)(sizeof(tx) - 3U))
    {
        return RUNCAM_BAD_PARAM;
    }

    tx[0] = RUNCAM_HEADER;
    tx[1] = cmd;

    if ((payloadLen > 0U) && (payload != NULL))
    {
        memcpy(&tx[2], payload, payloadLen);
    }

    txLen = (uint16_t)(2U + payloadLen + 1U);
    tx[txLen - 1U] = RunCam_Crc8(tx, (uint16_t)(txLen - 1U));

    halStatus = HAL_UART_Transmit(hrc->huart, tx, txLen, hrc->timeoutMs);
    return RunCam_HalToStatus(halStatus);
}

RunCam_StatusTypeDef RunCam_SendPacketAndRead(RunCam_HandleTypeDef *hrc,
                                              uint8_t cmd,
                                              const uint8_t *payload,
                                              uint16_t payloadLen,
                                              uint8_t *rx,
                                              uint16_t rxLen)
{
    RunCam_StatusTypeDef status;
    HAL_StatusTypeDef halStatus;

    if ((hrc == NULL) || (hrc->huart == NULL) || (rx == NULL) || (rxLen == 0U))
    {
        return RUNCAM_BAD_PARAM;
    }

    RunCam_FlushRx(hrc);

    status = RunCam_SendPacket(hrc, cmd, payload, payloadLen);
    if (status != RUNCAM_OK)
    {
        return status;
    }

    halStatus = HAL_UART_Receive(hrc->huart, rx, rxLen, hrc->timeoutMs);
    status = RunCam_HalToStatus(halStatus);
    if (status != RUNCAM_OK)
    {
        return status;
    }

    return RunCam_VerifyResponse(rx, rxLen);
}

RunCam_StatusTypeDef RunCam_GetDeviceInfo(RunCam_HandleTypeDef *hrc)
{
    uint8_t rx[5];
    RunCam_StatusTypeDef status;

    status = RunCam_SendPacketAndRead(hrc,
                                      RUNCAM_CMD_GET_DEVICE_INFO,
                                      NULL,
                                      0U,
                                      rx,
                                      sizeof(rx));
    if (status != RUNCAM_OK)
    {
        return status;
    }

    hrc->protocolVersion = rx[1];
    hrc->features = (uint16_t)rx[2] | ((uint16_t)rx[3] << 8U);

    return RUNCAM_OK;
}

RunCam_StatusTypeDef RunCam_CameraControl(RunCam_HandleTypeDef *hrc, uint8_t action)
{
    return RunCam_SendPacket(hrc, RUNCAM_CMD_CAMERA_CONTROL, &action, 1U);
}

RunCam_StatusTypeDef RunCam_WiFiButton(RunCam_HandleTypeDef *hrc)
{
    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_SIMULATE_WIFI_BUTTON))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_WIFI_BTN);
}

RunCam_StatusTypeDef RunCam_PowerButton(RunCam_HandleTypeDef *hrc)
{
    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_SIMULATE_POWER_BUTTON))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_POWER_BTN);
}

RunCam_StatusTypeDef RunCam_ChangeMode(RunCam_HandleTypeDef *hrc)
{
    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_CHANGE_MODE))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_CHANGE_MODE);
}

RunCam_StatusTypeDef RunCam_StartRecording(RunCam_HandleTypeDef *hrc)
{
    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_START_RECORDING))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_START_RECORDING);
}

RunCam_StatusTypeDef RunCam_StopRecording(RunCam_HandleTypeDef *hrc)
{
    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_STOP_RECORDING))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_STOP_RECORDING);
}

RunCam_StatusTypeDef RunCam_5KeyOpenConnection(RunCam_HandleTypeDef *hrc)
{
    uint8_t action = RUNCAM_5KEY_OPEN;
    uint8_t rx[3];
    RunCam_StatusTypeDef status;

    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
    {
        return RUNCAM_UNSUPPORTED;
    }

    status = RunCam_SendPacketAndRead(hrc,
                                      RUNCAM_CMD_5KEY_CONNECTION,
                                      &action,
                                      1U,
                                      rx,
                                      sizeof(rx));
    if (status != RUNCAM_OK)
    {
        return status;
    }

    return (((rx[1] >> 4U) == RUNCAM_5KEY_OPEN) && ((rx[1] & 0x0FU) == 1U)) ? RUNCAM_OK : RUNCAM_ERROR;
}

RunCam_StatusTypeDef RunCam_5KeyCloseConnection(RunCam_HandleTypeDef *hrc)
{
    uint8_t action = RUNCAM_5KEY_CLOSE;
    uint8_t rx[3];
    RunCam_StatusTypeDef status;

    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
    {
        return RUNCAM_UNSUPPORTED;
    }

    status = RunCam_SendPacketAndRead(hrc,
                                      RUNCAM_CMD_5KEY_CONNECTION,
                                      &action,
                                      1U,
                                      rx,
                                      sizeof(rx));
    if (status != RUNCAM_OK)
    {
        return status;
    }

    return (((rx[1] >> 4U) == RUNCAM_5KEY_CLOSE) && ((rx[1] & 0x0FU) == 1U)) ? RUNCAM_OK : RUNCAM_ERROR;
}

RunCam_StatusTypeDef RunCam_5KeyPress(RunCam_HandleTypeDef *hrc, uint8_t key)
{
    uint8_t rx[2];

    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
    {
        return RUNCAM_UNSUPPORTED;
    }

    switch (key)
    {
        case RUNCAM_5KEY_SET:
        case RUNCAM_5KEY_LEFT:
        case RUNCAM_5KEY_RIGHT:
        case RUNCAM_5KEY_UP:
        case RUNCAM_5KEY_DOWN:
            break;
        default:
            return RUNCAM_BAD_PARAM;
    }

    return RunCam_SendPacketAndRead(hrc,
                                    RUNCAM_CMD_5KEY_PRESS,
                                    &key,
                                    1U,
                                    rx,
                                    sizeof(rx));
}

RunCam_StatusTypeDef RunCam_5KeyRelease(RunCam_HandleTypeDef *hrc)
{
    uint8_t rx[2];

    if ((hrc != NULL) && (hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_SendPacketAndRead(hrc,
                                    RUNCAM_CMD_5KEY_RELEASE,
                                    NULL,
                                    0U,
                                    rx,
                                    sizeof(rx));
}

RunCam_StatusTypeDef RunCam_RequestFcAttitude(RunCam_HandleTypeDef *hrc,
                                               RunCam_FcAttitudeTypeDef *attitude)
{
    uint8_t rx[8];
    RunCam_StatusTypeDef status;

    if ((hrc == NULL) || (attitude == NULL))
    {
        return RUNCAM_BAD_PARAM;
    }

    if ((hrc->features != 0U) && !RunCam_HasFeature(hrc, RUNCAM_FEATURE_FC_ATTITUDE))
    {
        return RUNCAM_UNSUPPORTED;
    }

    status = RunCam_SendPacketAndRead(hrc,
                                      RUNCAM_CMD_REQUEST_FC_ATTITUDE,
                                      NULL,
                                      0U,
                                      rx,
                                      sizeof(rx));
    if (status != RUNCAM_OK)
    {
        return status;
    }

    attitude->roll  = (int16_t)((uint16_t)rx[1] | ((uint16_t)rx[2] << 8U));
    attitude->pitch = (int16_t)((uint16_t)rx[3] | ((uint16_t)rx[4] << 8U));
    attitude->yaw   = (int16_t)((uint16_t)rx[5] | ((uint16_t)rx[6] << 8U));

    return RUNCAM_OK;
}
