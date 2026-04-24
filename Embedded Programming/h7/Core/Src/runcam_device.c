/**
  ******************************************************************************
  * @file    runcam_device.c
  * @brief   RunCam protocol driver — camera control over UART.
  *
  * Implements the RunCam Device Protocol v1/v2:
  *   https://support.runcam.com/hc/en-us/articles/360014537794
  *
  * All public API functions follow the same null-guard convention:
  *   - If the handle or a required pointer is NULL, return RUNCAM_BAD_PARAM.
  *   - If the feature bitmap is non-zero and the required feature is absent,
  *     return RUNCAM_UNSUPPORTED (avoids sending commands the camera rejects).
  ******************************************************************************
  */

#include "runcam_device.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Private helpers                                                             */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Map a HAL status to a RunCam status.
  */
static RunCam_StatusTypeDef RunCam_HalToStatus(HAL_StatusTypeDef halStatus)
{
    switch (halStatus)
    {
        case HAL_OK:      return RUNCAM_OK;
        case HAL_TIMEOUT: return RUNCAM_TIMEOUT;
        default:          return RUNCAM_ERROR;
    }
}

/**
  * @brief  Verify that a received buffer contains a valid RunCam response.
  *
  * Checks for the protocol header byte and validates the trailing CRC-8.
  *
  * @param  rx     Pointer to the received byte buffer.
  * @param  rxLen  Number of bytes in the buffer (must be >= 2).
  * @retval RUNCAM_OK on success, error code otherwise.
  */
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

/**
  * @brief  Return non-zero if the handle's features field is initialised AND
  *         does NOT contain the requested feature mask.
  *
  *         When features == 0 the device info has not been queried yet, so we
  *         allow the command through rather than incorrectly blocking it.
  */
static uint8_t RunCam_FeatureMissing(const RunCam_HandleTypeDef *hrc,
                                     uint16_t featureMask)
{
    if (hrc == NULL)
    {
        return 1U; /* treat NULL handle as "feature absent" */
    }

    if (hrc->features == 0U)
    {
        return 0U; /* features not yet known — allow through */
    }

    return ((hrc->features & featureMask) != featureMask) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void RunCam_AttachUart(RunCam_HandleTypeDef *hrc,
                       UART_HandleTypeDef   *huart,
                       uint32_t              timeoutMs)
{
    if (hrc == NULL)
    {
        return;
    }

    hrc->huart           = huart;
    hrc->protocolVersion = 0U;
    hrc->features        = 0U;
    hrc->timeoutMs       = (timeoutMs == 0U) ? RUNCAM_DEFAULT_TIMEOUT_MS : timeoutMs;
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

    /* Drain the UART RX FIFO one byte at a time with a 1 ms timeout each */
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

RunCam_StatusTypeDef RunCam_EnsureVideoMode(RunCam_HandleTypeDef *hrc)
{
    RunCam_StatusTypeDef status;

    if (hrc == NULL)
    {
        return RUNCAM_BAD_PARAM;
    }

    /* Re-query device info to get a fresh feature set */
    status = RunCam_GetDeviceInfo(hrc);
    if (status != RUNCAM_OK)
    {
        return status;
    }

    if (!RunCam_HasFeature(hrc, RUNCAM_FEATURE_START_RECORDING))
    {
        /*
         * START_RECORDING not advertised — camera is likely in photo mode.
         * Send one ChangeMode and allow time for the mode transition before
         * re-querying.
         */
        (void)RunCam_ChangeMode(hrc);
        HAL_Delay(1500U);

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
                                       uint8_t               cmd,
                                       const uint8_t        *payload,
                                       uint16_t              payloadLen)
{
    /* Header (1) + cmd (1) + payload + CRC (1) */
    uint8_t           tx[80];
    uint16_t          txLen;
    HAL_StatusTypeDef halStatus;

    if ((hrc == NULL) || (hrc->huart == NULL))
    {
        return RUNCAM_BAD_PARAM;
    }

    /* Ensure payload fits leaving room for header, cmd, and CRC */
    if (payloadLen > (uint16_t)(sizeof(tx) - 3U))
    {
        return RUNCAM_BAD_PARAM;
    }

    tx[0] = RUNCAM_HEADER;
    tx[1] = cmd;

    if ((payloadLen > 0U) && (payload != NULL))
    {
        memcpy(&tx[2], payload, (size_t)payloadLen);
    }

    txLen          = (uint16_t)(2U + payloadLen + 1U);
    tx[txLen - 1U] = RunCam_Crc8(tx, (uint16_t)(txLen - 1U));

    halStatus = HAL_UART_Transmit(hrc->huart, tx, txLen, hrc->timeoutMs);
    return RunCam_HalToStatus(halStatus);
}

RunCam_StatusTypeDef RunCam_SendPacketAndRead(RunCam_HandleTypeDef *hrc,
                                              uint8_t               cmd,
                                              const uint8_t        *payload,
                                              uint16_t              payloadLen,
                                              uint8_t              *rx,
                                              uint16_t              rxLen)
{
    RunCam_StatusTypeDef status;
    HAL_StatusTypeDef    halStatus;

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
    status    = RunCam_HalToStatus(halStatus);
    if (status != RUNCAM_OK)
    {
        return status;
    }

    return RunCam_VerifyResponse(rx, rxLen);
}

RunCam_StatusTypeDef RunCam_GetDeviceInfo(RunCam_HandleTypeDef *hrc)
{
    /*
     * Response layout (5 bytes):
     *   [0] Header
     *   [1] Protocol version
     *   [2] Features low byte
     *   [3] Features high byte
     *   [4] CRC-8
     */
    uint8_t              rx[5];
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
    hrc->features        = (uint16_t)rx[2] | ((uint16_t)rx[3] << 8U);

    return RUNCAM_OK;
}

RunCam_StatusTypeDef RunCam_CameraControl(RunCam_HandleTypeDef *hrc, uint8_t action)
{
    if (hrc == NULL)
    {
        return RUNCAM_BAD_PARAM;
    }

    return RunCam_SendPacket(hrc, RUNCAM_CMD_CAMERA_CONTROL, &action, 1U);
}

RunCam_StatusTypeDef RunCam_WiFiButton(RunCam_HandleTypeDef *hrc)
{
    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_SIMULATE_WIFI_BUTTON))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_WIFI_BTN);
}

RunCam_StatusTypeDef RunCam_PowerButton(RunCam_HandleTypeDef *hrc)
{
    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_SIMULATE_POWER_BUTTON))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_POWER_BTN);
}

RunCam_StatusTypeDef RunCam_ChangeMode(RunCam_HandleTypeDef *hrc)
{
    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_CHANGE_MODE))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_CHANGE_MODE);
}

RunCam_StatusTypeDef RunCam_StartRecording(RunCam_HandleTypeDef *hrc)
{
    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_START_RECORDING))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_START_RECORDING);
}

RunCam_StatusTypeDef RunCam_StopRecording(RunCam_HandleTypeDef *hrc)
{
    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_STOP_RECORDING))
    {
        return RUNCAM_UNSUPPORTED;
    }

    return RunCam_CameraControl(hrc, RUNCAM_ACTION_STOP_RECORDING);
}

RunCam_StatusTypeDef RunCam_5KeyOpenConnection(RunCam_HandleTypeDef *hrc)
{
    uint8_t              action = RUNCAM_5KEY_OPEN;
    uint8_t              rx[3];
    RunCam_StatusTypeDef status;

    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
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

    return (((rx[1] >> 4U) == RUNCAM_5KEY_OPEN) && ((rx[1] & 0x0FU) == 1U))
               ? RUNCAM_OK
               : RUNCAM_ERROR;
}

RunCam_StatusTypeDef RunCam_5KeyCloseConnection(RunCam_HandleTypeDef *hrc)
{
    uint8_t              action = RUNCAM_5KEY_CLOSE;
    uint8_t              rx[3];
    RunCam_StatusTypeDef status;

    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
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

    return (((rx[1] >> 4U) == RUNCAM_5KEY_CLOSE) && ((rx[1] & 0x0FU) == 1U))
               ? RUNCAM_OK
               : RUNCAM_ERROR;
}

RunCam_StatusTypeDef RunCam_5KeyPress(RunCam_HandleTypeDef *hrc, uint8_t key)
{
    uint8_t rx[2];

    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
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

    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_SIMULATE_5_KEY_OSD))
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

RunCam_StatusTypeDef RunCam_RequestFcAttitude(RunCam_HandleTypeDef    *hrc,
                                               RunCam_FcAttitudeTypeDef *attitude)
{
    /*
     * Response layout (8 bytes):
     *   [0]     Header
     *   [1..2]  Roll  (int16, little-endian)
     *   [3..4]  Pitch (int16, little-endian)
     *   [5..6]  Yaw   (int16, little-endian)
     *   [7]     CRC-8
     */
    uint8_t              rx[8];
    RunCam_StatusTypeDef status;

    if ((hrc == NULL) || (attitude == NULL))
    {
        return RUNCAM_BAD_PARAM;
    }

    if (RunCam_FeatureMissing(hrc, RUNCAM_FEATURE_FC_ATTITUDE))
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
