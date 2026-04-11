#include "ina219.h"

#include <stdint.h>

static INA219_StatusTypeDef INA219_WriteReg16(INA219_HandleTypeDef *hina,
                                              uint8_t reg,
                                              uint16_t value)
{
    uint8_t tx[2];

    tx[0] = (uint8_t)(value >> 8);
    tx[1] = (uint8_t)(value & 0xFFU);

    if (HAL_I2C_Mem_Write(hina->hi2c,
                          (uint16_t)(INA219_I2C_ADDR << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          tx,
                          sizeof(tx),
                          INA219_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return INA219_ERROR;
    }

    return INA219_OK;
}

static INA219_StatusTypeDef INA219_ReadReg16(INA219_HandleTypeDef *hina,
                                             uint8_t reg,
                                             uint16_t *value)
{
    uint8_t rx[2];

    if ((hina == NULL) || (value == NULL))
        return INA219_BAD_PARAM;

    if (HAL_I2C_Mem_Read(hina->hi2c,
                         (uint16_t)(INA219_I2C_ADDR << 1),
                         reg,
                         I2C_MEMADD_SIZE_8BIT,
                         rx,
                         sizeof(rx),
                         INA219_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return INA219_ERROR;
    }

    *value = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return INA219_OK;
}

INA219_StatusTypeDef INA219_Init(INA219_HandleTypeDef *hina,
                                 I2C_HandleTypeDef *hi2c)
{
    float current_lsb;
    float cal_f;

    if ((hina == NULL) || (hi2c == NULL))
        return INA219_BAD_PARAM;

    hina->hi2c = hi2c;

    current_lsb = INA219_MAX_CURRENT_A / 32768.0f;
    cal_f = 0.04096f / (current_lsb * INA219_SHUNT_OHMS);

    if (cal_f < 1.0f)
        cal_f = 1.0f;
    if (cal_f > 65535.0f)
        cal_f = 65535.0f;

    hina->currentLsb_A = current_lsb;
    hina->calValue = (uint16_t)(cal_f);

    if (INA219_WriteReg16(hina, INA219_REG_CONFIG, INA219_CONFIG_VALUE) != INA219_OK)
        return INA219_ERROR;

    if (INA219_WriteReg16(hina, INA219_REG_CALIBRATION, hina->calValue) != INA219_OK)
        return INA219_ERROR;

    return INA219_OK;
}

INA219_StatusTypeDef INA219_ReadCurrent_mA(INA219_HandleTypeDef *hina,
                                           float *current_mA)
{
    uint16_t raw_u16;
    int16_t raw_i16;

    if ((hina == NULL) || (current_mA == NULL) || (hina->hi2c == NULL))
        return INA219_BAD_PARAM;

    if (INA219_ReadReg16(hina, INA219_REG_CURRENT, &raw_u16) != INA219_OK)
        return INA219_ERROR;

    raw_i16 = (int16_t)raw_u16;
    *current_mA = ((float)raw_i16 * hina->currentLsb_A) * 1000.0f;
    return INA219_OK;
}
