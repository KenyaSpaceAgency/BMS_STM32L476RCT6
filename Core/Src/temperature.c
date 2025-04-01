/*
 * temperature.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "temperature.h"

/* Private constants ---------------------------------------------------------*/
#define TMP100_I2C_ADDRESS_1    0x48  // I2C address for TMP100 on I2C1
#define TMP100_I2C_ADDRESS_2    0x49  // I2C address for TMP100 on I2C2
#define TMP100_TEMP_REG         0x00

/**
  * @brief  Reads temperature from the TMP100 sensor
  * @param  hi2c: Pointer to the I2C handle
  * @param  temperature: Pointer to store the temperature (in °C)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c, int16_t *temperature)
{
    uint8_t data[2];
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (TMP100_I2C_ADDRESS_1 << 1) : (TMP100_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    int16_t raw = (data[0] << 4) | (data[1] >> 4);
    *temperature = (raw * 0.0625); // Convert to °C
    return HAL_OK;
}
