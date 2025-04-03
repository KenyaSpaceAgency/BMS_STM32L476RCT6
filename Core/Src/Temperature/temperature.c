/*
 * temperature.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "temperature.h"
#include "main.h"

HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, int16_t *temperature_1, int16_t *temperature_2)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    // Read from NTC-1 (I2C address 0x48, using hi2c1)
    uint16_t i2c_addr_1 = (TMP100_I2C_ADDRESS_1 << 1);
    status = HAL_I2C_Mem_Read(hi2c1, i2c_addr_1, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Convert temperature for NTC-1
    int16_t temp_raw = (data[0] << 4) | (data[1] >> 4);
    if (temp_raw & 0x800) { // Sign bit check
        temp_raw -= 4096; // Convert to signed value
    }
    *temperature_1 = (temp_raw * 625) / 10000; // Convert to degrees Celsius (0.0625°C per LSB)

    // Read from NTC-2 (I2C address 0x49, using hi2c2)
    uint16_t i2c_addr_2 = (TMP100_I2C_ADDRESS_2 << 1);
    status = HAL_I2C_Mem_Read(hi2c2, i2c_addr_2, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Convert temperature for NTC-2
    temp_raw = (data[0] << 4) | (data[1] >> 4);
    if (temp_raw & 0x800) { // Sign bit check
        temp_raw -= 4096; // Convert to signed value
    }
    *temperature_2 = (temp_raw * 625) / 10000; // Convert to degrees Celsius (0.0625°C per LSB)

    return HAL_OK;
}
