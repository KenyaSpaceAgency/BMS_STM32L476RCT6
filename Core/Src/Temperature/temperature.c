#include "temperature.h"
#include "main.h" // Include main.h to access hi2c1 and hi2c2

#define TMP100_I2C_ADDRESS_1 0x48 // Example I2C address for TMP100 on I2C1
#define TMP100_I2C_ADDRESS_2 0x49 // Example I2C address for TMP100 on I2C2
#define TMP100_TEMP_REG 0x00

HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c, int16_t *temperature)
{
    uint8_t data[2];
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (TMP100_I2C_ADDRESS_1 << 1) : (TMP100_I2C_ADDRESS_2 << 1);

    // Read temperature register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Convert the temperature data (assuming 12-bit resolution)
    int16_t temp_raw = (data[0] << 4) | (data[1] >> 4);
    if (temp_raw & 0x800) { // Sign bit check
        temp_raw -= 4096; // Convert to signed value
    }
    *temperature = (temp_raw * 625) / 10000; // Convert to degrees Celsius (0.0625°C per LSB)

    return HAL_OK;
}
