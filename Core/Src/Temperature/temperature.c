/*
 * temperature.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 *
 * @brief  Implements temperature reading functionality for the Battery Management System
 *         (BMS) in a CubeSat, using two TMP100 temperature sensors (NTC-1 and NTC-2) over I2C.
 *         Reads and converts temperature data for battery and PCB monitoring.
 * @note   - Uses the TMP100 temperature sensor, a 12-bit I2C device with 0.0625°C resolution.
 *         - NTC-1 (I2C address 0x48) and NTC-2 (I2C address 0x49) are connected to I2C1 and I2C2,
 *           respectively, on the STM32 microcontroller.
 *         - Temperature values are critical for battery safety (e.g., over/undertemperature protection).
 * @context In a CubeSat, the BMS monitors battery temperatures (NTC-1, NTC-2) and PCB temperature
 *          to ensure safe operation in space’s extreme thermal environment. This file provides
 *          temperature data used in `main.c` for fault detection (e.g., ERROR_OVERTEMP) and PID
 *          control of heaters (`pid.c`).
 */

/* Include necessary headers */
#include "temperature.h"
#include "main.h"

/**
  * @brief  Reads temperature data from two TMP100 sensors (NTC-1 and NTC-2) over I2C.
  * @param  hi2c1: Pointer to the I2C handle for NTC-1 (I2C1 on STM32).
  * @param  hi2c2: Pointer to the I2C handle for NTC-2 (I2C2 on STM32).
  * @param  temperature_1: Pointer to store the temperature from NTC-1 (in °C).
  * @param  temperature_2: Pointer to store the temperature from NTC-2 (in °C).
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_ERROR on I2C communication failure.
  * @note   - NTC-1 (address 0x48) and NTC-2 (address 0x49) are read using I2C1 and I2C2.
  *         - Each sensor returns a 12-bit temperature value (2 bytes) with 0.0625°C/LSB resolution.
  *         - Temperatures are converted to signed 16-bit integers representing °C.
  *         - If I2C communication fails, the function returns early with the error status.
  * @context Called in `main.c` within the main loop to monitor battery temperatures for safety
  *          checks (e.g., ERROR_OVERTEMP, ERROR_UNDERTEMP) and to provide input for the PID
  *          heater control in `pid.c`. Accurate temperature monitoring is critical for battery
  *          health and system reliability in space.
  * @integration The STM32 uses I2C1 and I2C2 (configured in `main.c`) to communicate with the
  *              TMP100 sensors. The function integrates with the BMS’s fault detection and heater
  *              control systems, ensuring thermal safety through real-time temperature data.
  * @debug  - If HAL_ERROR occurs, check I2C bus connections, sensor addresses (0x48, 0x49), and
  *           ensure I2C1/I2C2 are properly initialized (400 kHz, `main.c`).
  *         - If temperatures are incorrect, verify the conversion formula and sign bit handling.
  *         - Use an I2C debugger or oscilloscope to capture bus transactions for troubleshooting.
  */
HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, int16_t *temperature_1, int16_t *temperature_2)
{
    uint8_t data[2]; // Buffer for 2-byte temperature data from TMP100
    HAL_StatusTypeDef status;

    // Read from NTC-1 (I2C address 0x48, using hi2c1)
    uint16_t i2c_addr_1 = (TMP100_I2C_ADDRESS_1 << 1); // Shift address for HAL (7-bit to 8-bit)
    // Read 2 bytes from the temperature register (TMP100_TEMP_REG, typically 0x00)
    status = HAL_I2C_Mem_Read(hi2c1, i2c_addr_1, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status; // Return early on I2C failure
    }
	// Check if the read operation was successful
    // Convert temperature for NTC-1
    // Combine bytes into a 12-bit value: data[0] (MSB) and 4 bits of data[1] (LSB)
    int16_t temp_raw = (data[0] << 4) | (data[1] >> 4);
    if (temp_raw & 0x800) { // Check sign bit (bit 11, 0x800 for 12-bit)
        temp_raw -= 4096; // Extend sign for negative values (2’s complement)
    }
    // Convert to °C: 0.0625°C/LSB → (temp_raw * 625) / 10000 = temp_raw * 0.0625
    *temperature_1 = (temp_raw * 625) / 10000;

    // Read from NTC-2 (I2C address 0x49, using hi2c2)
    uint16_t i2c_addr_2 = (TMP100_I2C_ADDRESS_2 << 1); // Shift address for HAL
    // Read 2 bytes from the temperature register
    status = HAL_I2C_Mem_Read(hi2c2, i2c_addr_2, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status; // Return early on I2C failure
    }

    // Convert temperature for NTC-2
    // Same conversion process as NTC-1
    temp_raw = (data[0] << 4) | (data[1] >> 4);
    if (temp_raw & 0x800) { // Check sign bit
        temp_raw -= 4096; // Extend sign for negative values
    }
    *temperature_2 = (temp_raw * 625) / 10000; // Convert to °C

    return HAL_OK; // Success: Both temperatures read and converted
}
