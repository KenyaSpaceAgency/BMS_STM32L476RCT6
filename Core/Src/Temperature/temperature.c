/* Role of temperature.c in the BMS Project:
 * This file provides functions to initialize and read temperature data from two TMP100
 * temperature sensors (NTC-1 and NTC-2) connected via I2C to the BMS for a CubeSat. It
 * configures the sensors and converts their readings into degrees Celsius, supplying
 * critical temperature data for battery and PCB monitoring.
 */

/* Importance of temperature.c in the BMS Project:
 * - Ensures Thermal Safety: Detects over-temperature (>60°C) or under-temperature (<-20°C)
 *   conditions, enabling protective actions to prevent battery damage.
 * - Enhances Battery Performance: Maintains optimal temperatures, improving lithium-ion
 *   battery efficiency and longevity in space’s extreme environment.
 * - Improves Reliability: Supports fault detection and heater control, ensuring the EPS
 *   operates reliably.
 * - Integrates with RS485: Logs temperature data sent to the OBC via SSP, enabling remote
 *   diagnostics.
 * - Provides Redundancy: Uses two sensors for cross-checking, enhancing system reliability.
 */

/* Objective of temperature.c in the BMS Project:
 * The objective is to initialize TMP100 sensors and provide accurate temperature readings
 * for the BMS, supporting thermal safety, battery performance, and diagnostics. This ensures
 * safe power management, heater control, and communication with the OBC, contributing to
 * the CubeSat’s EPS reliability and mission success.
 */

/* Include temperature header file for function declarations and constants */
#include "temperature.h"
/* Include main header file for BMS-specific definitions and I2C handles */
#include "main.h"

/* Define function to initialize TMP100 temperature sensors
 * Inputs:
 * - hi2c1: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C1 interface
 *          for NTC-1 (address 0x48), used to communicate with the first sensor.
 * - hi2c2: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C2 interface
 *          for NTC-2 (address 0x49), used to communicate with the second sensor.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if both sensors are configured
 *          and verified correctly, or failure (HAL_ERROR) if I2C communication fails or
 *          configuration verification does not match.
 * What it does: Configures NTC-1 and NTC-2 for 12-bit resolution, sets fault queue to
 *               reduce noise, and verifies settings to ensure sensors are ready.
 */
HAL_StatusTypeDef Temperature_Init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2) {
    HAL_StatusTypeDef status; /* Create a variable to store I2C operation status */
    uint8_t config_value = 0; /* Create a variable to hold configuration value */
    uint8_t read_config = 0; /* Create a variable to hold read-back configuration */
    config_value |= (1 << 6) | (1 << 5); /* Set bits 6 and 5 for 12-bit resolution */
    config_value |= (1 << TMP100_F0); /* Set bit for fault queue of 2 */
    uint16_t i2c_addr_1 = (TMP100_I2C_ADDRESS_1 << 1); /* Shift NTC-1 address (0x48) for HAL */
    status = HAL_I2C_Mem_Write(hi2c1, i2c_addr_1, TMP100_CONFIG_REG, 1, &config_value, 1, HAL_MAX_DELAY); /* Write configuration to NTC-1 */
    if (status != HAL_OK) return status; /* Return error if I2C write fails */
    status = HAL_I2C_Mem_Read(hi2c1, i2c_addr_1, TMP100_CONFIG_REG, 1, &read_config, 1, HAL_MAX_DELAY); /* Read back NTC-1 configuration */
    if (status != HAL_OK || read_config != config_value) return HAL_ERROR; /* Return error if read fails or config mismatch */
    uint16_t i2c_addr_2 = (TMP100_I2C_ADDRESS_2 << 1); /* Shift NTC-2 address (0x49) for HAL */
    status = HAL_I2C_Mem_Write(hi2c2, i2c_addr_2, TMP100_CONFIG_REG, 1, &config_value, 1, HAL_MAX_DELAY); /* Write configuration to NTC-2 */
    if (status != HAL_OK) return status; /* Return error if I2C write fails */
    status = HAL_I2C_Mem_Read(hi2c2, i2c_addr_2, TMP100_CONFIG_REG, 1, &read_config, 1, HAL_MAX_DELAY); /* Read back NTC-2 configuration */
    if (status != HAL_OK || read_config != config_value) return HAL_ERROR; /* Return error if read fails or config mismatch */
    return HAL_OK; /* Return success status */
}

/* Define function to read temperatures from TMP100 sensors
 * Inputs:
 * - hi2c1: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies I2C1 for NTC-1
 *          (address 0x48), used to read temperature data.
 * - hi2c2: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies I2C2 for NTC-2
 *          (address 0x49), used to read temperature data.
 * - temperature_1: Pointer to a variable (int16_t *) to store NTC-1 temperature in °C.
 * - temperature_2: Pointer to a variable (int16_t *) to store NTC-2 temperature in °C.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if both temperatures are read
 *          and converted successfully, or failure (HAL_ERROR) if I2C communication fails
 *          for either sensor. On failure, temperature values are not updated.
 * What it does: Reads 12-bit temperature data from NTC-1 and NTC-2, converts it to
 *               degrees Celsius, and stores the results in the provided variables.
 */
HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, int16_t *temperature_1, int16_t *temperature_2) {
    uint8_t data[2]; /* Create a buffer to hold 2 bytes of temperature data */
    HAL_StatusTypeDef status; /* Create a variable to store I2C operation status */
    uint16_t i2c_addr_1 = (TMP100_I2C_ADDRESS_1 << 1); /* Shift NTC-1 address (0x48) for HAL */
    status = HAL_I2C_Mem_Read(hi2c1, i2c_addr_1, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY); /* Read 2 bytes from NTC-1 temperature register */
    if (status != HAL_OK) { /* Check if I2C read failed */
        return status; /* Return error status */
    }
    int16_t temp_raw = (data[0] << 4) | (data[1] >> 4); /* Combine bytes into 12-bit temperature value */
    if (temp_raw & 0x800) { /* Check sign bit for negative temperature */
        temp_raw -= 4096; /* Adjust for 2’s complement negative values */
    }
    *temperature_1 = (temp_raw * 625) / 10000; /* Convert raw value to °C using 0.0625°C/LSB */
    uint16_t i2c_addr_2 = (TMP100_I2C_ADDRESS_2 << 1); /* Shift NTC-2 address (0x49) for HAL */
    status = HAL_I2C_Mem_Read(hi2c2, i2c_addr_2, TMP100_TEMP_REG, 1, data, 2, HAL_MAX_DELAY); /* Read 2 bytes from NTC-2 temperature register */
    if (status != HAL_OK) { /* Check if I2C read failed */
        return status; /* Return error status */
    }
    temp_raw = (data[0] << 4) | (data[1] >> 4); /* Combine bytes into 12-bit temperature value */
    if (temp_raw & 0x800) { /* Check sign bit for negative temperature */
        temp_raw -= 4096; /* Adjust for 2’s complement negative values */
    }
    *temperature_2 = (temp_raw * 625) / 10000; /* Convert raw value to °C using 0.0625°C/LSB */
    return HAL_OK; /* Return success status */
}
