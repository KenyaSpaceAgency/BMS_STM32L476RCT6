/*
 * eeprom.c
 *
 *  Created on: Jul 11, 2025
 *      Author: yomue
 */


#include "main.h"
#include "eeprom.h"
#include "log.h"
#include <string.h>

#define EEPROM_I2C_ADDR       0xA0 // Adjust based on your EEPROM (e.g., AT24C256 = 0x50 << 1)
#define EEPROM_PAGE_SIZE      64
#define EEPROM_START_ADDRESS  0x0000

extern I2C_HandleTypeDef hi2c3;
extern TelemetryData telemetry;

// Calculate a CRC16 over a block of memory (used for data integrity)
static uint16_t EEPROM_CalculateCRC16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

bool EEPROM_SaveTelemetry(void)
{
    uint8_t buffer[sizeof(TelemetryData) + 2];
    memcpy(buffer, &telemetry, sizeof(TelemetryData));

    uint16_t crc = EEPROM_CalculateCRC16((uint8_t*)&telemetry, sizeof(TelemetryData));
    buffer[sizeof(TelemetryData)] = (uint8_t)(crc >> 8);
    buffer[sizeof(TelemetryData) + 1] = (uint8_t)(crc & 0xFF);

    for (uint16_t i = 0; i < sizeof(buffer); i += EEPROM_PAGE_SIZE) {
        uint16_t chunk_size = ((sizeof(buffer) - i) > EEPROM_PAGE_SIZE) ? EEPROM_PAGE_SIZE : (sizeof(buffer) - i);

        if (HAL_I2C_Mem_Write(&hi2c3, EEPROM_I2C_ADDR, EEPROM_START_ADDRESS + i, I2C_MEMADD_SIZE_16BIT, &buffer[i], chunk_size, HAL_MAX_DELAY) != HAL_OK)
        {
            Log_Message(BMS_MSG_LEVEL_ERROR, "EEPROM write failed at offset %u", i);
            return false;
        }
        HAL_Delay(5); // EEPROM write cycle time
    }
    return true;
}

bool EEPROM_LoadTelemetry(void)
{
    uint8_t buffer[sizeof(TelemetryData) + 2];
    if (HAL_I2C_Mem_Read(&hi2c3, EEPROM_I2C_ADDR, EEPROM_START_ADDRESS, I2C_MEMADD_SIZE_16BIT, buffer, sizeof(buffer), HAL_MAX_DELAY) != HAL_OK)
    {
        Log_Message(BMS_MSG_LEVEL_ERROR, "EEPROM read failed");
        return false;
    }

    uint16_t stored_crc = ((uint16_t)buffer[sizeof(TelemetryData)] << 8) | buffer[sizeof(TelemetryData) + 1];
    uint16_t computed_crc = EEPROM_CalculateCRC16(buffer, sizeof(TelemetryData));

    if (stored_crc != computed_crc) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "EEPROM CRC mismatch");
        return false;
    }

    memcpy(&telemetry, buffer, sizeof(TelemetryData));
    return true;
}
