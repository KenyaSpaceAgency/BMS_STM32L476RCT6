/*
 * eeprom.h
 *
 *  Created on: Jul 11, 2025
 *      Author: yomue
 */

#ifndef __EEPROM_H
#define __EEPROM_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>

// ------------------- EEPROM I2C Addresses -------------------
#define EPSBMS_I2C_ADDR_MEMORY     0x50    // M24M01-A125 main memory (E2,E1 = 0)
#define EPSBMS_I2C_ADDR_ID_PAGE    0x58    // Identification page

// ------------------- EEPROM Memory Map ----------------------
#define EEPROM_ADDR_LOCK                   0x0000  // Lock struct (shared)
#define EEPROM_ADDR_BMS_TELsEMETRY_START    0x0100  // Rotating telemetry buffer
#define EEPROM_ENTRY_SIZE                  sizeof(TelemetrySnapshot)
#define EEPROM_ENTRY_COUNT                 16      // Total telemetry snapshots (can increase if needed)
#define EEPROM_ADDR_BMS_TELEMETRY_END      (EEPROM_ADDR_BMS_TELEMETRY_START + EEPROM_ENTRY_SIZE * EEPROM_ENTRY_COUNT)

#define TELEMETRY_VERSION 0x01
#define SNAPSHOT_PADDING  0xFFFF



// ------------------- Public APIs ----------------------------
HAL_StatusTypeDef EEPROM_BMS_WriteSnapshot(TelemetryData *data);
HAL_StatusTypeDef EEPROM_BMS_ReadLatestSnapshot(TelemetryData *data);
void EEPROM_BMS_RecoverWriteIndex(void);
uint16_t EEPROM_BMS_CalculateCRC16(const uint8_t *data, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H */

