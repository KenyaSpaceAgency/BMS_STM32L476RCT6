/*
 * flash.h
 *
 *  Created on: Jul 2, 2025
 *      Author: yomue
 */

#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32l4xx_hal.h"

// --- Flash memory configuration ---
#define FLASH_USER_START_ADDR  ((uint32_t)0x0803F800)  // Last 2KB of 256KB flash
#define FLASH_USER_END_ADDR    ((uint32_t)0x0803FFFF)
#define FLASH_PAGE_SIZE        ((uint32_t)0x800)       // 2KB per page

// --- Telemetry versioning ---
#define TELEMETRY_VERSION      0x01

// --- Flash API ---
HAL_StatusTypeDef Flash_WriteDoubleWord(uint32_t address, uint64_t data);
uint64_t Flash_ReadDoubleWord(uint32_t address);
HAL_StatusTypeDef Flash_ErasePage(void);
void Flash_WriteTelemetry(void);
void Flash_ReadTelemetry(void);

// --- CRC16 utility (used internally but may be reused externally) ---
uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);

// Structure: TelemetrySnapshot
// Purpose:
//   - A container to store telemetry data in flash with version and CRC for integrity.
//   - Ensures data is aligned to 64-bit boundaries (required for STM32 flash writes).
// Fields:
//   - version: A uint8_t, tracks the telemetry format version (0x01)
//   - reserved: A 3-byte array, padding for future use or alignment
//   - telemetry: The TelemetryData structure with all battery data
//   - crc: A uint16_t, the CRC-16 checksum for data integrity
//   - padding: A uint16_t, ensures 64-bit alignment for flash writes
typedef struct {
    uint8_t version;              // Version number to identify telemetry format
    uint8_t reserved[3];          // Reserved bytes for future use or alignment
    TelemetryData telemetry;      // The actual telemetry data (voltages, currents, etc.)
    uint16_t crc;                 // CRC-16 checksum to verify data integrity
    uint16_t padding;             // Padding to align to 8 bytes (64 bits)
} TelemetrySnapshot;

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H */
