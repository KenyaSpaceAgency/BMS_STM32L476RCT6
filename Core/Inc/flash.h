/**
 * @file flash.h
 * @brief Flash memory interface for telemetry logging and generic read/write
 * @author yomue
 * @date Jul 2, 2025
 */

#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32l4xx_hal.h"


// --- Flash Region Constants ---
#define STM32_FLASH_END        ((uint32_t)(FLASH_BASE + 512 * 1024)) // End of 512KB flash
#define FLASH_USER_START_ADDR  ((uint32_t)0x0803F800)          // Last 2KB of flash
#define FLASH_USER_END_ADDR    ((uint32_t)0x0803FFFF)
#define FLASH_PAGE_SIZE        ((uint32_t)0x800)               // 2KB per page

// --- Snapshot Layout ---
#define SNAPSHOT_SIZE          sizeof(TelemetrySnapshot)
#define MAX_SNAPSHOTS          (FLASH_PAGE_SIZE / SNAPSHOT_SIZE)
#define TELEMETRY_VERSION      0x01

// --- API: Low-level Flash Access ---
HAL_StatusTypeDef Flash_WriteDoubleWord(uint32_t address, uint64_t data);
uint64_t          Flash_ReadDoubleWord(uint32_t address);
HAL_StatusTypeDef Flash_ErasePage(uint32_t address);  ///< Erase page containing address
bool              Flash_WriteBytes(uint32_t address, uint8_t *data, uint16_t len);

// --- API: Snapshot Access ---
void Flash_WriteTelemetry(void);      ///< Writes current telemetry to flash
void Flash_ReadTelemetry(void);       ///< Reads snapshot from flash into global telemetry
void Flash_RecoverWritePointer(void); ///< Scans for last used flash slot after reboot

// --- Utility: CRC ---
uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);

// --- Data Structure: Telemetry Snapshot Format ---
typedef struct {
    uint8_t       version;              ///< Format version
    uint8_t       reserved[3];          ///< Alignment padding
    TelemetryData telemetry;            ///< Actual data
    uint16_t      crc;                  ///< CRC-16 checksum
    uint16_t      padding;              ///< Align to 64-bit boundary
} TelemetrySnapshot;

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H */
