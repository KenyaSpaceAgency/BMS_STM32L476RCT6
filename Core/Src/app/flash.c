#include "flash.h"
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_flash_ex.h"
#include "log.h"
#include "main.h"
#include <string.h>

extern TelemetryData telemetry;

static uint32_t current_flash_addr = FLASH_USER_START_ADDR;

uint16_t CalculateCRC16(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0x0000;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

HAL_StatusTypeDef Flash_WriteDoubleWord(uint32_t address, uint64_t data) {
    if (address < FLASH_BASE || address > STM32_FLASH_END || (address % 8 != 0)) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Flash_WriteDoubleWord: Invalid address 0x%08lX", address);
        return HAL_ERROR;
    }

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Flash write failed at 0x%08lX, err=0x%lX", address, HAL_FLASH_GetError());
    }

    return status;
}

uint64_t Flash_ReadDoubleWord(uint32_t address) {
    if (address < FLASH_BASE || address > STM32_FLASH_END || (address % 8 != 0)) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Flash_ReadDoubleWord: Invalid address 0x%08lX", address);
        return 0xFFFFFFFFFFFFFFFFULL;
    }
    return *(uint64_t*)address;
}

static uint32_t Flash_GetPage(uint32_t address) {
    return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

HAL_StatusTypeDef Flash_ErasePage(uint32_t address) {
    if (address < FLASH_BASE || address >= STM32_FLASH_END) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "ErasePage: Invalid address 0x%08lX", address);
        return HAL_ERROR;
    }

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks = FLASH_BANK_1,
        .Page = Flash_GetPage(address),
        .NbPages = 1
    };
    uint32_t page_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);

    HAL_FLASH_Lock();

    if (status != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "ErasePage failed at page %lu, err=0x%lX", erase_init.Page, page_error);
    }

    return status;
}

bool Flash_WriteBytes(uint32_t address, uint8_t *data, uint16_t len) {
    if (address < FLASH_BASE || (address + len) > STM32_FLASH_END) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "WriteBytes: address out of range");
        return false;
    }

    if (address % 8 != 0) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "WriteBytes: unaligned address 0x%08lX", address);
        return false;
    }

    HAL_FLASH_Unlock();

    for (uint16_t i = 0; i < len; i += 8) {
        uint64_t word = 0xFFFFFFFFFFFFFFFFULL;
        memcpy(&word, &data[i], (len - i >= 8) ? 8 : (len - i));

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, word) != HAL_OK) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "WriteBytes failed at 0x%08lX", address + i);
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

void Flash_WriteTelemetry(void) {
    TelemetrySnapshot snapshot;
    snapshot.version = TELEMETRY_VERSION;
    memset(snapshot.reserved, 0, sizeof(snapshot.reserved));
    snapshot.telemetry = telemetry;
    snapshot.crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
    snapshot.padding = 0xFFFF;

    if ((current_flash_addr + SNAPSHOT_SIZE) > FLASH_USER_END_ADDR) {
        Log_Message(BMS_MSG_LEVEL_INFO, "Flash full. Erasing and wrapping.");
        if (Flash_ErasePage(FLASH_USER_START_ADDR) != HAL_OK) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "Flash erase failed");
            return;
        }
        current_flash_addr = FLASH_USER_START_ADDR;
    }

    uint8_t *raw = (uint8_t*)&snapshot;
    for (uint32_t i = 0; i < SNAPSHOT_SIZE; i += 8) {
        uint64_t data = *(uint64_t*)&raw[i];
        if (Flash_WriteDoubleWord(current_flash_addr + i, data) != HAL_OK) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "Write failed at 0x%08lX", current_flash_addr + i);
            return;
        }
    }

    current_flash_addr += SNAPSHOT_SIZE;
}

void Flash_ReadTelemetry(void) {
    TelemetrySnapshot snapshot;
    memcpy(&snapshot, (void*)FLASH_USER_START_ADDR, sizeof(TelemetrySnapshot));

    uint16_t expected_crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
    if (snapshot.crc != expected_crc) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "CRC mismatch: expected 0x%04X, got 0x%04X", expected_crc, snapshot.crc);
        return;
    }

    if (snapshot.version != TELEMETRY_VERSION) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Version mismatch: expected 0x%02X, got 0x%02X", TELEMETRY_VERSION, snapshot.version);
        return;
    }

    telemetry = snapshot.telemetry;
}

void Flash_RecoverWritePointer(void) {
    uint32_t addr = FLASH_USER_START_ADDR;
    uint32_t count = 0;
    TelemetrySnapshot snapshot;

    while (addr + SNAPSHOT_SIZE <= FLASH_USER_END_ADDR) {
        memcpy(&snapshot, (void*)addr, SNAPSHOT_SIZE);
        uint16_t expected_crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));

        if (snapshot.version != TELEMETRY_VERSION || snapshot.crc != expected_crc) {
            break;
        }

        addr += SNAPSHOT_SIZE;
        count++;
    }

    Log_Message(BMS_MSG_LEVEL_INFO, "Recovered %lu telemetry entries from flash.", count);
    current_flash_addr = addr;
}
