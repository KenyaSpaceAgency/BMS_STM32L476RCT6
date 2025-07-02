// Include header files needed for the code to work
#include "flash.h"                      // Contains function prototypes and constants for flash operations
#include "stm32l4xx_hal_flash.h"       // STM32 HAL library for basic flash operations (e.g., erase, program)
#include "stm32l4xx_hal_flash_ex.h"    // STM32 HAL library for extended flash operations (e.g., page erase)
#include "main.h"                      // Main project header with TelemetryData and GPIO definitions
#include <string.h>                    // Includes memory functions like memset and memcpy for data handling

// Define constants for the flash memory region used to store telemetry data
#define FLASH_USER_END_ADDR    ((uint32_t)0x0803FFFF) // End of last 2KB page
#define FLASH_PAGE_SIZE        ((uint32_t)0x800)      // 2KB (2048 bytes) per page (Reference Manual, Section 3.3.2)
#define TELEMETRY_VERSION      0x01                   // Version number for telemetry data format

// External variables declared in other files (e.g., main.c)
extern TelemetryData telemetry;                        // Global structure to store battery data (voltages, temperatures, etc.)
extern UART_HandleTypeDef huart1;                      // UART interface (PA9/PA10) for logging errors

// Declare error logging function (defined in main.c)
void Log_Error(const char *format, ...);

// Function: CalculateCRC16
// Inputs:
//   - data: A pointer to an array of bytes (uint8_t*) to calculate the CRC for
//   - length: A uint32_t, the number of bytes in the data array
// Output:
//   - Returns a uint16_t, the 16-bit CRC value
// Significance:
//   - Calculates a CRC-16-CCITT checksum to verify that telemetry data stored in flash is not corrupted.
//     Used to ensure data integrity when saving/restoring telemetry (used in Flash_WriteTelemetry and Flash_ReadTelemetry).
uint16_t CalculateCRC16(const uint8_t *data, uint32_t length) {
    uint16_t crc = 0x0000; // Initialize CRC to 0 (starting value for CRC-16-CCITT)
    for (uint32_t i = 0; i < length; i++) { // Loop through each byte in the data array
        crc ^= (uint16_t)data[i] << 8; // XOR the current byte (shifted left by 8) with the CRC
        for (uint8_t j = 0; j < 8; j++) { // Process each bit in the byte (8 bits)
            // If the most significant bit (MSB) of crc is 1, shift left and XOR with polynomial 0x1021
            // Otherwise, just shift left
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc; // Return the final CRC value
}



// Function: Flash_WriteDoubleWord
// Inputs:
//   - address: A uint32_t, the flash memory address to write to (must be 8-byte aligned)
//   - data: A uint64_t, the 8-byte (64-bit) value to write
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if successful, HAL_ERROR if failed
// Significance:
//   - Writes 8 bytes of data to flash memory, used to store telemetry data persistently
//     across power cycles (Reference Manual, Section 3.3.2, page 93).
HAL_StatusTypeDef Flash_WriteDoubleWord(uint32_t address, uint64_t data) {
    // Check if the address is within the allowed range (0x0803F800 to 0x0803FFFF)
    if (address < FLASH_USER_START_ADDR || address > FLASH_USER_END_ADDR) {
        // Log an error if the address is invalid
        Log_Error("Flash write address out of range: 0x%08lX", address);
        // Return error status
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status; // Variable to store the status of the write operation
    HAL_FLASH_Unlock(); // Unlock the flash memory to allow writing (required by STM32)
    // Write 8 bytes (64 bits) to the specified address
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, data);
    HAL_FLASH_Lock(); // Lock the flash memory to prevent accidental writes
    // If the write failed, log the error with the address and error code
    if (status != HAL_OK) {
        Log_Error("Flash write failed at 0x%08lX, error: %lu", address, HAL_FLASH_GetError());
    }
    return status; // Return the status (HAL_OK or HAL_ERROR)
}

// Function: Flash_ReadDoubleWord
// Inputs:
//   - address: A uint32_t, the flash memory address to read from
// Output:
//   - Returns a uint64_t, the 8-byte (64-bit) value read from flash
// Significance:
//   - Reads 8 bytes from flash memory, used to retrieve telemetry data saved previously
//     (Reference Manual, Section 3.3.2, page 93).
uint64_t Flash_ReadDoubleWord(uint32_t address) {
    // Check if the address is within the allowed range
    if (address < FLASH_USER_START_ADDR || address > FLASH_USER_END_ADDR) {
        // Log an error if the address is invalid
        Log_Error("Flash read address out of range: 0x%08lX", address);
        // Return all 1s (0xFFFFFFFFFFFFFFFF) to indicate an error
        return 0xFFFFFFFFFFFFFFFFULL;
    }
    // Read 8 bytes directly from the address and return the value
    return *(uint64_t*)address;
}

// Function: Flash_ErasePage
// Inputs:
//   - None (void)
// Output:
//   - Returns HAL_StatusTypeDef, HAL_OK if successful, HAL_ERROR if failed
// Significance:
//   - Erases the 2KB flash page (0x0803F800–0x0803FFFF) to prepare it for new data.
//     STM32 flash must be erased before writing (Reference Manual, Section 3.3.2, page 94).
HAL_StatusTypeDef Flash_ErasePage(void) {
    // Declare a structure to set up erase parameters
    FLASH_EraseInitTypeDef eraseInit = {0};
    // Variable to store any page errors during erase
    uint32_t pageError = 0;
    // Variable to store the status of the erase operation
    HAL_StatusTypeDef status;

    // Unlock the flash memory to allow erasing
    HAL_FLASH_Unlock();

    // Set up erase parameters: erase one page in Bank 1
    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES; // Specify page erase mode
    eraseInit.Banks = FLASH_BANK_1; // Use Bank 1 (STM32L476 has one bank for flash)
    // Calculate page number (address 0x0803F800 is page 127)
    eraseInit.Page = (FLASH_USER_START_ADDR - 0x08000000) / FLASH_PAGE_SIZE;
    eraseInit.NbPages = 1; // Erase only one page (2KB)

    // Perform the page erase operation
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    // Lock the flash memory to prevent accidental writes
    HAL_FLASH_Lock();

    // If the erase failed, log the error with page number and error code
    if (status != HAL_OK) {
        Log_Error("Flash erase failed at page %u, error: %lu", eraseInit.Page, HAL_FLASH_GetError());
    }
    return status; // Return the status (HAL_OK or HAL_ERROR)
}

// Function: Flash_WriteTelemetry
// Inputs:
//   - None (void), uses the global `telemetry` structure
// Output:
//   - None (void), writes telemetry data to flash
// Significance:
//   - Saves the current telemetry data (voltages, currents, temperatures) to flash memory,
//     ensuring it persists across power cycles. Uses CRC16 and versioning for data integrity.
//     Called by BMS_Service.c when SOC changes or every 5 minutes.
void Flash_WriteTelemetry(void) {
    // Declare a TelemetrySnapshot structure to hold data, version, and CRC
    TelemetrySnapshot snapshot;

    // Set the version number to track the telemetry format
    snapshot.version = TELEMETRY_VERSION; // Set to 0x01
    // Clear the reserved bytes (set to 0) for future use or alignment
    memset(snapshot.reserved, 0, sizeof(snapshot.reserved));
    // Copy the global telemetry data into the snapshot
    snapshot.telemetry = telemetry;
    // Calculate CRC-16 over the snapshot (excluding the CRC field itself)
    snapshot.crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
    // Set padding to 0xFFFF to ensure 64-bit alignment
    snapshot.padding = 0xFFFF;

    // Erase the flash page to prepare for writing
    if (Flash_ErasePage() != HAL_OK) {
        // Log an error if the erase fails
        Log_Error("Failed to erase Flash page for telemetry");
        // Exit the function to avoid writing to an unprepared page
        return;
    }

    // Pointer to the snapshot as raw bytes for writing
    uint8_t *raw = (uint8_t*)&snapshot;
    // Start writing at the beginning of the flash page
    uint32_t address = FLASH_USER_START_ADDR;

    // Loop through the snapshot, writing 8 bytes at a time
    for (uint32_t i = 0; i < sizeof(snapshot); i += 8) {
        // Read 8 bytes from the snapshot into a 64-bit value
        uint64_t data = *(uint64_t*)&raw[i];
        // Write the 8 bytes to flash
        if (Flash_WriteDoubleWord(address, data) != HAL_OK) {
            // Log an error if the write fails
            Log_Error("Flash write failed at 0x%08lX", address);
            // Exit to avoid partial writes
            return;
        }
        // Move to the next 8-byte address
        address += 8;
    }
}

// Function: Flash_ReadTelemetry
// Inputs:
//   - None (void), reads from a fixed flash address
// Output:
//   - None (void), updates the global `telemetry` structure if valid
// Significance:
//   - Loads telemetry data from flash memory, verifying it with CRC16 and version checks.
//     Called by BMS_Service.c at startup to restore the last saved state.
void Flash_ReadTelemetry(void) {
    // Declare a TelemetrySnapshot structure to hold the read data
    TelemetrySnapshot snapshot;
    // Copy data from flash (starting at 0x0803F800) into the snapshot
    memcpy(&snapshot, (void*)FLASH_USER_START_ADDR, sizeof(TelemetrySnapshot));

    // Calculate the expected CRC-16 over the snapshot (excluding the CRC field)
    uint16_t expected_crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
    // Check if the stored CRC matches the calculated CRC
    if (snapshot.crc != expected_crc) {
        // Log an error if the CRC doesn’t match (data may be corrupted)
        Log_Error("CRC16 mismatch: expected 0x%04X, found 0x%04X", expected_crc, snapshot.crc);
        // Exit to avoid using corrupted data
        return;
    }

    // Check if the stored version matches the expected version
    if (snapshot.version != TELEMETRY_VERSION) {
        // Log an error if the version doesn’t match (incompatible data format)
        Log_Error("Version mismatch: expected 0x%02X, found 0x%02X", TELEMETRY_VERSION, snapshot.version);
        // Exit to avoid using incompatible data
        return;
    }

    // If CRC and version are valid, copy the snapshot’s telemetry to the global structure
    telemetry = snapshot.telemetry;
}// only use functions used here, if you are taking telemetry from the flash memory, we use cr8 only
