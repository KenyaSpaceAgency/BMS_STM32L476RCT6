/*
 * i2c_command_handlers.c
 *
 *  Created on: Jul 10, 2025
 *      Author: yomue
 */


// File: i2c_command_handlers.c

#include "i2c_command_handlers.h"
#include "log.h"
#include "bms_service.h"
#include <string.h>
#include "flash.h"

extern TelemetryData telemetry;
extern uint64_t g_sync_counter;




/**
 * @brief Handles PD (Put Data) command as defined in ICD 6.3.1 (CMD_ID 0x05).
 *
 * This command enables the master to send blocks of data to a slave subsystem,
 * intended for memory write operations. Multiple PD frames are used for block
 * transfers. The payload consists of:
 *
 *  - [2]   Module memory address offset (1 byte, 0x00 to 0xFF)
 *  - [3-4] Sequence number of the block (2 bytes)
 *  - [5..] Data block payload (up to 2048 bytes)
 *
 * Reference:
 *  - ICD Section 6.3.1 – PD Command
 *  - STM32L476RC Flash: Ref Manual RM0351, Section 3.3.1 (Table 13), Flash @ 0x08000000
 */
void Handle_PD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len) {
    // Sanity check: must at least have addr (1) + seq (2) + 1 byte payload = 5 bytes minimum
    if (len < 5 || len > 2048 + 5) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "PD error: invalid length %u", len);
        *tx_len = 0;
        return;
    }

    // Parse fields from rx_data
    uint32_t base_address = 0x08000000;  // STM32L476RC Flash start (RM0351 §3.3.1)
    uint8_t address_offset = rx_data[2]; // page offset (0x00 = 0x08000000, 0x01 = 0x08000800, ...)
    uint32_t target_address = base_address + (address_offset * 0x800); // 2KB page size
    uint16_t sequence = ((uint16_t)rx_data[3] << 8) | rx_data[4];
    uint8_t *payload = &rx_data[5];
    uint16_t payload_len = len - 5;

    // Validate address range: must stay within 0x08000000–0x0807FFFF (512KB)
    if (target_address < base_address || target_address >= (base_address + 512 * 1024)) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "PD error: invalid address 0x%08lX", target_address);
        tx_data[0] = 0x05; tx_data[1] = 1; tx_data[2] = 0x02;  // Error code 0x02: invalid address
        *tx_len = 3;
        return;
    }

    // Alignment check: STM32 Flash must be written in 64-bit aligned blocks (RM0351 §3.3.3)
    if ((target_address + sequence * payload_len) % 8 != 0) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "PD error: unaligned write");
        tx_data[0] = 0x05; tx_data[1] = 1; tx_data[2] = 0x03;  // Error code 0x03: unaligned write
        *tx_len = 3;
        return;
    }

    Log_Message(BMS_MSG_LEVEL_INFO, "PD write to 0x%08lX, seq %u, len %u", target_address, sequence, payload_len);

    // Erase flash page only on sequence 0 (first block transfer)
    if (sequence == 0) {
        if (!Flash_ErasePage(target_address)) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "PD flash erase failed at 0x%08lX", target_address);
            tx_data[0] = 0x05; tx_data[1] = 1; tx_data[2] = 0x04;  // Error code 0x04: erase failed
            *tx_len = 3;
            return;
        }
    }

    // Perform the write at computed offset
    uint32_t write_addr = target_address + (sequence * payload_len);
    if (!Flash_WriteBytes(write_addr, payload, payload_len)) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "PD flash write failed");
        tx_data[0] = 0x05; tx_data[1] = 1; tx_data[2] = 0x05;  // Error code 0x05: write failed
        *tx_len = 3;
        return;
    }

    // Success response
    tx_data[0] = 0x05;  // CMD ID
    tx_data[1] = 1;     // LEN
    tx_data[2] = 0x00;  // Status: OK
    *tx_len = 3;
}


/**
 * @brief Handle_RD - Responds to RD (Read Data) command from the master.
 *
 * @param rx_data  Input buffer: [CMD=0x06][LEN][ADDR_MSB][ADDR_LSB][READ_LEN]
 * @param len      Length of rx_data (must be >= 5).
 * @param tx_data  Output buffer: [CMD=0x06][LEN][ADDR_MSB][ADDR_LSB][DATA...]
 * @param tx_len   Output length (set to 0 on error).
 *
 * Protocol Reference:
 * - AFDEVSAT ICD Section 6.3.1, CMD 0x06 (RD)
 * - STM32L476RC Flash memory map: RM0351 Rev 8, Table 13
 *
 * Memory Range Constraints:
 * - Valid STM32L476RC Flash address range: 0x08000000 to 0x0807FFFF (512 KB)
 * - Max read length: 255 bytes (1-byte field)
 * - Response format: CMD, LEN, ADDR_MSB, ADDR_LSB, DATA[n]
 */
void Handle_RD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len) {
    // --- Input Validation ---
    if (len < 5 || rx_data[4] == 0) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "RD error: invalid packet length %u or zero read", len);
        *tx_len = 0;
        return;
    }

    // --- Parse Address and Length ---
    uint16_t offset = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    uint8_t read_len = rx_data[4];

    const uint32_t base_address = 0x08000000;          // STM32L476RC Flash base
    const uint32_t max_flash_address = base_address + (512 * 1024);  // 512 KB Flash
    uint32_t target_address = base_address + offset;

    // --- Alignment Check (optional but recommended) ---
    if ((target_address % 4) != 0) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "RD warning: unaligned read at 0x%08lX", target_address);
    }

    // --- Bounds Check ---
    if ((target_address + read_len) > max_flash_address || read_len > 255) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "RD error: read out-of-bounds at 0x%08lX, len=%u",
                    target_address, read_len);
        *tx_len = 0;
        return;
    }

    // --- Prepare Response ---
    tx_data[0] = 0x06;              // CMD ID for RD
    tx_data[1] = 2 + read_len;      // LEN = 2 (address) + data
    tx_data[2] = rx_data[2];        // Echo address MSB
    tx_data[3] = rx_data[3];        // Echo address LSB

    // Copy from Flash to output buffer
    memcpy(&tx_data[4], (void *)target_address, read_len);
    *tx_len = 4 + read_len;

    Log_Message(BMS_MSG_LEVEL_DEBUG, "RD: read %u bytes from 0x%08lX", read_len, target_address);
}


void Handle_WD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len) {
    const uint8_t CMD_WD = 0x07;
    const uint32_t base_address = 0x08000000;  // STM32L476RC Flash base
    const uint32_t max_flash_address = base_address + (512 * 1024); // 512 KB
    const uint16_t min_len = 5;  // CMD + LEN + ADDR_MSB + ADDR_LSB + 1 data byte

    if (len < min_len) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "WD error: insufficient packet length %u", len);
        tx_data[0] = CMD_WD; tx_data[1] = 1; tx_data[2] = 0x01;  // Error: invalid length
        *tx_len = 3;
        return;
    }

    // --- Parse fields ---
    uint16_t offset = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    uint32_t target_address = base_address + offset;
    uint8_t *payload = &rx_data[4];
    uint16_t payload_len = len - 4;

    if (target_address < base_address || (target_address + payload_len) > max_flash_address) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "WD error: address 0x%08lX out of bounds (len=%u)", target_address, payload_len);
        tx_data[0] = CMD_WD; tx_data[1] = 1; tx_data[2] = 0x02;  // Error: out-of-bounds
        *tx_len = 3;
        return;
    }

    if (target_address % 8 != 0) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "WD error: unaligned write address 0x%08lX", target_address);
        tx_data[0] = CMD_WD; tx_data[1] = 1; tx_data[2] = 0x03;  // Error: unaligned
        *tx_len = 3;
        return;
    }

    // Optional: Erase if writing to a fresh flash page
    if ((target_address % FLASH_PAGE_SIZE) == 0) {
        if (Flash_ErasePage(target_address) != HAL_OK) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "WD error: flash erase failed at 0x%08lX", target_address);
            tx_data[0] = CMD_WD; tx_data[1] = 1; tx_data[2] = 0x04;  // Error: erase failed
            *tx_len = 3;
            return;
        }
    }

    if (!Flash_WriteBytes(target_address, payload, payload_len)) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "WD error: flash write failed");
        tx_data[0] = CMD_WD; tx_data[1] = 1; tx_data[2] = 0x05;  // Error: write failed
        *tx_len = 3;
        return;
    }

    Log_Message(BMS_MSG_LEVEL_INFO, "WD write %u bytes to 0x%08lX", payload_len, target_address);
    tx_data[0] = CMD_WD;
    tx_data[1] = 1;
    tx_data[2] = 0x00;  // Success
    *tx_len = 3;
}


