/*
 * crc16.c
 *
 * @brief  Implements a CRC-16 (Cyclic Redundancy Check) calculation for data integrity verification.
 * @note   Uses the CRC-16-IBM polynomial (0xA001) with an initial value of 0xFFFF.
 *         Commonly used in embedded systems for error detection in communication protocols
 *         and firmware updates, such as in the BQ76920 BMS firmware.
 */

#include "crc16.h"

/**
  * @brief  Calculates the CRC-16 checksum for a given data buffer
  * @param  data: Pointer to the input data buffer
  * @param  length: Number of bytes in the data buffer
  * @retval uint16_t: Calculated CRC-16 value
  * @note   Implements the CRC-16-IBM algorithm:
  *         - Polynomial: 0xA001 (x^16 + x^15 + x^2 + 1, reversed)
  *         - Initial value: 0xFFFF
  *         - No final XOR
  * @reason Provides a robust method to detect errors in data transmission or storage,
  *         critical for ensuring firmware update integrity and communication reliability
  *         in the BMS system.
  */
uint16_t CalculateCRC16(const uint8_t *data, uint32_t length) {
    // Initialize CRC to 0xFFFF (standard for CRC-16-IBM)
    uint16_t crc = 0xFFFF;

    // Process each byte in the input data
    for (uint32_t i = 0; i < length; i++) {
        // XOR the current byte with the CRC
        crc ^= data[i];

        // Process each bit of the byte (8 bits)
        for (uint8_t j = 0; j < 8; j++) {
            // Check if the least significant bit is 1
            if (crc & 0x0001) {
                // Shift right and XOR with polynomial 0xA001
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                // Shift right only
                crc >>= 1;
            }
        }
    }

    // Return the final CRC value
    return crc;
}
