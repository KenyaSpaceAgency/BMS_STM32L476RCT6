/*
 * i2c_command_handlers.h
 *
 *  Created on: Jul 10, 2025
 *      Author: yomue
 */

#ifndef INC_I2C_COMMAND_HANDLERS_H_
#define INC_I2C_COMMAND_HANDLERS_H_

#include <stdint.h>


#define ERR_OK                      0x00  // Success
#define ERR_INVALID_LENGTH          0x01  // Malformed input or too short
#define ERR_INVALID_PARAM           0x02  // e.g., zero read_len or out of range
#define ERR_OUT_OF_BOUNDS           0x03  // Memory access exceeds region
#define ERR_BUFFER_OVERFLOW         0x04  // Response too large for buffer
#define ERR_FLASH_ERASE_FAIL        0x05  // Failed to erase page
#define ERR_FLASH_WRITE_FAIL        0x06  // Failed to write
#define ERR_FLASH_ALIGNMENT         0x07  // Write not 8-byte aligned
#define ERR_FLASH_RDP_PROTECTED     0x08  // Flash read blocked by RDP





// Handler function prototypes for Generic CMDs
void Handle_PING(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len);
void Handle_GD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len);
void Handle_PD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len);
void Handle_RD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len);
void Handle_WD(uint8_t *rx_data, uint8_t len, uint8_t *tx_data, uint8_t *tx_len);





#endif /* INC_I2C_COMMAND_HANDLERS_H_ */
