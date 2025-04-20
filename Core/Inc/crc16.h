/*
 * crc32.h
 *
 *  Created on: Apr 3, 2025
 *      Author: yomue
 */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_
#include <stdint.h>
#include "main.h"
uint16_t CalculateCRC16(const uint8_t *data, uint32_t length);
#endif /* INC_CRC16_H_ */
