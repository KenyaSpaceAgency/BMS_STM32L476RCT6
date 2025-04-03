/*
 * crc32.h
 *
 *  Created on: Apr 3, 2025
 *      Author: yomue
 */

#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include <stdint.h>
#include "main.h"

uint32_t CalculateCRC32(const uint8_t *data, uint32_t length);

#endif /* INC_CRC32_H_ */
