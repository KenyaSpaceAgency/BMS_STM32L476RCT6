/*
 * delay.h
 *
 *  Created on: Jul 7, 2025
 *      Author: yomue
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stm32l4xx_hal.h"

typedef struct {
    uint32_t start_time;
    uint32_t delay_ms;
} SoftwareDelay_t;

void SoftwareDelay(uint32_t delay_ms);

#endif /* INC_DELAY_H_ */
