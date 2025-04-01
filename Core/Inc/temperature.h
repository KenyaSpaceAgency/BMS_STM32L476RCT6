/*
 * temperature.h
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c, int16_t *temperature);

#ifdef __cplusplus
}
#endif

#endif /* __TEMPERATURE_H */
