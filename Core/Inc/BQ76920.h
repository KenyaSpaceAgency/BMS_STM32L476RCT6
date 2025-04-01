/*
 * BQ76920.h
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#ifndef __BQ76920_H
#define __BQ76920_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported constants --------------------------------------------------------*/
#define BQ76920_I2C_ADDRESS_1   0x08  // I2C1 address
#define BQ76920_I2C_ADDRESS_2   0x48  // I2C2 address

/* Register addresses (simplified for example) */
#define SYS_STAT_REG            0x00
#define CELLBAL1_REG            0x01
#define VC1_HI_REG              0x04
#define CC_HI_REG               0x32
#define TS1_HI_REG              0x2C

/* Protection thresholds */
#define OV_THRESHOLD            4200  // Overvoltage threshold in mV
#define UV_THRESHOLD            2800  // Undervoltage threshold in mV

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *voltages, uint8_t offset);
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current);
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *voltages, uint8_t offset);
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag);

#ifdef __cplusplus
}
#endif

#endif /* __BQ76920_H */
