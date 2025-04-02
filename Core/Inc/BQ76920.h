/*
 * BQ76920.h
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#ifndef BQ76920_H_
#define BQ76920_H_

#include "main.h"

// Register Definitions
#define SYS_STAT_REG        0x00
#define VC1_HI_REG          0x0C
#define CC_HI_REG           0x32
#define CELLBAL1_REG        0x01
#define SYS_CTRL2_REG       0x05  // Added for CHG_ON and DSG_ON control

// I2C Addresses (shifted left by 1 for HAL)
#define BQ76920_I2C_ADDRESS_1 0x08
#define BQ76920_I2C_ADDRESS_2 0x09

// Constants
#define NUM_GROUPS_PER_IC   3  // 3 cells in series (1S, 2S, 3S)

// Thresholds (example values, adjust as needed)
#define OV_THRESHOLD        4200  // 4.2V per cell (in mV)
#define UV_THRESHOLD        2800  // 2.8V per cell (in mV)

HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset);
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current);
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask);
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag);
HAL_StatusTypeDef BQ76920_CheckOvercurrent(I2C_HandleTypeDef *hi2c, uint8_t *occ_flag, uint8_t *ocd_flag);
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag);

// Added function to enable/disable charging and discharging
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable);

#endif /* BQ76920_H_ */
