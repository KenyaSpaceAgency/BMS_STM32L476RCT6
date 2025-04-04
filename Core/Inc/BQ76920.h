#ifndef BQ76920_H_
#define BQ76920_H_

#include "main.h"  // This brings in tools we need from the STM32 kit

// These are the "notebooks" inside the BQ76920 chip we’ll read or write
#define SYS_STAT_REG        0x00  // Warning lights notebook
#define VC1_HI_REG          0x0C  // Battery voltage notebook
#define CC_HI_REG           0x32   // Power flow notebook
#define CELLBAL1_REG        0x01  // Balancing control notebook
#define SYS_CTRL2_REG       0x05  // Charging/discharging switch notebook

// Addresses to talk to the two BQ76920 chips (like phone numbers)
#define BQ76920_I2C_ADDRESS_1 0x08
#define BQ76920_I2C_ADDRESS_2 0x09

// How many battery groups we’re watching (3 in this case)
#define NUM_GROUPS_PER_IC   3

// Safety limits for the batteries (in millivolts)
#define OV_THRESHOLD        4200  // Too full! (4.2 volts)
#define UV_THRESHOLD        2800  // Too empty! (2.8 volts)

// These are the instructions (functions) we can give the chip
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c);  // Wake up the chip
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset);  // Check battery fullness
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current);  // Check power flow
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask);  // Balance the batteries
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag);  // Look for trouble
HAL_StatusTypeDef BQ76920_CheckOvercurrent(I2C_HandleTypeDef *hi2c, uint8_t *occ_flag, uint8_t *ocd_flag);  // Check for too much power
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag);  // Compare two chips
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable);  // Turn charging on/off
HAL_StatusTypeDef BQ76920_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data);  // Read a specific notebook
HAL_StatusTypeDef BQ76920_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data);  // Write to a specific notebook
HAL_StatusTypeDef BQ76920_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);  // Check all warning lights
HAL_StatusTypeDef BQ76920_ClearStatus(I2C_HandleTypeDef *hi2c, uint8_t flags_to_clear);  // Turn off warning lights
void BQ76920_CheckStatus(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, uint32_t *error_flags);  // Check both chips’ warnings

#endif /* BQ76920_H_ */
