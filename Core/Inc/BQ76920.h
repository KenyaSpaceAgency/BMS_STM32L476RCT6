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

// Turn on the chip and get it ready
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c);

// Read how much power each battery part has
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset);

// Check how much energy is flowing
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current);

// Even out the battery parts
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask);

// Look for power problems
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag);

// Turn charging or discharging on/off
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable);
#endif /* BQ76920_H_ */
