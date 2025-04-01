#ifndef BQ76920_H_
#define BQ76920_H_

#include "main.h"

/* BQ76920 Register Addresses */
#define SYS_STAT_REG    0x00
#define CELLBAL1_REG    0x01
#define VC1_HI_REG      0x0A
#define CC_HI_REG       0x32

/* BQ76920 I2C Addresses */
#define BQ76920_I2C_ADDRESS_1   0x08  // I2C address for BQ76920 on I2C1
#define BQ76920_I2C_ADDRESS_2   0x18  // I2C address for BQ76920 on I2C2

/* Protection Thresholds */
#define OV_THRESHOLD    4200  // Overvoltage threshold in mV (per parallel group)
#define UV_THRESHOLD    2500  // Undervoltage threshold in mV (per parallel group)

/* Extern variables */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

/* Function prototypes */
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset);
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current);
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask);
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag);
HAL_StatusTypeDef BQ76920_CheckOvercurrent(I2C_HandleTypeDef *hi2c, uint8_t *occ_flag, uint8_t *ocd_flag);
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag);

#endif /* BQ76920_H_ */
