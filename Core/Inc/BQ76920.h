/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BQ76920.h
  * @brief   This file contains the headers of the BQ76920 battery management functions.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BQ76920_H_
#define BQ76920_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"  // This brings in tools we need from the STM32 kit

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// These are the "notebooks" inside the BQ76920 chip we’ll read or write
#define SYS_STAT_REG        0x00  // Warning lights notebook
#define VC1_HI_REG          0x0C  // Battery voltage notebook
#define CC_HI_REG           0x32  // Power flow notebook
#define CELLBAL1_REG        0x01  // Balancing control notebook
#define SYS_CTRL2_REG       0x05  // Charging/discharging switch notebook

// Addresses to talk to the two BQ76920 chips (like phone numbers)
#define BQ76920_I2C_ADDRESS_1 0x08
#define BQ76920_I2C_ADDRESS_2 0x09

// Number of battery groups we’re watching (4 in this case for 4S configuration)
#define NUM_GROUPS_PER_IC   4

// Safety limits for the batteries (in millivolts)
#define OV_THRESHOLD        4200  // Too full! (4.2 volts)
#define UV_THRESHOLD        2800  // Too empty! (2.8 volts)

// Note: Pin configuration: VCO=ground, VC1=Cell 2 positive, VC2=Cell 3 positive,
// VC3=VC4=Cell 3 positive, VC5=Cell 4 positive. Cell 1 is not monitored.
/* USER CODE END EC */

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset);
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current);
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask);
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag);
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable);
HAL_StatusTypeDef BQ76920_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
void BQ76920_CheckRedundancy(uint16_t *voltages_1, uint16_t *voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag);
void BQ76920_CheckStatus(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, uint32_t *error_flags);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* BQ76920_H_ */
