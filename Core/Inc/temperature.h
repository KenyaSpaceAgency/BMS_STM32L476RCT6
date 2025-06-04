/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : temperature.h
  * @brief          : Header for temperature.c file.
  *                   This file contains the common defines for temperature sensing.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TMP100_I2C_ADDRESS_1 0x48 // I2C address for NTC-1 (ADD1=0, ADD0=0)
#define TMP100_I2C_ADDRESS_2 0x49 // I2C address for NTC-2 (ADD1=0, ADD0=Float)
#define TMP100_TEMP_REG      0x00 // Temperature Register
#define TMP100_CONFIG_REG    0x01 // Configuration Register

// Configuration Register bit definitions (datasheet Section 7.5.3)
#define TMP100_SD           (1 << 0) // Shutdown Mode (0=continuous, 1=shutdown)
#define TMP100_TM           (1 << 1) // Thermostat Mode (0=comparator, 1=interrupt)
#define TMP100_POL          (1 << 2) // Polarity (0=active low, 1=active high)
#define TMP100_F0           (1 << 3) // Fault Queue bit 0
#define TMP100_F1           (1 << 4) // Fault Queue bit 1
#define TMP100_R0           (1 << 5) // Resolution bit 0
#define TMP100_R1           (1 << 6) // Resolution bit 1
#define TMP100_OS           (1 << 7) // One-Shot (0=continuous, 1=one-shot in shutdown)
/* USER CODE END EC */

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Temperature_Init(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2);
HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, int16_t *temperature_1, int16_t *temperature_2);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __TEMPERATURE_H */
