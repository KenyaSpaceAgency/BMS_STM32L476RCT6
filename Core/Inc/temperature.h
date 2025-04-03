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
#define TMP100_I2C_ADDRESS_1 0x48 // I2C address for NTC-1
#define TMP100_I2C_ADDRESS_2 0x49 // I2C address for NTC-2
#define TMP100_TEMP_REG 0x00
/* USER CODE END EC */

/* Exported functions prototypes ---------------------------------------------*/
HAL_StatusTypeDef Temperature_Read(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, int16_t *temperature_1, int16_t *temperature_2);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __TEMPERATURE_H */
