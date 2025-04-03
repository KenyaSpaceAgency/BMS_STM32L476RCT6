/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.h
  * @brief          : Header for pid.c file.
  *                   This file contains the common defines of the PID controller.
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
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define KP 10.0f          // Proportional gain
#define KI 0.1f           // Integral gain
#define KD 1.0f           // Derivative gain
#define DT 0.1f           // Time step (seconds), assuming 100ms loop time
#define TARGET_TEMP 25    // Target temperature in °C
#define TEMP_UPPER_LIMIT 60 // Upper temperature limit in °C to disable heaters
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void PID_Init(void); // Added declaration for PID_Init
void PID_Control(int16_t temperature);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
