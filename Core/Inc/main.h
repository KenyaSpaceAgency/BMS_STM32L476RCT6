/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BQ76920.h"
#include "kalman_filter.h"
#include "pid.h"
#include "temperature.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Battery configuration */
#define NUM_CELLS_PER_IC        3    // 3S configuration per IC
#define TOTAL_CELLS             6    // Total cells (3S3P, but monitoring 3S twice)
#define NOMINAL_CAPACITY        7800 // Total capacity in mAh (3P * 2600 mAh)
#define INITIAL_SOC             50.0 // Initial State of Charge in %
#define INITIAL_SOH             100.0 // Initial State of Health in %

/* Timing */
#define LOOP_TIME               1.0  // Loop time in seconds (1 second per iteration)

/* Flash memory configuration for logging */
#define FLASH_LOG_PAGE          448                  // Page 448 (address 0x080E0000)
#define FLASH_LOG_PAGE_ADDR     0x080E0000           // Page 448 start address
#define LOG_ENTRY_SIZE          64                   // Each log entry is 64 bytes
#define TIMESTAMP_SIZE          8                    // 8 bytes for Unix timestamp
#define MESSAGE_SIZE            (LOG_ENTRY_SIZE - TIMESTAMP_SIZE) // 56 bytes for message
#define NUM_LOG_ENTRIES         63                   // 63 slots (4032 bytes / 64)
#define NEXT_SLOT_ADDR          FLASH_LOG_PAGE_ADDR  // Store next_slot at the start
#define LOG_START_ADDR          (NEXT_SLOT_ADDR + 4) // Logs start after next_slot
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
/* Declare I2C handles as extern so they can be accessed in other files */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim4;
extern USART_HandleTypeDef husart2;
extern void Log_Error(const char *message);
/* USER CODE END EFP */


/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOC
#define RS4852_DE_Pin GPIO_PIN_1
#define RS4852_DE_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define BOOT2_Pin GPIO_PIN_7
#define BOOT2_GPIO_Port GPIOC
#define ALERT2_Pin GPIO_PIN_12
#define ALERT2_GPIO_Port GPIOA
#define BOOT_Pin GPIO_PIN_4
#define BOOT_GPIO_Port GPIOB
#define ALERT_Pin GPIO_PIN_5
#define ALERT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define HEATER1_Pin GPIO_PIN_9
#define HEATER1_GPIO_Port GPIOB
#define HEATER2_Pin GPIO_PIN_8
#define HEATER2_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
