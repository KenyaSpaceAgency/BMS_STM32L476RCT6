/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                This file contains the common defines of the application.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// BMS operation modes
typedef enum {
    MODE_CHARGING = 0,
    MODE_DISCHARGING,
    MODE_FAULT,
    MODE_SLEEP
} BMS_ModeTypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// Initial SOC and SOH values
#define INITIAL_SOC 50.0f // 50%
#define INITIAL_SOH 100.0f // 100%

// Battery capacity
#define NOMINAL_CAPACITY 5000.0f // 5000 mAh

// Number of cell groups per IC
#define NUM_GROUPS_PER_IC 3

// Logging constants
#define LOG_ENTRY_SIZE 64
#define TIMESTAMP_SIZE 8
#define MESSAGE_SIZE (LOG_ENTRY_SIZE - TIMESTAMP_SIZE)
#define LOG_START_ADDR 0x08080000 // Example address in flash
#define NUM_LOG_ENTRIES 100
#define NEXT_SLOT_ADDR 0x0807F800 // Example address to store next_slot
#define FLASH_LOG_PAGE 128 // Example flash page number

// Error flags
#define ERROR_OVERVOLTAGE  (1UL << 0)
#define ERROR_UNDERVOLTAGE (1UL << 1)
#define ERROR_OVERCURRENT  (1UL << 2)
#define ERROR_OVERTEMP     (1UL << 3)
#define ERROR_UNDERTEMP    (1UL << 4)
#define ERROR_DISCREPANCY  (1UL << 5)

// BMS thresholds and constants
#define SOC_LOW_THRESHOLD 20.0f // 20%
#define LOOP_TIME 1.0f // 1 second loop time
#define MAX_CHARGE_TIME 14400 // 4 hours in seconds
#define CV_VOLTAGE_THRESHOLD 4200 // 4200 mV (example for a Li-ion cell)
#define CC_CURRENT_TARGET 1000 // 1000 mA (example charging current)
#define CV_CURRENT_THRESHOLD 50 // 50 mA (example termination current)

// Extern declarations for I2C handles
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOOT2_Pin GPIO_PIN_13
#define BOOT2_GPIO_Port GPIOC
#define ALERT2_Pin GPIO_PIN_14
#define ALERT2_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOA
#define RS4852_DE_Pin GPIO_PIN_1
#define RS4852_DE_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2 // Added definition for USART2 TX pin
#define BOOT_Pin GPIO_PIN_2
#define BOOT_GPIO_Port GPIOB
#define ALERT_Pin GPIO_PIN_10
#define ALERT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
