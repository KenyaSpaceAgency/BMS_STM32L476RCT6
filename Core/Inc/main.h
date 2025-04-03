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
#include <string.h> // For strncpy in Log_Error
#include <stdarg.h> // For variable arguments in Log_Error
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern USART_HandleTypeDef husart2;
extern ADC_HandleTypeDef hadc1; // Add this line

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    MODE_CHARGING,
    MODE_DISCHARGING,
    MODE_FAULT,
    MODE_SLEEP
} BMS_ModeTypeDef;

// Battery pack configuration structure
typedef struct {
    float nominal_capacity;        // Battery capacity in mAh
    uint16_t ov_threshold;        // Overvoltage threshold in mV
    uint16_t uv_threshold;        // Undervoltage threshold in mV
    int16_t occ_threshold;        // Overcurrent charge threshold in mA
    int16_t ocd_threshold;        // Overcurrent discharge threshold in mA
    int16_t overtemp_threshold;   // Overtemperature threshold in °C
    int16_t undertemp_threshold;  // Undertemperature threshold in °C
    float soc_low_threshold;      // SOC low threshold in percentage
    uint32_t max_charge_time;     // Maximum charge time in seconds
    uint16_t cv_threshold;        // Constant voltage threshold in mV
} BatteryConfig;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define INITIAL_SOC 50.0f
#define INITIAL_SOH 100.0f

#define BACKUP_START_ADDR 0x08040000  // Example: Middle of flash
#define BACKUP_END_ADDR   0x0807F7FF
#define APP_VALIDITY_FLAG_ADDR 0x0807F820  // Flag to indicate valid app

#define LOG_START_ADDR 0x08080000
#define NEXT_SLOT_ADDR 0x0807F800
#define FIRMWARE_UPDATE_FLAG_ADDR 0x0807F810 // Offset within the same page as NEXT_SLOT_ADDR
#define LOG_ENTRY_SIZE 64
#define TIMESTAMP_SIZE 8
#define MESSAGE_SIZE (LOG_ENTRY_SIZE - TIMESTAMP_SIZE)
#define NUM_LOG_ENTRIES 1024
#define FLASH_LOG_PAGE 128

#define LOOP_TIME 0.1f // Loop time in seconds (100 ms)
#define NUM_GROUPS_PER_IC 3 // Define here for consistency across files

// Flash memory layout
//#define FLASH_PAGE_SIZE 2048 // 2 KB pages for STM32L476RCT6
#define BOOTLOADER_START_ADDR 0x08000000
#define BOOTLOADER_END_ADDR   0x08003FFF
#define APP_START_ADDR        0x08004000
#define APP_END_ADDR          0x0807F7FF
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ERROR_OVERVOLTAGE   (1UL << 0)
#define ERROR_UNDERVOLTAGE  (1UL << 1)
#define ERROR_OCC           (1UL << 2) // Overcurrent charge
#define ERROR_OCD           (1UL << 3) // Overcurrent discharge
#define ERROR_SCD           (1UL << 4) // Short-circuit discharge
#define ERROR_OVERTEMP      (1UL << 5)
#define ERROR_UNDERTEMP     (1UL << 6)
#define ERROR_DISCREPANCY   (1UL << 7)
#define ERROR_DEVICE_XREADY (1UL << 8)
#define ERROR_OVRD_ALERT    (1UL << 9)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Log_Error(const char *format, ...);
extern BatteryConfig battery_config;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
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
