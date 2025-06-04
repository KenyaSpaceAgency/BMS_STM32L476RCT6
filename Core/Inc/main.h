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

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    MODE_CHARGING = 0,
    MODE_DISCHARGING,
    MODE_FAULT,
    MODE_SLEEP
} BMS_ModeTypeDef;

typedef struct {
    float nominal_capacity;
    uint16_t ov_threshold;
    uint16_t uv_threshold;
    uint16_t occ_threshold;
    uint16_t ocd_threshold;
    int16_t overtemp_threshold;
    int16_t undertemp_threshold;
    float soc_low_threshold;
    uint32_t max_charge_time;
    uint16_t cv_threshold;
} BatteryConfig;

extern BatteryConfig battery_config;

#define NUM_GROUPS_PER_IC 4
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
#define HEATER2_Pin GPIO_PIN_8
#define HEATER2_GPIO_Port GPIOB
#define HEATER1_Pin GPIO_PIN_9
#define HEATER1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
