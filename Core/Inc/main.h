#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* Exported constants --------------------------------------------------------*/
#define NUM_GROUPS_PER_IC       3    // 3S configuration per IC (each "group" is 3 cells in parallel)
#define TOTAL_GROUPS            3    // Total monitored groups (3S x 1 pack, second BQ76920 is redundant)
#define TOTAL_PHYSICAL_CELLS    9    // Total physical cells (3S3P = 9 cells)
#define CELLS_PER_GROUP         3    // Number of cells in parallel per group
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

/* Operation modes */
typedef enum {
    MODE_CHARGING = 0,
    MODE_DISCHARGING,
    MODE_SLEEP,
    MODE_FAULT
} BMS_ModeTypeDef;

/* Error flags */
#define ERROR_OVERVOLTAGE   (1 << 0)
#define ERROR_UNDERVOLTAGE  (1 << 1)
#define ERROR_OVERCURRENT   (1 << 2)
#define ERROR_OVERTEMP      (1 << 3)
#define ERROR_UNDERTEMP     (1 << 4)
#define ERROR_DISCREPANCY   (1 << 5) // Redundancy discrepancy

/* Protection thresholds */
#define OVERTEMP_THRESHOLD  45  // °C
#define UNDERTEMP_THRESHOLD 0   // °C
#define SOC_LOW_THRESHOLD   10.0 // % (for charge immediately status)

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SystemClock_Config(void);
/* Declare I2C handles as extern so they can be accessed in other files */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
/* Declare TIM handle as extern for PWM control */
extern TIM_HandleTypeDef htim4;
/* Declare USART handle as extern for RS485 communication */
extern USART_HandleTypeDef husart2;
/* Declare ADC handle for internal temperature sensor */
extern ADC_HandleTypeDef hadc1;
/* Declare logging function */
extern void Log_Error(const char *message);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
