/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "BQ76920.h"
#include "temperature.h"
#include "pid.h"
#include "kalman_filter.h"
#include "ssp.h"
#include "adc.h"
#include "crc32.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> // For va_list in Log_Error
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */
uint16_t group_voltages_1[NUM_GROUPS_PER_IC];
uint16_t group_voltages_2[NUM_GROUPS_PER_IC];
int16_t pack_current_1;
int16_t pack_current_2;
int16_t temperature_1;
int16_t temperature_2;
int16_t pcb_temperature;
float soc = INITIAL_SOC;
float soh = INITIAL_SOH;
KalmanFilter soc_kf;
KalmanFilter soh_kf;

static uint32_t next_slot = 0;
static uint8_t log_buffer[LOG_ENTRY_SIZE];

static float coulomb_count;
static float initial_capacity;
static float actual_capacity;

static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];

static BMS_ModeTypeDef bms_mode = MODE_DISCHARGING;
static uint8_t charge_enabled = 0;
static uint8_t discharge_enabled = 1;
static uint8_t charge_immediately = 0;
static uint8_t bms_online = 0;
static uint32_t error_flags = 0;

static uint32_t charge_cycle_count = 0;
static uint32_t total_charge_time = 0;
static uint32_t total_discharge_time = 0;
static uint32_t total_operating_time = 0;
static uint8_t charging_started = 0;

// Added for CC-CV charging
static uint32_t charge_start_time = 0; // Timestamp when charging starts
static uint8_t in_cv_mode = 0; // Flag to indicate CV phase

// Added for KEN command
static uint8_t mission_termination_enabled = 0; // Set by GCS via SFP

static uint8_t balancing_mask_1 = 0;
static uint8_t balancing_mask_2 = 0;
static uint8_t balancing_active = 0;

// Power line states (simplified for SON/SOF commands)
static uint8_t power_lines[16] = {0}; // 0 = OFF, 1 = ON (PWRL0 to PWRL15)

// Debug LED timing for flashing indication
static uint32_t startup_blink_start = 0;
static uint32_t last_blink_toggle = 0;
#define STARTUP_BLINK_DURATION 5000 // 5 seconds
#define BLINK_INTERVAL 200 // 200 ms on/off for rapid blinking

// Battery configuration
BatteryConfig battery_config = {
    .nominal_capacity = 2000.0f,      // Default: 2000 mAh
    .ov_threshold = 4200,             // Default: 4.2V per cell (in mV)
    .uv_threshold = 2800,             // Default: 2.8V per cell (in mV)
    .occ_threshold = 5000,            // Default: 5000 mA (charging)
    .ocd_threshold = 5000,            // Default: 5000 mA (discharging)
    .overtemp_threshold = 60,         // Default: 60°C
    .undertemp_threshold = -20,       // Default: -20°C
    .soc_low_threshold = 20.0f,       // Default: 20%
    .max_charge_time = 3600,          // Default: 3600 seconds
    .cv_threshold = 4200              // Default: 4200 mV
};

// Firmware update variables
static uint8_t firmware_update_mode = 0; // Flag to indicate firmware update mode
#define FIRMWARE_UPDATE_PACKET_SIZE 128 // Size of each firmware packet
#define FIRMWARE_UPDATE_TIMEOUT 10000   // Timeout for firmware update (10 seconds)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_Init(void);

/* USER CODE BEGIN PFP */
void Flash_Erase(uint32_t page);
void Log_Error(const char *format, ...);
void Log_Read_All(void);
void Log_Init(void);
void Update_SOC_SOH(void);
void SSP_SendStatus(void);
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame);
void Update_BMS_Mode(void);
int16_t Read_Internal_Temperature(void);
HAL_StatusTypeDef ChargeBattery(void);
void Bootloader_Check(void);
void Bootloader_FirmwareUpdate(void);
void JumpToApplication(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Erases a specified page in flash memory
  * @param  page: Page number to erase
  * @retval None
  */
void Flash_Erase(uint32_t page)
{
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = page;
    erase_init.NbPages = 1;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();
}

/**
  * @brief  Logs an error message to flash memory with a timestamp
  * @param  format: Format string (printf-style)
  * @param  ...: Variable arguments for the format string
  * @retval None
  */
void Log_Error(const char *format, ...)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    uint64_t timestamp = HAL_GetTick();
    char message_buffer[MESSAGE_SIZE];

    // Format the message using vsnprintf
    va_list args;
    va_start(args, format);
    vsnprintf(message_buffer, MESSAGE_SIZE, format, args);
    va_end(args);

    memset(log_buffer, 0, LOG_ENTRY_SIZE);
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE);
    strncpy((char *)(log_buffer + TIMESTAMP_SIZE), message_buffer, MESSAGE_SIZE - 1);

    uint32_t address = LOG_START_ADDR + (next_slot * LOG_ENTRY_SIZE);
    HAL_FLASH_Unlock();
    for (uint8_t i = 0; i < LOG_ENTRY_SIZE; i += 8) {
        uint64_t data = *(uint64_t *)(log_buffer + i);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data);
    }
    HAL_FLASH_Lock();

    next_slot = (next_slot + 1) % NUM_LOG_ENTRIES;
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot);
    HAL_FLASH_Lock();
}
/**
  * @brief  Reads and sends all logs over RS485
  * @retval None
  */
void Log_Read_All(void)
{
    char buffer[128];
    for (uint32_t i = 0; i < NUM_LOG_ENTRIES; i++) {
        uint32_t address = LOG_START_ADDR + (i * LOG_ENTRY_SIZE);
        uint64_t timestamp = *(uint64_t *)address;
        char *message = (char *)(address + TIMESTAMP_SIZE);
        snprintf(buffer, sizeof(buffer), "Log %lu: Time=%llu, Msg=%s\r\n", i, timestamp, message);
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

/**
  * @brief  Initializes the logging system
  * @retval None
  */
void Log_Init(void)
{
    next_slot = *(uint32_t *)NEXT_SLOT_ADDR;
    if (next_slot >= NUM_LOG_ENTRIES) {
        Flash_Erase(FLASH_LOG_PAGE);
        next_slot = 0;
        HAL_FLASH_Unlock();
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot);
        HAL_FLASH_Lock();
    }
}

/**
  * @brief  Updates SOC and SOH using coulomb counting and Kalman Filter
  * @retval None
  */
void Update_SOC_SOH(void)
{
    coulomb_count += ((pack_current_1 + pack_current_2) / 2.0 * LOOP_TIME) / 3600.0;
    float soc_measured = (coulomb_count / battery_config.nominal_capacity) * 100.0;
    soc = KalmanFilter_Update(&soc_kf, soc_measured);
    if (soc > 100.0) soc = 100.0;
    if (soc < 0.0) soc = 0.0;

    if (soc >= 100.0) {
        actual_capacity = coulomb_count;
        soh = (actual_capacity / initial_capacity) * 100.0;
        soh = KalmanFilter_Update(&soh_kf, soh);
        if (soh > 100.0) soh = 100.0;
        if (soh < 0.0) soh = 0.0;
    }
}

/**
  * @brief  Updates the BMS operation mode and charge/discharge status
  * @retval None
  */
void Update_BMS_Mode(void)
{
    static uint32_t fault_start_time = 0;
    static uint8_t in_fault_mode = 0;
    static uint8_t recovery_attempts = 0; // Counter for recovery attempts (e.g., for DEVICE_XREADY)
    static const uint8_t MAX_RECOVERY_ATTEMPTS = 3; // Maximum number of recovery attempts before reset
    static const uint32_t FAULT_TIMEOUT = 30000; // 30 seconds timeout for most faults
    static const uint32_t TEMP_FAULT_TIMEOUT = 60000; // 60 seconds timeout for temperature faults
    static const uint32_t COOLDOWN_PERIOD = 10000; // 10 seconds cooldown for overcurrent faults
    static const uint32_t RECOVERY_DELAY = 5000; // 5 seconds delay before recovery attempt

    // Check for faults
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OCC | ERROR_OCD | ERROR_SCD | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY | ERROR_DEVICE_XREADY | ERROR_OVRD_ALERT))
    {
        // If not already in fault mode, record the start time
        if (!in_fault_mode)
        {
            fault_start_time = HAL_GetTick();
            in_fault_mode = 1;
        }

        bms_mode = MODE_FAULT;

        // Protective actions and recovery logic based on specific faults
        if (error_flags & ERROR_OVERVOLTAGE)
        {
            charge_enabled = 0; // Disable charging to prevent further voltage increase
            discharge_enabled = 1; // Allow discharging to reduce voltage
            Log_Error("Protective action: Disabled charging due to overvoltage");

            // Recovery: Check if all cell voltages are below the threshold
            uint8_t all_below_threshold = 1;
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                if (group_voltages_1[i] > battery_config.ov_threshold || group_voltages_2[i] > battery_config.ov_threshold)
                {
                    all_below_threshold = 0;
                    break;
                }
            }
            if (all_below_threshold)
            {
                Log_Error("Overvoltage fault cleared");
                error_flags &= ~ERROR_OVERVOLTAGE;
                in_fault_mode = 0;
            }
            else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
            {
                Log_Error("Overvoltage fault persists, triggering system reset");
                HAL_NVIC_SystemReset();
            }
        }
        else if (error_flags & ERROR_UNDERVOLTAGE)
        {
            charge_enabled = 1; // Allow charging to recover battery
            discharge_enabled = 0; // Disable discharging to prevent deep discharge
            Log_Error("Protective action: Disabled discharging due to undervoltage");

            // Recovery: Check if all cell voltages are above the threshold
            uint8_t all_above_threshold = 1;
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                if (group_voltages_1[i] < battery_config.uv_threshold || group_voltages_2[i] < battery_config.uv_threshold)
                {
                    all_above_threshold = 0;
                    break;
                }
            }
            if (all_above_threshold)
            {
                Log_Error("Undervoltage fault cleared");
                error_flags &= ~ERROR_UNDERVOLTAGE;
                in_fault_mode = 0;
            }
            else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
            {
                Log_Error("Undervoltage fault persists, triggering system reset");
                HAL_NVIC_SystemReset();
            }
        }
        else if (error_flags & ERROR_OCC)
        {
            charge_enabled = 0; // Disable charging to prevent damage
            discharge_enabled = 1; // Allow discharging
            Log_Error("Protective action: Disabled charging due to overcurrent charge");

            // Recovery: Wait for a cooldown period and check current
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD)
            {
                int16_t total_current = (pack_current_1 + pack_current_2) / 2;
                if (total_current >= 0) // Ensure no charging current
                {
                    Log_Error("Overcurrent charge fault cleared");
                    error_flags &= ~ERROR_OCC;
                    in_fault_mode = 0;
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("Overcurrent charge fault persists, triggering system reset");
                    HAL_NVIC_SystemReset();
                }
            }
        }
        else if (error_flags & ERROR_OCD)
        {
            charge_enabled = 1; // Allow charging
            discharge_enabled = 0; // Disable discharging to prevent damage
            Log_Error("Protective action: Disabled discharging due to overcurrent discharge");

            // Recovery: Wait for a cooldown period and check current
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD)
            {
                int16_t total_current = (pack_current_1 + pack_current_2) / 2;
                if (total_current <= 0) // Ensure no discharging current
                {
                    Log_Error("Overcurrent discharge fault cleared");
                    error_flags &= ~ERROR_OCD;
                    in_fault_mode = 0;
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("Overcurrent discharge fault persists, triggering system reset");
                    HAL_NVIC_SystemReset();
                }
            }
        }
        else if (error_flags & ERROR_SCD)
        {
            charge_enabled = 0; // Disable both charging and discharging
            discharge_enabled = 0;
            Log_Error("Protective action: Disabled charging and discharging due to short-circuit discharge");

            // Recovery: Wait and check if the fault clears in SYS_STAT
            if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
            {
                uint8_t status1, status2;
                uint8_t scd_cleared = 1;
                if (BQ76920_ReadStatus(&hi2c1, &status1) == HAL_OK && (status1 & (1 << 3)))
                {
                    scd_cleared = 0;
                }
                if (BQ76920_ReadStatus(&hi2c2, &status2) == HAL_OK && (status2 & (1 << 3)))
                {
                    scd_cleared = 0;
                }
                if (scd_cleared)
                {
                    Log_Error("Short-circuit discharge fault cleared");
                    error_flags &= ~ERROR_SCD;
                    in_fault_mode = 0;
                }
                else
                {
                    Log_Error("Short-circuit discharge fault persists, triggering system reset");
                    HAL_NVIC_SystemReset();
                }
            }
        }
        else if (error_flags & ERROR_OVERTEMP)
        {
            charge_enabled = 0; // Disable both charging and discharging
            discharge_enabled = 0;
            Log_Error("Protective action: Disabled charging and discharging due to overtemperature");

            // Recovery: Check if temperatures are below a safe threshold
            if (temperature_1 < (battery_config.overtemp_threshold - 10) && temperature_2 < (battery_config.overtemp_threshold - 10) && pcb_temperature < (battery_config.overtemp_threshold - 10))
            {
                Log_Error("Overtemperature fault cleared");
                error_flags &= ~ERROR_OVERTEMP;
                in_fault_mode = 0;
            }
            else if (HAL_GetTick() - fault_start_time >= TEMP_FAULT_TIMEOUT)
            {
                Log_Error("Overtemperature fault persists, triggering system reset");
                HAL_NVIC_SystemReset();
            }
        }
        else if (error_flags & ERROR_UNDERTEMP)
        {
            charge_enabled = 0; // Disable charging to prevent lithium plating
            discharge_enabled = 1; // Allow discharging if safe
            Log_Error("Protective action: Disabled charging due to undertemperature");

            // Recovery: Check if temperatures are above a safe threshold
            if (temperature_1 > (battery_config.undertemp_threshold + 10) && temperature_2 > (battery_config.undertemp_threshold + 10))
            {
                Log_Error("Undertemperature fault cleared");
                error_flags &= ~ERROR_UNDERTEMP;
                in_fault_mode = 0;
            }
            else if (HAL_GetTick() - fault_start_time >= TEMP_FAULT_TIMEOUT)
            {
                Log_Error("Undertemperature fault persists, triggering system reset");
                HAL_NVIC_SystemReset();
            }
        }
        else if (error_flags & ERROR_DISCREPANCY)
        {
            charge_enabled = 0; // Disable both charging and discharging
            discharge_enabled = 0;
            Log_Error("Protective action: Disabled charging and discharging due to redundancy discrepancy");

            // Recovery: Attempt to reinitialize BQ76920 ICs
            if (HAL_GetTick() - fault_start_time >= RECOVERY_DELAY)
            {
                Log_Error("Attempting to reinitialize BQ76920 ICs to resolve discrepancy");
                if (BQ76920_Init(&hi2c1) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C1)");
                }
                if (BQ76920_Init(&hi2c2) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C2)");
                }

                // Recheck redundancy
                uint8_t discrepancy_flag = 0;
                BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag);
                if (!discrepancy_flag)
                {
                    Log_Error("Redundancy discrepancy fault cleared");
                    error_flags &= ~ERROR_DISCREPANCY;
                    in_fault_mode = 0;
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("Redundancy discrepancy fault persists, triggering system reset");
                    HAL_NVIC_SystemReset();
                }
            }
        }
        else if (error_flags & ERROR_DEVICE_XREADY)
        {
            charge_enabled = 0; // Disable both charging and discharging
            discharge_enabled = 0;
            Log_Error("Protective action: Disabled charging and discharging due to DEVICE_XREADY");

            // Recovery: Attempt to reinitialize BQ76920 ICs after a delay
            if (HAL_GetTick() - fault_start_time >= RECOVERY_DELAY)
            {
                recovery_attempts++;
                Log_Error("Attempting to reinitialize BQ76920 ICs (attempt %d)", recovery_attempts);
                uint8_t init_success = 1;
                if (BQ76920_Init(&hi2c1) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C1)");
                    init_success = 0;
                }
                if (BQ76920_Init(&hi2c2) != HAL_OK)
                {
                    Log_Error("Failed to reinitialize BQ76920 (I2C2)");
                    init_success = 0;
                }

                if (init_success)
                {
                    Log_Error("DEVICE_XREADY fault cleared");
                    error_flags &= ~ERROR_DEVICE_XREADY;
                    in_fault_mode = 0;
                    recovery_attempts = 0; // Reset the counter
                }
                else if (recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    Log_Error("Failed to recover from DEVICE_XREADY after %d attempts, triggering system reset", MAX_RECOVERY_ATTEMPTS);
                    HAL_NVIC_SystemReset();
                }
            }
        }
        else if (error_flags & ERROR_OVRD_ALERT)
        {
            charge_enabled = 0; // Disable both charging and discharging as a precaution
            discharge_enabled = 0;
            Log_Error("Protective action: Disabled charging and discharging due to OVRD_ALERT");

            // Recovery: Wait and check if the fault clears in SYS_STAT
            if (HAL_GetTick() - fault_start_time >= COOLDOWN_PERIOD)
            {
                uint8_t status1, status2;
                uint8_t alert_cleared = 1;
                if (BQ76920_ReadStatus(&hi2c1, &status1) == HAL_OK && (status1 & (1 << 6)))
                {
                    alert_cleared = 0;
                }
                if (BQ76920_ReadStatus(&hi2c2, &status2) == HAL_OK && (status2 & (1 << 6)))
                {
                    alert_cleared = 0;
                }
                if (alert_cleared)
                {
                    Log_Error("OVRD_ALERT fault cleared");
                    error_flags &= ~ERROR_OVRD_ALERT;
                    in_fault_mode = 0;
                }
                else if (HAL_GetTick() - fault_start_time >= FAULT_TIMEOUT)
                {
                    Log_Error("OVRD_ALERT fault persists, triggering system reset");
                    HAL_NVIC_SystemReset();
                }
            }
        }

        // Update BQ76920 charge/discharge settings
        BQ76920_SetChargeEnable(&hi2c1, charge_enabled, discharge_enabled);
        BQ76920_SetChargeEnable(&hi2c2, charge_enabled, discharge_enabled);
        return;
    }

    // If no faults, clear fault mode flag and recovery attempts
    in_fault_mode = 0;
    recovery_attempts = 0;

    // Check if the battery is too low and needs to charge right away
    charge_immediately = (soc < battery_config.soc_low_threshold) ? 1 : 0;

    // Look at the current to decide what to do
    int16_t total_current = (pack_current_1 + pack_current_2) / 2;
    if (total_current < 0) { // Charging (negative current)
        bms_mode = MODE_CHARGING;
        charge_enabled = 1;
        discharge_enabled = 0;
    } else if (total_current > 0) { // Discharging (positive current)
        bms_mode = MODE_DISCHARGING;
        charge_enabled = 0;
        discharge_enabled = 1;
    } else { // Idle (no current)
        if (soc < battery_config.soc_low_threshold) { // Start charging if SOC is low
            bms_mode = MODE_CHARGING;
            charge_enabled = 1;
            discharge_enabled = 0;
        } else { // Go to sleep to save power
            bms_mode = MODE_SLEEP;
            charge_enabled = 0;
            discharge_enabled = 0;
        }
    }

    // Update BQ76920 charge/discharge settings
    BQ76920_SetChargeEnable(&hi2c1, charge_enabled, discharge_enabled);
    BQ76920_SetChargeEnable(&hi2c2, charge_enabled, discharge_enabled);
}

/**
  * @brief  Reads the internal temperature sensor of the STM32
  * @retval Temperature in °C
  */
int16_t Read_Internal_Temperature(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    int32_t temp = ((raw * 3300 / 4096) - 760) * 100 / 250 + 25;
    return (int16_t)temp;
}

/**
  * @brief  Sends the current BMS status to the OBC using SSP
  * @retval None
  */
void SSP_SendStatus(void)
{
    SSP_TelemetryTypeDef telemetry = {0};
    SSP_FrameTypeDef frame = {0};
    uint16_t frame_len;

    uint32_t pack_voltage = group_voltages_1[0] + group_voltages_1[1] + group_voltages_1[2];

    telemetry.mode = bms_mode;
    telemetry.charge_enabled = charge_enabled;
    telemetry.discharge_enabled = discharge_enabled;
    telemetry.charge_immediately = charge_immediately;
    telemetry.bms_online = bms_online;
    telemetry.error_flags = error_flags;
    telemetry.pack_voltage_1 = (uint16_t)pack_voltage;
    telemetry.pack_voltage_2 = (uint16_t)pack_voltage;
    telemetry.pack_current_1 = pack_current_1;
    telemetry.pack_current_2 = pack_current_2;
    telemetry.soc = (uint8_t)soc;
    telemetry.soh = (uint8_t)soh;
    telemetry.temp_1 = temperature_1;
    telemetry.temp_2 = temperature_2;
    telemetry.pcb_temp = pcb_temperature;
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        telemetry.group_voltages[i] = group_voltages_1[i];
    }
    telemetry.balancing_active = balancing_active;
    telemetry.balancing_mask_1 = balancing_mask_1;
    telemetry.balancing_mask_2 = balancing_mask_2;
    telemetry.charge_cycle_count = charge_cycle_count;
    telemetry.total_charge_time = total_charge_time;
    telemetry.total_discharge_time = total_discharge_time;
    telemetry.total_operating_time = total_operating_time;

    SSP_PackTelemetry(&telemetry, &frame);
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len);
    // Set DE pin high to transmit
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET);
    SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len);
    // Set DE pin low to receive
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Processes a received SSP frame
  * @param  frame: Pointer to the received SSP frame
  * @retval None
  */
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame)
{
    // Check if the frame is for the EPS (BMS)
    if (frame->dest != SSP_ADDR_EPS && frame->dest != SSP_ADDR_BROADCAST && frame->dest != SSP_ADDR_MULTICAST) {
        return;
    }

    // Check if it's a command frame (bit 6 = 0)
    if (frame->cmd_id & SSP_FRAME_TYPE_REPLY) {
        return; // Ignore reply frames
    }

    // Check if it's a time-tagged command (bit 7 = 1)
    if (frame->cmd_id & SSP_CMD_TYPE_TIMETAG) {
        // Time-tagged commands are handled by OBC, not EPS
        return;
    }

    // Log the received command
    char log_msg[MESSAGE_SIZE];
    snprintf(log_msg, sizeof(log_msg), "Received CMD: ID=0x%02X, SRC=0x%02X, LEN=%d", frame->cmd_id, frame->src, frame->data_len);
    Log_Error(log_msg);

    // Prepare a response frame
    SSP_FrameTypeDef response = {0};
    response.dest = frame->src;
    response.src = SSP_ADDR_EPS;
    response.data_len = 1; // Default to 1 byte for ACK/NACK data
    response.data[0] = frame->cmd_id; // Echo the command ID in the response

    // Process the command
    switch (frame->cmd_id & 0x3F) { // Mask out the type bits
        case SSP_CMD_PING: // 0x00
            response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
            break;

        case SSP_CMD_SON: // 0x0B
            if (frame->data_len == 1) {
                uint8_t pwrl_id = frame->data[0];
                if (pwrl_id <= 15) { // PWRL0 to PWRL15
                    power_lines[pwrl_id] = 1; // Turn ON
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    snprintf(log_msg, sizeof(log_msg), "SON command: PWRL%d ON", pwrl_id);
                    Log_Error(log_msg);
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            }
            break;

        case SSP_CMD_SOF: // 0x0C
            if (frame->data_len == 1) {
                uint8_t pwrl_id = frame->data[0];
                if (pwrl_id <= 15) { // PWRL0 to PWRL15
                    power_lines[pwrl_id] = 0; // Turn OFF
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    snprintf(log_msg, sizeof(log_msg), "SOF command: PWRL%d OFF", pwrl_id);
                    Log_Error(log_msg);
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            }
            break;

        case SSP_CMD_KEN: // 0x31
            if (frame->data_len == 0) {
                if (mission_termination_enabled) {
                    // In a real implementation, this would disconnect the batteries
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    Log_Error("KEN command executed: Mission termination enabled");
                } else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                    Log_Error("KEN command rejected: Mission termination not enabled");
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            }
            break;

        case SSP_CMD_KDIS: // 0x32
            if (frame->data_len == 0) {
                // In a real implementation, this would reconnect the batteries
                response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                Log_Error("KDIS command executed: Batteries reconnected");
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            }
            break;

        case SSP_CMD_SM: // 0x15
            if (frame->data_len >= 1) {
                bms_mode = frame->data[0];
                Update_BMS_Mode();
                response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                Log_Error("Mode changed by OBC");
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            }
            break;

        case SSP_CMD_GM: // 0x16
            response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
            response.data_len = 1;
            response.data[0] = bms_mode;
            break;

        case SSP_CMD_GOSTM: // 0x25
            SSP_SendStatus();
            return; // SSP_SendStatus handles the response

        case SSP_CMD_SFP: // 0x1B
            if (frame->data_len >= 2) {
                uint8_t param_id = frame->data[0];
                uint8_t param_value = frame->data[1];
                if (param_id == 0x01) { // Mission termination enable parameter
                    mission_termination_enabled = param_value;
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    Log_Error("Mission termination enable set");
                }
                else if (param_id == 0x02) { // Firmware update trigger parameter
                    if (param_value == 1) {
                        // Set the firmware update flag and reboot
                        HAL_FLASH_Unlock();
                        Flash_Erase(FLASH_LOG_PAGE); // Erase the page containing the flag
                        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FIRMWARE_UPDATE_FLAG_ADDR, 0xDEADBEEF);
                        HAL_FLASH_Lock();
                        Log_Error("Firmware update requested, rebooting...");
                        HAL_Delay(100); // Allow time for logging
                        HAL_NVIC_SystemReset();
                    }
                    response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                }
                else {
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                }
            } else {
                response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            }
            break;

        case SSP_CMD_GSTLM: // 0x22
        case SSP_CMD_GOTLM: // 0x21
            SSP_SendStatus();
            return; // SSP_SendStatus handles the response

        default:
            response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
            break;
    }

    // Send the response
    uint16_t frame_len;
    SSP_ConstructFrame(&response, ssp_tx_buffer, &frame_len);
    // Set DE pin high to transmit
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET);
    SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len);
    // Set DE pin low to receive
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Implements the CC-CV charging algorithm
  * @retval None
  */
HAL_StatusTypeDef ChargeBattery(void)
{
    int16_t temperature_1, temperature_2;
    HAL_StatusTypeDef status;

    // Read temperatures from both sensors
    status = Temperature_Read(&hi2c1, &hi2c2, &temperature_1, &temperature_2);
    if (status != HAL_OK || temperature_1 == INT16_MIN || temperature_2 == INT16_MIN)
    {
        // Handle I2C read error or sensor failure
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Disable EPS power (EPS_EN)
        in_cv_mode = 0; // Reset CV mode on error
        charge_start_time = 0; // Reset charge start time
        return HAL_ERROR;
    }

    // Compute the highest temperature
    int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2;

    // Check for over-temperature condition
    if (highest_temp > battery_config.overtemp_threshold || pcb_temperature > battery_config.overtemp_threshold)
    {
        // Over-temperature detected, disable charging
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Disable EPS power (EPS_EN)
        in_cv_mode = 0; // Reset CV mode on error
        charge_start_time = 0; // Reset charge start time
        return HAL_ERROR;
    }

    // CC-CV charging logic
    if (!in_cv_mode)
    {
        // Constant Current (CC) mode
        // (Add your CC charging logic here, e.g., set charging current)
        if (charge_start_time == 0)
        {
            charge_start_time = HAL_GetTick(); // Record start time
        }

        // Check if we should switch to CV mode (based on voltage)
        int16_t max_voltage = 0;
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
        {
            if (group_voltages_1[i] > max_voltage) max_voltage = group_voltages_1[i];
            if (group_voltages_2[i] > max_voltage) max_voltage = group_voltages_2[i];
        }
        if (max_voltage > battery_config.cv_threshold)
        {
            in_cv_mode = 1; // Switch to CV mode
        }
    }
    else
    {
        // Constant Voltage (CV) mode
        // (Add your CV charging logic here, e.g., maintain constant voltage)
        uint32_t charge_duration = (HAL_GetTick() - charge_start_time) / 1000; // Duration in seconds
        if (charge_duration > battery_config.max_charge_time)
        {
            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // Disable charging
            in_cv_mode = 0; // Reset CV mode
            charge_start_time = 0; // Reset charge start time
        }
    }

    return HAL_OK;
}

/**
  * @brief  Checks if the system should enter bootloader mode for firmware update
  * @retval None
  */
void Bootloader_Check(void)
{
    // Read the firmware update flag from flash
    uint32_t firmware_update_flag = *(volatile uint32_t *)FIRMWARE_UPDATE_FLAG_ADDR;

    if (firmware_update_flag == 0xDEADBEEF) // Magic number to indicate firmware update mode
    {
        firmware_update_mode = 1;
        Log_Error("Entering firmware update mode");

        // Clear the firmware update flag
        HAL_FLASH_Unlock();
        Flash_Erase(FLASH_LOG_PAGE); // Erase the page containing the flag
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FIRMWARE_UPDATE_FLAG_ADDR, 0xFFFFFFFF);
        HAL_FLASH_Lock();
    }
    else
    {
        firmware_update_mode = 0;
        Log_Error("Booting to application");
    }
}



/**
  * @brief  Handles the firmware update process over RS485
  * @retval None
  */
void Bootloader_FirmwareUpdate(void) {
    SSP_FrameTypeDef received_frame = {0};
    uint32_t last_packet_time = HAL_GetTick();
    uint32_t current_address = APP_START_ADDR;
    uint32_t total_bytes_received = 0;
    uint32_t expected_firmware_size = 0;
    uint8_t firmware_buffer[FIRMWARE_UPDATE_PACKET_SIZE];
    uint32_t calculated_crc = 0xFFFFFFFF; // Initialize for CRC32

    Log_Error("Waiting for firmware update packets...");

    // Erase the application flash region
    HAL_FLASH_Unlock();
    for (uint32_t addr = APP_START_ADDR; addr < APP_END_ADDR; addr += FLASH_PAGE_SIZE) {
        Flash_Erase((addr - FLASH_BASE) / FLASH_PAGE_SIZE);
    }
    HAL_FLASH_Lock();

    while (1) {
        if (HAL_GetTick() - last_packet_time > FIRMWARE_UPDATE_TIMEOUT) {
            Log_Error("Firmware update timeout, rebooting...");
            HAL_NVIC_SystemReset();
        }

        if (SSP_ReceiveFrame(&husart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK) {
            last_packet_time = HAL_GetTick();

            if (received_frame.dest != SSP_ADDR_EPS && received_frame.dest != SSP_ADDR_BROADCAST) {
                continue;
            }

            if (received_frame.cmd_id & SSP_FRAME_TYPE_REPLY) {
                continue;
            }

            SSP_FrameTypeDef response = {0};
            response.dest = received_frame.src;
            response.src = SSP_ADDR_EPS;
            response.data_len = 1;
            response.data[0] = received_frame.cmd_id;

            switch (received_frame.cmd_id & 0x3F) {
                case SSP_CMD_FIRMWARE_UPDATE:
                    if (received_frame.data_len < 4) {
                        response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                        break;
                    }

                    if (total_bytes_received == 0) {
                        expected_firmware_size = (received_frame.data[0] << 24) |
                                                 (received_frame.data[1] << 16) |
                                                 (received_frame.data[2] << 8) |
                                                 received_frame.data[3];
                        Log_Error("Firmware update started, expected size: %lu bytes", expected_firmware_size);
                        response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;
                    } else {
                        if (received_frame.data_len > FIRMWARE_UPDATE_PACKET_SIZE) {
                            response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                            break;
                        }

                        memcpy(firmware_buffer, received_frame.data, received_frame.data_len);

                        HAL_FLASH_Unlock();
                        for (uint32_t i = 0; i < received_frame.data_len; i += 8) {
                            uint64_t data = *(uint64_t *)(firmware_buffer + i);
                            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, current_address + i, data);
                        }
                        HAL_FLASH_Lock();

                        // Update CRC32 incrementally
                        calculated_crc = CalculateCRC32(firmware_buffer, received_frame.data_len);

                        total_bytes_received += received_frame.data_len;
                        current_address += received_frame.data_len;

                        Log_Error("Received %lu/%lu bytes", total_bytes_received, expected_firmware_size);
                        response.cmd_id = SSP_CMD_ACK | SSP_FRAME_TYPE_REPLY;

                        if (total_bytes_received >= expected_firmware_size) {
                            uint32_t received_crc = (firmware_buffer[received_frame.data_len - 4] << 24) |
                                                    (firmware_buffer[received_frame.data_len - 3] << 16) |
                                                    (firmware_buffer[received_frame.data_len - 2] << 8) |
                                                    (firmware_buffer[received_frame.data_len - 1]);
                            if (calculated_crc == received_crc) {
                                HAL_FLASH_Unlock();
                                HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, APP_VALIDITY_FLAG_ADDR, 0xA5A5A5A5);
                                HAL_FLASH_Lock();
                                Log_Error("Firmware update completed successfully, rebooting...");
                                HAL_Delay(100);
                                HAL_NVIC_SystemReset();
                            } else {
                                Log_Error("Firmware CRC32 mismatch, rebooting without setting validity flag...");
                                HAL_Delay(100);
                                HAL_NVIC_SystemReset();
                            }
                        }
                    }
                    break;

                default:
                    response.cmd_id = SSP_CMD_NACK | SSP_FRAME_TYPE_REPLY;
                    break;
            }

            uint16_t frame_len;
            SSP_ConstructFrame(&response, ssp_tx_buffer, &frame_len);
            HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET);
            SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len);
            HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);
        }
    }
}

/**
  * @brief  Jumps to the application code
  * @retval None
  */
static uint8_t IsApplicationValid(uint32_t start_addr) {
    uint32_t crc = CalculateCRC32((uint8_t *)start_addr, APP_END_ADDR - start_addr - 4);
    uint32_t stored_crc = *(uint32_t *)(APP_END_ADDR - 4);
    return (crc == stored_crc) && (*(uint32_t *)APP_VALIDITY_FLAG_ADDR == 0xA5A5A5A5);
}

void JumpToApplication(void) {
    if (IsApplicationValid(APP_START_ADDR)) {
        uint32_t app_jump_address = *(volatile uint32_t *)(APP_START_ADDR + 4);
        void (*app_reset_handler)(void) = (void (*)(void))app_jump_address;
        __set_MSP(*(volatile uint32_t *)APP_START_ADDR);
        app_reset_handler();
    } else {
        Log_Error("Main application invalid, falling back to backup...");
        if (IsApplicationValid(BACKUP_START_ADDR)) {
            uint32_t backup_jump_address = *(volatile uint32_t *)(BACKUP_START_ADDR + 4);
            void (*backup_reset_handler)(void) = (void (*)(void))backup_jump_address;
            __set_MSP(*(volatile uint32_t *)BACKUP_START_ADDR);
            backup_reset_handler();
        } else {
            Log_Error("Backup application also invalid, halting...");
            while (1) {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                HAL_Delay(500);
            }
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_RTC_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART2_Init();
    MX_ADC1_Init();
    /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    sTime.Hours = 12;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sDate.Year = 25;
    sDate.Month = 3;
    sDate.Date = 28;
    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    Log_Init();

    // Check if we should enter bootloader mode
    Bootloader_Check();

    if (firmware_update_mode)
    {
        Bootloader_FirmwareUpdate();
        // If we return from Bootloader_FirmwareUpdate, something went wrong, so reboot
        HAL_NVIC_SystemReset();
    }

    // Jump to the application
    JumpToApplication();

    // The following code will only execute if the jump fails
    Log_Error("Failed to jump to application, entering normal operation");

    if (BQ76920_Init(&hi2c1) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C1) initialization failed");
        Error_Handler();
    }

    if (BQ76920_Init(&hi2c2) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C2) initialization failed");
        Error_Handler();
    }

    KalmanFilter_Init(&soc_kf, INITIAL_SOC, 1.0, 0.01, 1.0);
    KalmanFilter_Init(&soh_kf, INITIAL_SOH, 1.0, 0.01, 1.0);

    PID_Init();

    bms_online = 1;

    // Debug LED: Start blinking to indicate application startup (post-flashing)
    startup_blink_start = HAL_GetTick();
    last_blink_toggle = HAL_GetTick();

    // Initialize battery configuration (in a real system, load from flash or EEPROM)
    initial_capacity = battery_config.nominal_capacity;
    actual_capacity = battery_config.nominal_capacity;
    coulomb_count = (INITIAL_SOC / 100.0) * battery_config.nominal_capacity;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t last_log_read = 0;
    uint32_t last_status_send = 0;
    uint32_t last_time_sync = 0;

    while (1)
    {
        // Debug LED: Blink rapidly for 5 seconds after startup
        if (HAL_GetTick() - startup_blink_start < STARTUP_BLINK_DURATION)
        {
            if (HAL_GetTick() - last_blink_toggle >= BLINK_INTERVAL)
            {
                HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                last_blink_toggle = HAL_GetTick();
            }
        }
        else
        {
            // After 5 seconds, turn off the LED
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }

        // Step 1: Read data from the first BQ76920 (IC2)
        if (BQ76920_ReadVoltages(&hi2c1, group_voltages_1, 0) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C1) group voltages");
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                group_voltages_1[i] = 0;
            }
        }
        if (BQ76920_ReadCurrent(&hi2c1, &pack_current_1) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C1) current");
        }

        // Step 2: Read data from the second BQ76920 (IC4)
        if (BQ76920_ReadVoltages(&hi2c2, group_voltages_2, 0) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C2) group voltages");
            for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
            {
                group_voltages_2[i] = 0;
            }
        }
        if (BQ76920_ReadCurrent(&hi2c2, &pack_current_2) != HAL_OK)
        {
            Log_Error("Error reading BQ76920 (I2C2) current");
        }

        // Step 2.5: Read temperatures from both sensors (NTC-1 and NTC-2)
        if (Temperature_Read(&hi2c1, &hi2c2, &temperature_1, &temperature_2) != HAL_OK)
        {
            Log_Error("Error reading temperatures (I2C1/I2C2)");
            temperature_1 = INT16_MIN; // Sentinel value to indicate error
            temperature_2 = INT16_MIN; // Sentinel value to indicate error
        }

        // Step 3: Check for redundancy discrepancies
        uint8_t discrepancy_flag = 0;
        BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag);
        if (discrepancy_flag)
        {
            error_flags |= ERROR_DISCREPANCY;
            Log_Error("Redundancy discrepancy detected");
        }

        // Step 4: Read internal temperature
        pcb_temperature = Read_Internal_Temperature();

        // Step 5: Check BQ76920 status flags (replaces manual OV/UV and overcurrent checks)
        BQ76920_CheckStatus(&hi2c1, &hi2c2, &error_flags);

        // Step 6: Check temperature limits
        int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2;
        int16_t lowest_temp = (temperature_1 < temperature_2) ? temperature_1 : temperature_2;
        if (highest_temp > battery_config.overtemp_threshold || pcb_temperature > battery_config.overtemp_threshold)
        {
            error_flags |= ERROR_OVERTEMP;
            Log_Error("Overtemperature detected");
        }
        if (lowest_temp < battery_config.undertemp_threshold)
        {
            error_flags |= ERROR_UNDERTEMP;
            Log_Error("Undertemperature detected");
        }

        // Step 7: Update timers
        total_operating_time = HAL_GetTick() / 1000;
        int16_t total_current = (pack_current_1 + pack_current_2) / 2;
        if (total_current < 0) {
            total_charge_time += (uint32_t)LOOP_TIME;
            if (soc < 20.0 && !charging_started) {
                charging_started = 1;
            }
            if (soc >= 100.0 && charging_started) {
                charge_cycle_count++;
                charging_started = 0;
            }
        } else if (total_current > 0) {
            total_discharge_time += (uint32_t)LOOP_TIME;
        }

        // Step 8: Balance cells
        if (BQ76920_BalanceCells(&hi2c1, group_voltages_1, 0, &balancing_mask_1) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C1)");
        }
        if (BQ76920_BalanceCells(&hi2c2, group_voltages_2, 0, &balancing_mask_2) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C2)");
        }
        balancing_active = (balancing_mask_1 || balancing_mask_2) ? 1 : 0;

        // Step 9: Control heaters
        PID_Control(lowest_temp);

        // Step 10: Update SOC and SOH
        Update_SOC_SOH();

        // Step 11: Update BMS mode
        Update_BMS_Mode();

        // Step 12: Apply CC-CV charging control
        ChargeBattery();

        // Step 13: Log data
        char message[MESSAGE_SIZE];
        snprintf(message, sizeof(message), "Time: %lu | ", HAL_GetTick());
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
        {
            char group_data[20];
            snprintf(group_data, sizeof(group_data), "Group%d: %dmV ", i + 1, group_voltages_1[i]);
            strncat(message, group_data, MESSAGE_SIZE - strlen(message) - 1);
        }
        char temp_data[88];
        snprintf(temp_data, sizeof(temp_data), "I1: %dmA I2: %dmA T1: %dC T2: %dC PCB: %dC SOC: %.1f%% SOH: %.1f%%",
                 pack_current_1, pack_current_2, temperature_1, temperature_2, pcb_temperature, soc, soh);
        strncat(message, temp_data, MESSAGE_SIZE - strlen(message) - 1);
        Log_Error(message);

        // Step 14: Send logs every 10 seconds
        if (HAL_GetTick() - last_log_read >= 10000)
        {
            Log_Read_All();
            last_log_read = HAL_GetTick();
        }

        // Step 15: Send status every 5 seconds
        if (HAL_GetTick() - last_status_send >= 5000)
        {
            SSP_SendStatus();
            last_status_send = HAL_GetTick();
        }

        // Step 16: Synchronize time with OBC every 60 seconds
        if (HAL_GetTick() - last_time_sync >= 60000)
        {
            SSP_TimeTypeDef time = {0};
            if (SSP_RequestTime(&husart2, &time) == HAL_OK)
            {
                RTC_TimeTypeDef sTime = {0};
                RTC_DateTypeDef sDate = {0};
                sTime.Hours = time.hour;
                sTime.Minutes = time.minute;
                sTime.Seconds = time.second;
                sDate.Year = (uint8_t)(time.year - 2000); // Assuming year is since 2000
                sDate.Month = time.month;
                sDate.Date = time.day;
                HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
                HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
                Log_Error("Time synchronized with OBC");
            }
            else
            {
                Log_Error("Failed to synchronize time with OBC");
            }
            last_time_sync = HAL_GetTick();
        }

        // Step 17: Process received SSP frames
        SSP_FrameTypeDef received_frame = {0};
        if (SSP_ReceiveFrame(&husart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK)
        {
            SSP_ProcessReceivedFrame(&received_frame);
        }

        HAL_Delay((uint32_t)(LOOP_TIME * 1000));
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00210607; // 400 kHz with 8 MHz system clock
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00210607; // 400 kHz with 8 MHz system clock
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = 0x00210607; // 400 kHz with 8 MHz system clock
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }

    sTime.Hours = 0x0;
    sTime.Minutes = 0x0;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 0x1;
    sDate.Year = 0x0;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    if (htim->Instance == TIM4) {
//        __HAL_RCC_GPIOB_CLK_ENABLE();
//        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // Adjust pins for TIM4 CH3 and CH4
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    }
//}

static void MX_TIM4_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 79;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 999;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */
    HAL_TIM_PWM_MspInit(&htim4);

        /* USER CODE END TIM4_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{
    husart2.Instance = USART2;
    husart2.Init.BaudRate = 115200;
    husart2.Init.WordLength = USART_WORDLENGTH_8B;
    husart2.Init.StopBits = USART_STOPBITS_1;
    husart2.Init.Parity = USART_PARITY_NONE;
    husart2.Init.Mode = USART_MODE_TX_RX;
    husart2.Init.CLKPolarity = USART_POLARITY_LOW;
    husart2.Init.CLKPhase = USART_PHASE_1EDGE;
    husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
    if (HAL_USART_Init(&husart2) != HAL_OK)
    {
        Error_Handler();
    }
    // Manually control the DE pin for RS485
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); // Receive mode by default
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_Pin (PC3) */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : RS4852_DE_Pin */
    GPIO_InitStruct.Pin = RS4852_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RS4852_DE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PB4 for BOOT output (first BQ76920, with diode D2) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PC7 for BOOT2 output (second BQ76920, with diode D1) */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PB5 for ALERT input (first BQ76920) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Use pull-up for open-drain alert signal
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /*Configure GPIO pin : PA12 for ALERT2 input (second BQ76920) */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Use pull-up for open-drain alert signal
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Debug LED: Blink rapidly to indicate error
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(BLINK_INTERVAL);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
