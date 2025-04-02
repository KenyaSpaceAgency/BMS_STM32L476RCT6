/*
 * main.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "main.h"
#include "BQ76920.h"
#include "temperature.h"
#include "pid.h"
#include "kalman_filter.h"
#include "ssp.h"
#include "adc.h" // Added for MX_ADC1_Init
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
USART_HandleTypeDef husart2;

//ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint16_t group_voltages_1[NUM_GROUPS_PER_IC]; // Group voltages from BQ76920 on I2C1 (in mV)
uint16_t group_voltages_2[NUM_GROUPS_PER_IC]; // Group voltages from BQ76920 on I2C2 (in mV, redundant)
int16_t pack_current_1;                      // Pack current from BQ76920 on I2C1 (in mA)
int16_t pack_current_2;                      // Pack current from BQ76920 on I2C2 (in mA, redundant)
int16_t temperature_1;                       // Temperature from Pack 1 (in °C)
int16_t temperature_2;                       // Temperature from Pack 2 (in °C)
int16_t pcb_temperature;                     // PCB temperature (in °C)
float soc = INITIAL_SOC;                     // State of Charge (in %)
float soh = INITIAL_SOH;                     // State of Health (in %)
KalmanFilter soc_kf;                         // Kalman Filter for SOC
KalmanFilter soh_kf;                         // Kalman Filter for SOH
/* Logging variables */
static uint32_t next_slot = 0;               // Tracks the next slot to write to
static uint8_t log_buffer[LOG_ENTRY_SIZE];   // Temporary buffer for log entries
/* Battery capacity tracking */
static float coulomb_count = (INITIAL_SOC / 100.0) * NOMINAL_CAPACITY; // Initial coulomb count in mAh
static float initial_capacity = NOMINAL_CAPACITY; // Initial capacity for SOH calculation
static float actual_capacity = NOMINAL_CAPACITY;  // Actual capacity for SOH calculation
/* SSP communication variables */
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];
/* Operation mode and status flags */
static BMS_ModeTypeDef bms_mode = MODE_DISCHARGING; // Default mode
static uint8_t charge_enabled = 0;    // 0 = disabled, 1 = enabled
static uint8_t discharge_enabled = 1; // 0 = disabled, 1 = enabled
static uint8_t charge_immediately = 0; // 0 = no, 1 = yes
static uint8_t bms_online = 0;        // 0 = offline, 1 = online
static uint32_t error_flags = 0;      // Bitmask for error flags
/* Counters */
static uint32_t charge_cycle_count = 0;
static uint32_t total_charge_time = 0;    // seconds
static uint32_t total_discharge_time = 0; // seconds
static uint32_t total_operating_time = 0; // seconds
static uint8_t charging_started = 0;
/* Balancing status */
static uint8_t balancing_mask_1 = 0;
static uint8_t balancing_mask_2 = 0;
static uint8_t balancing_active = 0;
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
void Log_Error(const char *message);
void Log_Read_All(void);
void Log_Init(void);
void Update_SOC_SOH(void);
void SSP_SendStatus(void);
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame);
void Update_BMS_Mode(void);
int16_t Read_Internal_Temperature(void);
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
  * @param  message: The message to log
  * @retval None
  */
void Log_Error(const char *message)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    uint64_t timestamp = HAL_GetTick(); // Simplified timestamp
    memset(log_buffer, 0, LOG_ENTRY_SIZE);
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE); // Fixed typo: ×tamp -> timestamp
    strncpy((char *)(log_buffer + TIMESTAMP_SIZE), message, MESSAGE_SIZE - 1);

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
    // Coulomb counting
    coulomb_count += ((pack_current_1 + pack_current_2) / 2.0 * LOOP_TIME) / 3600.0; // mAh
    float soc_measured = (coulomb_count / NOMINAL_CAPACITY) * 100.0;
    soc = KalmanFilter_Update(&soc_kf, soc_measured);
    if (soc > 100.0) soc = 100.0;
    if (soc < 0.0) soc = 0.0;

    // Update SOH (simplified: based on capacity fade)
    if (soc >= 100.0) {
        actual_capacity = coulomb_count; // Update actual capacity at full charge
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
    // Check for faults
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OVERCURRENT | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY)) {
        bms_mode = MODE_FAULT;
        charge_enabled = 0;
        discharge_enabled = 0;
        Log_Error("Entering fault mode");
        return;
    }

    // Check SOC for charge immediately status
    charge_immediately = (soc < SOC_LOW_THRESHOLD) ? 1 : 0;

    // Determine mode based on current and SOC
    int16_t total_current = (pack_current_1 + pack_current_2) / 2; // Average for redundancy
    if (total_current < 0) { // Charging (negative current)
        bms_mode = MODE_CHARGING;
        charge_enabled = 1;
        discharge_enabled = 0;
    } else if (total_current > 0) { // Discharging (positive current)
        bms_mode = MODE_DISCHARGING;
        charge_enabled = 0;
        discharge_enabled = 1;
    } else { // Idle
        if (soc < SOC_LOW_THRESHOLD) {
            bms_mode = MODE_CHARGING; // Try to charge if SOC is low
            charge_enabled = 1;
            discharge_enabled = 0;
        } else {
            bms_mode = MODE_SLEEP;
            charge_enabled = 0;
            discharge_enabled = 0;
        }
    }
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

    // Convert raw ADC value to temperature (refer to STM32L476 datasheet)
    int32_t temp = ((raw * 3300 / 4096) - 760) * 100 / 250 + 25; // Simplified formula
    return (int16_t)temp; // °C
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

    // Calculate pack voltage
    uint32_t pack_voltage = group_voltages_1[0] + group_voltages_1[1] + group_voltages_1[2]; // mV

    // Fill telemetry data
    telemetry.mode = bms_mode;
    telemetry.charge_enabled = charge_enabled;
    telemetry.discharge_enabled = discharge_enabled;
    telemetry.charge_immediately = charge_immediately;
    telemetry.bms_online = bms_online;
    telemetry.error_flags = error_flags;
    telemetry.pack_voltage_1 = (uint16_t)pack_voltage;
    telemetry.pack_voltage_2 = (uint16_t)pack_voltage; // Same pack, redundant reading
    telemetry.pack_current_1 = pack_current_1;
    telemetry.pack_current_2 = pack_current_2;
    telemetry.soc = (uint8_t)soc; // Scale to 0-100
    telemetry.soh = (uint8_t)soh; // Scale to 0-100
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

    // Pack and send the frame
    SSP_PackTelemetry(&telemetry, &frame);
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len);
    SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len);
}

/**
  * @brief  Processes a received SSP frame
  * @param  frame: Pointer to the received SSP frame
  * @retval None
  */
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame)
{
    // Check if the frame is for the BMS
    if (frame->dest != SSP_ADDR_BMS) {
        return;
    }

    switch (frame->cmd_id) {
        case SSP_CMD_STATUS:
        case SSP_CMD_TELEMETRY:
            // Respond with telemetry data
            SSP_SendStatus();
            break;

        case SSP_CMD_SET_MODE:
            // Set the BMS mode (e.g., from OBC)
            if (frame->data_len >= 1) {
                bms_mode = frame->data[0];
                Update_BMS_Mode();
                Log_Error("Mode changed by OBC");
            }
            break;

        case SSP_CMD_LOG_DATA:
            // Send all logs
            Log_Read_All();
            break;

        default:
            break;
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
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_I2C3_Init();
    MX_RTC_Init();
    MX_TIM4_Init();
    MX_USART1_UART_Init();
    MX_USART2_Init();
    MX_ADC1_Init(); // Call the generated function

    /* USER CODE BEGIN 2 */
    // Turn off the LED at the start
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    // Initialize PWM for heaters (start with 0% duty cycle)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Start PWM on Channel 3 (HEATER2)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Start PWM on Channel 4 (HEATER1)
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // 0% duty cycle for HEATER2
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // 0% duty cycle for HEATER1

    // Set the initial RTC time to a known UTC value (e.g., 2025-03-28 12:00:00)
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    sTime.Hours = 12;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sDate.Year = 25; // 2025 - 2000
    sDate.Month = 3;
    sDate.Date = 28;
    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Initialize the logging system
    Log_Init();
    Log_Error("System started");

    // Initialize BQ76920 on I2C1
    if (BQ76920_Init(&hi2c1) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C1) initialization failed");
        Error_Handler();
    }

    // Initialize BQ76920 on I2C2 (redundant)
    if (BQ76920_Init(&hi2c2) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C2) initialization failed");
        Error_Handler();
    }

    // Initialize Kalman Filters
    KalmanFilter_Init(&soc_kf, INITIAL_SOC, 1.0, 0.01, 1.0); // Q=0.01, R=1.0
    KalmanFilter_Init(&soh_kf, INITIAL_SOH, 1.0, 0.01, 1.0);

    // Initialize PID for heaters
    PID_Init();

    // Set BMS online status
    bms_online = 1;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t last_log_read = 0; // Track the last time we read logs
    uint32_t last_status_send = 0; // Track the last time we sent a status update
    while (1)
    {
        // Step 1: Read data from BQ76920 on I2C1 (3 parallel groups)
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
        if (Temperature_Read(&hi2c1, &temperature_1) != HAL_OK)
        {
            Log_Error("Error reading temperature (I2C1)");
        }

        // Step 2: Read data from BQ76920 on I2C2 (redundant readings of the same pack)
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
        if (Temperature_Read(&hi2c2, &temperature_2) != HAL_OK)
        {
            Log_Error("Error reading temperature (I2C2)");
        }

        // Step 3: Check redundancy between the two BQ76920 ICs
        uint8_t discrepancy_flag = 0;
        BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag);
        if (discrepancy_flag)
        {
            error_flags |= ERROR_DISCREPANCY;
            Log_Error("Redundancy discrepancy detected");
        }

        // Step 4: Read PCB temperature
        pcb_temperature = Read_Internal_Temperature();

        // Step 5: Check for overvoltage/undervoltage protection
        uint8_t ov_flag_1, uv_flag_1, ov_flag_2, uv_flag_2;
        BQ76920_CheckProtection(&hi2c1, group_voltages_1, 0, &ov_flag_1, &uv_flag_1);
        BQ76920_CheckProtection(&hi2c2, group_voltages_2, 0, &ov_flag_2, &uv_flag_2);
        if (ov_flag_1 || ov_flag_2)
        {
            error_flags |= ERROR_OVERVOLTAGE;
            Log_Error("Overvoltage detected");
        }
        if (uv_flag_1 || uv_flag_2)
        {
            error_flags |= ERROR_UNDERVOLTAGE;
            Log_Error("Undervoltage detected");
        }

        // Step 6: Check for overcurrent protection
        uint8_t occ_flag_1, ocd_flag_1, occ_flag_2, ocd_flag_2;
        BQ76920_CheckOvercurrent(&hi2c1, &occ_flag_1, &ocd_flag_1);
        BQ76920_CheckOvercurrent(&hi2c2, &occ_flag_2, &ocd_flag_2);
        if (occ_flag_1 || occ_flag_2 || ocd_flag_1 || ocd_flag_2)
        {
            error_flags |= ERROR_OVERCURRENT;
            Log_Error("Overcurrent detected");
        }

        // Step 7: Check for temperature protection
        int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2;
        int16_t lowest_temp = (temperature_1 < temperature_2) ? temperature_1 : temperature_2;
        if (highest_temp > OVERTEMP_THRESHOLD)
        {
            error_flags |= ERROR_OVERTEMP;
            Log_Error("Overtemperature detected");
        }
        if (lowest_temp < UNDERTEMP_THRESHOLD)
        {
            error_flags |= ERROR_UNDERTEMP;
            Log_Error("Undertemperature detected");
        }

        // Step 8: Update counters
        total_operating_time = HAL_GetTick() / 1000; // seconds
        int16_t total_current = (pack_current_1 + pack_current_2) / 2; // Average for redundancy
        if (total_current < 0) { // Charging
            total_charge_time += (uint32_t)LOOP_TIME;
            if (soc < 20.0 && !charging_started) {
                charging_started = 1;
            }
            if (soc >= 100.0 && charging_started) {
                charge_cycle_count++;
                charging_started = 0;
            }
        } else if (total_current > 0) { // Discharging
            total_discharge_time += (uint32_t)LOOP_TIME;
        }

        // Step 9: Balance groups (between parallel groups)
        if (BQ76920_BalanceCells(&hi2c1, group_voltages_1, 0, &balancing_mask_1) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C1)");
        }
        if (BQ76920_BalanceCells(&hi2c2, group_voltages_2, 0, &balancing_mask_2) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C2)");
        }
        balancing_active = (balancing_mask_1 || balancing_mask_2) ? 1 : 0;

        // Step 10: Control the heaters using PID and PWM
        PID_Control(lowest_temp);

        // Step 11: Update SOC and SOH
        Update_SOC_SOH();

        // Step 12: Update BMS mode and status
        Update_BMS_Mode();

        // Step 13: Log the group voltages, current, temperature, SOC, and SOH to flash
        char message[MESSAGE_SIZE];
        snprintf(message, sizeof(message), "Time: %lu | ", HAL_GetTick());
        for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
        {
            char group_data[20]; // Increased from 16 to 20 to accommodate worst-case (17 bytes + margin)
            snprintf(group_data, sizeof(group_data), "Group%d: %dmV ", i + 1, group_voltages_1[i]);
            strncat(message, group_data, MESSAGE_SIZE - strlen(message) - 1);
        }
        char temp_data[88]; // Increased from 32 to 88 to accommodate worst-case (88 bytes)
        snprintf(temp_data, sizeof(temp_data), "I1: %dmA I2: %dmA T1: %dC T2: %dC PCB: %dC SOC: %.1f%% SOH: %.1f%%",
                 pack_current_1, pack_current_2, temperature_1, temperature_2, pcb_temperature, soc, soh);
        strncat(message, temp_data, MESSAGE_SIZE - strlen(message) - 1);
        Log_Error(message);

        // Step 14: Periodically read and send all logs over RS485 (every 10 seconds)
        if (HAL_GetTick() - last_log_read >= 10000)
        {
            Log_Read_All();
            last_log_read = HAL_GetTick();
        }

        // Step 15: Periodically send status to OBC using SSP (every 5 seconds)
        if (HAL_GetTick() - last_status_send >= 5000)
        {
            SSP_SendStatus();
            last_status_send = HAL_GetTick();
        }

        // Step 16: Check for incoming SSP frames
        SSP_FrameTypeDef received_frame = {0};
        if (SSP_ReceiveFrame(&husart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK)
        {
            SSP_ProcessReceivedFrame(&received_frame);
        }

        HAL_Delay((uint32_t)(LOOP_TIME * 1000)); // Delay for the loop time
        /* USER CODE END WHILE */
    }
    /* USER CODE END 3 */
}
/* Peripheral initialization functions (omitted for brevity) */
void SystemClock_Config(void) { /* ... */ }
static void MX_GPIO_Init(void) { /* ... */ }
static void MX_I2C1_Init(void) { /* ... */ }
static void MX_I2C2_Init(void) { /* ... */ }
static void MX_I2C3_Init(void) { /* ... */ }
static void MX_RTC_Init(void) { /* ... */ }
static void MX_TIM4_Init(void) { /* ... */ }
static void MX_USART1_UART_Init(void) { /* ... */ }
static void MX_USART2_Init(void) { /* ... */ }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
