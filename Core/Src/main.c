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
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
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

static float coulomb_count = (INITIAL_SOC / 100.0) * NOMINAL_CAPACITY;
static float initial_capacity = NOMINAL_CAPACITY;
static float actual_capacity = NOMINAL_CAPACITY;

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

// Temperature thresholds (example values, adjust as needed)
#define OVERTEMP_THRESHOLD  60  // 60°C
#define UNDERTEMP_THRESHOLD -20 // -20°C

// Power line states (simplified for SON/SOF commands)
static uint8_t power_lines[16] = {0}; // 0 = OFF, 1 = ON (PWRL0 to PWRL15)
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
void ChargeBattery(void);
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

    uint64_t timestamp = HAL_GetTick();
    memset(log_buffer, 0, LOG_ENTRY_SIZE);
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE);
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
    coulomb_count += ((pack_current_1 + pack_current_2) / 2.0 * LOOP_TIME) / 3600.0;
    float soc_measured = (coulomb_count / NOMINAL_CAPACITY) * 100.0;
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
    // Check for faults
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OVERCURRENT | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY)) {
        bms_mode = MODE_FAULT;
        charge_enabled = 0;
        discharge_enabled = 0;
        Log_Error("Entering fault mode");
        return;
    }

    // Check if the battery is too low and needs to charge right away
    charge_immediately = (soc < SOC_LOW_THRESHOLD) ? 1 : 0;

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
        if (soc < SOC_LOW_THRESHOLD) { // Start charging if SOC is low
            bms_mode = MODE_CHARGING;
            charge_enabled = 1;
            discharge_enabled = 0;
        } else { // Go to sleep to save power
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
                } else {
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
void ChargeBattery(void)
{
    // Calculate total pack voltage
    uint32_t pack_voltage = group_voltages_1[0] + group_voltages_1[1] + group_voltages_1[2];
    int16_t total_current = (pack_current_1 + pack_current_2) / 2;
    int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2;

    // Check safety conditions before charging
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OVERCURRENT | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY)) {
        charge_enabled = 0;
        in_cv_mode = 0;
        Log_Error("Charging aborted due to fault");
        return;
    }

    // Check if charging should start
    if (bms_mode != MODE_CHARGING || !charge_enabled) {
        in_cv_mode = 0;
        charge_start_time = 0;
        return;
    }

    // Start charging timer if not already started
    if (charge_start_time == 0) {
        charge_start_time = HAL_GetTick() / 1000;
        Log_Error("Charging started");
    }

    // Check for maximum charge time
    uint32_t charge_duration = (HAL_GetTick() / 1000) - charge_start_time;
    if (charge_duration > MAX_CHARGE_TIME) {
        charge_enabled = 0;
        in_cv_mode = 0;
        charge_start_time = 0;
        Log_Error("Charging stopped: Maximum charge time exceeded");
        return;
    }

    // CC-CV Charging Logic
    if (!in_cv_mode) { // Constant Current (CC) phase
        if (pack_voltage >= CV_VOLTAGE_THRESHOLD) {
            in_cv_mode = 1;
            Log_Error("Transitioning to CV phase");
        } else {
            // In CC phase, ensure charging is enabled
            // Current is controlled by the LT3652HV, but we monitor it
            if (total_current > CC_CURRENT_TARGET + 200 || total_current < CC_CURRENT_TARGET - 200) {
                // Log if current is significantly off target
                char message[MESSAGE_SIZE];
                snprintf(message, sizeof(message), "CC phase current deviation: %d mA", total_current);
                Log_Error(message);
            }
        }
    } else { // Constant Voltage (CV) phase
        if (total_current <= CV_CURRENT_THRESHOLD) {
            // Charging complete
            charge_enabled = 0;
            in_cv_mode = 0;
            charge_start_time = 0;
            Log_Error("Charging complete");
        } else if (pack_voltage < CV_VOLTAGE_THRESHOLD - 100) {
            // Voltage dropped below threshold, revert to CC
            in_cv_mode = 0;
            Log_Error("Reverting to CC phase");
        }
    }

    // Apply charge control
    if (charge_enabled) {
        // Ensure charging is enabled on both BQ76920 ICs
        BQ76920_SetChargeEnable(&hi2c1, 1, 0);
        BQ76920_SetChargeEnable(&hi2c2, 1, 0);
    } else {
        BQ76920_SetChargeEnable(&hi2c1, 0, discharge_enabled);
        BQ76920_SetChargeEnable(&hi2c2, 0, discharge_enabled);
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
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

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
    Log_Error("System started");

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
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    uint32_t last_log_read = 0;
    uint32_t last_status_send = 0;
    uint32_t last_time_sync = 0;

    while (1)
    {
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
        if (Temperature_Read(&hi2c1, &temperature_1) != HAL_OK)
        {
            Log_Error("Error reading temperature (I2C1)");
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
        if (Temperature_Read(&hi2c2, &temperature_2) != HAL_OK)
        {
            Log_Error("Error reading temperature (I2C2)");
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

        // Step 5: Check for overvoltage and undervoltage
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

        // Step 6: Check for overcurrent
        uint8_t occ_flag_1, ocd_flag_1, occ_flag_2, ocd_flag_2;
        BQ76920_CheckOvercurrent(&hi2c1, &occ_flag_1, &ocd_flag_1);
        BQ76920_CheckOvercurrent(&hi2c2, &occ_flag_2, &ocd_flag_2);
        if (occ_flag_1 || occ_flag_2 || ocd_flag_1 || ocd_flag_2)
        {
            error_flags |= ERROR_OVERCURRENT;
            Log_Error("Overcurrent detected");
        }

        // Step 7: Check temperature limits
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

        // Step 8: Update timers
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

        // Step 9: Balance cells
        if (BQ76920_BalanceCells(&hi2c1, group_voltages_1, 0, &balancing_mask_1) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C1)");
        }
        if (BQ76920_BalanceCells(&hi2c2, group_voltages_2, 0, &balancing_mask_2) != HAL_OK)
        {
            Log_Error("Error balancing groups (I2C2)");
        }
        balancing_active = (balancing_mask_1 || balancing_mask_2) ? 1 : 0;

        // Step 10: Control heaters
        PID_Control(lowest_temp);

        // Step 11: Update SOC and SOH
        Update_SOC_SOH();

        // Step 12: Update BMS mode
        Update_BMS_Mode();

        // Step 13: Apply CC-CV charging control
        ChargeBattery();

        // Step 14: Log data
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

        // Step 15: Send logs every 10 seconds
        if (HAL_GetTick() - last_log_read >= 10000)
        {
            Log_Read_All();
            last_log_read = HAL_GetTick();
        }

        // Step 16: Send status every 5 seconds
        if (HAL_GetTick() - last_status_send >= 5000)
        {
            SSP_SendStatus();
            last_status_send = HAL_GetTick();
        }

        // Step 17: Synchronize time with OBC every 60 seconds
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

        // Step 18: Process received SSP frames
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
    /* USER CODE BEGIN I2C1_Init 0 */
    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */
    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00201D2B;
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

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */
    /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
    /* USER CODE BEGIN I2C2_Init 0 */
    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */
    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x00201D2B;
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

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */
    /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
    /* USER CODE BEGIN I2C3_Init 0 */
    /* USER CODE END I2C3_Init 0 */

    /* USER CODE BEGIN I2C3_Init 1 */
    /* USER CODE END I2C3_Init 1 */
    hi2c3.Instance = I2C3;
    hi2c3.Init.Timing = 0x00201D2B;
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

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C3_Init 2 */
    /* USER CODE END I2C3_Init 2 */
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
    /* USER CODE BEGIN RTC_Init 0 */
    /* USER CODE END RTC_Init 0 */

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    /* USER CODE BEGIN RTC_Init 1 */
    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
    */
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

    /* USER CODE BEGIN Check_RTC_BKUP */
    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
    */
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

    /** Enable the TimeStamp
    */
    if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */
    /* USER CODE END RTC_Init 2 */
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */
    /* USER CODE END TIM4_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM4_Init 1 */
    /* USER CODE END TIM4_Init 1 */
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
    HAL_TIM_IC_MspInit(&htim4);
    /* USER CODE END TIM4_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
    /* USER CODE BEGIN USART1_Init 0 */
    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */
    /* USER CODE END USART1_Init 1 */
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
    /* USER CODE BEGIN USART1_Init 2 */
    /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{
    /* USER CODE BEGIN USART2_Init 0 */
    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */
    /* USER CODE END USART2_Init 1 */
    husart2.Instance = USART2;
    husart2.Init.BaudRate = 115200; // Matches ICD requirement
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
    /* USER CODE BEGIN USART2_Init 2 */
    // Manually control the DE pin for RS485
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET); // Receive mode by default
    /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : LED_Pin */
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

    /*Configure GPIO pin : BOOT2_Pin */
    GPIO_InitStruct.Pin = BOOT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BOOT2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : ALERT2_Pin */
    GPIO_InitStruct.Pin = ALERT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ALERT2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : BOOT_Pin ALERT_Pin */
    GPIO_InitStruct.Pin = BOOT_Pin | ALERT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
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
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
