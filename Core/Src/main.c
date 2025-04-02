//* USER CODE BEGIN Header */
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
#include "main.h" // This brings in the main settings for our chip, like which pins to use for things like the heaters.
#include "BQ76920.h" // This helps us talk to the battery chip that measures voltage and current.
#include "temperature.h" // This helps us read the temperature sensors on the battery.
#include "pid.h" // This helps us control the heaters to keep the battery at the right temperature.
#include "kalman_filter.h" // This helps us make smart guesses about the battery's charge and health.
#include "ssp.h" // This helps us talk to the satellite's main computer.
#include "adc.h" // This helps us read the temperature sensor inside our chip.
#include <stdio.h> // This helps us write messages, like for the log.
#include <string.h> // This helps us work with text, like copying messages.

/* Private variables ---------------------------------------------------------*/
// These are the tools we use to talk to the chip's hardware.
ADC_HandleTypeDef hadc1; // Tool to read the chip's temperature sensor.
I2C_HandleTypeDef hi2c1; // Tool to talk to the first battery chip.
I2C_HandleTypeDef hi2c2; // Tool to talk to the second battery chip (a backup).
I2C_HandleTypeDef hi2c3; // Tool to talk to the temperature sensors.
RTC_HandleTypeDef hrtc; // Tool to keep track of time, like a clock.
TIM_HandleTypeDef htim4; // Tool to control the heaters.
UART_HandleTypeDef huart1; // Tool to send logs to the main computer.
USART_HandleTypeDef husart2; // Tool to talk to the main computer about the battery status.

/* USER CODE BEGIN PV */
// These are like notebooks where we write down important information about the battery.
uint16_t group_voltages_1[NUM_GROUPS_PER_IC]; // Voltages of the battery groups from the first chip (in millivolts, mV).
uint16_t group_voltages_2[NUM_GROUPS_PER_IC]; // Voltages of the battery groups from the second chip (a backup, also in mV).
int16_t pack_current_1; // Current flowing through the battery, measured by the first chip (in milliamps, mA).
int16_t pack_current_2; // Current measured by the second chip (a backup, in mA).
int16_t temperature_1; // Temperature of one part of the battery (in degrees Celsius, °C).
int16_t temperature_2; // Temperature of another part of the battery (in °C).
int16_t pcb_temperature; // Temperature of the chip itself (in °C).
float soc = INITIAL_SOC; // How much charge the battery has, like a fuel gauge (in %, starts at 50%).
float soh = INITIAL_SOH; // How healthy the battery is (in %, starts at 100%).
KalmanFilter soc_kf; // A smart tool to guess the battery's charge more accurately.
KalmanFilter soh_kf; // A smart tool to guess the battery's health more accurately.

// These are for keeping a diary (log) of what happens.
static uint32_t next_slot = 0; // Keeps track of where to write the next log entry, like a page number in a diary.
static uint8_t log_buffer[LOG_ENTRY_SIZE]; // A temporary space to write a log message before saving it.

// These track how much charge the battery has used.
static float coulomb_count = (INITIAL_SOC / 100.0) * NOMINAL_CAPACITY; // How much charge has gone in or out (in mAh, like counting water in a bucket).
static float initial_capacity = NOMINAL_CAPACITY; // The battery's full capacity when it was new (in mAh).
static float actual_capacity = NOMINAL_CAPACITY; // The battery's current capacity (to see if it's getting old).

// These are for talking to the satellite's main computer.
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN]; // A space to write messages to send to the main computer.
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN]; // A space to store messages we get from the main computer.

// These keep track of what the BMS is doing and if there are any problems.
static BMS_ModeTypeDef bms_mode = MODE_DISCHARGING; // What the BMS is doing (e.g., charging, discharging, or sleeping).
static uint8_t charge_enabled = 0; // Can the battery charge? (0 = no, 1 = yes).
static uint8_t discharge_enabled = 1; // Can the battery discharge? (0 = no, 1 = yes).
static uint8_t charge_immediately = 0; // Does the battery need to charge right away? (0 = no, 1 = yes).
static uint8_t bms_online = 0; // Is the BMS working and talking to the main computer? (0 = no, 1 = yes).
static uint32_t error_flags = 0; // Keeps track of problems, like if the battery is too hot or the voltage is too high.

// These count things over time.
static uint32_t charge_cycle_count = 0; // How many times the battery has been fully charged.
static uint32_t total_charge_time = 0; // How long the battery has been charging (in seconds).
static uint32_t total_discharge_time = 0; // How long the battery has been discharging (in seconds).
static uint32_t total_operating_time = 0; // How long the BMS has been running (in seconds).
static uint8_t charging_started = 0; // Remembers if the battery started charging when it was low.

// These keep track of balancing the battery (making sure all parts have the same charge).
static uint8_t balancing_mask_1 = 0; // Which parts of the battery need balancing (first chip).
static uint8_t balancing_mask_2 = 0; // Which parts need balancing (second chip).
static uint8_t balancing_active = 0; // Is balancing happening right now? (0 = no, 1 = yes).
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// This is like a to-do list for the program, listing all the jobs it will do.
void SystemClock_Config(void); // Sets up the chip's clock, like setting the time on a watch.
static void MX_GPIO_Init(void); // Sets up the pins for the LED.
static void MX_I2C1_Init(void); // Sets up the communication line for the first battery chip.
static void MX_I2C2_Init(void); // Sets up the communication line for the second battery chip.
static void MX_I2C3_Init(void); // Sets up the communication line for the temperature sensors.
static void MX_RTC_Init(void); // Sets up the clock for keeping time.
static void MX_TIM4_Init(void); // Sets up the heaters.
static void MX_USART1_UART_Init(void); // Sets up the communication line to send logs.
static void MX_USART2_Init(void); // Sets up the communication line to talk to the main computer.
static void MX_ADC1_Init(void); // Sets up the temperature sensor inside the chip.
/* USER CODE BEGIN PFP */
void Flash_Erase(uint32_t page); // Clears a page in memory to make space for new logs.
void Log_Error(const char *message); // Writes a message to the log with a time stamp.
void Log_Read_All(void); // Reads all the logs and sends them to the main computer.
void Log_Init(void); // Gets the logging system ready.
void Update_SOC_SOH(void); // Updates the battery's charge and health.
void SSP_SendStatus(void); // Sends the BMS status to the main computer.
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame); // Listens for messages from the main computer.
void Update_BMS_Mode(void); // Decides what the BMS should be doing (e.g., charging or discharging).
int16_t Read_Internal_Temperature(void); // Reads the chip's temperature.
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
    FLASH_EraseInitTypeDef erase_init; // A tool to set up how we erase the memory.
    uint32_t page_error; // A place to store any errors that happen.

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES; // We want to erase a whole page.
    erase_init.Page = page; // The page number we want to erase.
    erase_init.NbPages = 1; // We only want to erase one page.

    HAL_FLASH_Unlock(); // Unlock the memory so we can erase it.
    HAL_FLASHEx_Erase(&erase_init, &page_error); // Erase the page.
    HAL_FLASH_Lock(); // Lock the memory again to keep it safe.
}

/**
  * @brief  Logs an error message to flash memory with a timestamp
  * @param  message: The message to log
  * @retval None
  */
void Log_Error(const char *message)
{
    // Get the current time and date from the clock.
    RTC_TimeTypeDef sTime = {0}; // A place to store the time (hours, minutes, seconds).
    RTC_DateTypeDef sDate = {0}; // A place to store the date (year, month, day).
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // Get the time.
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // Get the date.

    // For now, we're using a simple time stamp (how long the program has been running).
    uint64_t timestamp = HAL_GetTick(); // Get the time in milliseconds since the program started.
    memset(log_buffer, 0, LOG_ENTRY_SIZE); // Clear the temporary space for the log message.
    memcpy(log_buffer, &timestamp, TIMESTAMP_SIZE); // Write the time stamp to the temporary space.
    strncpy((char *)(log_buffer + TIMESTAMP_SIZE), message, MESSAGE_SIZE - 1); // Write the message after the time stamp.

    // Find the right spot in memory to save the log.
    uint32_t address = LOG_START_ADDR + (next_slot * LOG_ENTRY_SIZE);
    HAL_FLASH_Unlock(); // Unlock the memory so we can write to it.
    // Write the log message to memory, 8 bytes at a time.
    for (uint8_t i = 0; i < LOG_ENTRY_SIZE; i += 8) {
        uint64_t data = *(uint64_t *)(log_buffer + i);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, data);
    }
    HAL_FLASH_Lock(); // Lock the memory again.

    // Update the next spot to write a log.
    next_slot = (next_slot + 1) % NUM_LOG_ENTRIES;
    HAL_FLASH_Unlock(); // Unlock the memory again.
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot); // Save the new spot.
    HAL_FLASH_Lock(); // Lock the memory.
}

/**
  * @brief  Reads and sends all logs over RS485
  * @retval None
  */
void Log_Read_All(void)
{
    char buffer[128]; // A temporary space to write the log message to send.
    // Loop through all the log entries in memory.
    for (uint32_t i = 0; i < NUM_LOG_ENTRIES; i++) {
        uint32_t address = LOG_START_ADDR + (i * LOG_ENTRY_SIZE); // Find the spot in memory for this log.
        uint64_t timestamp = *(uint64_t *)address; // Read the time stamp.
        char *message = (char *)(address + TIMESTAMP_SIZE); // Read the message.
        // Write a new message like "Log 1: Time=1234, Msg=System started".
        snprintf(buffer, sizeof(buffer), "Log %lu: Time=%llu, Msg=%s\r\n", i, timestamp, message);
        // Send the message to the main computer.
        HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}

/**
  * @brief  Initializes the logging system
  * @retval None
  */
void Log_Init(void)
{
    next_slot = *(uint32_t *)NEXT_SLOT_ADDR; // Check where the last log was written.
    // If we've run out of space for logs, erase the memory and start over.
    if (next_slot >= NUM_LOG_ENTRIES) {
        Flash_Erase(FLASH_LOG_PAGE); // Clear the memory.
        next_slot = 0; // Start at the beginning again.
        HAL_FLASH_Unlock(); // Unlock the memory.
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NEXT_SLOT_ADDR, next_slot); // Save the new starting spot.
        HAL_FLASH_Lock(); // Lock the memory.
    }
}

/**
  * @brief  Updates SOC and SOH using coulomb counting and Kalman Filter
  * @retval None
  */
void Update_SOC_SOH(void)
{
    // Count how much charge has gone in or out of the battery, like counting water in a bucket.
    coulomb_count += ((pack_current_1 + pack_current_2) / 2.0 * LOOP_TIME) / 3600.0; // mAh
    float soc_measured = (coulomb_count / NOMINAL_CAPACITY) * 100.0; // Calculate the charge level as a percentage.
    soc = KalmanFilter_Update(&soc_kf, soc_measured); // Use a smart tool to make a better guess about the charge.
    if (soc > 100.0) soc = 100.0; // Make sure the charge doesn't go above 100%.
    if (soc < 0.0) soc = 0.0; // Make sure the charge doesn't go below 0%.

    // If the battery is fully charged, check its health.
    if (soc >= 100.0) {
        actual_capacity = coulomb_count; // See how much charge the battery can hold now.
        soh = (actual_capacity / initial_capacity) * 100.0; // Compare it to when the battery was new to get the health.
        soh = KalmanFilter_Update(&soh_kf, soh); // Use a smart tool to make a better guess about the health.
        if (soh > 100.0) soh = 100.0; // Make sure the health doesn't go above 100%.
        if (soh < 0.0) soh = 0.0; // Make sure the health doesn't go below 0%.
    }
}

/**
  * @brief  Updates the BMS operation mode and charge/discharge status
  * @retval None
  */
void Update_BMS_Mode(void)
{
    // Check if there are any problems, like the battery being too hot or the voltage too high.
    if (error_flags & (ERROR_OVERVOLTAGE | ERROR_UNDERVOLTAGE | ERROR_OVERCURRENT | ERROR_OVERTEMP | ERROR_UNDERTEMP | ERROR_DISCREPANCY)) {
        bms_mode = MODE_FAULT; // Go into fault mode if there's a problem.
        charge_enabled = 0; // Stop charging.
        discharge_enabled = 0; // Stop discharging.
        Log_Error("Entering fault mode"); // Write a log message.
        return; // Stop here.
    }

    // Check if the battery is too low and needs to charge right away.
    charge_immediately = (soc < SOC_LOW_THRESHOLD) ? 1 : 0;

    // Look at the current to decide what to do.
    int16_t total_current = (pack_current_1 + pack_current_2) / 2; // Average the current from both chips.
    if (total_current < 0) { // If the current is negative, the battery is charging.
        bms_mode = MODE_CHARGING; // Set the mode to charging.
        charge_enabled = 1; // Allow charging.
        discharge_enabled = 0; // Stop discharging.
    } else if (total_current > 0) { // If the current is positive, the battery is discharging.
        bms_mode = MODE_DISCHARGING; // Set the mode to discharging.
        charge_enabled = 0; // Stop charging.
        discharge_enabled = 1; // Allow discharging.
    } else { // If there's no current, the battery is idle.
        if (soc < SOC_LOW_THRESHOLD) { // If the battery is too low, start charging.
            bms_mode = MODE_CHARGING;
            charge_enabled = 1;
            discharge_enabled = 0;
        } else { // Otherwise, go to sleep to save power.
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
    HAL_ADC_Start(&hadc1); // Start the temperature sensor.
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for the sensor to finish measuring.
    uint32_t raw = HAL_ADC_GetValue(&hadc1); // Get the measurement (a number).
    HAL_ADC_Stop(&hadc1); // Stop the sensor.

    // Turn the number into a temperature in degrees Celsius.
    int32_t temp = ((raw * 3300 / 4096) - 760) * 100 / 250 + 25; // A math formula to get the temperature.
    return (int16_t)temp; // Return the temperature (e.g., 25°C).
}

/**
  * @brief  Sends the current BMS status to the OBC using SSP
  * @retval None
  */
void SSP_SendStatus(void)
{
    SSP_TelemetryTypeDef telemetry = {0}; // A place to store the status message.
    SSP_FrameTypeDef frame = {0}; // A place to build the message to send.
    uint16_t frame_len; // How long the message will be.

    // Add up the voltages of the battery groups to get the total voltage.
    uint32_t pack_voltage = group_voltages_1[0] + group_voltages_1[1] + group_voltages_1[2]; // mV

    // Put all the important information into the status message.
    telemetry.mode = bms_mode; // What the BMS is doing (e.g., charging).
    telemetry.charge_enabled = charge_enabled; // Can the battery charge?
    telemetry.discharge_enabled = discharge_enabled; // Can the battery discharge?
    telemetry.charge_immediately = charge_immediately; // Does the battery need to charge right away?
    telemetry.bms_online = bms_online; // Is the BMS working?
    telemetry.error_flags = error_flags; // Any problems?
    telemetry.pack_voltage_1 = (uint16_t)pack_voltage; // Total voltage (first chip).
    telemetry.pack_voltage_2 = (uint16_t)pack_voltage; // Total voltage (second chip, same because it's the same battery).
    telemetry.pack_current_1 = pack_current_1; // Current (first chip).
    telemetry.pack_current_2 = pack_current_2; // Current (second chip).
    telemetry.soc = (uint8_t)soc; // Battery charge level (0-100%).
    telemetry.soh = (uint8_t)soh; // Battery health (0-100%).
    telemetry.temp_1 = temperature_1; // Temperature of one part of the battery.
    telemetry.temp_2 = temperature_2; // Temperature of another part.
    telemetry.pcb_temp = pcb_temperature; // Temperature of the chip.
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { // Add the voltages of each group.
        telemetry.group_voltages[i] = group_voltages_1[i];
    }
    telemetry.balancing_active = balancing_active; // Is balancing happening?
    telemetry.balancing_mask_1 = balancing_mask_1; // Which parts are being balanced (first chip).
    telemetry.balancing_mask_2 = balancing_mask_2; // Which parts are being balanced (second chip).
    telemetry.charge_cycle_count = charge_cycle_count; // How many times the battery has been charged.
    telemetry.total_charge_time = total_charge_time; // How long the battery has been charging.
    telemetry.total_discharge_time = total_discharge_time; // How long the battery has been discharging.
    telemetry.total_operating_time = total_operating_time; // How long the BMS has been running.

    // Pack the message and send it to the main computer.
    SSP_PackTelemetry(&telemetry, &frame); // Put the status into a message.
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len); // Build the message to send.
    SSP_TransmitFrame(&husart2, ssp_tx_buffer, frame_len); // Send the message.
}

/**
  * @brief  Processes a received SSP frame
  * @param  frame: Pointer to the received SSP frame
  * @retval None
  */
void SSP_ProcessReceivedFrame(SSP_FrameTypeDef *frame)
{
    // Check if the message is for the BMS.
    if (frame->dest != SSP_ADDR_BMS) {
        return; // If not, ignore it.
    }

    // Look at the message and decide what to do.
    switch (frame->cmd_id) {
        case SSP_CMD_STATUS: // If the message says "send status"...
        case SSP_CMD_TELEMETRY: // Or "send telemetry"...
            SSP_SendStatus(); // Send the status.
            break;

        case SSP_CMD_SET_MODE: // If the message says "change mode"...
            if (frame->data_len >= 1) { // Make sure the message has the new mode.
                bms_mode = frame->data[0]; // Set the new mode.
                Update_BMS_Mode(); // Update what the BMS is doing.
                Log_Error("Mode changed by OBC"); // Write a log message.
            }
            break;

        case SSP_CMD_LOG_DATA: // If the message says "send logs"...
            Log_Read_All(); // Send all the logs.
            break;

        default: // If the message is something else, ignore it for now.
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // Get the chip ready to work, like turning on a computer.

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config(); // Set up the chip's clock, like setting the time on a watch.

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init(); // Set up the pins for the LED.
  MX_I2C1_Init(); // Set up the communication line for the first battery chip.
  MX_I2C2_Init(); // Set up the communication line for the second battery chip.
  MX_I2C3_Init(); // Set up the communication line for the temperature sensors.
  MX_RTC_Init(); // Set up the clock for keeping time.
  MX_TIM4_Init(); // Set up the heaters.
  MX_USART1_UART_Init(); // Set up the communication line to send logs.
  MX_USART2_Init(); // Set up the communication line to talk to the main computer.
  MX_ADC1_Init(); // Set up the temperature sensor inside the chip.
  /* USER CODE BEGIN 2 */
    // Turn off the LED at the start, like turning off a light.
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    // Get the heaters ready, but start with them off (0% power).
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Start the heater for part 2 of the battery.
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Start the heater for part 1 of the battery.
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); // Set heater 2 to 0% power.
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // Set heater 1 to 0% power.

    // Set the clock to a starting time (March 28, 2025, 12:00:00).
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    sTime.Hours = 12;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sDate.Year = 25; // 2025 - 2000
    sDate.Month = 3;
    sDate.Date = 28;
    sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // Set the time.
    HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN); // Set the date.

    // Get the logging system ready.
    Log_Init();
    Log_Error("System started"); // Write a log message to say we started.

    // Get the first battery chip ready.
    if (BQ76920_Init(&hi2c1) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C1) initialization failed"); // Write a log if it fails.
        Error_Handler(); // Stop if there's a problem.
    }

    // Get the second battery chip ready (a backup).
    if (BQ76920_Init(&hi2c2) != HAL_OK)
    {
        Log_Error("BQ76920 (I2C2) initialization failed"); // Write a log if it fails.
        Error_Handler(); // Stop if there's a problem.
    }

    // Get the smart tools ready for guessing the battery's charge and health.
    KalmanFilter_Init(&soc_kf, INITIAL_SOC, 1.0, 0.01, 1.0); // Set up the tool for charge.
    KalmanFilter_Init(&soh_kf, INITIAL_SOH, 1.0, 0.01, 1.0); // Set up the tool for health.

    // Get the heater control ready.
    PID_Init();

    // Say that the BMS is working and ready to talk to the main computer.
    bms_online = 1;
  /* USER CODE END 2 */
    /* USER CODE BEGIN WHILE */
	// This is the main part of the program that keeps running over and over, like a daily routine.
	uint32_t last_log_read = 0; // This remembers the last time we sent our diary (logs) to the main computer.
	uint32_t last_status_send = 0; // This remembers the last time we told the main computer how the battery is doing.
	while (1) // This means "keep doing this forever," like a daily checklist that never stops.
	{
		// Step 1: Read data from the first battery chip (BQ76920 on I2C1) to check the battery's health.
		// We're checking the voltages of 3 groups of battery cells
		if (BQ76920_ReadVoltages(&hi2c1, group_voltages_1, 0) != HAL_OK)
		{
			// If we can't read the voltages, something's wrong, so we write a note in our diary.
			Log_Error("Error reading BQ76920 (I2C1) group voltages");
			// Since we couldn't get the real voltages, we set them to 0 to be safe.
			for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
			{
				group_voltages_1[i] = 0; // Set each group's voltage to 0.
			}
		}
		// Now we check how much electricity (current) is flowing through the battery, like checking how fast water is flowing.
		if (BQ76920_ReadCurrent(&hi2c1, &pack_current_1) != HAL_OK)
		{
			// If we can't read the current, we write a note in our diary.
			Log_Error("Error reading BQ76920 (I2C1) current");
		}
		// We also check the temperature of one part of the battery, like feeling how warm a room is.
		if (Temperature_Read(&hi2c1, &temperature_1) != HAL_OK)
		{
			// If we can't read the temperature, we write a note in our diary.
			Log_Error("Error reading temperature (I2C1)");
		}

		// Step 2: Read the same things from the second battery chip (BQ76920 on I2C2) as a backup.
		// This is like having a second person double-check the same things to make sure we didn't miss anything.
		if (BQ76920_ReadVoltages(&hi2c2, group_voltages_2, 0) != HAL_OK)
		{
			// If we can't read the voltages, we write a note in our diary.
			Log_Error("Error reading BQ76920 (I2C2) group voltages");
			// Set the voltages to 0 if we can't read them.
			for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
			{
				group_voltages_2[i] = 0;
			}
		}
		// Check the current again with the second chip.
		if (BQ76920_ReadCurrent(&hi2c2, &pack_current_2) != HAL_OK)
		{
			// If we can't read the current, we write a note.
			Log_Error("Error reading BQ76920 (I2C2) current");
		}
		// Check the temperature of another part of the battery with the second chip.
		if (Temperature_Read(&hi2c2, &temperature_2) != HAL_OK)
		{
			// If we can't read the temperature, we write a note.
			Log_Error("Error reading temperature (I2C2)");
		}

		// Step 3: Compare the two chips to make sure they agree.
		// We have two chips checking the same battery, so they should give us the same answers.
		uint8_t discrepancy_flag = 0; // This is like a warning flag to say if something's wrong.
		// Check if the voltages and currents from both chips match (within a small difference).
		BQ76920_CheckRedundancy(group_voltages_1, group_voltages_2, pack_current_1, pack_current_2, &discrepancy_flag);
		if (discrepancy_flag) // If the two chips don't agree, there's a problem.
		{
			error_flags |= ERROR_DISCREPANCY; // Mark that we found a problem.
			Log_Error("Redundancy discrepancy detected"); // Write a note in our diary.
		}

		// Step 4: Check the temperature of the chip itself.
		// This is like feeling the temperature of the computer to make sure it's not too hot.
		pcb_temperature = Read_Internal_Temperature(); // Read the chip's temperature and save it.

		// Step 5: Check if the battery voltage is too high or too low.
		// We don't want the battery to be overfilled or too empty, like a water tank.
		uint8_t ov_flag_1, uv_flag_1, ov_flag_2, uv_flag_2; // These are warning flags for too high or too low voltage.
		// Check the first chip.
		BQ76920_CheckProtection(&hi2c1, group_voltages_1, 0, &ov_flag_1, &uv_flag_1);
		// Check the second chip.
		BQ76920_CheckProtection(&hi2c2, group_voltages_2, 0, &ov_flag_2, &uv_flag_2);
		// If either chip says the voltage is too high, we have a problem.
		if (ov_flag_1 || ov_flag_2)
		{
			error_flags |= ERROR_OVERVOLTAGE; // Mark that the voltage is too high.
			Log_Error("Overvoltage detected"); // Write a note in our diary.
		}
		// If either chip says the voltage is too low, we have a problem.
		if (uv_flag_1 || uv_flag_2)
		{
			error_flags |= ERROR_UNDERVOLTAGE; // Mark that the voltage is too low.
			Log_Error("Undervoltage detected"); // Write a note in our diary.
		}

		// Step 6: Check if too much electricity is flowing through the battery.
		// This is like checking if too much water is flowing through a pipe—it could cause damage.
		uint8_t occ_flag_1, ocd_flag_1, occ_flag_2, ocd_flag_2; // Warning flags for too much current.
		// Check the first chip.
		BQ76920_CheckOvercurrent(&hi2c1, &occ_flag_1, &ocd_flag_1);
		// Check the second chip.
		BQ76920_CheckOvercurrent(&hi2c2, &occ_flag_2, &ocd_flag_2);
		// If either chip says there's too much current, we have a problem.
		if (occ_flag_1 || occ_flag_2 || ocd_flag_1 || ocd_flag_2)
		{
			error_flags |= ERROR_OVERCURRENT; // Mark that there's too much current.
			Log_Error("Overcurrent detected"); // Write a note in our diary.
		}

		// Step 7: Check if the battery is too hot or too cold.
		// We want the battery to be at a nice temperature, not too hot or too cold, like keeping a room comfy.
		int16_t highest_temp = (temperature_1 > temperature_2) ? temperature_1 : temperature_2; // Find the hottest temperature.
		int16_t lowest_temp = (temperature_1 < temperature_2) ? temperature_1 : temperature_2; // Find the coldest temperature.
		// If the hottest temperature is too high, we have a problem.
		if (highest_temp > OVERTEMP_THRESHOLD)
		{
			error_flags |= ERROR_OVERTEMP; // Mark that it's too hot.
			Log_Error("Overtemperature detected"); // Write a note in our diary.
		}
		// If the coldest temperature is too low, we have a problem.
		if (lowest_temp < UNDERTEMP_THRESHOLD)
		{
			error_flags |= ERROR_UNDERTEMP; // Mark that it's too cold.
			Log_Error("Undertemperature detected"); // Write a note in our diary.
		}

		// Step 8: Keep track of how long things have been happening.
		// This is like keeping a timer for how long we've been doing different jobs.
		total_operating_time = HAL_GetTick() / 1000; // How long the BMS has been running (in seconds).
		int16_t total_current = (pack_current_1 + pack_current_2) / 2; // Average the current from both chips.
		if (total_current < 0) { // If the current is negative, the battery is charging.
			total_charge_time += (uint32_t)LOOP_TIME; // Add to the time we've been charging.
			// If the battery is very low and we haven't started charging yet, mark that we started.
			if (soc < 20.0 && !charging_started) {
				charging_started = 1;
			}
			// If the battery is fully charged and we started charging, count it as a full charge cycle.
			if (soc >= 100.0 && charging_started) {
				charge_cycle_count++; // Add 1 to the number of full charges.
				charging_started = 0; // Reset the "started charging" marker.
			}
		} else if (total_current > 0) { // If the current is positive, the battery is discharging.
			total_discharge_time += (uint32_t)LOOP_TIME; // Add to the time we've been discharging.
		}

		// Step 9: Balance the battery groups so they all have the same charge.
		// This is like making sure all the buckets of water are filled to the same level.
		// Check the first chip.
		if (BQ76920_BalanceCells(&hi2c1, group_voltages_1, 0, &balancing_mask_1) != HAL_OK)
		{
			Log_Error("Error balancing groups (I2C1)"); // Write a note if we can't balance.
		}
		// Check the second chip.
		if (BQ76920_BalanceCells(&hi2c2, group_voltages_2, 0, &balancing_mask_2) != HAL_OK)
		{
			Log_Error("Error balancing groups (I2C2)"); // Write a note if we can't balance.
		}
		// If either chip is balancing, mark that balancing is happening.
		balancing_active = (balancing_mask_1 || balancing_mask_2) ? 1 : 0;

		// Step 10: Control the heaters to keep the battery at the right temperature.
		// This is like turning on a heater in a cold room to keep it warm.
		PID_Control(lowest_temp); // Use a smart tool to decide how much to turn on the heaters.

		// Step 11: Update the battery's charge and health.
		// This checks how full the battery is and how healthy it is.
		Update_SOC_SOH();

		// Step 12: Decide what the BMS should be doing (e.g., charging, discharging, or sleeping).
		Update_BMS_Mode();

		// Step 13: Write a log entry with all the important information.
		// This is like writing in our diary to keep a record of what’s happening.
		char message[MESSAGE_SIZE]; // A space to write our log message.
		// Start the message with the current time (how long we've been running).
		snprintf(message, sizeof(message), "Time: %lu | ", HAL_GetTick());
		// Add the voltages of each group, like writing down the water level in each bucket.
		for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++)
		{
			char group_data[20]; // A space to write the voltage for one group.
			snprintf(group_data, sizeof(group_data), "Group%d: %dmV ", i + 1, group_voltages_1[i]);
			strncat(message, group_data, MESSAGE_SIZE - strlen(message) - 1); // Add it to the message.
		}
		// Add the current, temperatures, charge, and health to the message.
		char temp_data[88]; // A space to write the rest of the information.
		snprintf(temp_data, sizeof(temp_data), "I1: %dmA I2: %dmA T1: %dC T2: %dC PCB: %dC SOC: %.1f%% SOH: %.1f%%",
				 pack_current_1, pack_current_2, temperature_1, temperature_2, pcb_temperature, soc, soh);
		strncat(message, temp_data, MESSAGE_SIZE - strlen(message) - 1); // Add it to the message.
		Log_Error(message); // Save the message in our diary.

		// Step 14: Every 10 seconds, send all our logs to the main computer.
		// This is like mailing our diary to someone every few days so they can read it.
		if (HAL_GetTick() - last_log_read >= 10000) // Check if 10 seconds have passed.
		{
			Log_Read_All(); // Send all the logs.
			last_log_read = HAL_GetTick(); // Remember the time we sent the logs.
		}

		// Step 15: Every 5 seconds, tell the main computer how the battery is doing.
		// This is like calling a friend every few minutes to give them an update.
		if (HAL_GetTick() - last_status_send >= 5000) // Check if 5 seconds have passed.
		{
			SSP_SendStatus(); // Send the status.
			last_status_send = HAL_GetTick(); // Remember the time we sent the status.
		}

		// Step 16: Check if the main computer sent us a message.
		// This is like checking the mailbox to see if we got a letter.
		SSP_FrameTypeDef received_frame = {0}; // A place to store the message.
		if (SSP_ReceiveFrame(&husart2, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &received_frame) == HAL_OK)
		{
			SSP_ProcessReceivedFrame(&received_frame); // Read the message and do what it says.
		}

		// Wait for 1 second before starting the checklist again.
		// This is like taking a short break before doing your daily routine again.
		HAL_Delay((uint32_t)(LOOP_TIME * 1000)); // Wait for 1 second.
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


