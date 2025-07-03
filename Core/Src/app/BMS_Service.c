 // File: BMS_Service.c
// Created on: Jun 29, 2025
// Author: yomue
// Purpose: Manages the core BMS functionality, including initialization, measurements,
//          alerts, SOC/SOH calculations, and flash storage.

// Include header files needed for the code
#include "main.h"             // Main project header with TelemetryData and GPIO definitions
#include "BMS_Service.h"      // BMS service definitions (e.g., thresholds, function prototypes)
#include "flash.h"            // Flash memory functions for storing telemetry
#include "temperature.h"      // Temperature sensor and heater control functions
#include <math.h>             // Math functions like fminf, fmaxf, fabs for calculations
#include <stdio.h>            // Printing functions for error logging (snprintf)
#include <string.h>           // String functions like strlen for logging

// External variables declared in main.c
extern I2C_HandleTypeDef hi2c1;         // I2C1 interface (PB6/PB7) for BQ76920 and TMP100
extern I2C_HandleTypeDef hi2c2;         // I2C2 interface (PB10/PB11) for second BQ76920
extern ADC_HandleTypeDef hadc1;         // ADC for PCB temperature (internal STM32 sensor)
extern UART_HandleTypeDef huart1;       // UART (PA9/PA10) for error logging



// Declare internal functions (static, only used in this file)
static float Read_PCB_Temperature(void); // Reads PCB temperature using STM32 ADC
static void Enter_SHIP_Mode(BQ76920_t *BMS); // Puts BQ76920 into low-power mode

// Static variables for state management (only accessible in this file)
static float last_soc = 0.0f;           // Stores the last SOC value for flash storage checks
static uint32_t last_save_time = 0;     // Tracks the last time telemetry was saved to flash
static uint32_t low_power_timer = 0;    // Tracks time in low SOC state for SHIP mode

// Function: BMS_CheckSanity
// Inputs:
//   - label: A string (const char*), the name of the value being checked (e.g., "Cell_IC1")
//   - value: A float, the value to check (e.g., voltage, current)
//   - min: A float, the minimum allowed value
//   - max: A float, the maximum allowed value
// Output:
//   - None (void), logs an error if the value is out of range
// Significance:
//   - Checks if a measurement (e.g., voltage, current) is within safe limits, logging errors
//     if not. Used to ensure reliable data for battery monitoring.
void BMS_CheckSanity(const char *label, float value, float min, float max) {
    // Check if the value is outside the allowed range
    if (value < min || value > max) {
        // Create a buffer to hold the error message
        char msg[128];
        // Format the error message with the label, value, and range
        Log_Message(BMS_MSG_LEVEL_ERROR, "[SANITY FAIL] %s=%.2f out of range (%.2f - %.2f)\r\n",
                label, value, min, max);
        snprintf(msg, sizeof(msg), "[SANITY FAIL] %s=%.2f out of range (%.2f - %.2f)\r\n",
                 label, value, min, max);
        // Send the error message over UART (huart1) for debugging
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

// Function: Read_PCB_Temperature
// Inputs:
//   - None (void), uses global hadc1 for ADC
// Output:
//   - Returns a float, the PCB temperature in Celsius, or -273.15 if invalid
// Significance:
//   - Reads the STM32’s internal temperature sensor via ADC to monitor the PCB temperature,
//     stored in telemetry.pcb_temperature for safety checks.
float Read_PCB_Temperature(void) {
    // Start the ADC to begin temperature measurement
    HAL_ADC_Start(&hadc1);
    // Wait for the ADC conversion to complete (timeout if too long)
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    // Get the raw ADC value (12-bit)
    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    // Stop the ADC to free it for other uses
    HAL_ADC_Stop(&hadc1); // Fixed: Added &hadc1

    // Read calibration values from STM32 memory (Reference Manual, Section 15.4.29, page 463)
    uint32_t TS_CAL1 = *((uint16_t*)0x1FFF75A8); // Calibration value at 30°C
    uint32_t TS_CAL2 = *((uint16_t*)0x1FFF75CA); // Calibration value at 110°C
    // Check if calibration values are invalid (equal means no calibration)
    if (TS_CAL2 == TS_CAL1) return -273.15f; // Return absolute zero for error
    // Calculate temperature using linear interpolation
    // Formula: temp = (TS_CAL2 - TS_CAL1)/(110 - 30) * (adc_value - TS_CAL1) + 30
    float temp = ((float)(TS_CAL2 - TS_CAL1) / (110.0f - 30.0f)) * ((float)adc_value - TS_CAL1) + 30.0f;
    return temp; // Return temperature in Celsius
}

// Function: Enter_SHIP_Mode
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), puts the BQ76920 into SHIP mode
// Significance:
//   - Puts the BQ76920 into a low-power state to save battery when SOC is critically low,
//     critical for extending battery life in your satellite (BQ76920 datasheet, Section 8.3.5, page 22).
void Enter_SHIP_Mode(BQ76920_t *BMS) {
    uint8_t step1 = (1 << 7); // SHUT_B = 1, SHUT_A = 0
    uint8_t step2 = (1 << 6); // SHUT_A = 1, SHUT_B = 0
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &step1, NULL);
    HAL_Delay(1); // Mandatory 1ms delay between steps
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &step2, NULL);
}

// Function: BMS_Service_Init
// Inputs:
//   - None (void), uses global variables (hi2c1, hi2c2, telemetry)
// Output:
//   - None (void), initializes the BMS
// Significance:
//   - Initializes the BQ76920 chips and loads saved telemetry from flash, called at startup
//     in main.c to set up the BMS.
void BMS_Service_Init(void) {
    // Check if BQ76920 on I2C1 is responsive
    telemetry.i2c_comm_error_ic1 = (HAL_I2C_IsDeviceReady(&hi2c1, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);
    // Check if BQ76920 on I2C2 is responsive
    telemetry.i2c_comm_error_ic2 = (HAL_I2C_IsDeviceReady(&hi2c2, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);
    // Set BMS online if no I2C errors
    telemetry.bms_online = !(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2);

    // If BMS is online, initialize both BQ76920 chips
    if (telemetry.bms_online) {
        // Initialize first BQ76920 on I2C1
        BQ76920_Initialise(&bms_instance1, &hi2c1);
        // Initialize second BQ76920 on I2C2
        BQ76920_Initialise(&bms_instance2, &hi2c2);
        // Wait 250ms for chips to stabilize
        HAL_Delay(250);
    } else {
        // Log an error if initialization fails
    	Log_Message(BMS_MSG_LEVEL_ERROR,"BMS initialization failed");
    }

    // Load saved telemetry from flash
    Flash_ReadTelemetry();
    // Record the current time for flash storage timing
    last_save_time = HAL_GetTick();
}

// Function: BMS_Service_HandleAlerts
// Inputs:
//   - None (void), uses global bms_instance1, bms_instance2, telemetry
// Output:
//   - None (void), updates alert flags and handles device-ready alerts
// Significance:
//   - Checks for alerts (e.g., overvoltage, overcurrent) from both BQ76920 chips and updates
//     communication status, ensuring the BMS responds to issues.
void BMS_Service_HandleAlerts(void) {
    // Read alerts from first BQ76920
    readAlert(&bms_instance1);
    // Read alerts from second BQ76920
    readAlert(&bms_instance2);

    // Update I2C communication status for first chip
    telemetry.i2c_comm_error_ic1 = (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
    // Update I2C communication status for second chip
    telemetry.i2c_comm_error_ic2 = (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    // Set BMS online if no I2C errors
    telemetry.bms_online = !(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2);

    // Check for device-ready alerts (XREADY, bit 6 in SYS_STAT)
    if (getAlert(&bms_instance1, 6) || getAlert(&bms_instance2, 6)) {
        // Declare variable for SYS_CTRL2
        uint8_t sys_ctrl2;
        // Read SYS_CTRL2 for first BQ76920
        BQ76920_ReadRegister(&bms_instance1, SYS_CTRL2, &sys_ctrl2, NULL);
        // Set bit 2 (ALERT_EN) to handle XREADY
        sys_ctrl2 |= (1 << 2);
        // Write updated SYS_CTRL2
        BQ76920_WriteRegister(&bms_instance1, SYS_CTRL2, &sys_ctrl2, NULL);

        // Read SYS_CTRL2 for second BQ76920
        BQ76920_ReadRegister(&bms_instance2, SYS_CTRL2, &sys_ctrl2, NULL);
        // Set bit 2 (ALERT_EN) to handle XREADY
        sys_ctrl2 |= (1 << 2);
        // Write updated SYS_CTRL2
        BQ76920_WriteRegister(&bms_instance2, SYS_CTRL2, &sys_ctrl2, NULL);
    }
}

// Function: BMS_Service_ReadMeasurements
// Inputs:
//   - None (void), uses global bms_instance1, bms_instance2, telemetry
// Output:
//   - None (void), updates telemetry with measurements
// Significance:
//   - Reads voltages, currents, and PCB temperature, storing them in telemetry with
//     sanity checks to ensure valid data.
void BMS_Service_ReadMeasurements(void) {
	float temp = Read_PCB_Temperature();
	telemetry.pcb_temperature = temp;
	BMS_CheckSanity("PCB_Temp", temp, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
	// --- TMP100 readings for pack temperature ---
	telemetry.pack_temperature_ic1 = TMP100_ReadTemperature(&hi2c1, TMP100_IC1_ADDR);
	telemetry.pack_temperature_ic2 = TMP100_ReadTemperature(&hi2c2, TMP100_IC2_ADDR);

	// Optional sanity check
	BMS_CheckSanity("TMP100_IC1", telemetry.pack_temperature_ic1, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
	BMS_CheckSanity("TMP100_IC2", telemetry.pack_temperature_ic2, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
    // Read current from first BQ76920 and convert to mA
    telemetry.current_ic1 = (int16_t)(getCurrent(&bms_instance1) * 1000);
    // Read current from second BQ76920 and convert to mA
    telemetry.current_ic2 = (int16_t)(getCurrent(&bms_instance2) * 1000);

    // Read cell voltages for both BQ76920 chips
    for (int i = 0; i < NUMBER_OF_CELLS; i++) { // NUMBER_OF_CELLS is 4 (BQ76920.h)
        // Read voltage for cell i from first BQ76920
        float v1 = getCellVoltage(&bms_instance1, VC1 + i * 2);
        // Read voltage for cell i from second BQ76920
        float v2 = getCellVoltage(&bms_instance2, VC1 + i * 2);

        // Store voltage in mV for first chip
        telemetry.vcell_ic1[i] = (uint16_t)(v1 * 1000);
        // Store voltage in mV for second chip
        telemetry.vcell_ic2[i] = (uint16_t)(v2 * 1000);

        // Check if voltages are within safe range (2.5V–4.3V, BMS_Service.h)
        BMS_CheckSanity("Cell_IC1", v1, MIN_CELL_VOLTAGE, MAX_CELL_VOLTAGE);
        BMS_CheckSanity("Cell_IC2", v2, MIN_CELL_VOLTAGE, MAX_CELL_VOLTAGE);
    }

    // Convert currents back to amps for sanity checks
    float current1 = telemetry.current_ic1 / 1000.0f;
    float current2 = telemetry.current_ic2 / 1000.0f;

    // Check if currents are within safe range (±30A, BMS_Service.h)
    BMS_CheckSanity("Current_IC1", current1, -MAX_PACK_CURRENT, MAX_PACK_CURRENT);
    BMS_CheckSanity("Current_IC2", current2, -MAX_PACK_CURRENT, MAX_PACK_CURRENT);


    // Store temperature in telemetry
    telemetry.pcb_temperature = temp;
    // Check if temperature is within safe range (-40°C to 85°C, BMS_Service.h)
    BMS_CheckSanity("PCB_Temp", temp, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
}

// Function: BMS_Service_ProcessData
// Inputs:
//   - None (void), uses global bms_instance1, bms_instance2, telemetry
// Output:
//   - None (void), updates telemetry with SOC, SOH, and alerts
// Significance:
//   - Calculates SOC and SOH, updates alerts, and manages cell balancing, ensuring
//     the BMS tracks battery state and safety.
void BMS_Service_ProcessData(void) {
    // Calculate SOC for first BQ76920 (current and voltage in amps and volts)
    float soc1 = SOCPack(&bms_instance1, telemetry.current_ic1 / 1000.0f,
                         telemetry.vpack_ic1 / 1000.0f);
    // Calculate SOC for second BQ76920
    float soc2 = SOCPack(&bms_instance2, telemetry.current_ic2 / 1000.0f,
                         telemetry.vpack_ic2 / 1000.0f);
    // Average the SOC values and clamp between 0% and 100%
    telemetry.soc = fminf(fmaxf((soc1 + soc2) / 2.0f, 0.0f), 100.0f);

    // Calculate SOH for first BQ76920
    float soh1 = SOHPack(&bms_instance1);
    // Calculate SOH for second BQ76920
    float soh2 = SOHPack(&bms_instance2);
    // Average the SOH values
    telemetry.soh = (soh1 + soh2) / 2.0f;

    // Update error flags for all 8 bits
    for (int i = 0; i < 8; i++) {
        // Set flag if either BQ76920 reports an alert
        telemetry.error_flags[i] = getAlert(&bms_instance1, i) || getAlert(&bms_instance2, i);
    }

    // Update specific alerts from SYS_STAT
    telemetry.ovrd_alert_ic1 = getAlert(&bms_instance1, 5); // Overcurrent/short-circuit alert
    telemetry.ovrd_alert_ic2 = getAlert(&bms_instance2, 5);
    telemetry.device_xready_ic1 = getAlert(&bms_instance1, 6); // Device ready alert
    telemetry.device_xready_ic2 = getAlert(&bms_instance2, 6);
    telemetry.load_present_ic1 = getAlert(&bms_instance1, 7); // Load detection alert
    telemetry.load_present_ic2 = getAlert(&bms_instance2, 7);

    // Enable cell balancing for first BQ76920 if charging
    EnableBalanceCell(&bms_instance1, telemetry.current_ic1 / 1000.0f);
    // Enable cell balancing for second BQ76920 if charging
    EnableBalanceCell(&bms_instance2, telemetry.current_ic2 / 1000.0f);

    // Read balancing status for first BQ76920
    telemetry.balancing_mask_ic1 = justRead1(&bms_instance1);
    // Read balancing status for second BQ76920
    telemetry.balancing_mask_ic2 = justRead1(&bms_instance2);
    // Set balancing active if either chip is balancing
    telemetry.balancing_active = (telemetry.balancing_mask_ic1 ||
                                telemetry.balancing_mask_ic2) ? 1 : 0;

    // Set charge flag if SOC is low (<20%)
    telemetry.charge_immediately = (telemetry.soc < 20.0f) ? 1 : 0;
}

// Function: BMS_Service_UpdateCounters
// Inputs:
//   - delta_time: A uint32_t, the time elapsed since the last update (in seconds)
// Output:
//   - None (void), updates telemetry counters
// Significance:
//   - Updates time-based counters (operating, charge, discharge time) and charge cycle count,
//     tracking battery usage for telemetry.
void BMS_Service_UpdateCounters(uint32_t delta_time) {
    // Add elapsed time to total operating time
    telemetry.total_operating_time += delta_time;

    // If either chip is charging (positive current)
    if (telemetry.current_ic1 > 0 || telemetry.current_ic2 > 0) {
        // Add elapsed time to total charge time
        telemetry.total_charge_time += delta_time;
        // If SOC is nearly full (≥99%) and current is low (≤100mA), count a charge cycle
        if (telemetry.soc >= 99.0f &&
            (abs(telemetry.current_ic1) <= 100 ||
            abs(telemetry.current_ic2) <= 100)) {
            telemetry.charge_cycle_count++;
        }
    }
    // If either chip is discharging (negative current)
    else if (telemetry.current_ic1 < 0 || telemetry.current_ic2 < 0) {
        // Add elapsed time to total discharge time
        telemetry.total_discharge_time += delta_time;
    }
}

// Function: BMS_Service_HandleLowPowerCondition
// Inputs:
//   - low_power_mode: A pointer to a uint8_t, the current low-power state (1=on, 0=off)
// Output:
//   - None (void), updates low_power_mode and may enter SHIP mode
// Significance:
//   - Monitors SOC and enters SHIP mode if critically low (<5%) for 5 minutes,
//     saving battery power in critical conditions.
void BMS_Service_HandleLowPowerCondition(uint8_t *low_power_mode) {
    // If SOC is critically low (<5%)
    if (telemetry.soc < 5.0f) {
        // Add elapsed time (in seconds) to low-power timer
        low_power_timer += (HAL_GetTick() - last_save_time) / 1000;
        // If low SOC persists for 5 minutes (300 seconds)
        if (low_power_timer > 300) {
            // Enter SHIP mode for first BQ76920
            Enter_SHIP_Mode(&bms_instance1);
            // Enter SHIP mode for second BQ76920
            Enter_SHIP_Mode(&bms_instance2);
            // Set low-power mode flag
            *low_power_mode = 1;
            // Reset the timer
            low_power_timer = 0;
        }
    } else {
        // Reset the timer if SOC is above 5%
        low_power_timer = 0;
    }
}

// Function: BMS_Service_HandleLowPowerMode
// Inputs:
//   - low_power_mode: A pointer to a uint8_t, the current low-power state
// Output:
//   - None (void), attempts to wake BQ76920 chips and update mode
// Significance:
//   - Attempts to wake the BQ76920 chips from SHIP mode by toggling boot pins,
//     reinitializing them if responsive.
void BMS_Service_HandleLowPowerMode(uint8_t *low_power_mode) {
    // Wake-up sequence: toggle boot pins (BOOT1_Pin, BOOT2_Pin)
    HAL_GPIO_WritePin(bms_instance1.bootPort, bms_instance1.bootPin, GPIO_PIN_SET); // Set BOOT1 high
    HAL_GPIO_WritePin(bms_instance2.bootPort, bms_instance2.bootPin, GPIO_PIN_SET); // Set BOOT2 high
    HAL_Delay(3); // Wait 3ms for chips to respond
    HAL_GPIO_WritePin(bms_instance1.bootPort, bms_instance1.bootPin, GPIO_PIN_RESET); // Set BOOT1 low
    HAL_GPIO_WritePin(bms_instance2.bootPort, bms_instance2.bootPin, GPIO_PIN_RESET); // Set BOOT2 low
    HAL_Delay(10); // Wait 10ms for stabilization

    // Check if BQ76920 chips are responsive
    telemetry.i2c_comm_error_ic1 = (HAL_I2C_IsDeviceReady(&hi2c1, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);
    telemetry.i2c_comm_error_ic2 = (HAL_I2C_IsDeviceReady(&hi2c2, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);

    // If no I2C errors, reinitialize chips
    if (!(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2)) {
        // Clear low-power mode flag
        *low_power_mode = 0;
        // Set BMS online
        telemetry.bms_online = 1;
        // Reinitialize first BQ76920
        BQ76920_Initialise(&bms_instance1, &hi2c1);
        // Reinitialize second BQ76920
        BQ76920_Initialise(&bms_instance2, &hi2c2);
        // Wait 250ms for stabilization
        HAL_Delay(250);
    }
}

// Function: BMS_Service_HandleFlashStorage
// Inputs:
//   - None (void), uses global telemetry and static variables
// Output:
//   - None (void), saves telemetry to flash if needed
// Significance:
//   - Saves telemetry to flash when SOC changes significantly (>1%) or every 5 minutes,
//     ensuring persistent data storage.
void BMS_Service_HandleFlashStorage(void) {
    // Get the current system tick (milliseconds)
    uint32_t current_time = HAL_GetTick();

    // If SOC has changed by more than 1%
    if (fabs(telemetry.soc - last_soc) > 1.0f) {
        // Save telemetry to flash
        Flash_WriteTelemetry();
        // Update last SOC value
        last_soc = telemetry.soc;
        // Update last save time
        last_save_time = current_time;
    }
    // If 5 minutes (300,000ms) have passed since last save
    else if (current_time - last_save_time > 300000) {
        // Save telemetry to flash
        Flash_WriteTelemetry();
        // Update last save time
        last_save_time = current_time;
    }
}
