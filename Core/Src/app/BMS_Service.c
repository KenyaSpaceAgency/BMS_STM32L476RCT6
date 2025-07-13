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
#include "delay.h"
#include "log.h"           // For Log_Message()

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
    if (value < min || value > max) {
        char msg[128];

        // Compose the error message once
        snprintf(msg, sizeof(msg), "[SANITY FAIL] %s=%.2f out of range (%.2f - %.2f)\r\n",
                 label, value, min, max);

        // Log it to the centralized logger
        Log_Message(BMS_MSG_LEVEL_ERROR, "%s", msg);

        // Also transmit over UART1
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
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "ADC poll for PCB temperature failed.");
        return -273.15f;
    }

    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t TS_CAL1 = *((uint16_t*)0x1FFF75A8); // 30°C calibration
    uint32_t TS_CAL2 = *((uint16_t*)0x1FFF75CA); // 110°C calibration

    if (TS_CAL2 == TS_CAL1) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Invalid STM32 temp sensor calibration data.");
        return -273.15f;
    }

    float temp = ((float)(TS_CAL2 - TS_CAL1) / 80.0f) * ((float)adc_value - TS_CAL1) + 30.0f;

    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "PCB Temperature: %.2f °C (ADC=0x%03lX, CAL1=%lu, CAL2=%lu)",
        temp, adc_value, TS_CAL1, TS_CAL2);

    return temp;
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
    // Step 1: Wake up both BQ76920 chips
    Log_Message(BMS_MSG_LEVEL_INFO, "Waking up BMS ICs...");
    HAL_GPIO_WritePin(BOOT_GPIO_Port, BOOT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOOT2_GPIO_Port, BOOT2_Pin, GPIO_PIN_RESET);
    SoftwareDelay(2);
    HAL_GPIO_WritePin(BOOT_GPIO_Port, BOOT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BOOT2_GPIO_Port, BOOT2_Pin, GPIO_PIN_SET);
    SoftwareDelay(2);

    // Step 2: Check I2C communication
    Log_Message(BMS_MSG_LEVEL_INFO, "Checking I2C readiness for BMS ICs...");
    telemetry.i2c_comm_error_ic1 = (HAL_I2C_IsDeviceReady(&hi2c1, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);
    telemetry.i2c_comm_error_ic2 = (HAL_I2C_IsDeviceReady(&hi2c2, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);
    telemetry.bms_online = !(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2);

    // Step 3: Initialize chips
    if (telemetry.bms_online) {
        Log_Message(BMS_MSG_LEVEL_INFO, "BMS ICs are online. Proceeding with initialization...");

        BQ76920_Initialise(&bms_instance1, &hi2c1);
        BQ76920_Initialise(&bms_instance2, &hi2c2);
        SoftwareDelay(10);

        BQ76920_WakeAndConfigure(&bms_instance1);
        BQ76920_WakeAndConfigure(&bms_instance2);
        SoftwareDelay(250);

        Log_Message(BMS_MSG_LEVEL_INFO, "BMS initialization and configuration completed.");
    } else {
        if (telemetry.i2c_comm_error_ic1)
            Log_Message(BMS_MSG_LEVEL_ERROR, "BMS IC1 (I2C1) failed to respond during initialization");
        if (telemetry.i2c_comm_error_ic2)
            Log_Message(BMS_MSG_LEVEL_ERROR, "BMS IC2 (I2C2) failed to respond during initialization");
        if (!telemetry.i2c_comm_error_ic1 && !telemetry.i2c_comm_error_ic2)
            Log_Message(BMS_MSG_LEVEL_ERROR, "BMS initialization failed for unknown reason");
    }

    // Step 4: Restore persistent telemetry
    Log_Message(BMS_MSG_LEVEL_INFO, "Restoring telemetry from flash...");
    Flash_ReadTelemetry();
    last_save_time = HAL_GetTick();
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Telemetry restored. Init timestamp = %lu ms", last_save_time);
}



void BQ76920_WakeAndConfigure(BQ76920_t *bms) {
    uint8_t sys_ctrl2 = 0;
    HAL_StatusTypeDef status;

    // --- Step 1: Read current SYS_CTRL2 ---
    status = BQ76920_ReadRegister(bms, SYS_CTRL2, &sys_ctrl2, NULL);
    if (status != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to read SYS_CTRL2 during wake/config.");
        return;
    }
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Original SYS_CTRL2 = 0x%02X", sys_ctrl2);

    // --- Step 2: Clear SHUT_A and SHUT_B bits (bits 2 and 3) ---
    sys_ctrl2 &= ~((1 << 2) | (1 << 3));

    // --- Step 3: Set FET_EN (bit 4), ADC_EN (bit 1), TEMP_SEL (bit 0) ---
    sys_ctrl2 |= (1 << 4) | (1 << 1) | (1 << 0);

    // --- Step 4: Write updated value back ---
    status = BQ76920_WriteRegister(bms, SYS_CTRL2, &sys_ctrl2, NULL);
    if (status != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to write SYS_CTRL2 during wake/config.");
    } else {
        Log_Message(BMS_MSG_LEVEL_INFO, "BQ76920 configured successfully (SYS_CTRL2 = 0x%02X)", sys_ctrl2);
    }

    SoftwareDelay(1); // Optional stabilization delay
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
    readAlert(&bms_instance1);
    readAlert(&bms_instance2);

    telemetry.i2c_comm_error_ic1 = (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
    telemetry.i2c_comm_error_ic2 = (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
    telemetry.bms_online = !(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2);

    if (telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "I2C error detected: IC1 = %d, IC2 = %d",
                    telemetry.i2c_comm_error_ic1, telemetry.i2c_comm_error_ic2);
        turnCHGOff(&bms_instance1);
        turnDSGOff(&bms_instance1);
        turnCHGOff(&bms_instance2);
        turnDSGOff(&bms_instance2);
    }

    float Vcell[NUMBER_OF_CELLS];
    for (int i = 0; i < NUMBER_OF_CELLS; i++) {
        float v1 = getCellVoltage(&bms_instance1, VC1 + i * 2);
        float v2 = getCellVoltage(&bms_instance2, VC1 + i * 2);
        Vcell[i] = (v1 + v2) / 2.0f;

        telemetry.vcell_ic1[i] = (uint16_t)(v1 * 1000);
        telemetry.vcell_ic2[i] = (uint16_t)(v2 * 1000);
    }

    float current1 = getCurrent(&bms_instance1);
    float current2 = getCurrent(&bms_instance2);
    float PackCurrent = (current1 + current2) / 2.0f;

    telemetry.current_ic1 = (int16_t)(current1 * 1000);
    telemetry.current_ic2 = (int16_t)(current2 * 1000);

    static uint8_t UV = 0, OV = 0, OC = 0, SC = 0;

    if (getAlert(&bms_instance1, 6) || getAlert(&bms_instance2, 6)) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "XREADY bit set. Re-configuring BQ76920 chips...");
        BQ76920_WakeAndConfigure(&bms_instance1);
        BQ76920_WakeAndConfigure(&bms_instance2);
    }

    if (getAlert(&bms_instance1, 3) || getAlert(&bms_instance2, 3)) {
        if (checkUV(Vcell)) {
            Log_Message(BMS_MSG_LEVEL_WARNING, "Undervoltage detected. Disabling discharge.");
            turnDSGOff(&bms_instance1);
            turnDSGOff(&bms_instance2);
        }
    } else if (checkNotUV(Vcell, UV)) {
        Log_Message(BMS_MSG_LEVEL_INFO, "Undervoltage cleared. Enabling discharge.");
        turnDSGOn(&bms_instance1);
        turnDSGOn(&bms_instance2);
    }

    if (getAlert(&bms_instance1, 2) || getAlert(&bms_instance2, 2)) {
        if (checkOV(Vcell) && PackCurrent >= 0) {
            Log_Message(BMS_MSG_LEVEL_WARNING, "Overvoltage detected. Disabling charge.");
            turnCHGOff(&bms_instance1);
            turnCHGOff(&bms_instance2);
            SOHPack(&bms_instance1);
            SOHPack(&bms_instance2);
        }
    } else if (checkNotOV(Vcell, OV)) {
        Log_Message(BMS_MSG_LEVEL_INFO, "Overvoltage cleared. Enabling charge.");
        turnCHGOn(&bms_instance1);
        turnCHGOn(&bms_instance2);
    }

    if (getAlert(&bms_instance1, 0) || getAlert(&bms_instance2, 0)) {
        if (checkOC(PackCurrent)) {
            Log_Message(BMS_MSG_LEVEL_WARNING, "Overcurrent in discharge detected. Disabling discharge.");
            turnDSGOff(&bms_instance1);
            turnDSGOff(&bms_instance2);
        }
    } else if (checkNotOC(PackCurrent, OC)) {
        Log_Message(BMS_MSG_LEVEL_INFO, "Overcurrent cleared. Enabling discharge.");
        turnDSGOn(&bms_instance1);
        turnDSGOn(&bms_instance2);
    }

    if (getAlert(&bms_instance1, 1) || getAlert(&bms_instance2, 1)) {
        if (checkSC(PackCurrent)) {
            Log_Message(BMS_MSG_LEVEL_CRITICAL, "Short-circuit detected! Disabling CHG and DSG.");
            turnDSGOff(&bms_instance1);
            turnDSGOff(&bms_instance2);
            turnCHGOff(&bms_instance1);
            turnCHGOff(&bms_instance2);
        }
    } else if (checkNotSC(PackCurrent, SC)) {
        Log_Message(BMS_MSG_LEVEL_INFO, "Short-circuit cleared. Re-enabling CHG and DSG.");
        turnDSGOn(&bms_instance1);
        turnDSGOn(&bms_instance2);
        turnCHGOn(&bms_instance1);
        turnCHGOn(&bms_instance2);
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
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Reading PCB and pack temperatures...");

    // 🧪 Read PCB temperature via STM32 internal sensor
    float temp = Read_PCB_Temperature();
    telemetry.pcb_temperature = temp;
    BMS_CheckSanity("PCB_Temp", temp, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
    Log_Message(BMS_MSG_LEVEL_VERBOSE, "PCB Temperature: %.2f C", temp);

    // 🧪 TMP100 external temperature sensors
    telemetry.pack_temperature_ic1 = TMP100_ReadTemperature(&hi2c1, TMP100_IC1_ADDR);
    telemetry.pack_temperature_ic2 = TMP100_ReadTemperature(&hi2c2, TMP100_IC2_ADDR);
    BMS_CheckSanity("TMP100_IC1", telemetry.pack_temperature_ic1, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
    BMS_CheckSanity("TMP100_IC2", telemetry.pack_temperature_ic2, MIN_TEMPERATURE_C, MAX_TEMPERATURE_C);
    Log_Message(BMS_MSG_LEVEL_VERBOSE, "Pack Temps: IC1 = %.2f C, IC2 = %.2f C",
                telemetry.pack_temperature_ic1,
                telemetry.pack_temperature_ic2);

    // 🧪 Read current in mA from both BQ76920s
    float current1 = getCurrent(&bms_instance1);
    float current2 = getCurrent(&bms_instance2);
    telemetry.current_ic1 = (int16_t)(current1 * 1000);
    telemetry.current_ic2 = (int16_t)(current2 * 1000);
    BMS_CheckSanity("Current_IC1", current1, -MAX_PACK_CURRENT, MAX_PACK_CURRENT);
    BMS_CheckSanity("Current_IC2", current2, -MAX_PACK_CURRENT, MAX_PACK_CURRENT);
    Log_Message(BMS_MSG_LEVEL_VERBOSE, "Pack Current: IC1 = %.3f A, IC2 = %.3f A", current1, current2);

    // 🧪 Read and log cell voltages
    for (int i = 0; i < NUMBER_OF_CELLS; i++) {
        float v1 = getCellVoltage(&bms_instance1, VC1 + i * 2);
        float v2 = getCellVoltage(&bms_instance2, VC1 + i * 2);
        telemetry.vcell_ic1[i] = (uint16_t)(v1 * 1000);  // in mV
        telemetry.vcell_ic2[i] = (uint16_t)(v2 * 1000);  // in mV
        BMS_CheckSanity("Cell_IC1", v1, MIN_CELL_VOLTAGE, MAX_CELL_VOLTAGE);
        BMS_CheckSanity("Cell_IC2", v2, MIN_CELL_VOLTAGE, MAX_CELL_VOLTAGE);
        Log_Message(BMS_MSG_LEVEL_VERBOSE, "Cell %d Voltages: IC1 = %.3f V, IC2 = %.3f V", i + 1, v1, v2);
    }

    Log_Message(BMS_MSG_LEVEL_DEBUG, "Measurement read complete.");
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
    // ⏱ SOC calculation for both BMS ICs
    float soc1 = SOCPack(&bms_instance1, telemetry.current_ic1 / 1000.0f,
                         telemetry.vpack_ic1 / 1000.0f);
    float soc2 = SOCPack(&bms_instance2, telemetry.current_ic2 / 1000.0f,
                         telemetry.vpack_ic2 / 1000.0f);
    telemetry.soc = fminf(fmaxf((soc1 + soc2) / 2.0f, 0.0f), 100.0f);
    Log_Message(BMS_MSG_LEVEL_INFO, "SOC: IC1 = %.2f%%, IC2 = %.2f%%, Avg = %.2f%%", soc1, soc2, telemetry.soc);

    // 🩺 SOH calculation
    float soh1 = SOHPack(&bms_instance1);
    float soh2 = SOHPack(&bms_instance2);
    telemetry.soh = (soh1 + soh2) / 2.0f;
    Log_Message(BMS_MSG_LEVEL_INFO, "SOH: IC1 = %.2f%%, IC2 = %.2f%%, Avg = %.2f%%", soh1, soh2, telemetry.soh);

    // 🚨 Alert bits from SYS_STAT (0–7)
    for (int i = 0; i < 8; i++) {
        telemetry.error_flags[i] = getAlert(&bms_instance1, i) || getAlert(&bms_instance2, i);
        if (telemetry.error_flags[i]) {
            Log_Message(BMS_MSG_LEVEL_WARNING, "Alert bit %d active on IC1 or IC2", i);
        }
    }

    // 🔍 Specific status flags
    telemetry.ovrd_alert_ic1     = getAlert(&bms_instance1, 5);
    telemetry.ovrd_alert_ic2     = getAlert(&bms_instance2, 5);
    telemetry.device_xready_ic1  = getAlert(&bms_instance1, 6);
    telemetry.device_xready_ic2  = getAlert(&bms_instance2, 6);
    telemetry.load_present_ic1   = getAlert(&bms_instance1, 7);
    telemetry.load_present_ic2   = getAlert(&bms_instance2, 7);

    Log_Message(BMS_MSG_LEVEL_VERBOSE, "SYS_STAT: OVRD[%d,%d], XREADY[%d,%d], LOAD[%d,%d]",
                telemetry.ovrd_alert_ic1, telemetry.ovrd_alert_ic2,
                telemetry.device_xready_ic1, telemetry.device_xready_ic2,
                telemetry.load_present_ic1, telemetry.load_present_ic2);

    // 🔄 Cell balancing mask
    telemetry.balancing_mask_ic1 = justRead1(&bms_instance1);
    telemetry.balancing_mask_ic2 = justRead1(&bms_instance2);
    telemetry.balancing_active = (telemetry.balancing_mask_ic1 || telemetry.balancing_mask_ic2) ? 1 : 0;
    if (telemetry.balancing_active) {
        Log_Message(BMS_MSG_LEVEL_INFO, "Cell balancing active: Mask IC1=0x%02X, IC2=0x%02X",
                    telemetry.balancing_mask_ic1, telemetry.balancing_mask_ic2);
    }

//    // 🔋 Charging recommendation
//    telemetry.charge_immediately = (telemetry.soc < 20.0f) ? 1 : 0;
//    if (telemetry.charge_immediately) {
//        Log_Message(BMS_MSG_LEVEL_WARNING, "SOC below 20%% — recommend charging immediately.");
//    }
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
    // Add to total operation time
    telemetry.total_operating_time += delta_time;

    // Average current in mA
    float avg_current = (telemetry.current_ic1 + telemetry.current_ic2) / 2.0f;

    // Charging state
    if (avg_current > 0) {
        telemetry.total_charge_time += delta_time;

        static uint8_t charge_debounce = 0;

        if (telemetry.soc >= 99.0f && fabsf(avg_current) <= 100.0f) {
            // Only count once every 60 seconds minimum (arbitrary cooldown)
            charge_debounce += delta_time;
            if (charge_debounce >= 60000) {
                telemetry.charge_cycle_count++;
                charge_debounce = 0;
                Log_Message(BMS_MSG_LEVEL_INFO, "Charge cycle counted. Total: %lu",
                            telemetry.charge_cycle_count);
            }
        } else {
            charge_debounce = 0; // reset debounce if conditions not met
        }
    }
    // Discharging state
    else if (avg_current < 0) {
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
    uint32_t now = HAL_GetTick();
    static uint32_t last_check = 0;

    if (telemetry.soc < 5.0f) {
        // Accumulate only if we're checking periodically
        if (last_check > 0) {
            low_power_timer += (now - last_check) / 1000; // seconds
        }

        if (low_power_timer >= 300) {
            Log_Message(BMS_MSG_LEVEL_WARNING, "Low SOC persisted > 5 min. Entering SHIP mode...");
            Enter_SHIP_Mode(&bms_instance1);
            Enter_SHIP_Mode(&bms_instance2);
            *low_power_mode = 1;
            low_power_timer = 0;
        }
    } else {
        low_power_timer = 0;
    }

    last_check = now;
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
    // Step 1: Pulse BOOT pins high -> low to wake up BQ76920s
    HAL_GPIO_WritePin(bms_instance1.bootPort, bms_instance1.bootPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bms_instance2.bootPort, bms_instance2.bootPin, GPIO_PIN_SET);
    SoftwareDelay(3);  // Minimum 2ms required, using 3ms for safety
    HAL_GPIO_WritePin(bms_instance1.bootPort, bms_instance1.bootPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bms_instance2.bootPort, bms_instance2.bootPin, GPIO_PIN_RESET);
    SoftwareDelay(10); // Allow boot circuitry to stabilize

    // Step 2: Check I2C readiness of both BQ76920 devices
    telemetry.i2c_comm_error_ic1 = (HAL_I2C_IsDeviceReady(&hi2c1, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);
    telemetry.i2c_comm_error_ic2 = (HAL_I2C_IsDeviceReady(&hi2c2, BQ76920_ADDRESS, 2, HAL_MAX_DELAY) != HAL_OK);

    // Step 3: If both chips are online, reinitialize and exit low-power mode
    if (!(telemetry.i2c_comm_error_ic1 || telemetry.i2c_comm_error_ic2)) {
        *low_power_mode = 0;
        telemetry.bms_online = 1;

        BQ76920_Initialise(&bms_instance1, &hi2c1);
        BQ76920_Initialise(&bms_instance2, &hi2c2);
        SoftwareDelay(10);  // Optional settle delay
        BQ76920_WakeAndConfigure(&bms_instance1);
        BQ76920_WakeAndConfigure(&bms_instance2);
        SoftwareDelay(250);  // Allow system stabilization

        Log_Message(BMS_MSG_LEVEL_INFO, "Exited SHIP mode. BQ76920s reinitialized.");
    } else {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to wake BQ76920. IC1_ready=%d, IC2_ready=%d",
                    !telemetry.i2c_comm_error_ic1, !telemetry.i2c_comm_error_ic2);
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
    // Get the current system tick (in milliseconds)
    uint32_t current_time = HAL_GetTick();

    // Compute time since last save, safely handling HAL_GetTick() overflow
    uint32_t time_since_last_save = current_time - last_save_time;

    // Check for significant SOC change (more than 1%)
    if (fabsf(telemetry.soc - last_soc) > 1.0f) {
        Flash_WriteTelemetry();
        last_soc = telemetry.soc;
        last_save_time = current_time;
        Log_Message(BMS_MSG_LEVEL_INFO, "Flash write triggered by SOC change: %.2f%%", telemetry.soc);
    }
    // Check if 5 minutes have passed since last save (300,000ms)
    else if (time_since_last_save > 300000) {
        Flash_WriteTelemetry();
        last_save_time = current_time;
        Log_Message(BMS_MSG_LEVEL_INFO, "Flash write triggered by periodic timeout (5 min)");
    }
}




float GetReliableCellVoltage(uint8_t cell_idx)
{
    if (cell_idx > 3) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Invalid cell index: %u", cell_idx);
        return 0.0f;
    }

    float v1 = getCellVoltage(&bms_instance1, VC1 + cell_idx * 2);
    float v2 = getCellVoltage(&bms_instance2, VC1 + cell_idx * 2);

    bool ic1_valid = (v1 >= MIN_CELL_VOLTAGE && v1 <= MAX_CELL_VOLTAGE);
    bool ic2_valid = (v2 >= MIN_CELL_VOLTAGE && v2 <= MAX_CELL_VOLTAGE);

    if (telemetry.i2c_comm_error_ic1 && !telemetry.i2c_comm_error_ic2 && ic2_valid) {
        // Primary IC1 is offline, backup IC2 is good
        return v2;
    }

    if (!telemetry.i2c_comm_error_ic1 && ic1_valid) {
        // Primary IC1 is online and data valid
        return v1;
    }

    if (!telemetry.i2c_comm_error_ic1 && !ic1_valid &&
        !telemetry.i2c_comm_error_ic2 && ic2_valid) {
        // IC1 online but invalid data; IC2 good
        return v2;
    }

    if (telemetry.i2c_comm_error_ic1 && telemetry.i2c_comm_error_ic2) {
        // Both ICs are offline
        Log_Message(BMS_MSG_LEVEL_CRITICAL, "Both BMS ICs offline for cell %u", cell_idx);
        return 0.0f;
    }

    if (ic1_valid && ic2_valid) {
        // Both online and valid – prefer primary, or average
        return v1; // or: return (v1 + v2) / 2.0f;
    }

    Log_Message(BMS_MSG_LEVEL_WARNING, "Unresolved state: IC1 err:%u valid:%u, IC2 err:%u valid:%u",
                telemetry.i2c_comm_error_ic1, ic1_valid,
                telemetry.i2c_comm_error_ic2, ic2_valid);

    return 0.0f; // Conservative fallback
}


