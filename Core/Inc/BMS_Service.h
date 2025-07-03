/*
 * BMS_Service.h
 *
 *  Created on: Jun 29, 2025
 *      Author: yomue
 *
 *  This header file declares the core service layer for the Battery Management System (BMS).
 *  It defines key safety thresholds, inline macros for sanity checks, and function prototypes
 *  for initializing, monitoring, and processing battery-related data.
 */

#ifndef BMS_SERVICE_H
#define BMS_SERVICE_H

// ---------------------- INCLUDE DEPENDENCIES ----------------------
#include "main.h"           // Project-wide configurations
#include "bq76920.h"        // Driver for the BQ76920 battery monitor
#include "flash.h"          // Flash read/write support for telemetry persistence
#include "stm32l4xx_hal.h"  // STM32 HAL for GPIO, I2C, etc.

// ---------------------- THRESHOLD DEFINITIONS ----------------------

// Minimum voltage for a healthy cell (in volts)
#define MIN_CELL_VOLTAGE     2.5f   // Below this, cell is at risk of undervoltage

// Maximum allowed voltage for a cell (in volts)
#define MAX_CELL_VOLTAGE     4.3f   // Above this, cell might be overcharged

// Maximum allowed current drawn from or pushed into the pack (in amperes)
#define MAX_PACK_CURRENT     30.0f  // Above this might indicate short or overload

// Safe operating temperature range (in Celsius)
#define MIN_TEMPERATURE_C   -40.0f  // Below this, lithium-ion cells degrade
#define MAX_TEMPERATURE_C    85.0f  // Above this is unsafe for most electronics

// ---------------------- SAFETY CHECK MACROS ----------------------

// Check if a voltage reading is within safe range
#define IS_VOLTAGE_VALID(v)    ((v) >= MIN_CELL_VOLTAGE && (v) <= MAX_CELL_VOLTAGE)

// Check if a current measurement is within limit
#define IS_CURRENT_VALID(c)    (fabsf(c) <= MAX_PACK_CURRENT)

// Check if temperature is in the safe range
#define IS_TEMPERATURE_VALID(t) ((t) >= MIN_TEMPERATURE_C && (t) <= MAX_TEMPERATURE_C)


// ====================== FUNCTION PROTOTYPES ======================

// ----------------------------------------------------------------
// BMS_Service_Init
//
// Initializes the BMS service, sensors, state tracking, and any required
// GPIO or peripherals.
//
// Inputs: None
// Output: None
// Significance: Should be called once at system startup.
// ----------------------------------------------------------------
void BMS_Service_Init(void);


// ----------------------------------------------------------------
// BMS_Service_HandleAlerts
//
// Checks and responds to alerts from the BQ76920 (e.g. overvoltage, short circuit).
//
// Inputs: None
// Output: None
// Significance: Keeps the system safe by reacting to fault conditions in real-time.
// ----------------------------------------------------------------
void BMS_Service_HandleAlerts(void);


// ----------------------------------------------------------------
// BMS_Service_ReadMeasurements
//
// Reads the current pack voltage, individual cell voltages, current, and temperature
// from the BQ76920.
//
// Inputs: None
// Output: Updates the global `telemetry` structure
// Significance: Core function for collecting raw data from the battery.
// ----------------------------------------------------------------
void BMS_Service_ReadMeasurements(void);


// ----------------------------------------------------------------
// BMS_Service_ProcessData
//
// Processes raw measurement data to compute SOC, SOH, and other battery health metrics.
//
// Inputs: None
// Output: Updates state estimates in `telemetry` and `bms_instance1/2`
// Significance: Converts low-level readings into useful information.
// ----------------------------------------------------------------
void BMS_Service_ProcessData(void);


// ----------------------------------------------------------------
// BMS_Service_UpdateCounters
//
// Updates internal timers or counters that track uptime, usage, etc.
//
// Input:
//   - delta_time: Time passed (in milliseconds) since the last update
//
// Output: Updates counters or accumulators in the system
// Significance: Useful for tracking time-based degradation, energy use, etc.
// ----------------------------------------------------------------
void BMS_Service_UpdateCounters(uint32_t delta_time);


// ----------------------------------------------------------------
// BMS_Service_HandleLowPowerCondition
//
// Determines if the system should enter low power mode based on battery condition.
//
// Input:
//   - low_power_mode: pointer to a flag (0 or 1) indicating low-power state
//
// Output: Sets *low_power_mode to 1 if conditions require entering low power
// Significance: Ensures energy efficiency when the battery is critically low.
// ----------------------------------------------------------------
void BMS_Service_HandleLowPowerCondition(uint8_t *low_power_mode);


// ----------------------------------------------------------------
// BMS_Service_HandleLowPowerMode
//
// Performs the necessary operations when in low-power mode (e.g., shut down heaters,
// turn off peripherals, reduce sampling rate).
//
// Input:
//   - low_power_mode: pointer to flag indicating if we're in low power
//
// Output: Executes power-saving steps
// Significance: Preserves battery life and protects hardware.
// ----------------------------------------------------------------
void BMS_Service_HandleLowPowerMode(uint8_t *low_power_mode);


// ----------------------------------------------------------------
// BMS_Service_HandleFlashStorage
//
// Periodically stores current telemetry to flash memory for persistence across resets.
//
// Inputs: None
// Output: Saves data into flash
// Significance: Prevents telemetry loss during power failure or unexpected reset.
// ----------------------------------------------------------------
void BMS_Service_HandleFlashStorage(void);


// ====================== EXTERNAL VARIABLES ======================

// These are shared variables that are defined in the `.c` file and used across the system

extern BQ76920_t bms_instance1;  // First BMS IC instance
extern BQ76920_t bms_instance2;  // Optional second BMS IC for redundancy or multi-stack
extern TelemetryData telemetry;  // Global structure holding current telemetry data

#endif /* BMS_SERVICE_H */



/* */
