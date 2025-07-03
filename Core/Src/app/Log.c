/*
 * Log.c
 *
 *  Created on: Jul 2, 2025
 *      Author: yomue
 */


// log.c - Logging implementation for BMS project

#include "log.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;  // USART1 handle defined in main.c

#define LOG_BUFFER_SIZE 256

static const char *log_prefixes[] = {
    "",        // BMS_MSG_LEVEL_OFF
    "[E] ",    // BMS_MSG_LEVEL_ERROR
    "[W] ",    // BMS_MSG_LEVEL_WARNING
    "[I] ",    // BMS_MSG_LEVEL_INFO
    "[V] ",    // BMS_MSG_LEVEL_VERBOSE
    "[D] "     // BMS_MSG_LEVEL_DEBUG
};

void Log_Message(BMS_LogLevel level, const char *format, ...) {
    if (level == BMS_MSG_LEVEL_OFF) return;

    char buffer[LOG_BUFFER_SIZE];
    int offset = snprintf(buffer, LOG_BUFFER_SIZE, "%s", log_prefixes[level]);

    if (offset < 0 || offset >= LOG_BUFFER_SIZE) return;

    va_list args;
    va_start(args, format);
    vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, format, args);
    va_end(args);

    size_t msg_len = strnlen(buffer, LOG_BUFFER_SIZE);
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, msg_len, HAL_MAX_DELAY);
}

// Function: Log_Telemetry
// Inputs:
//   - level: BMS_LogLevel, the logging level (e.g., BMS_MSG_LEVEL_INFO)
//   - telemetry: Pointer to TelemetryData structure containing BMS data
// Output:
//   - None (void), logs all telemetry fields to USART1
// Significance:
//   - Formats and logs every field of the TelemetryData structure to USART1
//     (PA9/PA10, 115200 baud) for monitoring and debugging in the BMS project
void Log_Telemetry(BMS_LogLevel level, const TelemetryData *telemetry) {
    // Skip logging if level is OFF
    if (level == BMS_MSG_LEVEL_OFF) return;

    // Log cell voltages for IC1
    for (uint8_t i = 0; i < NUMBER_OF_CELLS; i++) {
        Log_Message(level, "vcell_ic1[%d]: %u mV", i, telemetry->vcell_ic1[i]);
    }
    // Log cell voltages for IC2
    for (uint8_t i = 0; i < NUMBER_OF_CELLS; i++) {
        Log_Message(level, "vcell_ic2[%d]: %u mV", i, telemetry->vcell_ic2[i]);
    }
    // Log pack voltages
    Log_Message(level, "vpack_ic1: %u mV", telemetry->vpack_ic1);
    Log_Message(level, "vpack_ic2: %u mV", telemetry->vpack_ic2);
    // Log currents
    Log_Message(level, "current_ic1: %d mA", telemetry->current_ic1);
    Log_Message(level, "current_ic2: %d mA", telemetry->current_ic2);
    // Log state of charge and health
    Log_Message(level, "soc: %.2f %%", telemetry->soc);
    Log_Message(level, "soh: %.2f %%", telemetry->soh);
    // Log temperatures
    Log_Message(level, "pcb_temperature: %.2f C", telemetry->pcb_temperature);
    Log_Message(level, "pack_temperature_ic1: %.2f C", telemetry->pack_temperature_ic1);
    Log_Message(level, "pack_temperature_ic2: %.2f C", telemetry->pack_temperature_ic2);
    Log_Message(level, "die_temperature_ic1: %.2f C", telemetry->die_temperature_ic1);
    Log_Message(level, "die_temperature_ic2: %.2f C", telemetry->die_temperature_ic2);
    Log_Message(level, "thermistor_temperature_ic1: %.2f C", telemetry->thermistor_temperature_ic1);
    Log_Message(level, "thermistor_temperature_ic2: %.2f C", telemetry->thermistor_temperature_ic2);
    // Log heater states
    Log_Message(level, "heater1_state: %u", telemetry->heater1_state);
    Log_Message(level, "heater2_state: %u", telemetry->heater2_state);
    // Log balancing status
    Log_Message(level, "balancing_active: %u", telemetry->balancing_active);
    Log_Message(level, "balancing_mask_ic1: 0x%02X", telemetry->balancing_mask_ic1);
    Log_Message(level, "balancing_mask_ic2: 0x%02X", telemetry->balancing_mask_ic2);
    // Log charging and BMS status
    Log_Message(level, "charge_immediately: %u", telemetry->charge_immediately);
    Log_Message(level, "bms_online: %u", telemetry->bms_online);
    // Log error flags
    for (uint8_t i = 0; i < 8; i++) {
        Log_Message(level, "error_flags[%d]: 0x%02X", i, telemetry->error_flags[i]);
    }
    // Log alert statuses
    Log_Message(level, "ovrd_alert_ic1: %u", telemetry->ovrd_alert_ic1);
    Log_Message(level, "ovrd_alert_ic2: %u", telemetry->ovrd_alert_ic2);
    Log_Message(level, "device_xready_ic1: %u", telemetry->device_xready_ic1);
    Log_Message(level, "device_xready_ic2: %u", telemetry->device_xready_ic2);
    Log_Message(level, "load_present_ic1: %u", telemetry->load_present_ic1);
    Log_Message(level, "load_present_ic2: %u", telemetry->load_present_ic2);
    // Log cycle and time counters
    Log_Message(level, "charge_cycle_count: %u", telemetry->charge_cycle_count);
    Log_Message(level, "total_charge_time: %u s", telemetry->total_charge_time);
    Log_Message(level, "total_discharge_time: %u s", telemetry->total_discharge_time);
    Log_Message(level, "total_operating_time: %u s", telemetry->total_operating_time);
    // Log ADC calibration data
    Log_Message(level, "raw_adc_gain_ic1: %u", telemetry->raw_adc_gain_ic1);
    Log_Message(level, "raw_adc_offset_ic1: %u", telemetry->raw_adc_offset_ic1);
    Log_Message(level, "raw_adc_gain_ic2: %u", telemetry->raw_adc_gain_ic2);
    Log_Message(level, "raw_adc_offset_ic2: %u", telemetry->raw_adc_offset_ic2);
    // Log I2C communication errors
    Log_Message(level, "i2c_comm_error_ic1: %u", telemetry->i2c_comm_error_ic1);
    Log_Message(level, "i2c_comm_error_ic2: %u", telemetry->i2c_comm_error_ic2);
    // Log sync counter and validity
    Log_Message(level, "sync_counter: %llu", telemetry->sync_counter);
    Log_Message(level, "sync_valid: %u", telemetry->sync_valid);
}
