
#ifndef __MAIN_H
#define __MAIN_H

// Prevent multiple inclusions of this header file
#ifdef __cplusplus
extern "C" { // Ensure C linkage for C++ compatibility
#endif

// Include necessary STM32 HAL and project-specific headers
#include "stm32l4xx_hal.h" // STM32L4 HAL driver
#include "stm32l4xx_hal_tim.h" // STM32L4 HAL timer driver
#include <string.h> // Standard string operations
#include <stdio.h> // Standard I/O functions
#include <stdarg.h> // Variable argument functions
#include "BQ76920.h" // BQ76920 BMS IC driver
#include "kalman_filter.h" // Kalman filter for SOC/SOH calculations
#include "Temperature.h" // Temperature control and measurement


// Flash memory configuration for STM32L476RCT6 (256KB Flash)
#define FLASH_BASE_ADDRESS 0x0803F800 // Base address of last 2KB flash page
#define FLASH_TELEMETRY_ADDRESS (FLASH_BASE_ADDRESS + 0x100) // Telemetry storage offset

// Thermistor parameters for temperature calculations
#define R_NOMINAL 10000.0f // Nominal resistance of thermistor at 25°C
#define T_NOMINAL 25.0f // Nominal temperature for thermistor (in °C)
#define BETA_VALUE 3950.0f // Beta coefficient for thermistor

// Function prototype for error handling
void Error_Handler(void);

// GPIO pin definitions for hardware interfacing
#define LED_Pin GPIO_PIN_3 // LED indicator pin
#define LED_GPIO_Port GPIOC // LED GPIO port
#define RS4852_DE_Pin GPIO_PIN_1 // RS485 driver enable pin
#define RS4852_DE_GPIO_Port GPIOA // RS485 DE GPIO port
#define USART2_TX_Pin GPIO_PIN_2 // USART2 transmit pin
#define USART2_TX_GPIO_Port GPIOA // USART2 TX GPIO port
#define BOOT2_Pin GPIO_PIN_7 // Boot pin for second BMS IC
#define BOOT2_GPIO_Port GPIOC // Boot2 GPIO port
#define ALERT2_Pin GPIO_PIN_12 // Alert pin for second BMS IC
#define ALERT2_GPIO_Port GPIOA // Alert2 GPIO port
#define BOOT_Pin GPIO_PIN_4 // Boot pin for first BMS IC
#define BOOT_GPIO_Port GPIOB // Boot GPIO port
#define ALERT_Pin GPIO_PIN_5 // Alert pin for first BMS IC
#define ALERT_GPIO_Port GPIOB // Alert GPIO port
#define HEATER2_Pin GPIO_PIN_8 // Heater 2 control pin
#define HEATER2_GPIO_Port GPIOB // Heater 2 GPIO port
#define HEATER1_Pin GPIO_PIN_9 // Heater 1 control pin
#define HEATER1_GPIO_Port GPIOB // Heater 1 GPIO port

// Structure to hold telemetry data for BMS system
typedef struct {
    uint16_t vcell_ic1[NUMBER_OF_CELLS]; // Cell voltages for first BMS IC (mV)
    uint16_t vcell_ic2[NUMBER_OF_CELLS]; // Cell voltages for second BMS IC (mV)
    uint16_t vpack_ic1; // Pack voltage for first BMS IC (mV)
    uint16_t vpack_ic2; // Pack voltage for second BMS IC (mV)
    int16_t current_ic1; // Pack current for first BMS IC (mA)
    int16_t current_ic2; // Pack current for second BMS IC (mA)
    float soc; // State of Charge (%)
    float soh; // State of Health (%)
    float pcb_temperature; // PCB temperature (°C)
    float pack_temperature_ic1; // Pack temperature from first BMS IC (°C)
    float pack_temperature_ic2; // Pack temperature from second BMS IC (°C)
    float die_temperature_ic1; // Die temperature of first BMS IC (°C)
    float die_temperature_ic2; // Die temperature of second BMS IC (°C)
    float thermistor_temperature_ic1; // Thermistor temperature for first BMS IC (°C)
    float thermistor_temperature_ic2; // Thermistor temperature for second BMS IC (°C)
    uint8_t heater1_state; // Heater 1 state (1 = ON, 0 = OFF)
    uint8_t heater2_state; // Heater 2 state (1 = ON, 0 = OFF)
    uint8_t balancing_active; // Cell balancing status (1 = active, 0 = inactive)
    uint8_t balancing_mask_ic1; // Balancing mask for first BMS IC
    uint8_t balancing_mask_ic2; // Balancing mask for second BMS IC
    uint8_t charge_immediately; // Flag to trigger immediate charging (1 = yes, 0 = no)
    uint8_t bms_online; // BMS online status (1 = online, 0 = offline)
    uint8_t error_flags[8]; // Array of error flags for fault conditions
    uint8_t ovrd_alert_ic1; // Override alert for first BMS IC
    uint8_t ovrd_alert_ic2; // Override alert for second BMS IC
    uint8_t device_xready_ic1; // Device ready status for first BMS IC
    uint8_t device_xready_ic2; // Device ready status for second BMS IC
    uint8_t load_present_ic1; // Load presence for first BMS IC (1 = present, 0 = absent)
    uint8_t load_present_ic2; // Load presence for second BMS IC (1 = present, 0 = absent)
    uint16_t charge_cycle_count; // Number of charge cycles
    uint32_t total_charge_time; // Total charge time (seconds)
    uint32_t total_discharge_time; // Total discharge time (seconds)
    uint32_t total_operating_time; // Total operating time (seconds)
    uint16_t raw_adc_gain_ic1; // Raw ADC gain for first BMS IC (µV/LSB)
    uint16_t raw_adc_offset_ic1; // Raw ADC offset for first BMS IC (mV)
    uint16_t raw_adc_gain_ic2; // Raw ADC gain for second BMS IC (µV/LSB)
    uint16_t raw_adc_offset_ic2; // Raw ADC offset for second BMS IC (mV)
    uint8_t i2c_comm_error_ic1; // I2C communication error flag for first BMS IC
    uint8_t i2c_comm_error_ic2; // I2C communication error flag for second BMS IC
    uint64_t sync_counter; // Synchronization counter for telemetry updates
    uint8_t sync_valid; // Synchronization validity flag (1 = valid, 0 = invalid)
} TelemetryData;

// Include logging header for system logging
#include "log.h"

#ifdef __cplusplus
} // End C linkage for C++ compatibility
#endif

#endif // End header guard
