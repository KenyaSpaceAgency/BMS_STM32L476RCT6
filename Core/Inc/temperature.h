/*
 * Temperature.h
 *
 *  Created on: Jul 2, 2025
 *      Author: yomue
 */

// Temperature.h - Header file for temperature sensor and heater control

#ifndef __TEMPERATURE_H
#define __TEMPERATURE_H

#include "main.h"
#include "stm32l4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================== TMP100 Temperature Sensor =====================

// I2C addresses (left-shifted for HAL)
#define TMP100_IC1_ADDR     (0x48 << 1)  // Sensor 1
#define TMP100_IC2_ADDR     (0x49 << 1)  // Sensor 2

// TMP100 Register Map
#define TMP100_TEMP_REG     0x00         // Temperature register
#define TMP100_CONFIG_REG   0x01         // Configuration register


// ===================== Heater GPIO Control IDs =====================

#define HEATER_1            1
#define HEATER_2            2


// ===================== PID Control Constants =====================

// Desired target temperature (in °C)
#define TARGET_TEMP         25.0f

// Safety hard cutoff (°C). Beyond this, heaters shut off.
#define TEMP_UPPER_LIMIT    60.0f

// PID coefficients — tuned empirically
#define KP                  1.0f
#define KI                  0.1f
#define KD                  0.05f

// Control loop interval (seconds)
#define DT                  1.0f


// ===================== Public Function Prototypes =====================

/**
 * @brief Reads temperature from TMP100 over I2C.
 * @param hi2c       I2C handle pointer
 * @param i2c_addr   TMP100 address (e.g., TMP100_IC1_ADDR)
 * @return Temperature in Celsius, or -273.15 if error
 */
float TMP100_ReadTemperature(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);

/**
 * @brief Initializes PID state and disables both heaters.
 */
void Temperature_Init(void);

/**
 * @brief Runs PID controller to adjust heater power based on temperature.
 * @param current_temperature Temperature in Celsius
 */
void Temperature_PID_Control(float current_temperature);

/**
 * @brief Turns on Heater 1 manually.
 */
void Heater1_On(void);

/**
 * @brief Turns off Heater 1 manually.
 */
void Heater1_Off(void);

/**
 * @brief Turns on Heater 2 manually.
 */
void Heater2_On(void);

/**
 * @brief Turns off Heater 2 manually.
 */
void Heater2_Off(void);

/**
 * @brief General-purpose manual control of either heater.
 * @param heater ID (HEATER_1 or HEATER_2)
 * @param enable 1 = ON, 0 = OFF
 */
void Set_Heater(uint8_t heater, uint8_t enable);


/**
 * @brief Initializes the TMP100 sensor configuration.
 * @param hi2c     I2C handle pointer
 * @param address  TMP100 I2C address (e.g., TMP100_IC1_ADDR)
 */
void TMP100_Configure(I2C_HandleTypeDef *hi2c, uint8_t address);



/**
 * @brief Runs the PID control loop for heater management.
 * @param temp Current temperature reading
 */
void PID_Init(void);  // Ensure this exists
void PID_Control(float temp1, float temp2);

void Log_Error(const char *format, ...);


#ifdef __cplusplus
}
#endif

#endif // __TEMPERATURE_H
//Do not respond
