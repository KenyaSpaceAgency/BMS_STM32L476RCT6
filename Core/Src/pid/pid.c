/*
 * pid.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "pid.h"
#include <stdio.h>        // For snprintf
#include <stdlib.h>       // For abs

/* Private variables ---------------------------------------------------------*/
static float integral = 0.0;
static float previous_error = 0.0;
static uint8_t heater_1_enabled = 0;
static uint8_t heater_2_enabled = 0;

/**
  * @brief  Initializes the TPS22810 power switches for heaters.
  * @param  None
  * @retval None
  * @note   - Configures PB9 (Heater 1) and PB8 (Heater 2) as outputs for EN/UVLO control (active high).
  *         - Assumes CT capacitors are set externally (e.g., 4700 pF for 957 µs rise time at 12 V).
  *         - Assumes QOD is configured externally (e.g., tied to VOUT for 500 Ω discharge).
  *         - Both switches are initially disabled.
  */
static void PowerSwitch_Init(void) {
    // Ensure switches are off at startup
    HAL_GPIO_WritePin(POWER_SWITCH_1_EN_PORT, POWER_SWITCH_1_EN_PIN, GPIO_PIN_RESET); // PB9 (Heater 1)
    HAL_GPIO_WritePin(POWER_SWITCH_2_EN_PORT, POWER_SWITCH_2_EN_PIN, GPIO_PIN_RESET); // PB8 (Heater 2)
}

/**
  * @brief  Controls the TPS22810 power switches to enable/disable heaters.
  * @param  heater: The heater to control (HEATER_1 or HEATER_2).
  * @param  enable: 1 to enable the switch, 0 to disable.
  * @retval None
  * @note   - Uses PB9 (Heater 1) and PB8 (Heater 2) to control the EN/UVLO pin of the TPS22810.
  *         - Logs errors if an invalid heater is specified.
  */
static void PowerSwitch_Control(uint8_t heater, uint8_t enable) {
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (heater) {
        case HEATER_1:
            HAL_GPIO_WritePin(POWER_SWITCH_1_EN_PORT, POWER_SWITCH_1_EN_PIN, state); // PB9
            break;
        case HEATER_2:
            HAL_GPIO_WritePin(POWER_SWITCH_2_EN_PORT, POWER_SWITCH_2_EN_PIN, state); // PB8
            break;
        default:
            Log_Error("Invalid heater: %d", heater);
            break;
    }
}

/**
  * @brief  Initializes the PID controller.
  * @retval None
  * @note   - Initializes PID variables.
  *         - Initializes TPS22810 power switches.
  */
void PID_Init(void)
{
    integral = 0.0;
    previous_error = 0.0;
    heater_1_enabled = 0;
    heater_2_enabled = 0;

    // Initialize TPS22810 power switches (ensures they are off at startup)
    PowerSwitch_Init();
}

/**
  * @brief  Controls the heaters using PID output to toggle TPS22810 switches.
  * @param  temp: Current temperature in degrees Celsius (lowest of NTC-1/NTC-2).
  * @retval None
  * @note   - Disables heaters via TPS22810 if temperature exceeds TEMP_UPPER_LIMIT (60°C).
  *         - Turns heaters on if PID output > 0 AND temp < 15°C (error > 10).
  *         - Turns heaters off if PID output <= 0 OR temp > 35°C (error < -10).
  *         - Manages TPS22810 switches (PB9 for Heater 1, PB8 for Heater 2) via GPIO.
  *         - Logs heater state changes.
  */
void PID_Control(int16_t temp)
{
    // Check if temperature exceeds upper limit
    if (temp >= TEMP_UPPER_LIMIT) {
        // Disable both heaters via TPS22810
        if (heater_1_enabled || heater_2_enabled) {
            PowerSwitch_Control(HEATER_1, 0); // PB9 (Heater 1)
            PowerSwitch_Control(HEATER_2, 0); // PB8 (Heater 2)
            heater_1_enabled = 0;
            heater_2_enabled = 0;

            // Reset PID terms
            integral = 0.0;
            previous_error = 0.0;

            // Log state change
            char message[56];
            snprintf(message, sizeof(message), "Heaters disabled, Temp: %dC", temp);
            Log_Error(message);
        }
        return;
    }

    // Compute PID control
    float error = TARGET_TEMP - temp;
    integral += error * DT;
    float derivative = (error - previous_error) / DT;
    float output = KP * error + KI * integral + KD * derivative;

    // Determine heater state based on PID output and temperature range
    uint8_t new_heater_state;
    if (output > 0 && temp < 15) { // error > 10 (25 - temp > 10 => temp < 15°C)
        new_heater_state = 1; // Turn heaters on
    } else if (output <= 0 || temp > 35) { // error < -10 (25 - temp < -10 => temp > 35°C)
        new_heater_state = 0; // Turn heaters off
    } else {
        new_heater_state = heater_1_enabled; // Maintain current state if conditions aren't met
    }

    // Update heater state via TPS22810 switches
    if (new_heater_state != heater_1_enabled || new_heater_state != heater_2_enabled) {
        PowerSwitch_Control(HEATER_1, new_heater_state); // PB9 (Heater 1)
        PowerSwitch_Control(HEATER_2, new_heater_state); // PB8 (Heater 2)
        heater_1_enabled = new_heater_state;
        heater_2_enabled = new_heater_state;

        // Log state change
        char message[56];
        snprintf(message, sizeof(message), "Heaters %s, Temp: %dC, PID Output: %.2f, Error: %.2f",
                 new_heater_state ? "enabled" : "disabled", temp, output, error);
        Log_Error(message);
    }

    previous_error = error;
}
