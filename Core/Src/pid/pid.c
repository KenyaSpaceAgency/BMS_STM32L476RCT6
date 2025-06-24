/* Role of pid.c in the BMS Project:
 * This file implements a PID controller to regulate the temperature of the battery pack
 * in the BMS for a CubeSat. It controls two heaters using TPS22810 power switches on GPIO
 * pins (PB9 for Heater 1, PB8 for Heater 2), maintaining safe battery temperatures based
 * on NTC sensor readings.
 */

/* Importance of pid.c in the BMS Project:
 * - Regulates Temperature: Keeps the battery at an optimal temperature (around 25°C),
 *   preventing damage from extreme cold or heat, critical for lithium-ion batteries.
 * - Ensures Safety: Disables heaters at high temperatures (e.g., 60°C), protecting the
 *   battery and electronics.
 * - Extends Battery Life: Maintains consistent temperatures, reducing cell stress and
 *   improving longevity.
 * - Integrates with BMS: Uses temperature data from BQ76920 ICs and logs heater states,
 *   sent to the OBC via RS485 using SSP for remote monitoring.
 * - Saves Power: Efficiently controls heaters with PID, minimizing energy use in the EPS.
 */

/* Objective of pid.c in the BMS Project:
 * The objective is to provide functions to initialize and control a PID-based heater
 * system, ensuring the battery pack’s temperature stays within safe limits. This supports
 * battery performance, safety, and longevity, while enabling diagnostic logging and
 * communication with the OBC, contributing to the BMS’s reliable power management.
 */

/* Include PID header file for function declarations and constants */
#include "pid.h"
/* Include standard I/O library for string formatting with snprintf */
#include <stdio.h>
/* Include standard library for abs function to calculate differences */
#include <stdlib.h>

/* Declare static variable to store the integral term of the PID controller */
static float integral = 0.0;
/* Declare static variable to store the previous error for derivative calculation */
static float previous_error = 0.0;
/* Declare static variable to track if Heater 1 is enabled (1) or disabled (0) */
static uint8_t heater_1_enabled = 0;
/* Declare static variable to track if Heater 2 is enabled (1) or disabled (0) */
static uint8_t heater_2_enabled = 0;

/* Define function to initialize TPS22810 power switches for heaters
 * Inputs: None
 * Returns: void, meaning it returns nothing; it configures hardware directly by
 *          setting GPIO pins to ensure heaters are off at startup.
 * What it does: Sets up GPIO pins PB9 (Heater 1) and PB8 (Heater 2) as outputs
 *               to control the TPS22810 switches, ensuring both heaters are off.
 */
static void PowerSwitch_Init(void) {
    HAL_GPIO_WritePin(POWER_SWITCH_1_EN_PORT, POWER_SWITCH_1_EN_PIN, GPIO_PIN_RESET); /* Set PB9 (Heater 1) low to disable switch */
    HAL_GPIO_WritePin(POWER_SWITCH_2_EN_PORT, POWER_SWITCH_2_EN_PIN, GPIO_PIN_RESET); /* Set PB8 (Heater 2) low to disable switch */
}

/* Define function to control TPS22810 power switches for heaters
 * Inputs:
 * - heater: Identifier (uint8_t) for the heater to control (HEATER_1 or HEATER_2),
 *           specifies which heater’s switch to toggle.
 * - enable: Flag (uint8_t), 1 to turn the heater on, 0 to turn it off, controlling
 *           the switch’s state.
 * Returns: void, meaning it returns nothing; it sets GPIO pins to control the switch
 *          and logs errors for invalid inputs.
 * What it does: Sets the specified heater’s GPIO pin (PB9 or PB8) high or low to
 *               enable or disable the TPS22810 switch, logging errors for invalid
 *               heater IDs.
 */
static void PowerSwitch_Control(uint8_t heater, uint8_t enable) {
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET; /* Set pin state to high (SET) if enable is 1, low (RESET) if 0 */
    switch (heater) { /* Check heater ID to select action */
        case HEATER_1: /* If heater is HEATER_1 */
            HAL_GPIO_WritePin(POWER_SWITCH_1_EN_PORT, POWER_SWITCH_1_EN_PIN, state); /* Set PB9 to control Heater 1 switch */
            break; /* Exit switch statement */
        case HEATER_2: /* If heater is HEATER_2 */
            HAL_GPIO_WritePin(POWER_SWITCH_2_EN_PORT, POWER_SWITCH_2_EN_PIN, state); /* Set PB8 to control Heater 2 switch */
            break; /* Exit switch statement */
        default: /* If heater ID is invalid */
            Log_Error("Invalid heater: %d", heater); /* Log error message with heater ID */
            break; /* Exit switch statement */
    }
}

/* Define function to initialize the PID controller
 * Inputs: None
 * Returns: void, meaning it returns nothing; it initializes variables and hardware
 *          for PID control.
 * What it does: Resets PID variables (integral, previous error) and heater states,
 *               and initializes TPS22810 switches to ensure heaters are off.
 */
void PID_Init(void)
{
    integral = 0.0; /* Set integral term to zero */
    previous_error = 0.0; /* Set previous error to zero */
    heater_1_enabled = 0; /* Set Heater 1 state to disabled */
    heater_2_enabled = 0; /* Set Heater 2 state to disabled */
    PowerSwitch_Init(); /* Call function to initialize TPS22810 switches */
}

/* Define function to control heaters using PID output
 * Inputs:
 * - temp: Current temperature (int16_t) in degrees Celsius, typically the lowest
 *         of NTC-1 or NTC-2 sensor readings, used to calculate the PID error.
 * Returns: void, meaning it returns nothing; it adjusts heater states via GPIO pins
 *          and logs state changes.
 * What it does: Calculates PID output based on the temperature error, toggles heaters
 *               on or off using TPS22810 switches, and logs state changes to ensure
 *               the battery stays within safe temperature limits.
 */
void PID_Control(int16_t temp)
{
    if (temp >= TEMP_UPPER_LIMIT) { /* Check if temperature is at or above 60°C */
        if (heater_1_enabled || heater_2_enabled) { /* Check if either heater is enabled */
            PowerSwitch_Control(HEATER_1, 0); /* Disable Heater 1 by setting PB9 low */
            PowerSwitch_Control(HEATER_2, 0); /* Disable Heater 2 by setting PB8 low */
            heater_1_enabled = 0; /* Update Heater 1 state to disabled */
            heater_2_enabled = 0; /* Update Heater 2 state to disabled */
            integral = 0.0; /* Reset integral term to zero */
            previous_error = 0.0; /* Reset previous error to zero */
            char message[56]; /* Create a buffer for log message */
            snprintf(message, sizeof(message), "Heaters disabled, Temp: %dC", temp); /* Format log message with temperature */
            Log_Error(message); /* Log heater disabled message */
        }
        return; /* Exit function to prevent further processing */
    }
    float error = TARGET_TEMP - temp; /* Calculate error as target (25°C) minus current temperature */
    integral += error * DT; /* Update integral term by adding error times time step */
    float derivative = (error - previous_error) / DT; /* Calculate derivative as change in error divided by time step */
    float output = KP * error + KI * integral + KD * derivative; /* Compute PID output using proportional, integral, and derivative terms */
    uint8_t new_heater_state; /* Create variable to store new heater state */
    if (output > 0 && temp < 15) { /* Check if PID output is positive and temperature is below 15°C */
        new_heater_state = 1; /* Set state to enable heaters */
    } else if (output <= 0 || temp > 35) { /* Check if PID output is non-positive or temperature exceeds 35°C */
        new_heater_state = 0; /* Set state to disable heaters */
    } else { /* If conditions are not met */
        new_heater_state = heater_1_enabled; /* Keep current heater state */
    }
    if (new_heater_state != heater_1_enabled || new_heater_state != heater_2_enabled) { /* Check if heater state needs to change */
        PowerSwitch_Control(HEATER_1, new_heater_state); /* Update Heater 1 state via PB9 */
        PowerSwitch_Control(HEATER_2, new_heater_state); /* Update Heater 2 state via PB8 */
        heater_1_enabled = new_heater_state; /* Update Heater 1 enabled status */
        heater_2_enabled = new_heater_state; /* Update Heater 2 enabled status */
        char message[64]; /* Create a buffer for log message */
        snprintf(message, sizeof(message), "Heaters %s, Temp: %dC, PID Output: %.2f, Error: %.2f",
                 new_heater_state ? "enabled" : "disabled", temp, output, error); /* Format log message with state, temp, output, and error */
        Log_Error(message); /* Log heater state change message */
    }
    previous_error = error; /* Update previous error for next derivative calculation */
}
