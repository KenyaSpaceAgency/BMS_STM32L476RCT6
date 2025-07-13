//File: Temperature.c

#include "Temperature.h"     // Contains definitions for TMP100 addresses, heater pins, and function prototypes
#include <math.h>            // Provides math functions like logf() (used in PID calculations)
#include <stdio.h>           // Includes functions for printing (used in Log_Error for debugging)
#include <stdlib.h>          // Includes utility functions like abs() (not used here but included for completeness)
#include "delay.h"

// External variables declared in other files (e.g., main.c)
extern TelemetryData telemetry;         // Global structure to store battery data, including heater states
extern I2C_HandleTypeDef hi2c1;         // I2C interface for TMP100 sensor 1 (pins PB6/PB7 on STM32L476)
extern I2C_HandleTypeDef hi2c2;         // I2C interface for TMP100 sensor 2 (pins PB10/PB11, not used here)
extern BQ76920_t bms_instance1;
extern BQ76920_t bms_instance2;
// Function: Temperature_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes the temperature system
// Significance:
//   - Sets up the TMP100 temperature sensors and the PID controller for heater management.
//     Called at system startup in BMS_Service.c to prepare temperature monitoring and control.
void Temperature_Init(void) {
    // Configure the first TMP100 sensor at address 0x48 (defined in Temperature.h)
    TMP100_Configure(&hi2c1, TMP100_IC1_ADDR);  // Set up sensor 1 on I2C1
    // Configure the second TMP100 sensor at address 0x49 (defined in Temperature.h)
    TMP100_Configure(&hi2c2, TMP100_IC2_ADDR);  // Set up sensor 2 on I2C1 (note: hi2c1, not hi2c2)
    // Initialize the PID controller for heater control
    PID_Init();                                // Reset PID variables and turn off heaters
}

// Function: TMP100_ReadTemperature
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface to communicate with the TMP100
//   - address: A uint8_t, the I2C address of the TMP100 sensor (0x48 or 0x49, shifted left by 1)
// Output:
//   - Returns a float, the temperature in Celsius, or -273.15 if an error occurs
// Significance:
//   - Reads the temperature from a TMP100 sensor over I2C, used to monitor battery pack temperature
//     in BMS_Service.c. Returns absolute zero (-273.15°C) if communication fails (TMP100 datasheet, Section 7.5.1).
float TMP100_ReadTemperature(I2C_HandleTypeDef *hi2c, uint8_t address) {
    uint8_t temp_reg = 0x00;
    uint8_t data[2];

    if (HAL_I2C_Master_Transmit(hi2c, address, &temp_reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "TMP100 TX failed (addr 0x%02X)", address);
        return -273.15f;
    }

    if (HAL_I2C_Master_Receive(hi2c, address, data, 2, HAL_MAX_DELAY) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "TMP100 RX failed (addr 0x%02X)", address);
        return -273.15f;
    }

    int16_t raw_temp = ((int16_t)data[0] << 4) | (data[1] >> 4);
    if (raw_temp & 0x800) raw_temp |= 0xF000;

    return raw_temp * 0.0625f;
}

// Static variables for PID controller (accessible only in this file)
static float integral = 0.0f;            // Stores the accumulated error over time for PID control
static float previous_error = 0.0f;      // Stores the error from the previous PID loop
static uint8_t heater_1_enabled = 0;     // Tracks if Heater 1 is on (1) or off (0)
static uint8_t heater_2_enabled = 0;     // Tracks if Heater 2 is on (1) or off (0)

// Function: PowerSwitch_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), initializes heater GPIO pins
// Significance:
//   - Turns off both heaters by setting their GPIO pins to low, called during PID initialization
//     to ensure heaters start in a safe state.
static void PowerSwitch_Init(void) {
    // Set Heater 1 GPIO pin (defined in main.h) to low (off)
    HAL_GPIO_WritePin(HEATER1_GPIO_Port, HEATER1_Pin, GPIO_PIN_RESET);
    // Set Heater 2 GPIO pin (defined in main.h) to low (off)
    HAL_GPIO_WritePin(HEATER2_GPIO_Port, HEATER2_Pin, GPIO_PIN_RESET);
}

// Function: PowerSwitch_Control
// Inputs:
//   - heater: A uint8_t, the heater to control (HEATER_1 or HEATER_2, defined in Temperature.h)
//   - enable: A uint8_t, 1 to turn the heater on, 0 to turn it off
// Output:
//   - None (void), sets the heater’s GPIO pin and updates telemetry
// Significance:
//   - Controls the state of a heater (on or off) via GPIO and updates the telemetry structure,
//     used by PID control and manual heater functions to manage battery temperature.
static void PowerSwitch_Control(uint8_t heater, uint8_t enable) {
    // Set GPIO pin state to high (on) if enable is 1, or low (off) if enable is 0
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

    // Check which heater to control
    switch (heater) {
        case HEATER_1: // If controlling Heater 1
            // Set Heater 1 GPIO pin to the desired state
            HAL_GPIO_WritePin(HEATER1_GPIO_Port, HEATER1_Pin, state);
            // Update the local variable to track Heater 1’s state
            heater_1_enabled = enable;
            // Update the global telemetry structure with Heater 1’s state
            telemetry.heater1_state = enable;
            break;

        case HEATER_2: // If controlling Heater 2
            // Set Heater 2 GPIO pin to the desired state
            HAL_GPIO_WritePin(HEATER2_GPIO_Port, HEATER2_Pin, state);
            // Update the local variable to track Heater 2’s state
            heater_2_enabled = enable;
            // Update the global telemetry structure with Heater 2’s state
            telemetry.heater2_state = enable;
            break;

        default: // If an invalid heater ID is provided
            // Log an error message using the UART (defined in main.c)
        	Log_Message(BMS_MSG_LEVEL_ERROR,"PID: Invalid heater ID %d", heater);
            break;
    }
}

// Function: PID_Init
// Inputs:
//   - None (void)
// Output:
//   - None (void), resets PID variables and turns off heaters
// Significance:
//   - Initializes the PID controller by resetting error terms and turning off heaters,
//     called at startup to ensure a clean state for temperature control.
void PID_Init(void) {
    // Reset the accumulated error (integral term) to 0
    integral = 0.0f;
    // Reset the previous error (used for derivative term) to 0
    previous_error = 0.0f;
    // Set Heater 1’s state to off
    heater_1_enabled = 0;
    // Set Heater 2’s state to off
    heater_2_enabled = 0;
    // Turn off both heaters by setting their GPIO pins to low
    PowerSwitch_Init();  // Turn off both heaters
}

/**
 * PID (Proportional-Integral-Derivative) Control function for temperature regulation.
 *
 * This function takes two temperature readings (temp1 and temp2) as input, validates them,
 * and then applies PID control to adjust the heater state (on/off) to maintain a target temperature.
 *
 * @param temp1  First temperature reading
 * @param temp2  Second temperature reading
 */
void PID_Control(float temp1, float temp2) {
  // **Temperature Validation**
  // Check if each temperature reading is within a valid range (-100.0f to 150.0f)
  uint8_t temp1_valid = (temp1 >= -100.0f && temp1 <= 150.0f);
  uint8_t temp2_valid = (temp2 >= -100.0f && temp2 <= 150.0f);

  float temp; // variable to store the validated/averaged temperature

  // **Error Handling: Both Sensors Invalid**
  if (!temp1_valid && !temp2_valid) {
    // Log an error message with both temperatures
    Log_Message(BMS_MSG_LEVEL_ERROR, "PID: Both sensors invalid (T1: %.1f, T2: %.1f)", temp1, temp2);
    // Disable heaters and reset PID variables
    PowerSwitch_Control(HEATER_1, 0);
    PowerSwitch_Control(HEATER_2, 0);
    integral = 0.0f;
    previous_error = 0.0f;
    return; // exit the function
  }

  // **Temperature Selection**
  else if (temp1_valid && temp2_valid) {
    // Average both temperatures
    temp = (temp1 + temp2) / 2.0f;
    Log_Message(BMS_MSG_LEVEL_DEBUG, "PID: Using average temp (T1: %.1f, T2: %.1f, Avg: %.1f)", temp1, temp2, temp);
  } else if (temp1_valid) {
    // Use only temp1 if temp2 is invalid
    temp = temp1;
    Log_Message(BMS_MSG_LEVEL_WARNING, "PID: Using T1 only (T1: %.1f, T2 invalid: %.1f)", temp1, temp2);
  } else {
    // Use only temp2 if temp1 is invalid
    temp = temp2;
    Log_Message(BMS_MSG_LEVEL_WARNING, "PID: Using T2 only (T1 invalid: %.1f, T2: %.1f)", temp1, temp2);
  }

  // **Emergency Shutdown (Temperature Exceeded)**
  if (temp >= TEMP_UPPER_LIMIT) {
    // If heaters are currently enabled, shut them down and reset PID variables
    if (heater_1_enabled || heater_2_enabled) {
      PowerSwitch_Control(HEATER_1, 0);
      PowerSwitch_Control(HEATER_2, 0);
      integral = 0.0f;
      previous_error = 0.0f;
      Log_Message(BMS_MSG_LEVEL_ERROR, "PID: Emergency shutdown (T1: %.1f, T2: %.1f)", temp1, temp2);
    }
    return; // exit the function
  }

  // **PID Calculation**
  // Calculate the error between the target temperature and the current temperature
  float error = TARGET_TEMP - temp;

  // Update the integral term ( accumulate error over time )
  integral += error * DT;
  // Limit the integral term to prevent windup
  if (integral > 100.0f) integral = 100.0f;
  if (integral < -100.0f) integral = -100.0f;

  // Calculate the derivative term ( rate of change of error )
  float derivative = (error - previous_error) / DT;

  // Calculate the PID output ( weighted sum of P, I, and D terms )
  float output = KP * error + KI * integral + KD * derivative;

  // **Heater State Determination**
  uint8_t new_heater_state = 0;
  if (output > 0 && temp < 15.0f) {
    // Enable heaters if output is positive and temperature is below 15°C
    new_heater_state = 1;
  } else if (output <= 0 || temp > 35.0f) {
    // Disable heaters if output is non-positive or temperature exceeds 35°C
    new_heater_state = 0;
  } else {
    // Otherwise, maintain the current heater state
    new_heater_state = heater_1_enabled;
  }

  // **Apply New Heater State (if changed)**
  if (new_heater_state != heater_1_enabled || new_heater_state != heater_2_enabled) {
    PowerSwitch_Control(HEATER_1, new_heater_state);
    PowerSwitch_Control(HEATER_2, new_heater_state);
    Log_Message(BMS_MSG_LEVEL_INFO, "PID: Heaters %s | T1: %.1f, T2: %.1f | Out: %.2f | Err: %.2f",
                new_heater_state ? "ON" : "OFF", temp1, temp2, output, error);
  }

  // **Update Previous Error**
  previous_error = error;
}
// Function: TMP100_Configure
// Inputs:
//   - hi2c: A pointer to an I2C_HandleTypeDef, the I2C interface for communication
//   - address: A uint8_t, the I2C address of the TMP100 sensor (0x48 or 0x49, shifted left by 1)
// Output:
//   - None (void), configures the TMP100 sensor
// Significance:
//   - Sets up the TMP100 sensor’s resolution and mode, called during initialization to
//     prepare the sensor for accurate temperature readings (TMP100 datasheet, Section 7.5.2).
void TMP100_Configure(I2C_HandleTypeDef *hi2c, uint8_t address) {
    // Define the configuration register address (0x01 for TMP100, datasheet Section 7.5.2)
    uint8_t config_reg = TMP100_CONFIG_REG;
    // Set configuration: 12-bit resolution, 4-fault queue, active-low comparator mode
    // 0x68 = 01101000 (12-bit, 4 faults, comparator mode, datasheet Table 5)
    uint8_t config_data[2] = {config_reg, 0x68}; // Array with register address and config value

    // Send configuration data to the TMP100 sensor
    if (HAL_I2C_Master_Transmit(hi2c, address, config_data, 2, HAL_MAX_DELAY) != HAL_OK) {
        // Log an error if configuration fails
    	Log_Message(BMS_MSG_LEVEL_ERROR,"TMP100 config write failed (address 0x%02X)", address);
    }

    // Wait 10ms to ensure the TMP100 applies the configuration
    SoftwareDelay(10); // Wait for settings to take effect
}

// Function: Heater1_On
// Inputs:
//   - None (void)
// Output:
//   - None (void), turns Heater 1 on
// Significance:
//   - Manually turns on Heater 1 for testing or debugging, bypassing PID control.
void Heater1_On(void) {
    // Turn on Heater 1 and log the action
    PowerSwitch_Control(HEATER_1, 1);
    // Log a message to indicate Heater 1 was turned on
    Log_Message(BMS_MSG_LEVEL_ERROR,"Heater 1 manually turned ON");
}

// Function: Heater2_On
// Inputs:
//   - None (void)
// Output:
//   - None (void), turns Heater 2 on
// Significance:
//   - Manually turns on Heater 2 for testing or debugging, bypassing PID control.
void Heater2_On(void) {
    // Turn on Heater 2 and log the action
    PowerSwitch_Control(HEATER_2, 1);
    // Log a message to indicate Heater 2 was turned on
    Log_Message(BMS_MSG_LEVEL_ERROR,"Heater 2 manually turned ON");
}

// Function: Heater1_Off
// Inputs:
//   - None (void)
// Output:
//   - None (void), turns Heater 1 off
// Significance:
//   - Manually turns off Heater 1 for testing or debugging, bypassing PID control.
void Heater1_Off(void) {
    // Turn off Heater 1 and log the action
    PowerSwitch_Control(HEATER_1, 0);
    // Log a message to indicate Heater 1 was turned off
    Log_Message(BMS_MSG_LEVEL_ERROR,"Heater 1 manually turned OFF");
}

// Function: Heater2_Off
// Inputs:
//   - None (void)
// Output:
//   - None (void), turns Heater 2 off
// Significance:
//   - Manually turns off Heater 2 for testing or debugging, bypassing PID control.
void Heater2_Off(void) {
    // Turn off Heater 2 and log the action
    PowerSwitch_Control(HEATER_2, 0);
    // Log a message to indicate Heater 2 was turned off
    Log_Message(BMS_MSG_LEVEL_ERROR,"Heater 2 manually turned OFF");
}
