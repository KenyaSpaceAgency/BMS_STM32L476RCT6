void BMS_Service_EnableHeater1(void);
void BMS_Service_DisableHeater1(void);
void BMS_Service_EnableHeater2(void);
void BMS_Service_DisableHeater2(void);  //dont use these functions, use these functions // Include header files needed for the code to work
#include "Temperature.h"     // Contains definitions for TMP100 addresses, heater pins, and function prototypes
#include <math.h>            // Provides math functions like logf() (used in PID calculations)
#include <stdio.h>           // Includes functions for printing (used in Log_Error for debugging)
#include <stdlib.h>          // Includes utility functions like abs() (not used here but included for completeness)

// External variables declared in other files (e.g., main.c)
extern TelemetryData telemetry;         // Global structure to store battery data, including heater states
extern I2C_HandleTypeDef hi2c1;         // I2C interface for TMP100 sensor 1 (pins PB6/PB7 on STM32L476)
extern I2C_HandleTypeDef hi2c2;         // I2C interface for TMP100 sensor 2 (pins PB10/PB11, not used here)

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
    // Define the temperature register address (0x00 for TMP100, datasheet Section 7.5.1)
    uint8_t temp_reg = 0x00;  // Address of the TMP100 temperature register
    // Array to store two bytes of temperature data from the sensor
    uint8_t data[2];          // Buffer to receive 2 bytes

    // Send the register address (0x00) to tell the TMP100 we want to read temperature
    if (HAL_I2C_Master_Transmit(hi2c, address, &temp_reg, 1, HAL_MAX_DELAY) != HAL_OK)
        // If I2C communication fails, return -273.15°C (absolute zero) to indicate an error
        return -273.15f;  // Return absolute zero on error

    // Read 2 bytes of temperature data from the TMP100
    if (HAL_I2C_Master_Receive(hi2c, address, data, 2, HAL_MAX_DELAY) != HAL_OK)
        // If reading fails, return -273.15°C to indicate an error
        return -273.15f;

    // Combine the two bytes into a 12-bit signed value (datasheet Section 7.5.1.2)
    // Shift first byte left by 4 and combine with second byte’s upper 4 bits
    int16_t raw_temp = ((int16_t)data[0] << 4) | (data[1] >> 4);
    // If the temperature is negative (bit 11 is 1), sign-extend to 16 bits
    if (raw_temp & 0x800) raw_temp |= 0xF000; // Sign-extend negative values

    // Convert raw value to Celsius (TMP100 resolution is 0.0625°C per bit, datasheet Section 7.5.1.2)
    return raw_temp * 0.0625f; // Multiply by 0.0625 to get temperature in Celsius
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

// Function: PID_Control
// Inputs:
//   - temp: An int16_t, the current temperature in Celsius (scaled, not float, to match input)
// Output:
//   - None (void), adjusts heater states to control temperature
// Significance:
//   - Implements a PID controller to turn heaters on or off based on the difference between
//     the current temperature and a target (25°C), ensuring batteries stay at optimal temperature
//     (used in BMS_Service.c).
void PID_Control(float temp) {
    // Check if temperature is invalid (outside -100°C to 150°C)
    if (temp < -100 || temp > 150) {
        // Log an error message if temperature is unrealistic
    	Log_Message(BMS_MSG_LEVEL_ERROR,"PID: Skipping control, invalid temp: %d", temp);
        // Exit the function to avoid using bad data
        return;
    }

    // Safety check: if temperature exceeds upper limit (60°C, defined in Temperature.h)
    if (temp >= TEMP_UPPER_LIMIT) {
        // If either heater is on
        if (heater_1_enabled || heater_2_enabled) {
            // Turn off Heater 1
            PowerSwitch_Control(HEATER_1, 0);
            // Turn off Heater 2
            PowerSwitch_Control(HEATER_2, 0);
            // Reset the integral term to prevent windup
            integral = 0.0f;
            // Reset the previous error
            previous_error = 0.0f;
            // Log an emergency shutdown message
            Log_Message(BMS_MSG_LEVEL_ERROR,"PID: Emergency shutdown. Temp = %d°C", temp);
        }
        // Exit to prevent further processing
        return;
    }

    // Calculate the error: difference between target temperature (25°C) and current temperature
    float error = TARGET_TEMP - temp;    // Proportional error term
    // Add error times time step (DT=1.0s) to the integral term
    integral += error * DT;              // Accumulate integral

    // Prevent integral windup (integral getting too large)
    // Clamp integral to ±100 to avoid runaway values
    if (integral > 100.0f) integral = 100.0f;
    if (integral < -100.0f) integral = -100.0f;

    // Calculate derivative term: change in error over time
    float derivative = (error - previous_error) / DT;  // Change in error
    // Calculate PID output using proportional, integral, and derivative terms
    // KP, KI, KD are tuning constants from Temperature.h
    float output = KP * error + KI * integral + KD * derivative; // PID formula

    // Decide whether to turn heaters on or off
    uint8_t new_heater_state = 0;
    // If output is positive and temperature is cold (<15°C), turn heaters on
    if (output > 0 && temp < 15) {
        new_heater_state = 1; // Force ON if very cold
    // If output is zero or negative, or temperature is hot (>35°C), turn heaters off
    } else if (output <= 0 || temp > 35) {
        new_heater_state = 0; // Turn OFF if hot or unneeded
    // Otherwise, maintain the current heater state
    } else {
        new_heater_state = heater_1_enabled; // Maintain previous state
    }

    // If the new state differs from the current state for either heater
    if (new_heater_state != heater_1_enabled || new_heater_state != heater_2_enabled) {
        // Update Heater 1 to the new state
        PowerSwitch_Control(HEATER_1, new_heater_state);
        // Update Heater 2 to the new state
        PowerSwitch_Control(HEATER_2, new_heater_state);
        // Log the heater state change, temperature, PID output, and error
        Log_Message(BMS_MSG_LEVEL_ERROR,"PID: Heaters %s | Temp: %d°C | Out: %.2f | Err: %.2f",
                  new_heater_state ? "ON" : "OFF", temp, output, error);
    }

    // Save current error for the next loop’s derivative calculation
    previous_error = error;  // Save error for next loop
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
    HAL_Delay(10); // Wait for settings to take effect
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
