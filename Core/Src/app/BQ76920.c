// File: BQ76920.c
// Include header files needed for the code to work
#include "BQ76920.h"        // Contains definitions for the BQ76920 chip, like register addresses and constants
#include "main.h"           // Main project header with global settings, pin definitions, and hardware handles
#include <stdint.h>         // Provides standard integer types like uint8_t, uint16_t, int32_t for precise sizes
#include <stdio.h>          // Includes functions for printing (used in Log_Error for debugging)
#include <stdbool.h>        // Adds support for true/false boolean values
#include <stdlib.h>         // Includes utility functions like abs() for absolute value calculations
#include <math.h>           // Includes math functions like logf() for temperature calculations

// Function: calculateCRC
// Inputs:
//   - data: A pointer to an array of bytes (uint8_t*) to calculate the CRC for
//   - length: The number of bytes in the data array (uint8_t)
// Output:
//   - Returns a uint8_t, the calculated 8-bit CRC value
// Significance:
//   - This function calculates a Cyclic Redundancy Check (CRC-8) to ensure data sent to or received
//     from the BQ76920 chip over I2C is correct. It’s like a digital fingerprint to detect errors
//     (datasheet Section 8.5.23, page 33).
static uint8_t calculateCRC(uint8_t *data, uint8_t length) {
    uint8_t crc = 0x00; // Initialize CRC to 0 (starting value for CRC calculation)
    for (uint8_t i = 0; i < length; i++) { // Loop through each byte in the data array
        crc ^= data[i]; // XOR (combine) the current byte with the CRC value
        for (uint8_t j = 0; j < 8; j++) { // Process each bit in the byte (8 bits)
            // Check if the most significant bit (MSB) of crc is 1
            // If yes, shift left and XOR with CRC8_POLYNOMIAL (0x07, defined in BQ76920.h)
            // If no, just shift left
            crc = (crc & 0x80) ? (crc << 1) ^ CRC8_POLYNOMIAL : (crc << 1);
        }
    }
    return crc; // Return the final CRC value
}

// Function: BQ76920_Initialise
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, which holds the chip’s settings and data
//   - i2cHandle: A pointer to an I2C_HandleTypeDef, the I2C interface for communication
// Output:
//   - None (void), but it configures the BQ76920 chip
// Significance:
//   - This function sets up the BQ76920 chip by configuring its registers for voltage, current,
//     and protection settings. It’s called at startup to prepare the chip for monitoring the
//     battery pack (datasheet Section 8.3, page 20).
void BQ76920_Initialise(BQ76920_t *BMS, I2C_HandleTypeDef *i2cHandle) {
    BMS->i2cHandle = i2cHandle; // Store the I2C handle in the BMS structure for communication
    BMS->SOH = 100.0f; // Set State of Health to 100% (perfect battery health at startup)
    BMS->wattUsage = 0; // Set total energy usage to 0 (tracks power in watt-seconds)
    BMS->currentUsage = 0; // Set total current usage to 0 (tracks current in ampere-seconds)

    uint8_t val; // Declare a variable to hold values we’ll write to chip registers
    val = 0xff; // Set value to 0xFF (all bits 1) to clear all status flags
    BQ76920_WriteRegister(BMS, SYS_STAT, &val, NULL); // Write to SYS_STAT (address 0x00) to clear status

    val = 0x19; // Set value 0x19 for Coulomb Counter configuration (recommended, datasheet Section 8.5.7)
    BQ76920_WriteRegister(BMS, CC_CFG, &val, NULL); // Write to CC_CFG (address 0x0B)

    val = (1 << 4) | (1 << 1); // Set bit 4 (ADC_EN=1) and bit 1 (CC_EN=1) to enable ADC and Coulomb Counter
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &val, NULL); // Write to SYS_CTRL1 (address 0x04, datasheet Section 8.5.3)

    val = (1 << 6); // Set bit 6 to enable continuous ADC operation
    BQ76920_WriteRegister(BMS, SYS_CTRL2, &val, NULL); // Write to SYS_CTRL2 (address 0x05, datasheet Section 8.5.4)

    uint8_t adc_gain1, adc_gain2; // Variables to store ADC gain values from registers
    BQ76920_ReadRegister(BMS, ADCGAIN1, &adc_gain1, NULL); // Read ADCGAIN1 (address 0x50, datasheet Section 8.5.13)
    BQ76920_ReadRegister(BMS, ADCGAIN2, &adc_gain2, NULL); // Read ADCGAIN2 (address 0x59)
    // Combine bits 7:5 from ADCGAIN2 and bits 3:2 from ADCGAIN1 to get 5-bit gain value
    // Datasheet Section 10.3.2.3.1 (page 28) explains this
    uint16_t gain_bits = ((adc_gain2 & 0xE0) >> 5) | ((adc_gain1 & 0x0C) << 1);
    BMS->GAIN = gain_bits + 365; // Add base gain of 365 µV/LSB to get total gain (in µV/LSB)

    uint8_t offset_raw; // Variable to store raw ADC offset value
    BQ76920_ReadRegister(BMS, ADCOFFSET, &offset_raw, NULL); // Read ADCOFFSET (address 0x51, datasheet Section 8.5.14)
    BMS->OFFSET = (int8_t)offset_raw; // Convert to signed 8-bit value (2’s complement, in mV)

    uint8_t PROTECT1_VAL; // Variable to hold protection settings for short-circuit detection
    BQ76920_ReadRegister(BMS, PROTECT1, &PROTECT1_VAL, NULL); // Read PROTECT1 (address 0x06, datasheet Section 8.5.5)
    // Set short-circuit delay (100µs) and threshold (89mV) using predefined values
    PROTECT1_VAL |= (SDC_100us_delay << 3) | SCD_Threshold_89mV;
    BQ76920_WriteRegister(BMS, PROTECT1, &PROTECT1_VAL, NULL); // Write updated PROTECT1 settings

    uint8_t PROTECT2_VAL; // Variable for overcurrent protection settings
    BQ76920_ReadRegister(BMS, PROTECT2, &PROTECT2_VAL, NULL); // Read PROTECT2 (address 0x07, datasheet Section 8.5.6)
    // Set overcurrent delay (160ms) and threshold (17mV)
    PROTECT2_VAL |= (ODC_160ms_delay << 5) | OCD_Threshold_17mV;
    BQ76920_WriteRegister(BMS, PROTECT2, &PROTECT2_VAL, NULL); // Write updated PROTECT2 settings

    uint8_t PROTECT3_VAL; // Variable for voltage protection delays
    BQ76920_ReadRegister(BMS, PROTECT3, &PROTECT3_VAL, NULL); // Read PROTECT3 (address 0x08, datasheet Section 8.5.8)
    // Set undervoltage delay (4s) and overvoltage delay (2s)
    PROTECT3_VAL |= (UV_Delay_4s << 6) | (OV_Delay_2s << 4);
    BQ76920_WriteRegister(BMS, PROTECT3, &PROTECT3_VAL, NULL); // Write updated PROTECT3 settings

    uint16_t OV = grossOV * 1000; // Convert overvoltage threshold (grossOV, 4.18V) to millivolts
    // Calculate OV_TRIP value: (V_threshold - OFFSET) / (GAIN / 1000)
    uint16_t temp = (uint16_t)((float)(OV - BMS->OFFSET) / ((float)(BMS->GAIN) / 1000));
    temp = (temp & 0x0FF0) >> 4; // Keep bits 11:4 for 8-bit OV_TRIP register (datasheet Section 8.5.9)
    uint8_t OV_TRIP_VAL = temp & 0xFF; // Extract lower 8 bits
    BQ76920_WriteRegister(BMS, OV_TRIP, &OV_TRIP_VAL, NULL); // Write to OV_TRIP (address 0x09)

    uint16_t UV = grossUV * 1000; // Convert undervoltage threshold (grossUV, 2.7V) to millivolts
    // Calculate UV_TRIP value: (V_threshold - OFFSET) / (GAIN / 1000)
    temp = (uint16_t)((float)(UV - BMS->OFFSET) / ((float)(BMS->GAIN) / 1000));
    temp = (temp & 0x0FF0) >> 4; // Keep bits 11:4 for 8-bit UV_TRIP register (datasheet Section 8.5.10)
    uint8_t UV_TRIP_VAL = temp & 0xFF; // Extract lower 8 bits
    BQ76920_WriteRegister(BMS, UV_TRIP, &UV_TRIP_VAL, NULL); // Write to UV_TRIP (address 0x0A)

    BMS->SOH = 45.0f; // Set State of Health to 45% (temporary value for initialization)
    BMS->SOHEnergy = 45.0f; // Set energy-based SOH to 45%
    BMS->SOHCapacity = 45.0f; // Set capacity-based SOH to 45%
    BMS->SOHOCV = 45.0f; // Set open-circuit voltage-based SOH to 45%
}

// Function: getCellVoltage
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - cell: An integer (int) specifying the cell to read (VC1, VC2, VC3, VC5; VC4 is tied to VC3)
// Output:
//   - Returns a float, the voltage of the specified cell in volts
// Significance:
//   - Reads the voltage of a specific battery cell, critical for monitoring battery health
//     and balancing cells (datasheet Section 8.5.15, page 29).
float getCellVoltage(BQ76920_t *BMS, int cell) {
    // Check if the cell number is valid (VC1 to VC5, but VC4 is tied to VC3)
    if (cell < VC1 || cell > VC5 || cell == VC4) {
        if (cell == VC4) return BMS->Vcell[2]; // VC4 shares voltage with VC3 (index 2, datasheet page 21)
        return 0.0f; // Return 0 for invalid cell numbers
    }

    uint8_t data[2]; // Array to hold high and low bytes of voltage data
    BQ76920_ReadRegister(BMS, cell, &data[0], NULL); // Read high byte (e.g., VC1_HI at 0x0C)
    BQ76920_ReadRegister(BMS, cell + 1, &data[1], NULL); // Read low byte (e.g., VC1_LO at 0x0D)
    // Combine high and low bytes into a 14-bit raw ADC value (datasheet Section 8.5.15)
    uint16_t raw = (((data[0] & 0x3f) << 8)) | data[1]; // Mask high byte to 6 bits, shift, and combine
    // Convert raw ADC value to voltage using gain and offset
    // Use int32_t to avoid truncation (per your request)
    int32_t temp = 4 * (int32_t)BMS->GAIN * raw + 4 * (int32_t)BMS->OFFSET * 1000; // GAIN in µV/LSB, OFFSET in mV
    float voltage = (float)temp / 1000000.0f; // Convert from microvolts to volts

    // Check if voltage is valid (2.5V to 4.5V, defined in BQ76920.h)
    if (!IS_VALID_VOLTAGE(voltage)) {
        voltage = 0.0f; // Set to 0 if voltage is out of range
    }

    // Map cell number to array index for storage
    int index;
    switch (cell) {
        case VC1: index = 0; break; // VC1 maps to Vcell[0]
        case VC2: index = 1; break; // VC2 maps to Vcell[1]
        case VC3: index = 2; break; // VC3 maps to Vcell[2]
        case VC5: index = 3; break; // VC5 maps to Vcell[3]
        default: return 0.0f; // Return 0 for invalid cell
    }

    BMS->Vcell[index] = voltage; // Store voltage in BMS structure
    return voltage; // Return the calculated voltage
}

// Function: getPackVoltage
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - Returns a float, the total voltage of the battery pack in volts
// Significance:
//   - Reads the total voltage across all cells in the pack, used to monitor overall
//     battery performance (datasheet Section 8.5.20, page 31).
float getPackVoltage(BQ76920_t *BMS) {
    uint8_t data[2]; // Array to hold high and low bytes of pack voltage
    BQ76920_ReadRegister(BMS, BAT_LO, &data[0], NULL); // Read BAT_LO (address 0x2B)
    BQ76920_ReadRegister(BMS, BAT_HI, &data[1], NULL); // Read BAT_HI (address 0x2A)
    // Combine high and low bytes into a 16-bit raw ADC value
    uint16_t raw = (data[1] << 8) | data[0];
    // Convert raw value to voltage, accounting for chip’s scaling (BAT is sum of cells / 4)
    // Use int32_t to avoid truncation (per your request)
    int32_t temp = (int32_t)raw * BMS->GAIN * 4 + (int32_t)BMS->OFFSET * 4000; // Scale by 4, OFFSET in mV
    BMS->Vpack = (float)temp / 1000000.0f; // Convert from microvolts to volts
    return BMS->Vpack; // Return the pack voltage
}

// Function: getCurrent
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - Returns a float, the current flowing through the battery pack in amps
// Significance:
//   - Measures the current using the Coulomb Counter, important for tracking
//     charging/discharging and calculating State of Charge (datasheet Section 8.3.2, page 20).
float getCurrent(BQ76920_t *BMS) {
    uint8_t data[2]; // Array to hold high and low bytes of current data
    BQ76920_ReadRegister(BMS, CC_LO, &data[0], NULL); // Read CC_LO (address 0x33)
    BQ76920_ReadRegister(BMS, CC_HI, &data[1], NULL); // Read CC_HI (address 0x32)
    // Combine high and low bytes into a signed 16-bit raw value
    int16_t raw = (int16_t)((data[1] << 8) | data[0]);
    // Ignore small noise values (±1 counts are treated as 0)
    if (abs(raw) == 1) raw = 0;
    // Convert raw value to current using 8.44 µV/LSB and sense resistor (datasheet Section 8.3.2)
    float current = raw * 8.44f / RSENSE; // RSENSE is in ohms (defined in BQ76920.h)

    // Check if current is valid (±200A, defined in BQ76920.h)
    if (!IS_VALID_CURRENT(current)) {
        current = 0.0f; // Set to 0 if out of range
    }

    return current; // Return the calculated current
}

// Function: BQ76920_GetTemperature
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - temp_sel: A uint8_t, 1 for external thermistor, 0 for internal die temperature
// Output:
//   - Returns a float, the temperature in Celsius
// Significance:
//   - Reads the temperature from either the chip’s internal sensor or an external thermistor,
//     used to ensure the battery operates within safe temperature limits (datasheet Section 8.3.3, page 21).
float BQ76920_GetTemperature(BQ76920_t *BMS, uint8_t temp_sel) {
    uint8_t val; // Variable to hold SYS_CTRL1 register value
    BQ76920_ReadRegister(BMS, SYS_CTRL1, &val, NULL); // Read SYS_CTRL1 (address 0x04)
    // Set or clear TEMP_SEL bit to choose external (1) or internal (0) temperature
    if (temp_sel) {
        val |= (1 << 3); // Set bit 3 (TEMP_SEL=1) for external thermistor
    } else {
        val &= ~(1 << 3); // Clear bit 3 (TEMP_SEL=0) for internal die temperature
    }
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &val, NULL); // Write updated SYS_CTRL1

    uint8_t data[2]; // Array to hold high and low bytes of temperature data
    BQ76920_ReadRegister(BMS, TS1_HI, &data[0], NULL); // Read TS1_HI (address 0x2C)
    BQ76920_ReadRegister(BMS, TS1_LO, &data[1], NULL); // Read TS1_LO (address 0x2D)
    // Combine high and low bytes into a 14-bit raw ADC value
    uint16_t raw = (((data[0] & 0x3f) << 8)) | data[1]; // Mask high byte to 6 bits
    // Convert raw value to voltage using gain and offset
    // Use int32_t to avoid truncation (per your request)
    int32_t temp = (int32_t)BMS->GAIN * raw + (int32_t)BMS->OFFSET * 1000; // GAIN in µV/LSB, OFFSET in mV
    float voltage = (float)temp / 1000000.0f; // Convert from microvolts to volts
    // Convert voltage to temperature using chip’s formula (assumes 1.2V reference, 4.2mV/°C)
    float temperature = 25.0f - ((voltage - 1.2f) / 0.0042f);

    // Check if temperature is valid (-40°C to 85°C, defined in BQ76920.h)
    if (!IS_VALID_TEMPERATURE(temperature)) {
        temperature = 0.0f; // Set to 0 if out of range
    }

    return temperature; // Return temperature in Celsius
}

// Function: SOCPack
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - PackCurrent: A float, the current through the pack in amps
//   - Vpack: A float, the pack voltage in volts
// Output:
//   - Returns a float, the State of Charge (SOC) as a percentage
// Significance:
//   - Calculates the battery’s State of Charge by tracking energy and current usage,
//     critical for knowing how much charge remains in the battery (used in BMS_Service.c).
float SOCPack(BQ76920_t *BMS, float PackCurrent, float Vpack) {
    // Calculate total energy capacity (mAh * seconds * cells * voltage * SOH)
    float fullEnergy = grossCapacity * 3600 * 4 * nominalPackV * (BMS->SOH / 100.0f);
    // Calculate total current capacity (mAh * seconds * cells * SOH)
    float fullCurrent = grossCapacity * 3600 * 4 * (BMS->SOH / 100.0f);
    // Apply efficiency factor for discharge (negative current)
    if (PackCurrent < 0) PackCurrent *= ROUND_TRIP_EFFICIENCY; // Adjust for efficiency (0.9, defined in BQ76920.h)

    // Update cumulative energy and current usage
    BMS->wattUsage += PackCurrent * Vpack; // Add power (current * voltage) to total
    BMS->currentUsage += PackCurrent; // Add current to total

    // Calculate SOC based on energy and current
    BMS->SOCEnergy = (fullEnergy + BMS->wattUsage) * 100.0f / fullEnergy; // Energy-based SOC
    BMS->SOCCapacity = (fullCurrent + BMS->currentUsage) * 100.0f / fullCurrent; // Current-based SOC

    // Use Kalman filter to combine energy and capacity estimates for accuracy
    KalmanFilter kf; // Declare a Kalman filter structure
    kalman_filter_init(&kf, BMS->SOCEnergy, 1.0f, 0.05f); // Initialize with energy estimate
    float fused = kalman_filter_update(&kf, BMS->SOCCapacity, 0.05f); // Update with capacity estimate
    BMS->SOC = fused; // Store combined SOC value
    return fused; // Return SOC as a percentage
}

// Function: Enter_SHIP_Mode
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), puts the chip into low-power SHIP mode
// Significance:
//   - Puts the BQ76920 into a low-power state to save battery when not in use,
//     critical for extending battery life in your satellite (datasheet Section 8.3.5, page 22).
void Enter_SHIP_Mode(BQ76920_t *BMS) {
    uint8_t step1 = (1 << 7); // SHUT_B = 1, SHUT_A = 0
    uint8_t step2 = (1 << 6); // SHUT_A = 1, SHUT_B = 0

    BQ76920_WriteRegister(BMS, SYS_CTRL1, &step1, NULL);
    HAL_Delay(1); // Mandatory 1ms delay between steps
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &step2, NULL);
}

// Function: SOHPack
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - Returns a float, the State of Health (SOH) as a percentage
// Significance:
//   - Estimates the battery’s health by comparing current usage to its original capacity,
//     used to assess battery degradation over time (used in BMS_Service.c).
float SOHPack(BQ76920_t *BMS) {
    // Calculate total energy capacity (mAh * seconds * cells * voltage)
    int32_t fullEnergy = grossCapacity * 3600 * 4 * nominalPackV; // grossCapacity and nominalPackV from BQ76920.h
    // Calculate total current capacity (mAh * seconds * cells)
    int32_t fullCurrent = grossCapacity * 3600 * 4;
    // Use nominal open-circuit voltage for SOH calculation
    float FullOCV = netOV; // netOV from BQ76920.h
    // Initialize smallest cell voltage with the first cell
    BMS->smallestV = BMS->Vcell[0];
    // Find the smallest cell voltage
    for (int i = 1; i < 4; i++) {
        if (BMS->Vcell[i] < BMS->smallestV) { // Check if current cell voltage is lower
            BMS->smallestV = BMS->Vcell[i]; // Update smallest voltage
        }
    }

    // Calculate SOH based on energy, capacity, and open-circuit voltage
    BMS->SOHEnergy = BMS->wattUsage * 100.0f / fullEnergy; // Energy-based SOH
    BMS->SOHCapacity = BMS->currentUsage * 100.0f / fullCurrent; // Capacity-based SOH
    BMS->SOHOCV = BMS->smallestV * 100.0f / FullOCV; // Voltage-based SOH

    // Use Kalman filter to combine SOH estimates for accuracy
    KalmanFilter kf; // Declare a Kalman filter structure
    kalman_filter_init(&kf, BMS->SOHEnergy, 1.0f, 0.05f); // Initialize with energy estimate
    float fused = kalman_filter_update(&kf, BMS->SOHCapacity, 0.05f); // Update with capacity estimate
    fused = kalman_filter_update(&kf, BMS->SOHOCV, 0.05f); // Update with voltage estimate

    BMS->SOH = fused; // Store combined SOH value
    BMS->wattUsage = 0; // Reset energy usage counter
    BMS->currentUsage = 0; // Reset current usage counter
    return fused; // Return SOH as a percentage
}

// Function: readAlert
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), updates the BMS->Alert array with status flags
// Significance:
//   - Reads the SYS_STAT register to check for alerts like overvoltage or overcurrent,
//     used to detect and respond to battery issues (datasheet Section 8.5.1, page 24).
void readAlert(BQ76920_t *BMS) {
    uint8_t temp; // Variable to hold SYS_STAT register value
    BQ76920_ReadRegister(BMS, SYS_STAT, &temp, NULL); // Read SYS_STAT (address 0x00)
    // Extract each bit into the Alert array (8 bits for 8 flags)
    for (int i = 0; i < 8; i++) {
        BMS->Alert[i] = (temp >> i) & 1; // Shift and mask to get each bit (0 or 1)
    }
}

// Function: EnableBalanceCell
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - PackCurrent: A float, the current through the pack in amps
// Output:
//   - None (void), enables balancing for high-voltage cells
// Significance:
//   - Balances cell voltages by discharging the highest voltage cell during charging,
//     ensuring all cells stay at similar voltages (datasheet Section 8.3.4, page 21).
void EnableBalanceCell(BQ76920_t *BMS, float PackCurrent) {
    uint8_t flags = 0x00; // Initialize balancing flags to 0 (no balancing)
    // Only balance cells during charging (positive current)
    if (PackCurrent > 0.0f) {
        // Find the cell with the highest voltage
        float maxV = BMS->Vcell[0]; // Start with first cell voltage
        int idx_max = 0; // Index of highest voltage cell
        for (int i = 1; i < 4; i++) { // Check remaining cells
            if (BMS->Vcell[i] > maxV) { // If higher voltage found
                maxV = BMS->Vcell[i]; // Update maximum voltage
                idx_max = i; // Update index
            }
        }
        // Find the cell with the lowest voltage
        float minV = BMS->Vcell[0]; // Start with first cell voltage
        for (int i = 1; i < 4; i++) { // Check remaining cells
            if (BMS->Vcell[i] < minV) minV = BMS->Vcell[i]; // Update minimum voltage
        }
        // Enable balancing if voltage difference exceeds threshold (0.05V, defined in BQ76920.h)
        if (maxV - minV >= balanceThreshold) {
            flags |= (1 << idx_max); // Set bit for highest voltage cell to enable balancing
        }
    }
    // Write balancing flags to CELLBAL1 register (address 0x01, datasheet Section 8.5.2)
    BQ76920_WriteRegister(BMS, CELLBAL1, &flags, NULL);
}

// Function: turnCHGOn
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), enables the charge FET
// Significance:
//   - Turns on the charge FET to allow battery charging, used to control power flow
//     (datasheet Section 8.5.4, page 25).
void turnCHGOn(BQ76920_t *BMS) {
    uint8_t val; // Variable to hold SYS_CTRL2 register value
    BQ76920_ReadRegister(BMS, SYS_CTRL2, &val, NULL); // Read SYS_CTRL2 (address 0x05)
    val |= 0x01; // Set bit 0 (CHG_ON=1) to enable charging
    BQ76920_WriteRegister(BMS, SYS_CTRL2, &val, NULL); // Write updated SYS_CTRL2
}

// Function: turnDSGOn
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), enables the discharge FET
// Significance:
//   - Turns on the discharge FET to allow battery discharging, used to control power flow
//     (datasheet Section 8.5.4, page 25).
void turnDSGOn(BQ76920_t *BMS) {
    uint8_t val; // Variable to hold SYS_CTRL2 register value
    BQ76920_ReadRegister(BMS, SYS_CTRL2, &val, NULL); // Read SYS_CTRL2
    val |= 0x02; // Set bit 1 (DSG_ON=1) to enable discharging
    BQ76920_WriteRegister(BMS, SYS_CTRL2, &val, NULL); // Write updated SYS_CTRL2
}

// Function: turnCHGOff
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), disables the charge FET
// Significance:
//   - Turns off the charge FET to stop charging, used for safety or control
//     (datasheet Section 8.5.4, page 25).
void turnCHGOff(BQ76920_t *BMS) {
    uint8_t val; // Variable to hold SYS_CTRL2 register value
    BQ76920_ReadRegister(BMS, SYS_CTRL2, &val, NULL); // Read SYS_CTRL2
    val &= ~0x01; // Clear bit 0 (CHG_ON=0) to disable charging
    BQ76920_WriteRegister(BMS, SYS_CTRL2, &val, NULL); // Write updated SYS_CTRL2
}

// Function: turnDSGOff
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), disables the discharge FET
// Significance:
//   - Turns off the discharge FET to stop discharging, used for safety or control
//     (datasheet Section 8.5.4, page 25).
void turnDSGOff(BQ76920_t *BMS) {
    uint8_t val; // Variable to hold SYS_CTRL2 register value
    BQ76920_ReadRegister(BMS, SYS_CTRL2, &val, NULL); // Read SYS_CTRL2
    val &= ~0x02; // Clear bit 1 (DSG_ON=0) to disable discharging
    BQ76920_WriteRegister(BMS, SYS_CTRL2, &val, NULL); // Write updated SYS_CTRL2
}

// Function: CLEAR_SYS_STAT
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), clears all status flags
// Significance:
//   - Resets all alert flags in the SYS_STAT register to clear any active warnings
//     (datasheet Section 8.5.1, page 24).
void CLEAR_SYS_STAT(BQ76920_t *BMS) {
    uint8_t val = 0xFF; // Set all bits to 1 to clear all flags
    BQ76920_WriteRegister(BMS, SYS_STAT, &val, NULL); // Write to SYS_STAT (address 0x00)
}

// Function: getAlert
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - k: A uint8_t, the index of the alert flag to check (0 to 7)
// Output:
//   - Returns a uint8_t, the value of the specified alert flag (0 or 1)
// Significance:
//   - Retrieves a specific alert flag (e.g., overvoltage, overcurrent) to check
//     for issues, used in BMS_Service.c for monitoring (datasheet Section 8.5.1, page 24).
uint8_t getAlert(BQ76920_t *BMS, uint8_t k) {
    return BMS->Alert[k]; // Return the k-th alert flag from the BMS structure
}

// Function: checkUV
// Inputs:
//   - Vcell: An array of 4 floats, containing cell voltages
// Output:
//   - Returns a uint8_t, 1 if any cell is undervoltaged, 0 otherwise
// Significance:
//   - Checks if any cell voltage is too low, used to detect undervoltage conditions
//     that could harm the battery (datasheet Section 8.3.1, page 20).
uint8_t checkUV(float Vcell[4]) {
    for (int i = 0; i < 4; i++) { // Loop through all four cell voltages
        if (Vcell[i] <= netUV) return 1; // Return 1 if any cell is below threshold (netUV from BQ76920.h)
    }
    return 0; // Return 0 if no undervoltage detected
}

// Function: checkNotUV
// Inputs:
//   - Vcell: An array of 4 floats, containing cell voltages
//   - UV: A uint8_t, the current undervoltage flag status
// Output:
//   - Returns a uint8_t, 1 if undervoltage condition has cleared, 0 otherwise
// Significance:
//   - Checks if an undervoltage condition has been resolved, used to safely resume
//     normal operation (datasheet Section 8.3.1, page 20).
uint8_t checkNotUV(float Vcell[4], uint8_t UV) {
    if (!UV) return 0; // If no undervoltage flag is set, return 0
    for (int i = 0; i < 4; i++) { // Check all cell voltages
        // If any cell is still below threshold plus margin, return 0
        if (Vcell[i] < netUV + thresholdRange) return 0;
    }
    return 1; // Return 1 if all cells are above threshold (undervoltage cleared)
}

// Function: checkOV
// Inputs:
//   - Vcell: An array of 4 floats, containing cell voltages
// Output:
//   - Returns a uint8_t, 1 if any cell is overvoltaged, 0 otherwise
// Significance:
//   - Checks if any cell voltage is too high, used to detect overvoltage conditions
//     that could damage the battery (datasheet Section 8.3.1, page 20).
uint8_t checkOV(float Vcell[4]) {
    for (int i = 0; i < 4; i++) { // Loop through all four cell voltages
        if (Vcell[i] >= grossOV) return 1; // Return 1 if any cell is above threshold (grossOV from BQ76920.h)
    }
    return 0; // Return 0 if no overvoltage detected
}

// Function: checkNotOV
// Inputs:
//   - Vcell: An array of 4 floats, containing cell voltages
//   - OV: A uint8_t, the current overvoltage flag status
// Output:
//   - Returns a uint8_t, 1 if overvoltage condition has cleared, 0 otherwise
// Significance:
//   - Checks if an overvoltage condition has been resolved, used to safely resume
//     normal operation (datasheet Section 8.3.1, page 20).
uint8_t checkNotOV(float Vcell[4], uint8_t OV) {
    if (!OV) return 0; // If no overvoltage flag is set, return 0
    for (int i = 0; i < 4; i++) { // Check all cell voltages
        // If any cell is still above threshold minus margin, return 0
        if (Vcell[i] > grossOV - thresholdRange) return 0;
    }
    return 1; // Return 1 if all cells are below threshold (overvoltage cleared)
}

// Function: justWrite1
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - None (void), writes a test value to CC_CFG
// Significance:
//   - Writes a test value to the Coulomb Counter configuration register, likely for
//     debugging or specific test cases (datasheet Section 8.5.7, page 26).
void justWrite1(BQ76920_t *BMS) {
    uint8_t val = 0xcd; // Arbitrary test value for CC_CFG
    BQ76920_WriteRegister(BMS, CC_CFG, &val, NULL); // Write to CC_CFG (address 0x0B)
}

// Function: justRead1
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - Returns a uint8_t, the value of the CELLBAL1 register
// Significance:
//   - Reads the cell balancing register to check which cells are being balanced,
//     used in BMS_Service.c to monitor balancing status (datasheet Section 8.5.2, page 24).
uint8_t justRead1(BQ76920_t *BMS) {
    uint8_t temp; // Variable to hold register value
    BQ76920_ReadRegister(BMS, CELLBAL1, &temp, NULL); // Read CELLBAL1 (address 0x01)
    return temp; // Return the value
}

// Function: justRead2
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
// Output:
//   - Returns a uint8_t, the value of the SYS_CTRL2 register
// Significance:
//   - Reads the system control register to check charge/discharge FET status,
//     used for monitoring control settings (datasheet Section 8.5.4, page 25).
uint8_t justRead2(BQ76920_t *BMS) {
    uint8_t temp; // Variable to hold register value
    BQ76920_ReadRegister(BMS, SYS_CTRL2, &temp, NULL); // Read SYS_CTRL2 (address 0x05)
    return temp; // Return the value
}

// Function: BQ76920_GetMetrics
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - metrics: A pointer to a BMSMetrics_t structure to store metrics
// Output:
//   - None (void), fills the metrics structure with data
// Significance:
//   - Collects various battery metrics (SOC, SOH, balancing status) into a single
//     structure, used in BMS_Service.c for telemetry (datasheet Section 8.3, page 20).
void BQ76920_GetMetrics(BQ76920_t *BMS, BMSMetrics_t *metrics) {
    metrics->soc_energy = BMS->SOCEnergy; // Copy energy-based SOC
    metrics->soc_capacity = BMS->SOCCapacity; // Copy capacity-based SOC
    metrics->soh = BMS->SOH; // Copy combined SOH
    metrics->soh_energy = BMS->SOHEnergy; // Copy energy-based SOH
    metrics->soh_capacity = BMS->SOHCapacity; // Copy capacity-based SOH
    metrics->soh_ocv = BMS->SOHOCV; // Copy voltage-based SOH
    metrics->watt_usage = BMS->wattUsage; // Copy total energy usage
    metrics->current_usage = BMS->currentUsage; // Copy total current usage

    BQ76920_ReadRegister(BMS, CELLBAL1, &metrics->balancing_mask, NULL); // Read balancing status
    BQ76920_ReadRegister(BMS, SYS_CTRL2, &metrics->sys_ctrl2_flags, NULL); // Read control flags
}

// Function: BQ76920_ReadRegister
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - reg: A uint8_t, the register address to read (e.g., 0x00 for SYS_STAT)
//   - data: A pointer to a uint8_t to store the read value
//   - crc: A pointer to a uint8_t to store the CRC (NULL if not needed)
// Output:
//   - None (void), stores the read value in *data
// Significance:
//   - Reads a single byte from a BQ76920 register over I2C, used for all register
//     accesses (datasheet Section 8.5.23, page 33).
void BQ76920_ReadRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data, uint8_t *crc) {
    uint8_t buffer[2] = {reg, 0}; // Buffer with register address to send
    // Send the register address to the chip (BQ76920_ADDRESS is 0x18 << 1)
    HAL_I2C_Master_Transmit(BMS->i2cHandle, BQ76920_ADDRESS, buffer, 1, HAL_MAX_DELAY);
    // Receive the register data
    HAL_I2C_Master_Receive(BMS->i2cHandle, BQ76920_ADDRESS, data, 1, HAL_MAX_DELAY);
    // Calculate CRC for the transmitted address if requested
    if (crc) *crc = calculateCRC(buffer, 1);
}

// Function: BQ76920_WriteRegister
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - reg: A uint8_t, the register address to write (e.g., 0x00 for SYS_STAT)
//   - data: A pointer to a uint8_t, the value to write
//   - crc: A pointer to a uint8_t to store the CRC (NULL if not needed)
// Output:
//   - None (void), writes the value to the register
// Significance:
//   - Writes a single byte to a BQ76920 register over I2C, used for configuration
//     and control (datasheet Section 8.5.23, page 33).
void BQ76920_WriteRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data, uint8_t *crc) {
    uint8_t buffer[2] = {reg, *data}; // Buffer with register address and data
    // Calculate CRC for the address and data if requested
    if (crc) *crc = calculateCRC(buffer, 2);
    // Send the address and data to the chip
    HAL_I2C_Master_Transmit(BMS->i2cHandle, BQ76920_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}

// Function: BQ76920_GetExternalThermistorTemperature
// Inputs:
//   - BMS: A pointer to a BQ76920_t structure, holding chip data
//   - beta: A float, the thermistor’s beta value (3950, defined in main.h)
//   - R_nominal: A float, the thermistor’s nominal resistance (10kΩ)
//   - T_nominal: A float, the thermistor’s nominal temperature (25°C)
// Output:
//   - Returns a float, the temperature in Celsius
// Significance:
//   - Reads the temperature from an external thermistor connected to the BQ76920,
//     critical for monitoring battery temperature (datasheet Section 8.3.3, page 21).
float BQ76920_GetExternalThermistorTemperature(BQ76920_t *BMS, float beta, float R_nominal, float T_nominal) {
    uint8_t prev_val; // Store original SYS_CTRL1 value to restore later
    BQ76920_ReadRegister(BMS, SYS_CTRL1, &prev_val, NULL); // Read SYS_CTRL1 (address 0x04)
    uint8_t val = prev_val | (1 << 3); // Set TEMP_SEL=1 for external thermistor
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &val, NULL); // Write to enable thermistor

    uint8_t data[2]; // Array to hold high and low bytes of thermistor data
    BQ76920_ReadRegister(BMS, TS1_HI, &data[0], NULL); // Read TS1_HI (address 0x2C)
    BQ76920_ReadRegister(BMS, TS1_LO, &data[1], NULL); // Read TS1_LO (address 0x2D)
    // Combine high and low bytes into a 14-bit raw ADC value
    uint16_t raw = (((data[0] & 0x3F) << 8) | data[1]); // Mask high byte to 6 bits
    // Convert raw value to voltage using gain and offset
    // Use int32_t to avoid truncation (per your request)
    int32_t temp = (int32_t)BMS->GAIN * raw + (int32_t)BMS->OFFSET * 1000; // GAIN in µV/LSB, OFFSET in mV
    float v_ts1 = (float)temp / 1000000.0f; // Convert from microvolts to volts

    const float R_fixed = 10000.0f; // Fixed 10kΩ pull-up resistor (datasheet Section 8.3.3)
    // Calculate thermistor resistance using voltage divider formula (assumes 3V supply)
    float R_therm = R_fixed * v_ts1 / (3.0f - v_ts1);
    // Convert resistance to temperature using the Beta equation
    float T_kelvin = 1.0f / ((1.0f / (T_nominal + 273.15f)) + (1.0f / beta) * logf(R_therm / R_nominal));
    float temperature = T_kelvin - 273.15f; // Convert from Kelvin to Celsius

    // Check if temperature is valid (-40°C to 85°C)
    if (!IS_VALID_TEMPERATURE(temperature)) {
        temperature = 0.0f; // Set to 0 if out of range
    }

    BQ76920_WriteRegister(BMS, SYS_CTRL1, &prev_val, NULL); // Restore original SYS_CTRL1
    return temperature; // Return temperature in Celsius
}
