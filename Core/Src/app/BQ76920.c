// File: BQ76920.c
// Include header files needed for the code to work
#include "BQ76920.h"        // Contains definitions for the BQ76920 chip, like register addresses and constants
#include "main.h"           // Main project header with global settings, pin definitions, and hardware handles
#include <stdint.h>         // Provides standard integer types like uint8_t, uint16_t, int32_t for precise sizes
#include <stdio.h>          // Includes functions for printing (used in Log_Error for debugging)
#include <stdbool.h>        // Adds support for true/false boolean values
#include <stdlib.h>         // Includes utility functions like abs() for absolute value calculations
#include <math.h>           // Includes math functions like logf() for temperature calculations
#include <stdbool.h> // for bool type
#include "delay.h"

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
    BMS->i2cHandle = i2cHandle;
    BMS->SOH = 100.0f;
    BMS->wattUsage = 0;
    BMS->currentUsage = 0;

    Log_Message(BMS_MSG_LEVEL_INFO, "Initializing BQ76920...");

    uint8_t val;

    val = 0xFF;
    BQ76920_WriteRegister(BMS, SYS_STAT, &val, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Cleared SYS_STAT register");

    val = 0x19;
    BQ76920_WriteRegister(BMS, CC_CFG, &val, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Configured CC_CFG register: 0x%02X", val);

    val = (1 << 4) | (1 << 1);
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &val, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Enabled ADC and Coulomb Counter: SYS_CTRL1=0x%02X", val);

    val = (1 << 6);
    BQ76920_WriteRegister(BMS, SYS_CTRL2, &val, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Enabled continuous ADC operation: SYS_CTRL2=0x%02X", val);

    uint8_t adc_gain1, adc_gain2;
    BQ76920_ReadRegister(BMS, ADCGAIN1, &adc_gain1, NULL);
    BQ76920_ReadRegister(BMS, ADCGAIN2, &adc_gain2, NULL);
    uint16_t gain_bits = ((adc_gain2 & 0xE0) >> 5) | ((adc_gain1 & 0x0C) << 1);
    BMS->GAIN = gain_bits + 365;

    uint8_t offset_raw;
    BQ76920_ReadRegister(BMS, ADCOFFSET, &offset_raw, NULL);
    BMS->OFFSET = (int8_t)offset_raw;

    Log_Message(BMS_MSG_LEVEL_INFO, "GAIN: %duV/LSB, OFFSET: %dmV", BMS->GAIN, BMS->OFFSET);

    uint8_t PROTECT1_VAL;
    BQ76920_ReadRegister(BMS, PROTECT1, &PROTECT1_VAL, NULL);
    PROTECT1_VAL |= (SDC_100us_delay << 3) | SCD_Threshold_89mV;
    BQ76920_WriteRegister(BMS, PROTECT1, &PROTECT1_VAL, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Configured PROTECT1: 0x%02X", PROTECT1_VAL);

    uint8_t PROTECT2_VAL;
    BQ76920_ReadRegister(BMS, PROTECT2, &PROTECT2_VAL, NULL);
    PROTECT2_VAL |= (ODC_160ms_delay << 5) | OCD_Threshold_17mV;
    BQ76920_WriteRegister(BMS, PROTECT2, &PROTECT2_VAL, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Configured PROTECT2: 0x%02X", PROTECT2_VAL);

    uint8_t PROTECT3_VAL;
    BQ76920_ReadRegister(BMS, PROTECT3, &PROTECT3_VAL, NULL);
    PROTECT3_VAL |= (UV_Delay_4s << 6) | (OV_Delay_2s << 4);
    BQ76920_WriteRegister(BMS, PROTECT3, &PROTECT3_VAL, NULL);
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Configured PROTECT3: 0x%02X", PROTECT3_VAL);

    uint16_t OV = grossOV * 1000;
    uint16_t temp = (uint16_t)((float)(OV - BMS->OFFSET) / ((float)(BMS->GAIN) / 1000));
    temp = (temp & 0x0FF0) >> 4;
    uint8_t OV_TRIP_VAL = temp & 0xFF;
    BQ76920_WriteRegister(BMS, OV_TRIP, &OV_TRIP_VAL, NULL);
    Log_Message(BMS_MSG_LEVEL_INFO, "OV_TRIP set: %.2f V → 0x%02X", grossOV, OV_TRIP_VAL);

    uint16_t UV = grossUV * 1000;
    temp = (uint16_t)((float)(UV - BMS->OFFSET) / ((float)(BMS->GAIN) / 1000));
    temp = (temp & 0x0FF0) >> 4;
    uint8_t UV_TRIP_VAL = temp & 0xFF;
    BQ76920_WriteRegister(BMS, UV_TRIP, &UV_TRIP_VAL, NULL);
    Log_Message(BMS_MSG_LEVEL_INFO, "UV_TRIP set: %.2f V → 0x%02X", grossUV, UV_TRIP_VAL);

    BMS->SOH = 45.0f;
    BMS->SOHEnergy = 45.0f;
    BMS->SOHCapacity = 45.0f;
    BMS->SOHOCV = 45.0f;
    Log_Message(BMS_MSG_LEVEL_DEBUG, "Initial SOH metrics set to 45%% for all fields.");

    Log_Message(BMS_MSG_LEVEL_INFO, "BQ76920 initialization complete.");
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
        if (cell == VC4) {
            Log_Message(BMS_MSG_LEVEL_WARNING, "VC4 shares VC3 voltage: returning Vcell[2] = %.3f V", BMS->Vcell[2]);
            return BMS->Vcell[2];
        }
        Log_Message(BMS_MSG_LEVEL_WARNING, "Invalid cell ID: %d", cell);
        return 0.0f;
    }

    uint8_t data[2] = {0};
    BQ76920_ReadRegister(BMS, cell, &data[0], NULL);        // High byte
    BQ76920_ReadRegister(BMS, cell + 1, &data[1], NULL);    // Low byte

    uint16_t raw = (((data[0] & 0x3F) << 8) | data[1]);
    int32_t temp = 4 * (int32_t)BMS->GAIN * raw + 4 * (int32_t)BMS->OFFSET * 1000;
    float voltage = (float)temp / 1000000.0f;

    if (!IS_VALID_VOLTAGE(voltage)) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "Invalid voltage reading on cell %d: %.3f V (raw=0x%04X)", cell - VC1 + 1, voltage, raw);
        voltage = 0.0f;
    }

    int index;
    switch (cell) {
        case VC1: index = 0; break;
        case VC2: index = 1; break;
        case VC3: index = 2; break;
        case VC5: index = 3; break;
        default:
            Log_Message(BMS_MSG_LEVEL_ERROR, "Unexpected cell index mapping failed for cell %d", cell);
            return 0.0f;
    }

    BMS->Vcell[index] = voltage;

    Log_Message(BMS_MSG_LEVEL_DEBUG, "Cell %d voltage: %.3f V (raw=0x%04X, index=%d)", cell - VC1 + 1, voltage, raw, index);
    return voltage;
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
    uint8_t data[2] = {0};

    BQ76920_ReadRegister(BMS, BAT_LO, &data[0], NULL); // Read BAT_LO (0x2B)
    BQ76920_ReadRegister(BMS, BAT_HI, &data[1], NULL); // Read BAT_HI (0x2A)

    uint16_t raw = (data[1] << 8) | data[0];
    int32_t temp = (int32_t)raw * BMS->GAIN * 4 + (int32_t)BMS->OFFSET * 4000;
    float voltage = (float)temp / 1000000.0f;

    // Optional range check
#ifdef IS_VALID_VOLTAGE
    if (!IS_VALID_VOLTAGE(voltage)) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "Invalid pack voltage: %.3f V (raw=0x%04X)", voltage, raw);
        voltage = 0.0f;
    }
#endif

    BMS->Vpack = voltage;

    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "Pack voltage: %.3f V (raw=0x%04X, BAT_HI=0x%02X, BAT_LO=0x%02X, GAIN=%d, OFFSET=%d)",
        voltage, raw, data[1], data[0], BMS->GAIN, BMS->OFFSET);

    return voltage;
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
    uint8_t data[2] = {0};

    // Read Coulomb Counter registers
    BQ76920_ReadRegister(BMS, CC_LO, &data[0], NULL); // address 0x33
    BQ76920_ReadRegister(BMS, CC_HI, &data[1], NULL); // address 0x32

    // Combine into signed 16-bit value
    int16_t raw = (int16_t)((data[1] << 8) | data[0]);

    if (abs(raw) == 1) {
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Coulomb counter noise filtered: raw=±1");
        raw = 0;
    }

    float current = raw * 8.44f / RSENSE;

    if (!IS_VALID_CURRENT(current)) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "Invalid current reading: %.2f A (raw=0x%04X)", current, raw);
        current = 0.0f;
    } else {
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Current: %.2f A (raw=0x%04X, CC_HI=0x%02X, CC_LO=0x%02X, RSENSE=%.2f)",
            current, (uint16_t)raw, data[1], data[0], RSENSE);
    }

    return current;
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
    uint8_t val;
    BQ76920_ReadRegister(BMS, SYS_CTRL1, &val, NULL);

    // Configure TEMP_SEL bit (bit 3)
    if (temp_sel) {
        val |= (1 << 3);  // External thermistor
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Selecting external temperature sensor (TEMP_SEL = 1)");
    } else {
        val &= ~(1 << 3); // Internal die temperature
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Selecting internal temperature sensor (TEMP_SEL = 0)");
    }
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &val, NULL);

    uint8_t data[2] = {0};
    BQ76920_ReadRegister(BMS, TS1_HI, &data[0], NULL);
    BQ76920_ReadRegister(BMS, TS1_LO, &data[1], NULL);

    uint16_t raw = ((data[0] & 0x3F) << 8) | data[1];
    int32_t temp = (int32_t)BMS->GAIN * raw + (int32_t)BMS->OFFSET * 1000;
    float voltage = (float)temp / 1000000.0f;
    float temperature = 25.0f - ((voltage - 1.2f) / 0.0042f);

    if (!IS_VALID_TEMPERATURE(temperature)) {
        Log_Message(BMS_MSG_LEVEL_WARNING,
            "Invalid temperature reading: %.2f °C (voltage=%.3f V, raw=0x%04X)", temperature, voltage, raw);
        temperature = 0.0f;
    } else {
        Log_Message(BMS_MSG_LEVEL_DEBUG,
            "Temperature: %.2f °C (raw=0x%04X, voltage=%.3f V, GAIN=%d, OFFSET=%d)",
            temperature, raw, voltage, BMS->GAIN, BMS->OFFSET);
    }

    return temperature;
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
    // Calculate total energy and current capacity
    float fullEnergy = grossCapacity * 3600 * 4 * nominalPackV * (BMS->SOH / 100.0f);
    float fullCurrent = grossCapacity * 3600 * 4 * (BMS->SOH / 100.0f);

    // Log inputs
    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "SOCPack input: Current=%.2f A, Voltage=%.2f V, SOH=%.1f%%", PackCurrent, Vpack, BMS->SOH);

    // Apply efficiency factor for discharge
    if (PackCurrent < 0) {
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Discharge current detected. Applying %.2f efficiency factor.", ROUND_TRIP_EFFICIENCY);
        PackCurrent *= ROUND_TRIP_EFFICIENCY;
    }

    // Update cumulative usage
    BMS->wattUsage += PackCurrent * Vpack;
    BMS->currentUsage += PackCurrent;

    // Calculate SOC estimates
    BMS->SOCEnergy = (fullEnergy + BMS->wattUsage) * 100.0f / fullEnergy;
    BMS->SOCCapacity = (fullCurrent + BMS->currentUsage) * 100.0f / fullCurrent;

    // Kalman filter fusion
    KalmanFilter kf;
    kalman_filter_init(&kf, BMS->SOCEnergy, 1.0f, 0.05f);
    float fused = kalman_filter_update(&kf, BMS->SOCCapacity, 0.05f);

    BMS->SOC = fused;

    // Log estimates
    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "SOC estimates: Energy=%.2f%%, Capacity=%.2f%%, Fused=%.2f%%", BMS->SOCEnergy, BMS->SOCCapacity, BMS->SOC);

    return fused;
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

    Log_Message(BMS_MSG_LEVEL_INFO, "Entering SHIP mode...");

    HAL_StatusTypeDef status1 = BQ76920_WriteRegister(BMS, SYS_CTRL1, &step1, NULL);
    if (status1 != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to write step 1 (SHUT_B=1) to SYS_CTRL1");
    } else {
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Step 1 written: SYS_CTRL1 = 0x%02X", step1);
    }

    SoftwareDelay(1); // Mandatory 1ms delay between steps

    HAL_StatusTypeDef status2 = BQ76920_WriteRegister(BMS, SYS_CTRL1, &step2, NULL);
    if (status2 != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to write step 2 (SHUT_A=1) to SYS_CTRL1");
    } else {
        Log_Message(BMS_MSG_LEVEL_DEBUG, "Step 2 written: SYS_CTRL1 = 0x%02X", step2);
    }

    if (status1 == HAL_OK && status2 == HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_INFO, "BQ76920 successfully entered SHIP mode.");
    }
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
    int32_t fullEnergy = grossCapacity * 3600 * 4 * nominalPackV;
    int32_t fullCurrent = grossCapacity * 3600 * 4;
    float FullOCV = netOV;

    // Identify smallest cell voltage
    BMS->smallestV = BMS->Vcell[0];
    for (int i = 1; i < 4; i++) {
        if (BMS->Vcell[i] < BMS->smallestV) {
            BMS->smallestV = BMS->Vcell[i];
        }
    }

    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "SOHPack inputs: wattUsage=%.2f Ws, currentUsage=%.2f As, smallestV=%.3f V",
        BMS->wattUsage, BMS->currentUsage, BMS->smallestV);

    // Compute SOH estimates
    BMS->SOHEnergy = BMS->wattUsage * 100.0f / fullEnergy;
    BMS->SOHCapacity = BMS->currentUsage * 100.0f / fullCurrent;
    BMS->SOHOCV = BMS->smallestV * 100.0f / FullOCV;

    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "SOH estimates: Energy=%.2f%%, Capacity=%.2f%%, OCV=%.2f%%",
        BMS->SOHEnergy, BMS->SOHCapacity, BMS->SOHOCV);

    // Kalman filtering
    KalmanFilter kf;
    kalman_filter_init(&kf, BMS->SOHEnergy, 1.0f, 0.05f);
    float fused = kalman_filter_update(&kf, BMS->SOHCapacity, 0.05f);
    fused = kalman_filter_update(&kf, BMS->SOHOCV, 0.05f);

    BMS->SOH = fused;
    BMS->wattUsage = 0;
    BMS->currentUsage = 0;

    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "Fused SOH = %.2f%% (FullEnergy=%ld Ws, FullCurrent=%ld As, OCV ref=%.2f V)",
        BMS->SOH, fullEnergy, fullCurrent, FullOCV);

    return fused;
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


bool EnableBalanceCell(BQ76920_t *BMS, float PackCurrent) {
    uint8_t flags = 0x00;

    if (BMS == NULL || BMS->i2cHandle == NULL) {
        return false;
    }

    if (PackCurrent > 0.0f) {
        float maxV = BMS->Vcell[0];
        int idx_max = 0;

        for (int i = 1; i < 4; i++) {
            if (BMS->Vcell[i] > maxV) {
                maxV = BMS->Vcell[i];
                idx_max = i;
            }
        }

        float minV = BMS->Vcell[0];
        for (int i = 1; i < 4; i++) {
            if (BMS->Vcell[i] < minV) {
                minV = BMS->Vcell[i];
            }
        }

        if ((maxV - minV) >= balanceThreshold) {
            flags |= (1 << idx_max);
        } else {
            return true; // No balancing needed, still success
        }
    } else {
        return true; // Not charging, no balancing
    }

    // Write the balancing flags to CELLBAL1 register
    HAL_StatusTypeDef status = BQ76920_WriteRegister(BMS, CELLBAL1, &flags, NULL);
    return (status == HAL_OK);
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
    	if (Vcell[i] <= netUV) {
    		Log_Message(BMS_MSG_LEVEL_WARNING, "UV detected on Cell %d: %.3f V", i + 1, Vcell[i]);
    		return 1;  // Undervoltage detected// Return 1 if any cell is below threshold (netUV from BQ76920.h)
    	}
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


// Function: checkOC
// Input: PackCurrent (A) — should be negative for discharge
// Return: 1 if overcurrent in discharge is detected
uint8_t checkOC(float PackCurrent) {
    if (-PackCurrent >= MaxDischargeCurrent) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "OCD triggered: %.2f A", PackCurrent);
        return 1;
    }
    return 0;
}

// Function: checkNotOC
// Input: PackCurrent (A), OC flag
// Return: 1 if OCD condition has cleared
uint8_t checkNotOC(float PackCurrent, uint8_t OC) {
    if (!OC) return 0;
    if (-PackCurrent < (MaxDischargeCurrent - thresholdRange)) {
        Log_Message(BMS_MSG_LEVEL_INFO, "OCD cleared: %.2f A", PackCurrent);
        return 1;
    }
    return 0;
}

// Function: checkSC
// Input: PackCurrent (A)
// Return: 1 if short-circuit in discharge is detected
uint8_t checkSC(float PackCurrent) {
    // You can define a SC threshold separately, but for now we reuse MaxDischargeCurrent
    if (-PackCurrent >= MaxDischargeCurrent * 1.5f) { // SCD is typically more severe
        Log_Message(BMS_MSG_LEVEL_ERROR, "SCD triggered: %.2f A", PackCurrent);
        return 1;
    }
    return 0;
}

// Function: checkNotSC
// Input: PackCurrent (A), SC flag
// Return: 1 if SCD condition has cleared
uint8_t checkNotSC(float PackCurrent, uint8_t SC) {
    if (!SC) return 0;
    if (-PackCurrent < (MaxDischargeCurrent * 1.5f - thresholdRange)) {
        Log_Message(BMS_MSG_LEVEL_INFO, "SCD cleared: %.2f A", PackCurrent);
        return 1;
    }
    return 0;
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
HAL_StatusTypeDef BQ76920_ReadRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data, uint8_t *crc) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(BMS->i2cHandle, BQ76920_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    status = HAL_I2C_Master_Receive(BMS->i2cHandle, BQ76920_ADDRESS, data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    if (crc) *crc = calculateCRC(data, 1); // optional CRC
    return HAL_OK;
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
HAL_StatusTypeDef BQ76920_WriteRegister(BQ76920_t *BMS, uint8_t reg, uint8_t *data, uint8_t *crc) {
    uint8_t buffer[2] = {reg, *data};
    if (crc) *crc = calculateCRC(buffer, 2);
    return HAL_I2C_Master_Transmit(BMS->i2cHandle, BQ76920_ADDRESS, buffer, 2, HAL_MAX_DELAY);
}


void BQ76920_ReadGainAndOffset(BQ76920_t *BMS) {
    if (BMS == NULL || BMS->i2cHandle == NULL) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Invalid BMS handle in ReadGainAndOffset");
        return;
    }

    uint8_t cc_gain1 = 0, cc_gain2 = 0, cc_offset = 0;

    // Read registers as per datasheet (Table 11)
    if (BQ76920_ReadRegister(BMS, CC_GAIN1, &cc_gain1, NULL) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to read CC_GAIN1");
        return;
    }

    if (BQ76920_ReadRegister(BMS, CC_GAIN2, &cc_gain2, NULL) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to read CC_GAIN2");
        return;
    }

    if (BQ76920_ReadRegister(BMS, CC_OFFSET, &cc_offset, NULL) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Failed to read CC_OFFSET");
        return;
    }

    // Combine CC_GAIN1 and CC_GAIN2 into full 16-bit gain value (CC_GAIN = (CC_GAIN1 << 8) | CC_GAIN2)
    uint16_t raw_gain = ((uint16_t)cc_gain1 << 8) | cc_gain2;

    // Calculate GAIN and OFFSET per datasheet (Section 8.3.2.4)
    BMS->GAIN   = 365.0f + (float)(raw_gain);  // GAIN in µV/LSB
    BMS->OFFSET = (float)((int8_t)cc_offset);  // OFFSET in mV (2's complement signed)

    Log_Message(BMS_MSG_LEVEL_DEBUG,
        "Read Gain and Offset: CC_GAIN1=0x%02X, CC_GAIN2=0x%02X, raw_gain=%u → GAIN=%.1f uV/LSB, OFFSET=%.1f mV",
        cc_gain1, cc_gain2, raw_gain, BMS->GAIN, BMS->OFFSET);
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
    if (BMS == NULL || BMS->i2cHandle == NULL) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "Invalid BMS handle");
        return 0.0f;
    }

    // Ensure gain/offset are initialized
    if (BMS->GAIN == 0 || BMS->OFFSET == 0) {
        BQ76920_ReadGainAndOffset(BMS);
    }

    // Backup and modify SYS_CTRL1 to enable TEMP_SEL for external thermistor
    uint8_t prev_val = 0;
    BQ76920_ReadRegister(BMS, SYS_CTRL1, &prev_val, NULL);
    uint8_t val = prev_val | (1 << 3);  // Set TEMP_SEL = 1
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &val, NULL);
    SoftwareDelay(1);  // Allow settling

    // Read TS1_HI and TS1_LO (14-bit ADC value)
    uint8_t data[2] = {0};
    BQ76920_ReadRegister(BMS, TS1_HI, &data[0], NULL);  // Address 0x2C
    BQ76920_ReadRegister(BMS, TS1_LO, &data[1], NULL);  // Address 0x2D
    uint16_t raw = (((data[0] & 0x3F) << 8) | data[1]);  // Mask upper 2 bits of HI

    // Calculate voltage from ADC raw value
    int32_t temp_uV = (int32_t)BMS->GAIN * raw + (int32_t)(BMS->OFFSET * 1000);  // µV
    float v_ts1 = temp_uV / 1000000.0f;  // Convert to volts

    // Clamp voltage to avoid divide-by-zero
    if (v_ts1 >= 3.0f) v_ts1 = 2.999f;
    if (v_ts1 <= 0.0f) v_ts1 = 0.001f;

    const float R_fixed = 10000.0f;  // 10k pull-up
    float R_therm = R_fixed * v_ts1 / (3.0f - v_ts1);  // Voltage divider formula

    // Calculate temperature from Beta model
    float T_kelvin = 1.0f / ((1.0f / (T_nominal + 273.15f)) + (1.0f / beta) * logf(R_therm / R_nominal));
    float temperature = T_kelvin - 273.15f;

    // Range check
    if (!IS_VALID_TEMPERATURE(temperature)) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "Temperature out of range: %.2f °C", temperature);
        temperature = 0.0f;
    }

    // Restore original TEMP_SEL setting
    BQ76920_WriteRegister(BMS, SYS_CTRL1, &prev_val, NULL);

    // Debug output
    Log_Message(BMS_MSG_LEVEL_DEBUG,
                "TS1 raw=%u, V=%.3f V, R=%.1f Ω, T=%.2f °C",
                raw, v_ts1, R_therm, temperature);

    return temperature;
}

