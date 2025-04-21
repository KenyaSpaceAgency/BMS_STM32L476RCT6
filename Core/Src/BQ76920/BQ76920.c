/*
 * BQ76920.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 *
 * @brief  Driver for the BQ76920 battery monitoring IC, managing voltage, current,
 *         balancing, and protection for a 4S lithium-ion battery pack.
 * @note   Designed for a 4S2P configuration with two BQ76920 ICs for redundancy.
 *         Interfaces via I2C and uses a 5mΩ shunt resistor for current measurement.
 */

#include "BQ76920.h"
#include "main.h"
#include <stdlib.h> // For abs (to compare numbers)

/**
  * @brief  Initializes the BQ76920 IC
  * @param  hi2c: Pointer to the I2C handle (I2C1 or I2C2)
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   Verifies communication by reading the SYS_STAT register.
  *         Selects I2C address based on I2C handle (I2C1 or I2C2).
  * @reason Ensures the IC is responsive before further operations, critical for
  *         reliable battery monitoring.
  */
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t sys_stat = 0;  // Buffer for SYS_STAT register value
    // Select I2C address: BQ76920_I2C_ADDRESS_1 for I2C1, BQ76920_I2C_ADDRESS_2 for I2C2
    // Shift left by 1 as HAL expects 8-bit address (7-bit address + R/W bit)
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Read SYS_STAT to confirm communication
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Reads group voltages from the BQ76920 (4S configuration, all 4 cells monitored)
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array to store cell voltages (in mV)
  * @param  offset: Starting index in the array to store voltages
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   Pin configuration:
  *         - VCO=ground, VC1=Cell 1 positive, VC2=Cell 2 positive,
  *         - VC3=VC4=Cell 3 positive, VC5=Cell 4 positive.
  *         Voltages are calculated as:
  *         - Cell 1: VC1 - VC0 (~3600-4200 mV)
  *         - Cell 2: VC2 - VC1 (~3600-4200 mV)
  *         - Cell 3: VC3 - VC2 (~3600-4200 mV)
  *         - Cell 4: VC5 - VC4 (~3600-4200 mV)
  * @reason Reads raw ADC counts and converts to millivolts using the BQ76920’s
  *         scaling factor (0.382 mV/LSB) for accurate cell voltage monitoring.
  */
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset) {
    uint8_t data[10]; // Buffer for 10 bytes (5 cells, 2 bytes each)
    // Select I2C address based on handle
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Read voltage registers (VC1_HI to VC5_LO)
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, VC1_HI_REG, 1, data, 10, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Parse raw ADC counts for each cell input
    uint16_t vc[5];
    for (uint8_t i = 0; i < 5; i++) {
        vc[i] = (data[i * 2] << 8) | data[i * 2 + 1]; // Combine high and low bytes
    }

    // Convert ADC counts to millivolts and compute differential voltages
    // Scaling factor 0.382 mV/LSB from BQ76920 datasheet
    group_voltages[offset + 0] = (vc[1] - vc[0]) * 0.382; // Cell 1: VC1 - VC0
    group_voltages[offset + 1] = (vc[2] - vc[1]) * 0.382; // Cell 2: VC2 - VC1
    group_voltages[offset + 2] = (vc[3] - vc[2]) * 0.382; // Cell 3: VC3 - VC2
    group_voltages[offset + 3] = (vc[4] - vc[3]) * 0.382; // Cell 4: VC5 - VC4

    return HAL_OK;
}

/**
  * @brief  Reads pack current from the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  current: Pointer to store the current (in mA)
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   Uses a 5mΩ shunt resistor. Scaling:
  *         - Voltage per LSB = 8.44 µV (datasheet)
  *         - Current per LSB = 8.44 µV / 5 mΩ = 1.688 mA/LSB
  * @reason Accurately measures pack current for SOC estimation and overcurrent
  *         protection, critical for battery safety.
  */
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current) {
    uint8_t data[2];  // Buffer for high and low bytes
    // Select I2C address
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Read coulomb counter register (CC_HI and CC_LO)
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, CC_HI_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Combine bytes into a signed 16-bit value
    *current = (int16_t)((data[0] << 8) | data[1]);
    // Scale to milliamps: 8.44 µV/LSB ÷ 5 mΩ = 1.688 mA/LSB
    *current *= 1.688f; // Use float for precision

    return HAL_OK;
}

/**
  * @brief  Balances cells by enabling balancing on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array of cell voltages (in mV)
  * @param  offset: Starting index in the array for cells to balance
  * @param  balancing_mask: Pointer to store the balancing bitmask
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   CELLBAL1_REG bits:
  *         - Bit 0: Cell 1
  *         - Bit 1: Cell 2
  *         - Bit 2: Cell 3
  *         - Bit 3: Cell 4
  * @reason Balances cells >50 mV above the minimum to ensure uniform charge,
  *         preventing capacity loss and extending battery life.
  */
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask) {
    *balancing_mask = 0;
    // Find minimum voltage among valid cells (non-zero)
    uint16_t min_voltage = group_voltages[offset + 0];
    for (uint8_t i = 1; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] < min_voltage && group_voltages[offset + i] > 0) {
            min_voltage = group_voltages[offset + i];
        }
    }

    // Enable balancing for cells >50 mV above minimum
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] > min_voltage + 50 && group_voltages[offset + i] > 0) {
            *balancing_mask |= (1 << i); // Set bit for cell i
        }
    }

    // Write balancing mask to CELLBAL1_REG
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, CELLBAL1_REG, 1, balancing_mask, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Checks for overvoltage and undervoltage conditions on the cells
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array of cell voltages (in mV)
  * @param  offset: Starting index in the array for cells to check
  * @param  ov_flag: Pointer to store overvoltage flag (1 = OV detected)
  * @param  uv_flag: Pointer to store undervoltage flag (1 = UV detected)
  * @retval None
  * @note   Compares voltages against thresholds from battery_config.
  * @reason Provides software-based OV/UV detection as a backup to hardware
  *         protection, enhancing safety.
  */
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag) {
    *ov_flag = 0;
    *uv_flag = 0;
    // Check each cell against OV/UV thresholds
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] > battery_config.ov_threshold) *ov_flag = 1;
        if (group_voltages[offset + i] < battery_config.uv_threshold) *uv_flag = 1;
    }
}

/**
  * @brief  Enables or disables charging and discharging on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  charge_enable: 1 to enable charging, 0 to disable
  * @param  discharge_enable: 1 to enable discharging, 0 to disable
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   Modifies SYS_CTRL2 register bits:
  *         - Bit 0: CHG_ON
  *         - Bit 1: DSG_ON
  * @reason Controls charge/discharge FETs based on BMS state, ensuring safe
  *         operation during faults or mode changes.
  */
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable) {
    uint8_t sys_ctrl2 = 0;
    // Select I2C address
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);

    // Read current SYS_CTRL2 value to preserve other bits
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_CTRL2_REG, 1, &sys_ctrl2, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Update CHG_ON and DSG_ON bits
    if (charge_enable) {
        sys_ctrl2 |= (1 << 0); // Enable charging
    } else {
        sys_ctrl2 &= ~(1 << 0); // Disable charging
    }
    if (discharge_enable) {
        sys_ctrl2 |= (1 << 1); // Enable discharging
    } else {
        sys_ctrl2 &= ~(1 << 1); // Disable discharging
    }

    // Write updated value to SYS_CTRL2
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, SYS_CTRL2_REG, 1, &sys_ctrl2, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Reads the status register (SYS_STAT_REG) of the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  status: Pointer to store the status value
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   SYS_STAT_REG bits indicate faults (e.g., OV, UV, OCD).
  * @reason Retrieves hardware-detected faults for BMS to act upon, critical
  *         for real-time protection.
  */
HAL_StatusTypeDef BQ76920_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status) {
    // Select I2C address
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, status, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Clears specific status flags in the SYS_STAT_REG register
  * @param  hi2c: Pointer to the I2C handle
  * @param  flags_to_clear: The status flags to clear
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   Writing 1 to a flag clears it (per datasheet).
  * @reason Resets fault flags after handling to prevent repeated triggers,
  *         ensuring proper fault recovery.
  */
HAL_StatusTypeDef BQ76920_ClearStatus(I2C_HandleTypeDef *hi2c, uint8_t flags_to_clear) {
    // Select I2C address
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, SYS_STAT_REG, 1, &flags_to_clear, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Compares readings between two BQ76920 ICs for redundancy
  * @param  group_voltages_1: Cell voltages from the first BQ76920
  * @param  group_voltages_2: Cell voltages from the second BQ76920
  * @param  current_1: Current from the first BQ76920
  * @param  current_2: Current from the second BQ76920
  * @param  discrepancy_flag: Pointer to store discrepancy flag (1 = discrepancy)
  * @retval None
  * @note   Thresholds: 100 mV for voltages, 500 mA for currents.
  * @reason Ensures both ICs provide consistent measurements, detecting hardware
  *         faults for high-reliability applications.
  */
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag) {
    *discrepancy_flag = 0;

    // Compare voltages for all cells
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (abs(group_voltages_1[i] - group_voltages_2[i]) > 100) {
            *discrepancy_flag = 1;
            return;
        }
    }

    // Compare currents
    if (abs(current_1 - current_2) > 500) {
        *discrepancy_flag = 1;
    }
}

/**
  * @brief  Configures hardware protection thresholds for OCC, OCD, and SCD
  * @param  hi2c: Pointer to the I2C handle
  * @retval HAL_StatusTypeDef (HAL_OK on success, else error)
  * @note   For a 5mΩ shunt:
  *         - 60 mV = 12 A (OCC, OCD)
  *         - 100 mV = 20 A (SCD)
  * @reason Sets hardware thresholds for overcurrent and short-circuit protection,
  *         ensuring fast response to dangerous conditions.
  */
HAL_StatusTypeDef BQ76920_ConfigureProtection(I2C_HandleTypeDef *hi2c) {
    // Select I2C address
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status;

    // PROTECT1: Set OCC and OCD thresholds (60 mV, ~12 A)
    // Bits 5:3 = OCD_THRESHOLD, Bits 2:0 = OCC_THRESHOLD (per datasheet)
    uint8_t protect1 = (0x4 << 3) | (0x4); // 60 mV for both
    status = HAL_I2C_Mem_Write(hi2c, i2c_addr, PROTECT1_REG, 1, &protect1, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // PROTECT2: Set SCD threshold (100 mV, ~20 A) and delay (15 µs)
    // Bits 5:3 = SCD_THRESHOLD, Bits 2:0 = SCD_DELAY
    uint8_t protect2 = (0x5 << 3) | (0x1); // 100 mV, 15 µs
    status = HAL_I2C_Mem_Write(hi2c, i2c_addr, PROTECT2_REG, 1, &protect2, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
  * @brief  Checks the status registers of two BQ76920 ICs and logs any faults
  * @param  hi2c1: Pointer to the I2C handle for the first BQ76920
  * @param  hi2c2: Pointer to the I2C handle for the second BQ76920
  * @param  error_flags: Pointer to store error flags
  * @retval None
  * @note   SYS_STAT_REG bits:
  *         - Bit 7: DEVICE_XREADY
  *         - Bit 6: OVRD_ALERT
  *         - Bit 5: UV
  *         - Bit 4: OV
  *         - Bit 3: SCD
  *         - Bit 2: OCD
  *         - Bit 1: OCC
  * @reason Monitors hardware faults from both ICs, logs them, and clears flags
  *         to prevent repeated triggers, ensuring robust fault handling.
  */
void BQ76920_CheckStatus(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, uint32_t *error_flags) {
    uint8_t status1, status2;
    uint8_t clear_flags1 = 0, clear_flags2 = 0;

    // Read and process status from first BQ76920 (I2C1)
    if (BQ76920_ReadStatus(hi2c1, &status1) == HAL_OK) {
        // Check each fault bit and set corresponding error flag
        if (status1 & (1 << 7)) { // DEVICE_XREADY: IC initialization issue
            *error_flags |= ERROR_DEVICE_XREADY;
            Log_Error("BQ76920 (I2C1): DEVICE_XREADY fault");
            clear_flags1 |= (1 << 7);
        }
        if (status1 & (1 << 6)) { // OVRD_ALERT: General alert condition
            *error_flags |= ERROR_OVRD_ALERT;
            Log_Error("BQ76920 (I2C1): OVRD_ALERT condition");
            clear_flags1 |= (1 << 6);
        }
        if (status1 & (1 << 5)) { // UV: Undervoltage detected
            *error_flags |= ERROR_UNDERVOLTAGE;
            Log_Error("BQ76920 (I2C1): Undervoltage fault");
            clear_flags1 |= (1 << 5);
        }
        if (status1 & (1 << 4)) { // OV: Overvoltage detected
            *error_flags |= ERROR_OVERVOLTAGE;
            Log_Error("BQ76920 (I2C1): Overvoltage fault");
            clear_flags1 |= (1 << 4);
        }
        if (status1 & (1 << 3)) { // SCD: Short-circuit discharge
            *error_flags |= ERROR_SCD;
            Log_Error("BQ76920 (I2C1): Short-circuit discharge fault");
            clear_flags1 |= (1 << 3);
        }
        if (status1 & (1 << 2)) { // OCD: Overcurrent discharge
            *error_flags |= ERROR_OCD;
            Log_Error("BQ76920 (I2C1): Overcurrent discharge fault");
            clear_flags1 |= (1 << 2);
        }
        if (status1 & (1 << 1)) { // OCC: Overcurrent charge
            *error_flags |= ERROR_OCC;
            Log_Error("BQ76920 (I2C1): Overcurrent charge fault");
            clear_flags1 |= (1 << 1);
        }

        // Clear handled flags to reset fault state
        if (clear_flags1 != 0) {
            BQ76920_ClearStatus(hi2c1, clear_flags1);
        }
    } else {
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C1)");
    }

    // Read and process status from second BQ76920 (I2C2)
    if (BQ76920_ReadStatus(hi2c2, &status2) == HAL_OK) {
        // Check each fault bit and set corresponding error flag
        if (status2 & (1 << 7)) { // DEVICE_XREADY
            *error_flags |= ERROR_DEVICE_XREADY;
            Log_Error("BQ76920 (I2C2): DEVICE_XREADY fault");
            clear_flags2 |= (1 << 7);
        }
        if (status2 & (1 << 6)) { // OVRD_ALERT
            *error_flags |= ERROR_OVRD_ALERT;
            Log_Error("BQ76920 (I2C2): OVRD_ALERT condition");
            clear_flags2 |= (1 << 6);
        }
        if (status2 & (1 << 5)) { // UV
            *error_flags |= ERROR_UNDERVOLTAGE;
            Log_Error("BQ76920 (I2C2): Undervoltage fault");
            clear_flags2 |= (1 << 5);
        }
        if (status2 & (1 << 4)) { // OV
            *error_flags |= ERROR_OVERVOLTAGE;
            Log_Error("BQ76920 (I2C2): Overvoltage fault");
            clear_flags2 |= (1 << 4);
        }
        if (status2 & (1 << 3)) { // SCD
            *error_flags |= ERROR_SCD;
            Log_Error("BQ76920 (I2C2): Short-circuit discharge fault");
            clear_flags2 |= (1 << 3);
        }
        if (status2 & (1 << 2)) { // OCD
            *error_flags |= ERROR_OCD;
            Log_Error("BQ76920 (I2C2): Overcurrent discharge fault");
            clear_flags2 |= (1 << 2);
        }
        if (status2 & (1 << 1)) { // OCC
            *error_flags |= ERROR_OCC;
            Log_Error("BQ76920 (I2C2): Overcurrent charge fault");
            clear_flags2 |= (1 << 1);
        }

        // Clear handled flags
        if (clear_flags2 != 0) {
            BQ76920_ClearStatus(hi2c2, clear_flags2);
        }
    } else {
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C2)");
    }
}
