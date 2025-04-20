/*
 * BQ76920.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "BQ76920.h"
#include "main.h"
#include <stdlib.h> // For abs (to compare numbers)

/**
  * @brief  Initializes the BQ76920 IC
  * @param  hi2c: Pointer to the I2C handle
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t sys_stat = 0;  // Status register value
    // Select the correct I2C address for the BQ76920 IC
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Read the SYS_STAT register to verify communication
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Reads group voltages from the BQ76920 (4S configuration, all 4 cells monitored)
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array to store the group voltages (in mV)
  * @param  offset: Offset in the array to store the voltages
  * @retval HAL_StatusTypeDef
  * @note   Pin configuration: VCO=ground, VC1=Cell 1 positive, VC2=Cell 2 positive,
  *         VC3=VC4=Cell 3 positive, VC5=Cell 4 positive.
  *         Voltages are assigned as:
  *         - Group 1 (Cell 1): VC1 - VC0 (~3600-4200 mV)
  *         - Group 2 (Cell 2): VC2 - VC1 (~3600-4200 mV)
  *         - Group 3 (Cell 3): VC3 - VC2 (~3600-4200 mV)
  *         - Group 4 (Cell 4): VC5 - VC4 (~3600-4200 mV)
  */
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset) {
    uint8_t data[10]; // Read 10 bytes for 5 cells
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, VC1_HI_REG, 1, data, 10, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Raw voltages (in ADC counts)
    uint16_t vc[5];
    for (uint8_t i = 0; i < 5; i++) {
        vc[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }

    // Convert to millivolts and assign to group voltages
    group_voltages[offset + 0] = (vc[1] - vc[0]) * 0.382; // Cell 1: VC1 - VC0 (~3600-4200 mV)
    group_voltages[offset + 1] = (vc[2] - vc[1]) * 0.382; // Cell 2: VC2 - VC1 (~3600-4200 mV)
    group_voltages[offset + 2] = (vc[3] - vc[2]) * 0.382; // Cell 3: VC3 - VC2 (~3600-4200 mV)
    group_voltages[offset + 3] = (vc[4] - vc[3]) * 0.382; // Cell 4: VC5 - VC4 (~3600-4200 mV)

    return HAL_OK;
}

/**
  * @brief  Reads pack current from the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  current: Pointer to store the current (in mA)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current) {
    uint8_t data[2];  // Buffer for 2 bytes
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Read the coulomb counter register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, CC_HI_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Convert to milliamps
    *current = (int16_t)((data[0] << 8) | data[1]);  // Combine 2 bytes
    *current *= 8.44;  // Scale to milliamps
    return HAL_OK;
}

/**
  * @brief  Balances cells by enabling balancing on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array of group voltages (in mV)
  * @param  offset: Offset in the array for the groups to balance
  * @param  balancing_mask: Pointer to store the balancing bitmask
  * @retval HAL_StatusTypeDef
  * @note   Balances Cells 1, 2, 3, and 4. CELLBAL1_REG bits:
  *         Bit 0=Cell 1, Bit 1=Cell 2, Bit 2=Cell 3, Bit 3=Cell 4
  */
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask) {
    *balancing_mask = 0;
    // Find minimum voltage among all cells (Cells 1, 2, 3, 4)
    uint16_t min_voltage = group_voltages[offset + 0]; // Start with Cell 1
    for (uint8_t i = 1; i < NUM_GROUPS_PER_IC; i++) { // Check Cells 2, 3, 4
        if (group_voltages[offset + i] < min_voltage && group_voltages[offset + i] > 0) {
            min_voltage = group_voltages[offset + i];
        }
    }

    // Balance cells that are 50 mV above the minimum
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { // Check Cells 1, 2, 3, 4
        if (group_voltages[offset + i] > min_voltage + 50 && group_voltages[offset + i] > 0) {
            *balancing_mask |= (1 << i); // Bit 0=Cell 1, Bit 1=Cell 2, Bit 2=Cell 3, Bit 3=Cell 4
        }
    }

    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, CELLBAL1_REG, 1, balancing_mask, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Checks for overvoltage and undervoltage conditions on the cells
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array of group voltages (in mV)
  * @param  offset: Offset in the array for the groups to check
  * @param  ov_flag: Pointer to store overvoltage flag
  * @param  uv_flag: Pointer to store undervoltage flag
  * @retval None
  * @note   Checks all 4 cells (Cells 1, 2, 3, 4)
  */
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag) {
    *ov_flag = 0;
    *uv_flag = 0;
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { // Check Cells 1, 2, 3, 4
        if (group_voltages[offset + i] > battery_config.ov_threshold) *ov_flag = 1;
        if (group_voltages[offset + i] < battery_config.uv_threshold) *uv_flag = 1;
    }
}

/**
  * @brief  Enables or disables charging and discharging on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  charge_enable: 1 to enable charging, 0 to disable
  * @param  discharge_enable: 1 to enable discharging, 0 to disable
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable) {
    uint8_t sys_ctrl2 = 0;
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);

    // Read the current SYS_CTRL2 register value
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_CTRL2_REG, 1, &sys_ctrl2, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Modify CHG_ON (bit 0) and DSG_ON (bit 1)
    if (charge_enable) {
        sys_ctrl2 |= (1 << 0); // Set CHG_ON
    } else {
        sys_ctrl2 &= ~(1 << 0); // Clear CHG_ON
    }
    if (discharge_enable) {
        sys_ctrl2 |= (1 << 1); // Set DSG_ON
    } else {
        sys_ctrl2 &= ~(1 << 1); // Clear DSG_ON
    }

    // Write the updated value back to SYS_CTRL2
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, SYS_CTRL2_REG, 1, &sys_ctrl2, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Reads the status register (SYS_STAT_REG) of the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  status: Pointer to store the read status
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status) {
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, status, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Clears specific status flags in the SYS_STAT_REG register
  * @param  hi2c: Pointer to the I2C handle
  * @param  flags_to_clear: The status flags to be cleared
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_ClearStatus(I2C_HandleTypeDef *hi2c, uint8_t flags_to_clear) {
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, SYS_STAT_REG, 1, &flags_to_clear, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Compares readings between two BQ76920 ICs for redundancy
  * @param  group_voltages_1: Group voltages from the first BQ76920
  * @param  group_voltages_2: Group voltages from the second BQ76920
  * @param  current_1: Current from the first BQ76920
  * @param  current_2: Current from the second BQ76920
  * @param  discrepancy_flag: Pointer to store discrepancy flag (1 = discrepancy detected)
  * @retval None
  * @note   Compares all 4 cells (Cells 1, 2, 3, 4)
  */
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag) {
    *discrepancy_flag = 0;

    // Compare group voltages for Cells 1, 2, 3, 4
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (abs(group_voltages_1[i] - group_voltages_2[i]) > 100) { // 100 mV threshold
            *discrepancy_flag = 1;
            return;
        }
    }

    // Compare currents
    if (abs(current_1 - current_2) > 500) { // 500 mA threshold
        *discrepancy_flag = 1;
    }
}

/**
  * @brief  Checks the status registers of two BQ76920 ICs and logs any faults
  * @param  hi2c1: Pointer to the I2C handle for the first BQ76920
  * @param  hi2c2: Pointer to the I2C handle for the second BQ76920
  * @param  error_flags: Pointer to store error flags
  */
void BQ76920_CheckStatus(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, uint32_t *error_flags) {
    uint8_t status1, status2;
    uint8_t clear_flags1 = 0, clear_flags2 = 0;

    // Read status from first BQ76920 (I2C1)
    if (BQ76920_ReadStatus(hi2c1, &status1) == HAL_OK) {
        // Handle each status flag
        if (status1 & (1 << 7)) { // DEVICE_XREADY
            *error_flags |= ERROR_DEVICE_XREADY;
            Log_Error("BQ76920 (I2C1): DEVICE_XREADY fault");
            clear_flags1 |= (1 << 7);
        }
        if (status1 & (1 << 6)) { // OVRD_ALERT
            *error_flags |= ERROR_OVRD_ALERT;
            Log_Error("BQ76920 (I2C1): OVRD_ALERT condition");
            clear_flags1 |= (1 << 6);
        }
        if (status1 & (1 << 5)) { // UV
            *error_flags |= ERROR_UNDERVOLTAGE;
            Log_Error("BQ76920 (I2C1): Undervoltage fault");
            clear_flags1 |= (1 << 5);
        }
        if (status1 & (1 << 4)) { // OV
            *error_flags |= ERROR_OVERVOLTAGE;
            Log_Error("BQ76920 (I2C1): Overvoltage fault");
            clear_flags1 |= (1 << 4);
        }
        if (status1 & (1 << 3)) { // SCD
            *error_flags |= ERROR_SCD;
            Log_Error("BQ76920 (I2C1): Short-circuit discharge fault");
            clear_flags1 |= (1 << 3);
        }
        if (status1 & (1 << 2)) { // OCD
            *error_flags |= ERROR_OCD;
            Log_Error("BQ76920 (I2C1): Overcurrent discharge fault");
            clear_flags1 |= (1 << 2);
        }
        if (status1 & (1 << 1)) { // OCC
            *error_flags |= ERROR_OCC;
            Log_Error("BQ76920 (I2C1): Overcurrent charge fault");
            clear_flags1 |= (1 << 1);
        }

        // Clear the handled flags
        if (clear_flags1 != 0) {
            BQ76920_ClearStatus(hi2c1, clear_flags1);
        }
    } else {
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C1)");
    }

    // Read status from second BQ76920 (I2C2)
    if (BQ76920_ReadStatus(hi2c2, &status2) == HAL_OK) {
        // Handle each status flag
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

        // Clear the handled flags
        if (clear_flags2 != 0) {
            BQ76920_ClearStatus(hi2c2, clear_flags2);
        }
    } else {
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C2)");
    }
}
