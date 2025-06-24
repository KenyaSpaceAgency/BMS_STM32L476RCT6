/* Role of BQ76920.c in the BMS Project:
 * This file provides a driver for the BQ76920 battery monitoring IC, used in the BMS
 * to manage a 4S2P lithium-ion battery pack in a CubeSat. It handles voltage and current
 * measurements, cell balancing, fault detection, and charge/discharge control, interfacing
 * with two BQ76920 ICs via I2C for redundancy.
 */

/* Importance of BQ76920.c in the BMS Project:
 * - Monitors Battery Health: Measures cell voltages and pack current, critical for
 *   calculating SOC, SOH, and detecting faults, ensuring safe battery operation.
 * - Ensures Safety: Implements protections against overvoltage, undervoltage, and overcurrent,
 *   preventing battery damage through hardware and software checks.
 * - Provides Redundancy: Supports two BQ76920 ICs to ensure reliable data, vital for
 *   high-reliability space applications.
 * - Supports RS485: Sends battery data and faults to the OBC via RS485 using SSP, enabling
 *   remote monitoring and diagnostics.
 * - Extends Battery Life: Balances cells to maintain uniform charge, improving longevity.
 */

/* Objective of BQ76920.c in the BMS Project:
 * The objective is to provide functions to initialize, configure, and interact with the
 * BQ76920 ICs, enabling accurate monitoring and control of the battery pack. This ensures
 * the BMS can safely manage power, respond to faults, and communicate status to the OBC,
 * supporting the CubeSat’s EPS reliability and mission success.
 */

/* Include BQ76920 header file for function declarations and constants */
#include "BQ76920.h"
/* Include main header file for BMS-specific structures and variables */
#include "main.h"
/* Include standard library for abs function to calculate differences */
#include <stdlib.h>

/* Define function to initialize a BQ76920 IC
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies which I2C interface
 *         (I2C1 or I2C2) to use for communication with the BQ76920 IC. It determines
 *         which IC is being initialized.
 * Returns: HAL_StatusTypeDef, a status code indicating success (HAL_OK) or failure
 *          (e.g., HAL_ERROR, HAL_TIMEOUT). HAL_OK means the IC responded correctly,
 *          confirming communication is established.
 * What it does: Verifies the IC is responsive by reading its status register, setting
 *               up communication for further operations.
 */
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t sys_stat = 0; /* Create a variable to store the SYS_STAT register value */
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address based on whether hi2c is I2C1 or I2C2 */
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY); /* Read 1 byte from SYS_STAT register to confirm IC communication */
}

/* Define function to read cell voltages from a BQ76920
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface
 *         (I2C1 or I2C2) for communication with the BQ76920 IC.
 * - group_voltages: Pointer to an array (uint16_t *) where cell voltages will be stored
 *                   in millivolts.
 * - offset: Starting index (uint8_t) in the group_voltages array where voltages are stored.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if voltages are read and
 *          calculated correctly, or failure (e.g., HAL_ERROR) if the I2C read fails.
 * What it does: Reads raw voltage data from the BQ76920 and converts it to millivolts
 *               for the four cells in a 4S configuration.
 */
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset) {
    uint8_t data[10]; /* Create a buffer to hold 10 bytes (5 voltage inputs, 2 bytes each) */
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address based on I2C handle */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, VC1_HI_REG, 1, data, 10, HAL_MAX_DELAY); /* Read 10 bytes from voltage registers starting at VC1_HI */
    if (status != HAL_OK) return status; /* Return error status if I2C read fails */
    uint16_t vc[5]; /* Create an array to store raw ADC counts for 5 voltage inputs */
    for (uint8_t i = 0; i < 5; i++) { /* Loop through 5 voltage inputs */
        vc[i] = (data[i * 2] << 8) | data[i * 2 + 1]; /* Combine high and low bytes into a 16-bit value */
    }
    group_voltages[offset + 0] = (vc[1] - vc[0]) * 0.382; /* Calculate Cell 1 voltage (VC1 - VC0) in millivolts */
    group_voltages[offset + 1] = (vc[2] - vc[1]) * 0.382; /* Calculate Cell 2 voltage (VC2 - VC1) in millivolts */
    group_voltages[offset + 2] = (vc[3] - vc[2]) * 0.382; /* Calculate Cell 3 voltage (VC3 - VC2) in millivolts */
    group_voltages[offset + 3] = (vc[4] - vc[3]) * 0.382; /* Calculate Cell 4 voltage (VC5 - VC4) in millivolts */
    return HAL_OK; /* Return success status */
}

/* Define function to read pack current from a BQ76920
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface
 *         for communication with the BQ76920 IC.
 * - current: Pointer to a variable (int16_t *) to store the pack current in milliamps.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if current is read and scaled
 *          correctly, or failure (e.g., HAL_ERROR) if the I2C read fails.
 * What it does: Reads the current flowing through the battery pack using a shunt resistor,
 *               scaling the raw data to milliamps.
 */
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current) {
    uint8_t data[2]; /* Create a buffer to hold 2 bytes (high and low current values) */
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address based on I2C handle */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, CC_HI_REG, 1, data, 2, HAL_MAX_DELAY); /* Read 2 bytes from current registers */
    if (status != HAL_OK) return status; /* Return error status if I2C read fails */
    *current = (int16_t)((data[0] << 8) | data[1]); /* Combine high and low bytes into a signed 16-bit value */
    *current *= 1.688f; /* Scale raw value to milliamps using 1.688 mA per LSB */
    return HAL_OK; /* Return success status */
}

/* Define function to balance battery cells
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface
 *         for communication with the BQ76920 IC.
 * - group_voltages: Pointer to an array (uint16_t *) containing cell voltages in millivolts.
 * - offset: Starting index (uint8_t) in the group_voltages array for cells to balance.
 * - balancing_mask: Pointer to a variable (uint8_t *) to store the balancing bitmask.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if the balancing mask is written
 *          successfully, or failure (e.g., HAL_ERROR) if the I2C write fails.
 * What it does: Identifies cells with high voltages and enables balancing to equalize
 *               charge, writing a bitmask to the BQ76920 to control which cells are balanced.
 */
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask) {
    *balancing_mask = 0; /* Set balancing mask to zero to start with no cells selected */
    uint16_t min_voltage = group_voltages[offset + 0]; /* Set initial minimum voltage to first cell’s value */
    for (uint8_t i = 1; i < NUM_GROUPS_PER_IC; i++) { /* Loop through remaining cells to find minimum voltage */
        if (group_voltages[offset + i] < min_voltage && group_voltages[offset + i] > 0) { /* Check if voltage is lower and valid */
            min_voltage = group_voltages[offset + i]; /* Update minimum voltage */
        }
    }
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cells to determine balancing */
        if (group_voltages[offset + i] > min_voltage + 50 && group_voltages[offset + i] > 0) { /* Check if voltage is 50 mV above minimum */
            *balancing_mask |= (1 << i); /* Set corresponding bit in mask to enable balancing */
        }
    }
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address */
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, CELLBAL1_REG, 1, balancing_mask, 1, HAL_MAX_DELAY); /* Write balancing mask to CELLBAL1 register */
}

/* Define function to check for overvoltage and undervoltage conditions
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface
 *         for communication with the BQ76920 IC.
 * - group_voltages: Pointer to an array (uint16_t *) containing cell voltages in millivolts.
 * - offset: Starting index (uint8_t) in the group_voltages array for cells to check.
 * - ov_flag: Pointer to a variable (uint8_t *) to store overvoltage flag (1 if detected).
 * - uv_flag: Pointer to a variable (uint8_t *) to store undervoltage flag (1 if detected).
 * Returns: void, meaning it returns nothing; it modifies ov_flag and uv_flag directly to
 *          indicate detected conditions.
 * What it does: Compares cell voltages against predefined thresholds to detect overvoltage
 *               or undervoltage conditions, setting flags to indicate issues.
 */
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag) {
    *ov_flag = 0; /* Set overvoltage flag to zero */
    *uv_flag = 0; /* Set undervoltage flag to zero */
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through all cells */
        if (group_voltages[offset + i] > battery_config.ov_threshold) *ov_flag = 1; /* Set flag if voltage exceeds overvoltage threshold */
        if (group_voltages[offset + i] < battery_config.uv_threshold) *uv_flag = 1; /* Set flag if voltage is below undervoltage threshold */
    }
}

/* Define function to enable or disable charging and discharging
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface.
 * - charge_enable: Flag (uint8_t), 1 to enable charging, 0 to disable it.
 * - discharge_enable: Flag (uint8_t), 1 to enable discharging, 0 to disable it.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if the control register is
 *          updated successfully, or failure (e.g., HAL_ERROR) if I2C operations fail.
 * What it does: Modifies the BQ76920’s control register to turn on or off the charging
 *               and discharging FETs, controlling power flow.
 */
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable) {
    uint8_t sys_ctrl2 = 0; /* Create a variable to store SYS_CTRL2 register value */
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_CTRL2_REG, 1, &sys_ctrl2, 1, HAL_MAX_DELAY); /* Read current SYS_CTRL2 value */
    if (status != HAL_OK) return status; /* Return error if I2C read fails */
    if (charge_enable) { /* Check if charging should be enabled */
        sys_ctrl2 |= (1 << 0); /* Set CHG_ON bit to enable charging */
    } else { /* Charging should be disabled */
        sys_ctrl2 &= ~(1 << 0); /* Clear CHG_ON bit to disable charging */
    }
    if (discharge_enable) { /* Check if discharging should be enabled */
        sys_ctrl2 |= (1 << 1); /* Set DSG_ON bit to enable discharging */
    } else { /* Discharging should be disabled */
        sys_ctrl2 &= ~(1 << 1); /* Clear DSG_ON bit to disable discharging */
    }
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, SYS_CTRL2_REG, 1, &sys_ctrl2, 1, HAL_MAX_DELAY); /* Write updated SYS_CTRL2 value to register */
}

/* Define function to read the status register of a BQ76920
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface.
 * - status: Pointer to a variable (uint8_t *) to store the status register value.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if the status is read
 *          successfully, or failure (e.g., HAL_ERROR) if the I2C read fails.
 * What it does: Reads the SYS_STAT register to check for hardware-detected faults
 *               like overvoltage or overcurrent.
 */
HAL_StatusTypeDef BQ76920_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status) {
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address */
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, status, 1, HAL_MAX_DELAY); /* Read 1 byte from SYS_STAT register */
}

/* Define function to clear specific status flags in the BQ76920
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface.
 * - flags_to_clear: Value (uint8_t) indicating which status flags to clear.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if flags are cleared
 *          successfully, or failure (e.g., HAL_ERROR) if the I2C write fails.
 * What it does: Writes to the SYS_STAT register to reset specified fault flags,
 *               preventing repeated fault triggers.
 */
HAL_StatusTypeDef BQ76920_ClearStatus(I2C_HandleTypeDef *hi2c, uint8_t flags_to_clear) {
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address */
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, SYS_STAT_REG, 1, &flags_to_clear, 1, HAL_MAX_DELAY); /* Write flags to SYS_STAT to clear them */
}

/* Define function to compare readings between two BQ76920 ICs for redundancy
 * Inputs:
 * - group_voltages_1: Pointer to an array (uint16_t *) of voltages from the first IC.
 * - group_voltages_2: Pointer to an array (uint16_t *) of voltages from the second IC.
 * - current_1: Current value (int16_t) from the first IC in milliamps.
 * - current_2: Current value (int16_t) from the second IC in milliamps.
 * - discrepancy_flag: Pointer to a variable (uint8_t *) to store discrepancy flag
 *                     (1 if differences are found).
 * Returns: void, meaning it returns nothing; it modifies discrepancy_flag to indicate
 *          if significant differences are detected.
 * What it does: Checks if voltage or current readings from two ICs differ significantly,
 *               setting a flag to indicate potential hardware issues.
 */
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag) {
    *discrepancy_flag = 0; /* Set discrepancy flag to zero */
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cell voltages */
        if (abs(group_voltages_1[i] - group_voltages_2[i]) > 100) { /* Check if voltage difference exceeds 100 mV */
            *discrepancy_flag = 1; /* Set discrepancy flag */
            return; /* Exit function early */
        }
    }
    if (abs(current_1 - current_2) > 500) { /* Check if current difference exceeds 500 mA */
        *discrepancy_flag = 1; /* Set discrepancy flag */
    }
}

/* Define function to configure hardware protection thresholds
 * Inputs:
 * - hi2c: Pointer to an I2C handle (I2C_HandleTypeDef *), specifies the I2C interface.
 * Returns: HAL_StatusTypeDef, indicating success (HAL_OK) if protection thresholds
 *          are set successfully, or failure (e.g., HAL_ERROR) if I2C writes fail.
 * What it does: Sets thresholds for overcurrent and short-circuit protection in the
 *               BQ76920’s registers, enabling hardware-level safety features.
 */
HAL_StatusTypeDef BQ76920_ConfigureProtection(I2C_HandleTypeDef *hi2c) {
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1); /* Select I2C address */
    HAL_StatusTypeDef status; /* Create a variable to store I2C operation status */
    uint8_t protect1 = (0x4 << 3) | (0x4); /* Set OCC and OCD thresholds to 60 mV (12 A) */
    status = HAL_I2C_Mem_Write(hi2c, i2c_addr, PROTECT1_REG, 1, &protect1, 1, HAL_MAX_DELAY); /* Write PROTECT1 register */
    if (status != HAL_OK) return status; /* Return error if I2C write fails */
    uint8_t protect2 = (0x5 << 3) | (0x1); /* Set SCD threshold to 100 mV (20 A), delay to 15 µs */
    status = HAL_I2C_Mem_Write(hi2c, i2c_addr, PROTECT2_REG, 1, &protect2, 1, HAL_MAX_DELAY); /* Write PROTECT2 register */
    if (status != HAL_OK) return status; /* Return error if I2C write fails */
    return HAL_OK; /* Return success status */
}

/* Define function to check status registers of two BQ76920 ICs
 * Inputs:
 * - hi2c1: Pointer to an I2C handle (I2C_HandleTypeDef *) for the first BQ76920 IC.
 * - hi2c2: Pointer to an I2C handle (I2C_HandleTypeDef *) for the second BQ76920 IC.
 * - error_flags: Pointer to a variable (uint32_t *) to store BMS error flags.
 * Returns: void, meaning it returns nothing; it modifies error_flags and logs faults
 *          directly to indicate detected issues.
 * What it does: Reads status registers from both ICs, logs detected faults (e.g.,
 *               overvoltage, overcurrent), sets error flags, and clears fault conditions.
 */
void BQ76920_CheckStatus(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, uint32_t *error_flags) {
    uint8_t status1, status2; /* Create variables to store status register values */
    uint8_t clear_flags1 = 0, clear_flags2 = 0; /* Create variables to track flags to clear */
    if (BQ76920_ReadStatus(hi2c1, &status1) == HAL_OK) { /* Read status from first IC */
        if (status1 & (1 << 7)) { /* Check if DEVICE_XREADY fault bit is set */
            *error_flags |= ERROR_DEVICE_XREADY; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): DEVICE_XREADY fault"); /* Log fault message */
            clear_flags1 |= (1 << 7); /* Mark DEVICE_XREADY flag for clearing */
        }
        if (status1 & (1 << 6)) { /* Check if OVRD_ALERT condition bit is set */
            *error_flags |= ERROR_OVRD_ALERT; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): OVRD_ALERT condition"); /* Log condition message */
            clear_flags1 |= (1 << 6); /* Mark OVRD_ALERT flag for clearing */
        }
        if (status1 & (1 << 5)) { /* Check if undervoltage fault bit is set */
            *error_flags |= ERROR_UNDERVOLTAGE; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): Undervoltage fault"); /* Log fault message */
            clear_flags1 |= (1 << 5); /* Mark undervoltage flag for clearing */
        }
        if (status1 & (1 << 4)) { /* Check if overvoltage fault bit is set */
            *error_flags |= ERROR_OVERVOLTAGE; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): Overvoltage fault"); /* Log fault message */
            clear_flags1 |= (1 << 4); /* Mark overvoltage flag for clearing */
        }
        if (status1 & (1 << 3)) { /* Check if short-circuit discharge fault bit is set */
            *error_flags |= ERROR_SCD; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): Short-circuit discharge fault"); /* Log fault message */
            clear_flags1 |= (1 << 3); /* Mark short-circuit flag for clearing */
        }
        if (status1 & (1 << 2)) { /* Check if overcurrent discharge fault bit is set */
            *error_flags |= ERROR_OCD; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): Overcurrent discharge fault"); /* Log fault message */
            clear_flags1 |= (1 << 2); /* Mark overcurrent discharge flag for clearing */
        }
        if (status1 & (1 << 1)) { /* Check if overcurrent charge fault bit is set */
            *error_flags |= ERROR_OCC; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C1): Overcurrent charge fault"); /* Log fault message */
            clear_flags1 |= (1 << 1); /* Mark overcurrent charge flag for clearing */
        }
        if (clear_flags1 != 0) { /* Check if any flags need to be cleared */
            BQ76920_ClearStatus(hi2c1, clear_flags1); /* Clear marked flags in first IC */
        }
    } else { /* If status read fails */
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C1)"); /* Log error message */
    }
    if (BQ76920_ReadStatus(hi2c2, &status2) == HAL_OK) { /* Read status from second IC */
        if (status2 & (1 << 7)) { /* Check if DEVICE_XREADY fault bit is set */
            *error_flags |= ERROR_DEVICE_XREADY; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): DEVICE_XREADY fault"); /* Log fault message */
            clear_flags2 |= (1 << 7); /* Mark DEVICE_XREADY flag for clearing */
        }
        if (status2 & (1 << 6)) { /* Check if OVRD_ALERT condition bit is set */
            *error_flags |= ERROR_OVRD_ALERT; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): OVRD_ALERT condition"); /* Log condition message */
            clear_flags2 |= (1 << 6); /* Mark OVRD_ALERT flag for clearing */
        }
        if (status2 & (1 << 5)) { /* Check if undervoltage fault bit is set */
            *error_flags |= ERROR_UNDERVOLTAGE; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): Undervoltage fault"); /* Log fault message */
            clear_flags2 |= (1 << 5); /* Mark undervoltage flag for clearing */
        }
        if (status2 & (1 << 4)) { /* Check if overvoltage fault bit is set */
            *error_flags |= ERROR_OVERVOLTAGE; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): Overvoltage fault"); /* Log fault message */
            clear_flags2 |= (1 << 4); /* Mark overvoltage flag for clearing */
        }
        if (status2 & (1 << 3)) { /* Check if short-circuit discharge fault bit is set */
            *error_flags |= ERROR_SCD; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): Short-circuit discharge fault"); /* Log fault message */
            clear_flags2 |= (1 << 3); /* Mark short-circuit flag for clearing */
        }
        if (status2 & (1 << 2)) { /* Check if overcurrent discharge fault bit is set */
            *error_flags |= ERROR_OCD; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): Overcurrent discharge fault"); /* Log fault message */
            clear_flags2 |= (1 << 2); /* Mark overcurrent discharge flag for clearing */
        }
        if (status2 & (1 << 1)) { /* Check if overcurrent charge fault bit is set */
            *error_flags |= ERROR_OCC; /* Set corresponding BMS error flag */
            Log_Error("BQ76920 (I2C2): Overcurrent charge fault"); /* Log fault message */
            clear_flags2 |= (1 << 1); /* Mark overcurrent charge flag for clearing */
        }
        if (clear_flags2 != 0) { /* Check if any flags need to be cleared */
            BQ76920_ClearStatus(hi2c2, clear_flags2); /* Clear marked flags in second IC */
        }
    } else { /* If status read fails */
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C2)"); /* Log error message */
    }
}
