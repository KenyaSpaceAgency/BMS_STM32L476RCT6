/*
 * BQ76920.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "BQ76920.h"
#include "main.h"
#include <stdlib.h> // For abs (to compare numbers)


// Imagine the BQ76920 as a little helper with notebooks we can read or write using I2C wires

// This wakes up the BQ76920 chip to make sure it’s ready
/**
  * @brief  Initializes the BQ76920 IC
  * @param  hi2c: Pointer to the I2C handle
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t sys_stat = 0;  // A little box to hold the warning lights
    // Pick the right phone number for this chip (shifted because I2C needs it this way)
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Ask the chip: "Are you there?" by reading its status notebook
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Reads group voltages from the BQ76920 (each group is 3 cells in parallel)
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array to store the group voltages (in mV)
  * @param  offset: Offset in the array to store the voltages
  * @retval HAL_StatusTypeDef
  */
// This checks how full each battery is
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset) {
    uint8_t data[6];  // A box to hold 6 bytes (2 bytes per battery group)
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Ask the chip for the voltage notebook
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, VC1_HI_REG, 1, data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;  // Oops, something went wrong!

    // Turn the raw numbers into millivolts (like turning a code into a real number)
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        uint16_t raw = (data[i * 2] << 8) | data[i * 2 + 1];  // Combine 2 bytes into 1 number
        group_voltages[offset + i] = raw * 0.382;  // Multiply by a magic number to get millivolts
    }
    return HAL_OK;  // All done!
}




/**
  * @brief  Reads pack current from the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  current: Pointer to store the current (in mA)
  * @retval HAL_StatusTypeDef
  */
// This checks how much power is flowing
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current) {
    uint8_t data[2];  // A box for 2 bytes
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    // Ask the chip for the power flow notebook
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, CC_HI_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    // Turn the raw number into milliamps
    *current = (int16_t)((data[0] << 8) | data[1]);  // Combine 2 bytes
    *current *= 8.44;  // Multiply by a magic number to get milliamps
    return HAL_OK;
}





/**
  * @brief  Balances groups by enabling balancing on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array of group voltages (in mV)
  * @param  offset: Offset in the array for the groups to balance
  * @param  balancing_mask: Pointer to store the balancing bitmask
  * @retval HAL_StatusTypeDef
  */
// This balances the batteries so they’re all about the same
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask) {
    *balancing_mask = 0;  // Start with no balancing
    uint16_t min_voltage = group_voltages[offset];  // Find the emptiest battery
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] < min_voltage) min_voltage = group_voltages[offset + i];
    }

    // If a battery is much fuller (50 mV more), mark it for balancing
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] > min_voltage + 50) {
            *balancing_mask |= (1 << i);  // Turn on a switch for this battery
        }
    }

    // Tell the chip which batteries to balance
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, CELLBAL1_REG, 1, balancing_mask, 1, HAL_MAX_DELAY);
}





/**
  * @brief  Checks for overvoltage and undervoltage conditions on the groups
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array of group voltages (in mV)
  * @param  offset: Offset in the array for the groups to check
  * @param  ov_flag: Pointer to store overvoltage flag
  * @param  uv_flag: Pointer to store undervoltage flag
  * @retval None
  */
// This checks if batteries are too full or too empty
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag) {
    *ov_flag = 0;  // No overvoltage yet
    *uv_flag = 0;  // No undervoltage yet
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] > battery_config.ov_threshold) *ov_flag = 1;  // Too full!
        if (group_voltages[offset + i] < battery_config.uv_threshold) *uv_flag = 1;  // Too empty!
    }
}

/**
  * @brief  Checks for overcurrent conditions using the BQ76920 SYS_STAT register
  * @param  hi2c: Pointer to the I2C handle
  * @param  occ_flag: Pointer to store overcurrent in charge flag (1 = triggered)
  * @param  ocd_flag: Pointer to store overcurrent in discharge flag (1 = triggered)
  * @retval HAL_StatusTypeDef
  */
// This checks if too much power is flowing
HAL_StatusTypeDef BQ76920_CheckOvercurrent(I2C_HandleTypeDef *hi2c, uint8_t *occ_flag, uint8_t *ocd_flag) {
    uint8_t sys_stat = 0;
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    *occ_flag = (sys_stat & (1 << 2)) ? 1 : 0;  // Too much power going in
    *ocd_flag = (sys_stat & (1 << 1)) ? 1 : 0;  // Too much power going out
    return HAL_OK;
}

/**
  * @brief  Compares readings between two BQ76920 ICs for redundancy
  * @param  group_voltages_1: Group voltages from the first BQ76920
  * @param  group_voltages_2: Group voltages from the second BQ76920
  * @param  current_1: Current from the first BQ76920
  * @param  current_2: Current from the second BQ76920
  * @param  discrepancy_flag: Pointer to store discrepancy flag (1 = discrepancy detected)
  * @retval None
  */
void BQ76920_CheckRedundancy(uint16_t *group_voltages_1, uint16_t *group_voltages_2, int16_t current_1, int16_t current_2, uint8_t *discrepancy_flag)
{
    *discrepancy_flag = 0;

    // Compare group voltages (should be similar since they're measuring the same pack)
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (abs(group_voltages_1[i] - group_voltages_2[i]) > 100) { // 100 mV threshold
            *discrepancy_flag = 1;
            return;
        }
    }

    // Compare currents (should be similar since they're measuring the same pack)
    if (abs(current_1 - current_2) > 500) { // 500 mA threshold
        *discrepancy_flag = 1;
    }
}

/**
  * @brief  Enables or disables charging and discharging on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  charge_enable: 1 to enable charging, 0 to disable
  * @param  discharge_enable: 1 to enable discharging, 0 to disable
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_SetChargeEnable(I2C_HandleTypeDef *hi2c, uint8_t charge_enable, uint8_t discharge_enable)
{
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
  * @brief  Reads a specific register from the BQ76920 battery management IC using I2C communication.
  * @param  hi2c: Pointer to the I2C handle, which specifies the I2C peripheral to be used.
  * @param  reg_addr: The address of the register to be read.
  * @param  data: Pointer to a variable where the read data will be stored.
  * @retval HAL_StatusTypeDef: Status indicating the success or failure of the I2C read operation.
  */
HAL_StatusTypeDef BQ76920_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data)
{
    // Determine the I2C address based on the I2C handle provided
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);

    // Perform an I2C read operation to retrieve the value from the specified register address
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, reg_addr, 1, data, 1, HAL_MAX_DELAY);
}


/**
  * @brief  Writes a specific value to a register of the BQ76920 battery management IC using I2C communication.
  * @param  hi2c: Pointer to the I2C handle, which specifies the I2C peripheral to be used.
  * @param  reg_addr: The address of the register to be written to.
  * @param  data: The value to be written to the register.
  * @retval HAL_StatusTypeDef: Status indicating the success or failure of the I2C write operation.
  */
HAL_StatusTypeDef BQ76920_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data)
{
    // Determine the I2C address based on the I2C handle provided
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);

    // Perform an I2C write operation to write the value to the specified register address
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, reg_addr, 1, &data, 1, HAL_MAX_DELAY);
}


/**
  * @brief  Reads the status register (SYS_STAT_REG) of the BQ76920 battery management IC using I2C communication.
  * @param  hi2c: Pointer to the I2C handle, which specifies the I2C peripheral to be used.
  * @param  status: Pointer to a variable where the read status will be stored.
  * @retval HAL_StatusTypeDef: Status indicating the success or failure of the I2C read operation.
  */
HAL_StatusTypeDef BQ76920_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
    // Call the BQ76920_ReadRegister function to read the SYS_STAT_REG register
    return BQ76920_ReadRegister(hi2c, SYS_STAT_REG, status);
}


/**
  * @brief  Clears specific status flags in the SYS_STAT_REG register of the BQ76920 battery management IC using I2C communication.
  * @param  hi2c: Pointer to the I2C handle, which specifies the I2C peripheral to be used.
  * @param  flags_to_clear: The status flags to be cleared.
  * @retval HAL_StatusTypeDef: Status indicating the success or failure of the I2C write operation.
  */
HAL_StatusTypeDef BQ76920_ClearStatus(I2C_HandleTypeDef *hi2c, uint8_t flags_to_clear)
{
    // Call the BQ76920_WriteRegister function to write the flags_to_clear value to the SYS_STAT_REG register
    return BQ76920_WriteRegister(hi2c, SYS_STAT_REG, flags_to_clear);
}

/**
  * @brief  Checks the status registers of two BQ76920 battery management ICs and logs any faults.
  * @param  hi2c1: Pointer to the I2C handle for the first BQ76920.
  * @param  hi2c2: Pointer to the I2C handle for the second BQ76920.
  * @param  error_flags: Pointer to a variable where error flags will be stored.
  */
void BQ76920_CheckStatus(I2C_HandleTypeDef *hi2c1, I2C_HandleTypeDef *hi2c2, uint32_t *error_flags)
{
    uint8_t status1, status2;
    uint8_t clear_flags1 = 0, clear_flags2 = 0;

    // Read status from first BQ76920 (I2C1)
    if (BQ76920_ReadStatus(hi2c1, &status1) == HAL_OK)
    {
        // Handle each status flag
        if (status1 & (1 << 7)) // DEVICE_XREADY
        {
            *error_flags |= ERROR_DEVICE_XREADY;
            Log_Error("BQ76920 (I2C1): DEVICE_XREADY fault");
            clear_flags1 |= (1 << 7);
        }
        if (status1 & (1 << 6)) // OVRD_ALERT
        {
            *error_flags |= ERROR_OVRD_ALERT;
            Log_Error("BQ76920 (I2C1): OVRD_ALERT condition");
            clear_flags1 |= (1 << 6);
        }
        if (status1 & (1 << 5)) // UV
        {
            *error_flags |= ERROR_UNDERVOLTAGE;
            Log_Error("BQ76920 (I2C1): Undervoltage fault");
            clear_flags1 |= (1 << 5);
        }
        if (status1 & (1 << 4)) // OV
        {
            *error_flags |= ERROR_OVERVOLTAGE;
            Log_Error("BQ76920 (I2C1): Overvoltage fault");
            clear_flags1 |= (1 << 4);
        }
        if (status1 & (1 << 3)) // SCD
        {
            *error_flags |= ERROR_SCD;
            Log_Error("BQ76920 (I2C1): Short-circuit discharge fault");
            clear_flags1 |= (1 << 3);
        }
        if (status1 & (1 << 2)) // OCD
        {
            *error_flags |= ERROR_OCD;
            Log_Error("BQ76920 (I2C1): Overcurrent discharge fault");
            clear_flags1 |= (1 << 2);
        }
        if (status1 & (1 << 1)) // OCC
        {
            *error_flags |= ERROR_OCC;
            Log_Error("BQ76920 (I2C1): Overcurrent charge fault");
            clear_flags1 |= (1 << 1);
        }

        // Clear the handled flags
        if (clear_flags1 != 0)
        {
            BQ76920_ClearStatus(hi2c1, clear_flags1);
        }
    }
    else
    {
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C1)");
    }

    // Read status from second BQ76920 (I2C2)
    if (BQ76920_ReadStatus(hi2c2, &status2) == HAL_OK)
    {
        // Handle each status flag
        if (status2 & (1 << 7)) // DEVICE_XREADY
        {
            *error_flags |= ERROR_DEVICE_XREADY;
            Log_Error("BQ76920 (I2C2): DEVICE_XREADY fault");
            clear_flags2 |= (1 << 7);
        }
        if (status2 & (1 << 6)) // OVRD_ALERT
        {
            *error_flags |= ERROR_OVRD_ALERT;
            Log_Error("BQ76920 (I2C2): OVRD_ALERT condition");
            clear_flags2 |= (1 << 6);
        }
        if (status2 & (1 << 5)) // UV
        {
            *error_flags |= ERROR_UNDERVOLTAGE;
            Log_Error("BQ76920 (I2C2): Undervoltage fault");
            clear_flags2 |= (1 << 5);
        }
        if (status2 & (1 << 4)) // OV
        {
            *error_flags |= ERROR_OVERVOLTAGE;
            Log_Error("BQ76920 (I2C2): Overvoltage fault");
            clear_flags2 |= (1 << 4);
        }
        if (status2 & (1 << 3)) // SCD
        {
            *error_flags |= ERROR_SCD;
            Log_Error("BQ76920 (I2C2): Short-circuit discharge fault");
            clear_flags2 |= (1 << 3);
        }
        if (status2 & (1 << 2)) // OCD
        {
            *error_flags |= ERROR_OCD;
            Log_Error("BQ76920 (I2C2): Overcurrent discharge fault");
            clear_flags2 |= (1 << 2);
        }
        if (status2 & (1 << 1)) // OCC
        {
            *error_flags |= ERROR_OCC;
            Log_Error("BQ76920 (I2C2): Overcurrent charge fault");
            clear_flags2 |= (1 << 1);
        }

        // Clear the handled flags
        if (clear_flags2 != 0)
        {
            BQ76920_ClearStatus(hi2c2, clear_flags2);
        }
    }
    else
    {
        Log_Error("Failed to read SYS_STAT from BQ76920 (I2C2)");
    }
}


