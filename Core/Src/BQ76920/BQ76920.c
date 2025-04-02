/*
 * BQ76920.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "BQ76920.h"
#include "main.h"
#include <stdlib.h> // Added for abs function

/**
  * @brief  Initializes the BQ76920 IC
  * @param  hi2c: Pointer to the I2C handle
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t sys_stat = 0;
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Reads group voltages from the BQ76920 (each group is 3 cells in parallel)
  * @param  hi2c: Pointer to the I2C handle
  * @param  group_voltages: Array to store the group voltages (in mV)
  * @param  offset: Offset in the array to store the voltages
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset)
{
    uint8_t data[6];
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, VC1_HI_REG, 1, data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        uint16_t raw = (data[i * 2] << 8) | data[i * 2 + 1];
        group_voltages[offset + i] = raw * 0.382; // Convert to mV (approximate scaling)
    }
    return HAL_OK;
}

/**
  * @brief  Reads pack current from the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  current: Pointer to store the current (in mA)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_ReadCurrent(I2C_HandleTypeDef *hi2c, int16_t *current)
{
    uint8_t data[2];
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, CC_HI_REG, 1, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    *current = (int16_t)((data[0] << 8) | data[1]);
    *current *= 8.44; // Convert to mA (approximate scaling)
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
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *balancing_mask)
{
    *balancing_mask = 0;
    uint16_t min_voltage = group_voltages[offset];
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] < min_voltage) min_voltage = group_voltages[offset + i];
    }

    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] > min_voltage + 50) { // 50 mV threshold
            *balancing_mask |= (1 << i);
        }
    }

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
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *group_voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag)
{
    *ov_flag = 0;
    *uv_flag = 0;
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        if (group_voltages[offset + i] > OV_THRESHOLD) *ov_flag = 1;
        if (group_voltages[offset + i] < UV_THRESHOLD) *uv_flag = 1;
    }
}

/**
  * @brief  Checks for overcurrent conditions using the BQ76920 SYS_STAT register
  * @param  hi2c: Pointer to the I2C handle
  * @param  occ_flag: Pointer to store overcurrent in charge flag (1 = triggered)
  * @param  ocd_flag: Pointer to store overcurrent in discharge flag (1 = triggered)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_CheckOvercurrent(I2C_HandleTypeDef *hi2c, uint8_t *occ_flag, uint8_t *ocd_flag)
{
    uint8_t sys_stat = 0;
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, SYS_STAT_REG, 1, &sys_stat, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    *occ_flag = (sys_stat & (1 << 2)) ? 1 : 0; // OCC bit
    *ocd_flag = (sys_stat & (1 << 1)) ? 1 : 0; // OCD bit
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
