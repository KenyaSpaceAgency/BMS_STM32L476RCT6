/*
 * BQ76920.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "BQ76920.h"
#include "main.h"

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
  * @brief  Reads cell voltages from the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  voltages: Array to store the cell voltages (in mV)
  * @param  offset: Offset in the array to store the voltages
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_ReadVoltages(I2C_HandleTypeDef *hi2c, uint16_t *voltages, uint8_t offset)
{
    uint8_t data[6];
    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, i2c_addr, VC1_HI_REG, 1, data, 6, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;

    for (uint8_t i = 0; i < NUM_CELLS_PER_IC; i++) {
        uint16_t raw = (data[i * 2] << 8) | data[i * 2 + 1];
        voltages[offset + i] = raw * 0.382; // Convert to mV (approximate scaling)
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
  * @brief  Balances cells by enabling balancing on the BQ76920
  * @param  hi2c: Pointer to the I2C handle
  * @param  voltages: Array of cell voltages (in mV)
  * @param  offset: Offset in the array for the cells to balance
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76920_BalanceCells(I2C_HandleTypeDef *hi2c, uint16_t *voltages, uint8_t offset)
{
    uint8_t balance_bits = 0;
    uint16_t min_voltage = voltages[offset];
    for (uint8_t i = 0; i < NUM_CELLS_PER_IC; i++) {
        if (voltages[offset + i] < min_voltage) min_voltage = voltages[offset + i];
    }

    for (uint8_t i = 0; i < NUM_CELLS_PER_IC; i++) {
        if (voltages[offset + i] > min_voltage + 50) { // 50 mV threshold
            balance_bits |= (1 << i);
        }
    }

    uint16_t i2c_addr = (hi2c == &hi2c1) ? (BQ76920_I2C_ADDRESS_1 << 1) : (BQ76920_I2C_ADDRESS_2 << 1);
    return HAL_I2C_Mem_Write(hi2c, i2c_addr, CELLBAL1_REG, 1, &balance_bits, 1, HAL_MAX_DELAY);
}

/**
  * @brief  Checks for overvoltage and undervoltage conditions
  * @param  hi2c: Pointer to the I2C handle
  * @param  voltages: Array of cell voltages (in mV)
  * @param  offset: Offset in the array for the cells to check
  * @param  ov_flag: Pointer to store overvoltage flag
  * @param  uv_flag: Pointer to store undervoltage flag
  * @retval None
  */
void BQ76920_CheckProtection(I2C_HandleTypeDef *hi2c, uint16_t *voltages, uint8_t offset, uint8_t *ov_flag, uint8_t *uv_flag)
{
    *ov_flag = 0;
    *uv_flag = 0;
    for (uint8_t i = 0; i < NUM_CELLS_PER_IC; i++) {
        if (voltages[offset + i] > OV_THRESHOLD) *ov_flag = 1;
        if (voltages[offset + i] < UV_THRESHOLD) *uv_flag = 1;
    }
}
