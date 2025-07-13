/*
 * i2c_comm.h
 *
 *  Created on: Jul 2, 2025
 *      Author: yomue
 */


/*
 * i2c_comm.h
 *
 *  Created on: Jul 2, 2025
 *      Author: yomue
 */

#ifndef INC_I2C_COMM_H_
#define INC_I2C_COMM_H_

#include "stm32l4xx_hal.h"
#include "bms_service.h"      // For TelemetryData definition
#include "main.h"             // For hi2c3
#include <stdint.h>
#include "flash.h"                // For flash logging if needed
#include <stdio.h>                // For sprintf in Log_Error
#include <string.h>
#include "Temperature.h"      // For heater control functions
#include "BQ76920.h"        // For BQ76920 definitions

// Command Definitions
#define CMD_SYNC_COUNTER         0xA0
#define CMD_ENABLE_HEATER_1     0xB1
#define CMD_DISABLE_HEATER_1    0xB2
#define CMD_ENABLE_HEATER_2     0xB3
#define CMD_DISABLE_HEATER_2    0xB4
#define CMD_ENABLE_CHARGING     0xC1
#define CMD_DISABLE_CHARGING    0xC2
#define CMD_ENABLE_DISCHARGING  0xC3
#define CMD_DISABLE_DISCHARGING 0xC4
#define CMD_READ_TELEMETRY      0xD1
#define CMD_READ_FLASHED_TELEMETRY  0xE1
#define CMD_PUT_DATA                 0xF1  // PD – Put Data (block flash write)
#define CMD_READ_DATA                0xF2  // RD – Read Data (read from flash)
#define CMD_WRITE_DATA               0xF3  // WD – Write Data (direct write to flash)


// External variables used across modules
extern TelemetryData telemetry;
extern I2C_HandleTypeDef hi2c3;
extern uint8_t i2c3_rx_buffer[256];

// Function declarations
void I2C_Comm_Init(void);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_COMM_H_ */
