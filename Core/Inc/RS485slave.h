/*
 * RS485slave.h
 *
 *  Created on: Jul 6, 2025
 *      Author: yomue
 */

#ifndef __RS485SLAVE_H
#define __RS485SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bms_service.h"

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
#define BMS_SLAVE_ADDRESS   0x43
#define DST_BMS   0x43  // Unique ID for your BMS
#define SRC_OBC   0x01
#define SRC_EPS   0x02
#define MAX_SSP_FRAME_SIZE  256


void RS485_Comm_Init(void);
void RS485_ProcessFrame(uint8_t *buffer, uint16_t length);
void UpdateSyncCounterFromBytes(uint8_t *bytes);
void RS485_Handle_Received_Frame(uint8_t *frame, uint16_t length);
uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length);


extern uint64_t g_sync_counter;

#ifdef __cplusplus
}
#endif

#endif // __RS485SLAVE_H
