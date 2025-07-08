// File: rs485slave.c
#include "rs485slave.h"
#include "main.h"
#include "flash.h"
#include "Temperature.h"
#include "log.h"
#include <string.h>

#define SSP_FLAG           0xC0
#define MAX_SSP_FRAME_SIZE 256
#define DE_ENABLE()        HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_SET)
#define DE_DISABLE()       HAL_GPIO_WritePin(RS4852_DE_GPIO_Port, RS4852_DE_Pin, GPIO_PIN_RESET)

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern BQ76920_t bms_instance1;
extern BQ76920_t bms_instance2;
extern TelemetryData telemetry;

static uint8_t rs485_rx_buffer[MAX_SSP_FRAME_SIZE];
static uint8_t rs485_tx_buffer[MAX_SSP_FRAME_SIZE];

uint16_t Calculate_CRC16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void RS485_Comm_Init(void) {
    HAL_UART_Receive_DMA(&huart2, rs485_rx_buffer, MAX_SSP_FRAME_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (rs485_rx_buffer[0] != SSP_FLAG || rs485_rx_buffer[rs485_rx_buffer[1] + 8 + 1] != SSP_FLAG) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "SSP Frame missing start/end flag\n");
            return;
        }

        uint8_t dest = rs485_rx_buffer[1];
        if (dest != DST_BMS) {
            return; // Ignore commands not addressed to BMS
        }
        uint8_t src = rs485_rx_buffer[2];
        uint8_t cmd_id = rs485_rx_buffer[3];
        uint8_t len = rs485_rx_buffer[4];
        uint8_t *data = &rs485_rx_buffer[5];
        uint16_t crc_received = (rs485_rx_buffer[5 + len] | (rs485_rx_buffer[6 + len] << 8));
        uint16_t crc_calc = Calculate_CRC16(&rs485_rx_buffer[1], 4 + len);

        if (crc_calc != crc_received) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "CRC mismatch\n");
            return;
        }

        RS485_Handle_Received_Frame(&rs485_rx_buffer[1], 4 + len);

        HAL_UART_Receive_DMA(&huart2, rs485_rx_buffer, MAX_SSP_FRAME_SIZE);
    }
}

void RS485_Send_Response(uint8_t src, uint8_t dest, uint8_t cmd_id, uint8_t *payload, uint8_t len) {
    uint8_t pos = 0;
    rs485_tx_buffer[pos++] = SSP_FLAG;
    rs485_tx_buffer[pos++] = dest;
    rs485_tx_buffer[pos++] = src;
    rs485_tx_buffer[pos++] = cmd_id | 0x40; // Mark as reply
    rs485_tx_buffer[pos++] = len;
    memcpy(&rs485_tx_buffer[pos], payload, len);
    pos += len;
    uint16_t crc = Calculate_CRC16(&rs485_tx_buffer[1], 4 + len);
    rs485_tx_buffer[pos++] = crc & 0xFF;
    rs485_tx_buffer[pos++] = (crc >> 8) & 0xFF;
    rs485_tx_buffer[pos++] = SSP_FLAG;

    DE_ENABLE();
    HAL_UART_Transmit(&huart2, rs485_tx_buffer, pos, 100);
    DE_DISABLE();
}

void RS485_Handle_Received_Frame(uint8_t *frame, uint16_t length) {
    if (length < 9) return;  // Minimum SSP frame length (with empty data)

    // Validate start and end flags
    if (frame[0] != 0xC0 || frame[length - 1] != 0xC0) return;

    uint8_t dest = frame[1];
    uint8_t src = frame[2];
    uint8_t cmd_id = frame[3];
    uint8_t data_len = frame[4];

    if (dest != DST_BMS) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "RS485: Frame not addressed to BMS (got 0x%02X)", dest);
        return;
    }

    if (src != SRC_OBC && src != SRC_EPS) {
        Log_Message(BMS_MSG_LEVEL_WARNING, "RS485: Frame from unauthorized SRC 0x%02X", src);
        return;
    }

    // Validate CRC
    uint16_t crc_received = (frame[length - 3] << 8) | frame[length - 2];
    uint16_t crc_calculated = Calculate_CRC16_CCITT(&frame[1], length - 4);  // Exclude start flag and CRC and end flag

    if (crc_received != crc_calculated) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "RS485: CRC mismatch (expected 0x%04X, got 0x%04X)", crc_calculated, crc_received);
        return;
    }

    // ✅ Valid frame → Dispatch
    uint8_t *data = &frame[5];
    switch (cmd_id) {
        case CMD_ENABLE_HEATER_1: Heater1_On(); break;
        case CMD_DISABLE_HEATER_1: Heater1_Off(); break;
        case CMD_ENABLE_HEATER_2: Heater2_On(); break;
        case CMD_DISABLE_HEATER_2: Heater2_Off(); break;
        case CMD_ENABLE_CHARGING:
            turnCHGOn(&bms_instance1);
            turnCHGOn(&bms_instance2);
            break;
        case CMD_DISABLE_CHARGING:
            turnCHGOff(&bms_instance1);
            turnCHGOff(&bms_instance2);
            break;
        case CMD_ENABLE_DISCHARGING:
            turnDSGOn(&bms_instance1);
            turnDSGOn(&bms_instance2);
            break;
        case CMD_DISABLE_DISCHARGING:
            turnDSGOff(&bms_instance1);
            turnDSGOff(&bms_instance2);
            break;
        case CMD_READ_TELEMETRY:
            RS485_Send_Response(frame[1], frame[0], CMD_READ_TELEMETRY, (uint8_t*)&telemetry, sizeof(telemetry));
            break;
        case CMD_READ_FLASHED_TELEMETRY: {
            TelemetrySnapshot snapshot;
            memcpy(&snapshot, (void*)FLASH_USER_START_ADDR, sizeof(TelemetrySnapshot));
            uint16_t expected_crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
            if (snapshot.crc == expected_crc && snapshot.version == TELEMETRY_VERSION) {
                RS485_Send_Response(frame[1], frame[0], CMD_READ_FLASHED_TELEMETRY,
                    (uint8_t*)&snapshot.telemetry, sizeof(snapshot.telemetry));
            } else {
                Log_Message(BMS_MSG_LEVEL_ERROR, "Invalid snapshot in flash\n");
            }
            break;
        }
    }
}
