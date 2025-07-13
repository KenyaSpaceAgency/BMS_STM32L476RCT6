// File: i2c_comm.c

#include "i2c_comm.h"
#include "i2c_command_handlers.h"
#include "main.h"
#include "flash.h"
#include <string.h>
#include "Temperature.h"  // for Heater1_On/Off, Heater2_On/Off

extern BQ76920_t bms_instance1;
extern BQ76920_t bms_instance2;

extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

uint8_t i2c3_rx_buffer[256];
uint8_t i2c3_tx_buffer[256];

// 🟡 Global sync counter (64-bit)
uint64_t g_sync_counter = 0;

uint8_t EPS_I2C_CRC8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

void UpdateSyncCounterFromBytes(uint8_t *bytes) {
    g_sync_counter = 0;
    for (int i = 0; i < 8; i++) {
        g_sync_counter = (g_sync_counter << 8) | bytes[i];  // MSB first
    }
    Log_Message(BMS_MSG_LEVEL_INFO, "Sync counter updated: %llu", g_sync_counter);
}

void I2C_Comm_Init(void) {
    if (HAL_I2C_Slave_Receive_DMA(&hi2c3, i2c3_rx_buffer, sizeof(i2c3_rx_buffer)) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "I2C3 RX DMA init failed");
    } else {
        Log_Message(BMS_MSG_LEVEL_INFO, "I2C3 RX DMA initialized");
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C3) return;

    uint8_t cmd           = i2c3_rx_buffer[0];
    uint8_t len           = i2c3_rx_buffer[1];
    uint8_t crc_received  = i2c3_rx_buffer[2 + len];
    uint8_t crc_computed  = EPS_I2C_CRC8(&i2c3_rx_buffer[0], 2 + len);  // ✅ full-frame CRC
    uint8_t tx_len        = 0;

    Log_Message(BMS_MSG_LEVEL_DEBUG, "I2C3 received CMD: 0x%02X, len: %u", cmd, len);

    if (crc_computed != crc_received) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "I2C3 CRC mismatch: expected 0x%02X, got 0x%02X", crc_computed, crc_received);
        goto restart_dma;
    }

    if (cmd == CMD_SYNC_COUNTER) {
        if (len >= 8) {
            UpdateSyncCounterFromBytes(&i2c3_rx_buffer[2]);
            telemetry.sync_valid = 1;
        } else {
            Log_Message(BMS_MSG_LEVEL_WARNING, "SYNC command received with insufficient data");
        }
    }

    switch (cmd) {
        case CMD_ENABLE_HEATER_1: case CMD_DISABLE_HEATER_1:
        case CMD_ENABLE_HEATER_2: case CMD_DISABLE_HEATER_2:
        {
            if (len != 0) {
                Log_Message(BMS_MSG_LEVEL_ERROR, "CMD 0x%02X: Unexpected length %u", cmd, len);
                break;
            }

            if (cmd == CMD_ENABLE_HEATER_1)        Heater1_On();
            else if (cmd == CMD_DISABLE_HEATER_1)  Heater1_Off();
            else if (cmd == CMD_ENABLE_HEATER_2)   Heater2_On();
            else if (cmd == CMD_DISABLE_HEATER_2)  Heater2_Off();

            Log_Message(BMS_MSG_LEVEL_INFO, "Heater command 0x%02X executed", cmd);
            break;
        }

        case CMD_ENABLE_CHARGING:
            if (len == 0) {
                turnCHGOn(&bms_instance1);
                turnCHGOn(&bms_instance2);
                Log_Message(BMS_MSG_LEVEL_INFO, "Charging enabled");
            } else {
                Log_Message(BMS_MSG_LEVEL_ERROR, "CMD 0x%02X: Unexpected length %u", cmd, len);
            }
            break;

        case CMD_DISABLE_CHARGING:
            if (len == 0) {
                turnCHGOff(&bms_instance1);
                turnCHGOff(&bms_instance2);
                Log_Message(BMS_MSG_LEVEL_INFO, "Charging disabled");
            } else {
                Log_Message(BMS_MSG_LEVEL_ERROR, "CMD 0x%02X: Unexpected length %u", cmd, len);
            }
            break;

        case CMD_DISABLE_DISCHARGING:
            if (len == 0) {
                turnDSGOff(&bms_instance1);
                turnDSGOff(&bms_instance2);
                Log_Message(BMS_MSG_LEVEL_INFO, "Discharging disabled");
            } else {
                Log_Message(BMS_MSG_LEVEL_ERROR, "CMD 0x%02X: Unexpected length %u", cmd, len);
            }
            break;

        case CMD_READ_TELEMETRY:
        {
            // Struct to send over I2C
            typedef struct __attribute__((__packed__)) {
                uint8_t cmd;
                uint8_t len;
                TelemetryData data;
                uint8_t crc;
            } CMD_ReadTelemetryResponse_t;

            // Compute size
            const uint16_t response_size = sizeof(CMD_ReadTelemetryResponse_t);
            static CMD_ReadTelemetryResponse_t response;

            // Populate struct
            response.cmd = CMD_READ_TELEMETRY;
            response.len = sizeof(TelemetryData);
            memcpy(&response.data, &telemetry, sizeof(TelemetryData));
            response.crc = EPS_I2C_CRC8((uint8_t *)&response.cmd,
                                        offsetof(CMD_ReadTelemetryResponse_t, crc));

            // Cast to raw byte stream
            const uint8_t *raw_ptr = (const uint8_t *)&response;

            // Send in 253-byte chunks max (HAL limit is usually 255, allow margin)
            const uint16_t max_chunk = 253;
            uint16_t offset = 0;
            uint8_t frame_index = 0;

            while (offset < response_size) {
                uint16_t chunk_len = (response_size - offset > max_chunk)
                                   ? max_chunk
                                   : (response_size - offset);

                memcpy(i2c3_tx_buffer, &raw_ptr[offset], chunk_len);

                HAL_StatusTypeDef tx_status = HAL_I2C_Slave_Transmit(&hi2c3, i2c3_tx_buffer, chunk_len, 100);
                if (tx_status != HAL_OK) {
                    Log_Message(BMS_MSG_LEVEL_ERROR, "I2C3 TX failed at chunk %u (offset=%u)", frame_index, offset);
                    break;
                }

                Log_Message(BMS_MSG_LEVEL_DEBUG, "Sent telemetry chunk %u, bytes: %u", frame_index, chunk_len);

                offset += chunk_len;
                frame_index++;
            }

            break;
        }

        case CMD_READ_FLASHED_TELEMETRY:
        {
            TelemetrySnapshot snapshot;
            memcpy(&snapshot, (void*)FLASH_USER_START_ADDR, sizeof(TelemetrySnapshot));

            uint16_t expected_crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
            if (snapshot.crc != expected_crc || snapshot.version != TELEMETRY_VERSION) {
                Log_Message(BMS_MSG_LEVEL_ERROR, "Flashed telemetry invalid (CRC 0x%04X != 0x%04X or version mismatch)",
                            expected_crc, snapshot.crc);
                break;
            }

            typedef struct __attribute__((__packed__)) {
                uint8_t cmd;
                uint8_t len;
                TelemetryData data;
                uint8_t crc;
            } CMD_ReadFlashedTelemetryResponse_t;

            CMD_ReadFlashedTelemetryResponse_t response;

            response.cmd = CMD_READ_FLASHED_TELEMETRY;
            response.len = sizeof(TelemetryData);
            memcpy(&response.data, &snapshot.telemetry, sizeof(TelemetryData));
            response.crc = EPS_I2C_CRC8((uint8_t *)&response.cmd,
                                        offsetof(CMD_ReadFlashedTelemetryResponse_t, crc));

            const uint8_t *raw_ptr = (const uint8_t *)&response;
            const uint16_t total_size = sizeof(CMD_ReadFlashedTelemetryResponse_t);
            const uint16_t max_chunk = 253;

            uint16_t offset = 0;
            uint8_t chunk_index = 0;

            while (offset < total_size) {
                uint16_t chunk_len = (total_size - offset > max_chunk) ? max_chunk : (total_size - offset);
                memcpy(i2c3_tx_buffer, &raw_ptr[offset], chunk_len);

                HAL_StatusTypeDef tx_status = HAL_I2C_Slave_Transmit(&hi2c3, i2c3_tx_buffer, chunk_len, 100);
                if (tx_status != HAL_OK) {
                    Log_Message(BMS_MSG_LEVEL_ERROR, "I2C3 TX failed (Flashed) at chunk %u (offset=%u)", chunk_index, offset);
                    break;
                }

                Log_Message(BMS_MSG_LEVEL_DEBUG, "Sent flashed telemetry chunk %u, bytes: %u", chunk_index, chunk_len);
                offset += chunk_len;
                chunk_index++;
            }

            break;
        }


        case CMD_PUT_DATA:
            Handle_PD(i2c3_rx_buffer, len, i2c3_tx_buffer, &tx_len);
            break;

        case CMD_READ_DATA:
            Handle_RD(i2c3_rx_buffer, len, i2c3_tx_buffer, &tx_len);
            break;

        case CMD_WRITE_DATA:
            Handle_WD(i2c3_rx_buffer, len, i2c3_tx_buffer, &tx_len);
            break;

        default:
            Log_Message(BMS_MSG_LEVEL_WARNING, "Unknown I2C3 CMD: 0x%02X", cmd);
            break;
    }

    if (tx_len > 0) {
        if (HAL_I2C_Slave_Transmit(&hi2c3, i2c3_tx_buffer, tx_len, 100) != HAL_OK) {
            Log_Message(BMS_MSG_LEVEL_ERROR, "I2C3 TX failed: CMD 0x%02X", cmd);
        } else {
            Log_Message(BMS_MSG_LEVEL_INFO, "I2C3 TX success: CMD 0x%02X", cmd);
        }
    }

    restart_dma:
    if (HAL_I2C_Slave_Receive_DMA(&hi2c3, i2c3_rx_buffer, sizeof(i2c3_rx_buffer)) != HAL_OK) {
        Log_Message(BMS_MSG_LEVEL_ERROR, "I2C3 RX DMA restart failed");
    }
}
