#include "i2c_comm.h"
#include "main.h"
#include "flash.h"
#include <string.h>
#include "Temperature.h"  // for Heater1_On/Off, Heater2_On/Off


extern BQ76920_t bms_instance1;

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

// 🟡 Parse 8 bytes of SYNC_COUNTER and store in global variable
void UpdateSyncCounterFromBytes(uint8_t *bytes) {
    g_sync_counter = 0;
    for (int i = 0; i < 8; i++) {
        g_sync_counter = (g_sync_counter << 8) | bytes[i];  // MSB first
    }
}

void I2C_Comm_Init(void) {
    if (HAL_I2C_Slave_Receive_DMA(&hi2c3, i2c3_rx_buffer, sizeof(i2c3_rx_buffer)) != HAL_OK) {
        Log_Error("I2C3 RX DMA init failed\n");
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C3) {
        uint8_t cmd = i2c3_rx_buffer[0];
        uint8_t len = i2c3_rx_buffer[1];
        uint8_t crc_received = i2c3_rx_buffer[2 + len];
        uint8_t crc_computed = EPS_I2C_CRC8(i2c3_rx_buffer, 2 + len);

        if (crc_computed != crc_received) {
            Log_Error("I2C3 CRC mismatch: expected 0x%02X, got 0x%02X\n", crc_computed, crc_received);
            goto restart_dma;
        }

        if (len >= 8) {
            UpdateSyncCounterFromBytes(&i2c3_rx_buffer[2]);
            telemetry.sync_valid = 1;
        }

//        uint8_t *payload = &i2c3_rx_buffer[10];
//        uint8_t payload_len = (len > 8) ? (len - 8) : 0;

        switch (cmd) {
            case CMD_SYNC_COUNTER:
                break;

            case CMD_ENABLE_HEATER_1:
                Heater1_On();
                break;
            case CMD_DISABLE_HEATER_1:
                Heater1_Off();
                break;
            case CMD_ENABLE_HEATER_2:
                Heater2_On();
                break;
            case CMD_DISABLE_HEATER_2:
                Heater2_Off();
                break;
            case CMD_ENABLE_CHARGING:
                turnCHGOn(&bms_instance1);
                break;
            case CMD_DISABLE_CHARGING:
                turnCHGOff(&bms_instance1);
                break;
            case CMD_ENABLE_DISCHARGING:
                turnDSGOn(&bms_instance1);
                break;
            case CMD_DISABLE_DISCHARGING:
                turnDSGOff(&bms_instance1);
                break;

            case CMD_READ_TELEMETRY:
            {
                uint8_t pos = 0;
                i2c3_tx_buffer[pos++] = CMD_READ_TELEMETRY;
                i2c3_tx_buffer[pos++] = sizeof(telemetry);
                memcpy(&i2c3_tx_buffer[pos], &telemetry, sizeof(telemetry));
                pos += sizeof(telemetry);
                uint8_t crc = EPS_I2C_CRC8(i2c3_tx_buffer, pos);
                i2c3_tx_buffer[pos++] = crc;

                if (HAL_I2C_Slave_Transmit(&hi2c3, i2c3_tx_buffer, pos, 100) != HAL_OK)
                    Log_Error("I2C3 TX failed: RAM telemetry");
                break;
            }

            case CMD_READ_FLASHED_TELEMETRY:
            {
                TelemetrySnapshot snapshot;
                memcpy(&snapshot, (void*)FLASH_USER_START_ADDR, sizeof(TelemetrySnapshot));

                uint16_t expected_crc = CalculateCRC16((uint8_t*)&snapshot, sizeof(snapshot) - sizeof(uint16_t));
                if (snapshot.crc != expected_crc || snapshot.version != TELEMETRY_VERSION) {
                    Log_Error("Invalid flashed telemetry: CRC 0x%04X != 0x%04X or version mismatch",
                              expected_crc, snapshot.crc);
                    break;
                }

                uint8_t pos = 0;
                i2c3_tx_buffer[pos++] = CMD_READ_FLASHED_TELEMETRY;
                i2c3_tx_buffer[pos++] = sizeof(snapshot.telemetry);
                memcpy(&i2c3_tx_buffer[pos], &snapshot.telemetry, sizeof(snapshot.telemetry));
                pos += sizeof(snapshot.telemetry);
                uint8_t crc = EPS_I2C_CRC8(i2c3_tx_buffer, pos);
                i2c3_tx_buffer[pos++] = crc;

                if (HAL_I2C_Slave_Transmit(&hi2c3, i2c3_tx_buffer, pos, 100) != HAL_OK)
                    Log_Error("I2C3 TX failed: Flashed telemetry");
                break;
            }
        }

    restart_dma:
        if (HAL_I2C_Slave_Receive_DMA(&hi2c3, i2c3_rx_buffer, sizeof(i2c3_rx_buffer)) != HAL_OK) {
            Log_Error("I2C3 RX DMA restart failed\n");
        }
    }
}

