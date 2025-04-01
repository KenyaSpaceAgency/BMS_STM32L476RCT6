#ifndef SSP_H_
#define SSP_H_

#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define SSP_FLAG            0xC0    // Start and end flag for SSP frames
#define SSP_HEADER_SIZE     5       // Flag + DEST + SRC + CMD ID + D_Len
#define SSP_FOOTER_SIZE     3       // CRC_0 + CRC_1 + Flag
#define SSP_MAX_DATA_LEN    48      // Maximum data length (increased for more data)
#define SSP_MAX_FRAME_LEN   (SSP_HEADER_SIZE + SSP_MAX_DATA_LEN + SSP_FOOTER_SIZE)

// Command IDs
#define SSP_CMD_STATUS      0x01    // Request status (e.g., SOC, SOH, voltages)
#define SSP_CMD_SET_MODE    0x02    // Set operation mode (e.g., charging, sleep)
#define SSP_CMD_LOG_DATA    0x03    // Request logged data
#define SSP_CMD_TELEMETRY   0x04    // Send detailed telemetry data

// Subsystem addresses
#define SSP_ADDR_BMS        0x10    // BMS subsystem address
#define SSP_ADDR_OBC        0x20    // On-Board Computer (OBC) address
#define SSP_ADDR_POWER      0x30    // Power subsystem address

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t dest;           // Destination address
    uint8_t src;            // Source address
    uint8_t cmd_id;         // Command identifier
    uint8_t data_len;       // Data length
    uint8_t data[SSP_MAX_DATA_LEN]; // Data payload
    uint16_t crc;           // CRC (16-bit)
} SSP_FrameTypeDef;

// Telemetry data structure (fits within SSP_MAX_DATA_LEN)
typedef struct {
    uint8_t mode;           // BMS mode (1 byte)
    uint8_t charge_enabled; // Charge enable status (1 byte)
    uint8_t discharge_enabled; // Discharge enable status (1 byte)
    uint8_t charge_immediately; // Charge immediately status (1 byte)
    uint8_t bms_online;     // BMS online status (1 byte)
    uint32_t error_flags;   // Error flags (4 bytes)
    uint16_t pack_voltage_1;// Pack voltage (mV) (2 bytes)
    uint16_t pack_voltage_2;// Pack voltage (mV, redundant) (2 bytes)
    int16_t pack_current_1; // Pack current (mA) (2 bytes)
    int16_t pack_current_2; // Pack current (mA, redundant) (2 bytes)
    uint8_t soc;            // SOC (%) (1 byte, scaled)
    uint8_t soh;            // SOH (%) (1 byte, scaled)
    int16_t temp_1;         // Temperature 1 (°C) (2 bytes)
    int16_t temp_2;         // Temperature 2 (°C) (2 bytes)
    int16_t pcb_temp;       // PCB temperature (°C) (2 bytes)
    uint16_t group_voltages[3]; // Group voltages (mV) (6 bytes)
    uint8_t balancing_active; // Balancing active (1 byte)
    uint8_t balancing_mask_1; // Balancing mask for BQ76920 on I2C1 (1 byte)
    uint8_t balancing_mask_2; // Balancing mask for BQ76920 on I2C2 (1 byte)
    uint32_t charge_cycle_count; // Charge cycle count (4 bytes)
    uint32_t total_charge_time; // Total charge time (seconds) (4 bytes)
    uint32_t total_discharge_time; // Total discharge time (seconds) (4 bytes)
    uint32_t total_operating_time; // Total operating time (seconds) (4 bytes)
} SSP_TelemetryTypeDef; // Total: 41 bytes

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef SSP_ConstructFrame(SSP_FrameTypeDef *frame, uint8_t *buffer, uint16_t *frame_len);
HAL_StatusTypeDef SSP_TransmitFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t frame_len);
HAL_StatusTypeDef SSP_ReceiveFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame);
void SSP_PackTelemetry(SSP_TelemetryTypeDef *telemetry, SSP_FrameTypeDef *frame);
void SSP_UnpackTelemetry(SSP_FrameTypeDef *frame, SSP_TelemetryTypeDef *telemetry);

#endif /* SSP_H_ */
