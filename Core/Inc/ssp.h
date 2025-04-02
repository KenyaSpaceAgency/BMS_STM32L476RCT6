/*
 * ssp.h
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#ifndef SSP_H_
#define SSP_H_

#include "main.h"

/* Exported constants --------------------------------------------------------*/
#define SSP_FLAG            0xC0    // Start and end flag for SSP frames
#define SSP_HEADER_SIZE     5       // Flag + DEST + SRC + CMD ID + D_Len
#define SSP_FOOTER_SIZE     3       // CRC_0 + CRC_1 + Flag
#define SSP_MAX_DATA_LEN    248     // Maximum data length (256 - 8 bytes overhead)
#define SSP_MAX_FRAME_LEN   (SSP_HEADER_SIZE + SSP_MAX_DATA_LEN + SSP_FOOTER_SIZE)

// Command IDs (from ICD Table 38)
#define SSP_CMD_PING        0x00    // Check subsystem status
#define SSP_CMD_INIT        0x01    // Start UHF communication
#define SSP_CMD_ACK         0x02    // Acknowledge reply
#define SSP_CMD_NACK        0x03    // No acknowledge reply
#define SSP_CMD_GD          0x04    // Get parameter data
#define SSP_CMD_PD          0x05    // Put parameter data
#define SSP_CMD_RD          0x06    // Read data
#define SSP_CMD_WD          0x07    // Write data
#define SSP_CMD_SON         0x0B    // Switch ON subsystem (EPS)
#define SSP_CMD_SOF         0x0C    // Switch OFF subsystem (EPS)
#define SSP_CMD_END         0x0E    // End UHF communication
#define SSP_CMD_HRST        0x0F    // Hard reset
#define SSP_CMD_STIME       0x11    // Set satellite time
#define SSP_CMD_GTIME       0x12    // Get satellite time
#define SSP_CMD_SM          0x15    // Set subsystem mode
#define SSP_CMD_GM          0x16    // Get subsystem mode
#define SSP_CMD_GSC         0x17    // Get sync counter value
#define SSP_CMD_SSC         0x18    // Set sync counter value
#define SSP_CMD_GFP         0x1A    // Get function parameter
#define SSP_CMD_SFP         0x1B    // Set function parameter
#define SSP_CMD_FON         0x1C    // Function switch ON
#define SSP_CMD_FOF         0x1D    // Function switch OFF
#define SSP_CMD_GOTLM       0x21    // Get online telemetry
#define SSP_CMD_GSTLM       0x22    // Get stored telemetry
#define SSP_CMD_GOSTM       0x25    // Get online subsystem telemetry
#define SSP_CMD_KEN         0x31    // Kill enable (EPS)
#define SSP_CMD_KDIS        0x32    // Kill disable (EPS)

// Subsystem addresses (from ICD Table 21)
#define SSP_ADDR_OBC        0x01    // On-Board Controller
#define SSP_ADDR_EPS        0x02    // Electrical Power Supply (BMS)
#define SSP_ADDR_CCU        0x03    // Centralized Control Unit
#define SSP_ADDR_ADCS       0x04    // Attitude Determination and Control System
#define SSP_ADDR_PL         0x05    // Payload subsystem
#define SSP_ADDR_GCS        0x50    // Ground Control Station
#define SSP_ADDR_MULTICAST  0xF0    // Multicast address
#define SSP_ADDR_BROADCAST  0xFF    // Broadcast address

// Command type identifiers (from ICD Table 20)
#define SSP_CMD_TYPE_MASK   0xC0    // Bits 7 and 6 for command type
#define SSP_CMD_TYPE_DIRECT 0x00    // Bit 7 = 0: Direct command
#define SSP_CMD_TYPE_TIMETAG 0x80   // Bit 7 = 1: Time-tagged command
#define SSP_FRAME_TYPE_CMD  0x00    // Bit 6 = 0: Command frame
#define SSP_FRAME_TYPE_REPLY 0x40   // Bit 6 = 1: Reply frame

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

// Time structure for GTIME/STIME commands
typedef struct {
    uint16_t year;          // Year (e.g., 2025) (2 bytes)
    uint8_t month;          // Month (1-12) (1 byte)
    uint8_t day;            // Day (1-31) (1 byte)
    uint8_t hour;           // Hour (0-23) (1 byte)
    uint8_t minute;         // Minute (0-59) (1 byte)
    uint8_t second;         // Second (0-59) (1 byte)
} SSP_TimeTypeDef; // Total: 7 bytes

// Time-tagged command structure
typedef struct {
    uint8_t dest;           // Secondary destination address (1 byte)
    uint64_t exec_time;     // Execution time (8 bytes, e.g., Unix timestamp in seconds)
    uint8_t data[SSP_MAX_DATA_LEN - 9]; // Remaining data (max 239 bytes)
} SSP_TimeTaggedCmdTypeDef;

/* Exported functions --------------------------------------------------------*/
HAL_StatusTypeDef SSP_ConstructFrame(SSP_FrameTypeDef *frame, uint8_t *buffer, uint16_t *frame_len);
HAL_StatusTypeDef SSP_TransmitFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t frame_len);
HAL_StatusTypeDef SSP_ReceiveFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame);
void SSP_PackTelemetry(SSP_TelemetryTypeDef *telemetry, SSP_FrameTypeDef *frame);
void SSP_UnpackTelemetry(SSP_FrameTypeDef *frame, SSP_TelemetryTypeDef *telemetry);
HAL_StatusTypeDef SSP_RequestTime(USART_HandleTypeDef *husart, SSP_TimeTypeDef *time);

#endif /* SSP_H_ */
