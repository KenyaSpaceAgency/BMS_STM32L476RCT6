/* Role of ssp.c in the BMS Project:
 * This file implements the Simple Serial Protocol (SSP) for communication between the BMS
 * and the On-Board Computer (OBC) in a CubeSat. It handles sending and receiving data frames
 * over RS485, including telemetry (e.g., battery status) and time synchronization, enabling
 * the BMS to report its state and respond to OBC commands.
 */

/* Importance of ssp.c in the BMS Project:
 * - Enables Communication: SSP facilitates reliable data exchange over RS485, allowing the BMS
 *   to send critical battery data (voltages, currents, temperatures) to the OBC for monitoring.
 * - Supports Diagnostics: Telemetry and logs sent via SSP help ground control analyze BMS health,
 *   crucial for CubeSat operations in space.
 * - Ensures Safety: By processing OBC commands (e.g., mode changes, power line control), SSP
 *   allows remote management of the BMS, preventing unsafe conditions like overcharging.
 * - Integrates with BMS: Works with main.c to transmit status updates and receive instructions,
 *   supporting the BMS’s role in managing the 4S2P lithium-ion battery pack.
 */

/* Objective of ssp.c in the BMS Project:
 * The primary objective is to provide functions for constructing, transmitting, and receiving
 * SSP frames over RS485, packing telemetry data, and synchronizing time with the OBC. This
 * ensures the BMS can communicate battery health and operational status reliably, supporting
 * safe power management and remote control in the CubeSat’s Electrical Power System (EPS).
 */

/* Include STM32 HAL library for UART and hardware control functions */
#include "stm32l4xx_hal.h"
/* Include standard integer types for precise variable sizes */
#include <stdint.h>
/* Include SSP header file for protocol definitions and structures */
#include "ssp.h"
/* Include main header file for BMS-specific definitions */
#include "main.h"
/* Include CRC16 header file for checksum calculations */
#include "crc16.h"

/* Declare a static buffer to hold data for sending SSP frames */
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
/* Declare a static buffer to hold data for receiving SSP frames */
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];

/* Define function to calculate CRC checksum for SSP frames */
static uint16_t SSP_CalculateCRC(uint8_t *data, uint16_t len)
{
    return CalculateCRC16(data, len); /* Call external CRC16 function to compute checksum */
}

/* Define function to transmit an SSP frame over UART */
HAL_StatusTypeDef SSP_TransmitFrame(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t frame_len)
{
    return HAL_UART_Transmit(huart, buffer, frame_len, HAL_MAX_DELAY); /* Send frame data using UART with blocking delay */
}

/* Define function to receive and parse an SSP frame */
HAL_StatusTypeDef SSP_ReceiveFrame(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame)
{
    uint16_t index = 0; /* Initialize index to track buffer position */
    uint8_t byte; /* Declare variable to store received byte */
    while (1) { /* Start infinite loop to wait for frame start */
        if (HAL_UART_Receive(huart, &byte, 1, 100) != HAL_OK) return HAL_TIMEOUT; /* Receive one byte with 100ms timeout */
        if (byte == SSP_FLAG) break; /* Exit loop if start flag is received */
    }
    buffer[index++] = byte; /* Store start flag in buffer */
    while (index < SSP_HEADER_SIZE) { /* Loop to receive header bytes */
        if (HAL_UART_Receive(huart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; /* Receive next header byte */
    }
    frame->dest = buffer[1]; /* Set frame destination address from buffer */
    frame->src = buffer[2]; /* Set frame source address from buffer */
    frame->cmd_id = buffer[3]; /* Set frame command ID from buffer */
    frame->data_len = buffer[4]; /* Set frame data length from buffer */
    if (frame->data_len > SSP_MAX_DATA_LEN) return HAL_ERROR; /* Check if data length exceeds maximum */
    for (uint8_t i = 0; i < frame->data_len; i++) { /* Loop to receive data bytes */
        if (HAL_UART_Receive(huart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; /* Receive data byte */
        frame->data[i] = buffer[index - 1]; /* Store data byte in frame structure */
    }
    if (HAL_UART_Receive(huart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; /* Receive first CRC byte */
    if (HAL_UART_Receive(huart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; /* Receive second CRC byte */
    if (HAL_UART_Receive(huart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; /* Receive end flag */
    if (buffer[index - 1] != SSP_FLAG) return HAL_ERROR; /* Check if end flag is correct */
    frame->crc = (buffer[index - 3] << 8) | buffer[index - 2]; /* Combine CRC bytes into 16-bit value */
    uint16_t calc_crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4); /* Calculate CRC for received data */
    if (frame->crc != calc_crc) return HAL_ERROR; /* Check if CRC matches */
    return HAL_OK; /* Return success status */
}

/* Define function to request time from OBC */
HAL_StatusTypeDef SSP_RequestTime(UART_HandleTypeDef *huart, SSP_TimeTypeDef *time)
{
    SSP_FrameTypeDef frame = {0}; /* Create a structure for the request frame */
    uint16_t frame_len; /* Declare variable for frame length */
    frame.dest = SSP_ADDR_OBC; /* Set destination address to OBC */
    frame.src = SSP_ADDR_EPS; /* Set source address to EPS (BMS) */
    frame.cmd_id = SSP_CMD_GTIME; /* Set command ID to request time */
    frame.data_len = 0; /* Set data length to 0 (no data) */
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len); /* Construct the frame and set frame length */
    HAL_StatusTypeDef status = SSP_TransmitFrame(huart, ssp_tx_buffer, frame_len); /* Send request frame */
    if (status != HAL_OK) return status; /* Return error if transmission fails */
    SSP_FrameTypeDef response = {0}; /* Create a structure for the response frame */
    status = SSP_ReceiveFrame(huart, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &response); /* Receive response frame */
    if (status != HAL_OK) return status; /* Return error if reception fails */
    if (response.dest != SSP_ADDR_EPS || (response.cmd_id != SSP_CMD_ACK && response.cmd_id != SSP_CMD_NACK)) { /* Check response validity */
        return HAL_ERROR; /* Return error if response is invalid */
    }
    if (response.cmd_id == SSP_CMD_NACK) { /* Check negative acknowledgment */
        return HAL_ERROR; /* Return error for NACK */
    }
    status = SSP_ReceiveFrame(huart, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &response); /* Receive time data frame */
    if (status != HAL_OK) return status; /* Return error if reception fails */
    if (response.dest != SSP_ADDR_EPS || response.cmd_id != (SSP_CMD_GTIME | SSP_FRAME_TYPE_REPLY) || response.data_len != 7) { /* Check time frame validity */
        return HAL_ERROR; /* Return error if frame is invalid */
    }
    time->year = (response.data[0] << 8) | response.data[1]; /* Combine bytes to set year */
    time->month = response.data[2]; /* Set month from data */
    time->day = response.data[3]; /* Set day from data */
    time->hour = response.data[4]; /* Set hour from data */
    time->minute = response.data[5]; /* Set minute from data */
    time->second = response.data[6]; /* Set second from data */
    return HAL_OK; /* Return success status */
}

/* Define function to pack BMS telemetry into an SSP frame */
void SSP_PackTelemetry(SSP_TelemetryTypeDef *telemetry, SSP_FrameTypeDef *frame)
{
    frame->dest = SSP_ADDR_OBC; /* Set destination address to OBC */
    frame->src = SSP_ADDR_EPS; /* Set source address to EPS (BMS) */
    frame->cmd_id = SSP_CMD_GOSTM | SSP_FRAME_TYPE_REPLY; /* Set command ID for telemetry reply */
    frame->data_len = 43; /* Set data length to 43 bytes for telemetry */
    uint8_t *data = frame->data; /* Get pointer to frame’s data array */
    uint8_t index = 0; /* Initialize index for data array */
    data[index++] = telemetry->mode; /* Store BMS operating mode */
    data[index++] = telemetry->charge_enabled; /* Store charging enabled flag */
    data[index++] = telemetry->discharge_enabled; /* Store discharging enabled flag */
    data[index++] = telemetry->charge_immediately; /* Store immediate charge flag */
    data[index++] = telemetry->bms_online; /* Store BMS online status */
    data[index++] = (telemetry->error_flags >> 24) & 0xFF; /* Store high byte of error flags */
    data[index++] = (telemetry->error_flags >> 16) & 0xFF; /* Store next byte of error flags */
    data[index++] = (telemetry->error_flags >> 8) & 0xFF; /* Store next byte of error flags */
    data[index++] = telemetry->error_flags & 0xFF; /* Store low byte of error flags */
    data[index++] = (telemetry->pack_voltage_1 >> 8) & 0xFF; /* Store high byte of pack voltage 1 */
    data[index++] = telemetry->pack_voltage_1 & 0xFF; /* Store low byte of pack voltage 1 */
    data[index++] = (telemetry->pack_voltage_2 >> 8) & 0xFF; /* Store high byte of pack voltage 2 */
    data[index++] = telemetry->pack_voltage_2 & 0xFF; /* Store low byte of pack voltage 2 */
    data[index++] = (telemetry->pack_current_1 >> 8) & 0xFF; /* Store high byte of pack current 1 */
    data[index++] = telemetry->pack_current_1 & 0xFF; /* Store low byte of pack current 1 */
    data[index++] = (telemetry->pack_current_2 >> 8) & 0xFF; /* Store high byte of pack current 2 */
    data[index++] = telemetry->pack_current_2 & 0xFF; /* Store low byte of pack current 2 */
    data[index++] = telemetry->soc; /* Store state of charge */
    data[index++] = telemetry->soh; /* Store state of health */
    data[index++] = (telemetry->temp_1 >> 8) & 0xFF; /* Store high byte of temperature 1 */
    data[index++] = telemetry->temp_1 & 0xFF; /* Store low byte of temperature 1 */
    data[index++] = (telemetry->temp_2 >> 8) & 0xFF; /* Store high byte of temperature 2 */
    data[index++] = telemetry->temp_2 & 0xFF; /* Store low byte of temperature 2 */
    data[index++] = (telemetry->pcb_temp >> 8) & 0xFF; /* Store high byte of PCB temperature */
    data[index++] = telemetry->pcb_temp & 0xFF; /* Store low byte of PCB temperature */
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) { /* Loop through cell voltages */
        data[index++] = (telemetry->group_voltages[i] >> 8) & 0xFF; /* Store high byte of cell voltage */
        data[index++] = telemetry->group_voltages[i] & 0xFF; /* Store low byte of cell voltage */
    }
    data[index++] = telemetry->balancing_active; /* Store balancing active flag */
    data[index++] = telemetry->balancing_mask_1; /* Store balancing mask for IC 1 */
    data[index++] = telemetry->balancing_mask_2; /* Store balancing mask for IC 2 */
    data[index++] = (telemetry->charge_cycle_count >> 24) & 0xFF; /* Store high byte of charge cycle count */
    data[index++] = (telemetry->charge_cycle_count >> 16) & 0xFF; /* Store next byte of charge cycle count */
    data[index++] = (telemetry->charge_cycle_count >> 8) & 0xFF; /* Store next byte of charge cycle count */
    data[index++] = telemetry->charge_cycle_count & 0xFF; /* Store low byte of charge cycle count */
    data[index++] = (telemetry->total_charge_time >> 24) & 0xFF; /* Store high byte of total charge time */
    data[index++] = (telemetry->total_charge_time >> 16) & 0xFF; /* Store next byte of total charge time */
    data[index++] = (telemetry->total_charge_time >> 8) & 0xFF; /* Store next byte of total charge time */
    data[index++] = telemetry->total_charge_time & 0xFF; /* Store low byte of total charge time */
    data[index++] = (telemetry->total_discharge_time >> 24) & 0xFF; /* Store high byte of total discharge time */
    data[index++] = (telemetry->total_discharge_time >> 16) & 0xFF; /* Store next byte of total discharge time */
    data[index++] = (telemetry->total_discharge_time >> 8) & 0xFF; /* Store next byte of total discharge time */
    data[index++] = telemetry->total_discharge_time & 0xFF; /* Store low byte of total discharge time */
    data[index++] = (telemetry->total_operating_time >> 24) & 0xFF; /* Store high byte of total operating time */
    data[index++] = (telemetry->total_operating_time >> 16) & 0xFF; /* Store next byte of total operating time */
    data[index++] = (telemetry->total_operating_time >> 8) & 0xFF; /* Store next byte of total operating time */
    data[index++] = telemetry->total_operating_time & 0xFF; /* Store low byte of total operating time */
}

/* Define function to construct an SSP frame for transmission */
HAL_StatusTypeDef SSP_ConstructFrame(SSP_FrameTypeDef *frame, uint8_t *buffer, uint16_t *frame_len)
{
    uint8_t index = 0; /* Initialize index for buffer position */
    buffer[index++] = SSP_FLAG; /* Add start flag to buffer */
    buffer[index++] = frame->dest; /* Add destination address to buffer */
    buffer[index++] = frame->src; /* Add source address to buffer */
    buffer[index++] = frame->cmd_id; /* Add command ID to buffer */
    buffer[index++] = frame->data_len; /* Add data length to buffer */
    for (uint8_t i = 0; i < frame->data_len; i++) { /* Loop through data bytes */
        buffer[index++] = frame->data[i]; /* Add each data byte to buffer */
    }
    frame->crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4); /* Calculate CRC for frame */
    buffer[index++] = (frame->crc >> 8) & 0xFF; /* Add high byte of CRC to buffer */
    buffer[index++] = frame->crc & 0xFF; /* Add low byte of CRC to buffer */
    buffer[index++] = SSP_FLAG; /* Add end flag to buffer */
    *frame_len = index; /* Set frame length to total bytes */
    return HAL_OK; /* Return success status */
}
