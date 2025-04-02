/*
 * ssp.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "ssp.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Calculates CRC-16 for the SSP frame
  * @param  data: Pointer to the data
  * @param  len: Length of the data
  * @retval CRC-16 value
  */
static uint16_t SSP_CalculateCRC(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001; // CRC-16-CCITT polynomial (bit-reversed)
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief  Packs telemetry data into an SSP frame
  * @param  telemetry: Pointer to the telemetry data
  * @param  frame: Pointer to the SSP frame to fill
  * @retval None
  */
void SSP_PackTelemetry(SSP_TelemetryTypeDef *telemetry, SSP_FrameTypeDef *frame)
{
    frame->dest = SSP_ADDR_OBC;
    frame->src = SSP_ADDR_EPS;
    frame->cmd_id = SSP_CMD_GOSTM | SSP_FRAME_TYPE_REPLY; // Reply frame
    frame->data_len = 41; // Total size of telemetry data

    uint8_t *data = frame->data;
    uint8_t index = 0;

    data[index++] = telemetry->mode;
    data[index++] = telemetry->charge_enabled;
    data[index++] = telemetry->discharge_enabled;
    data[index++] = telemetry->charge_immediately;
    data[index++] = telemetry->bms_online;
    data[index++] = (telemetry->error_flags >> 24) & 0xFF;
    data[index++] = (telemetry->error_flags >> 16) & 0xFF;
    data[index++] = (telemetry->error_flags >> 8) & 0xFF;
    data[index++] = telemetry->error_flags & 0xFF;
    data[index++] = (telemetry->pack_voltage_1 >> 8) & 0xFF;
    data[index++] = telemetry->pack_voltage_1 & 0xFF;
    data[index++] = (telemetry->pack_voltage_2 >> 8) & 0xFF;
    data[index++] = telemetry->pack_voltage_2 & 0xFF;
    data[index++] = (telemetry->pack_current_1 >> 8) & 0xFF;
    data[index++] = telemetry->pack_current_1 & 0xFF;
    data[index++] = (telemetry->pack_current_2 >> 8) & 0xFF;
    data[index++] = telemetry->pack_current_2 & 0xFF;
    data[index++] = telemetry->soc;
    data[index++] = telemetry->soh;
    data[index++] = (telemetry->temp_1 >> 8) & 0xFF;
    data[index++] = telemetry->temp_1 & 0xFF;
    data[index++] = (telemetry->temp_2 >> 8) & 0xFF;
    data[index++] = telemetry->temp_2 & 0xFF;
    data[index++] = (telemetry->pcb_temp >> 8) & 0xFF;
    data[index++] = telemetry->pcb_temp & 0xFF;
    for (uint8_t i = 0; i < 3; i++) {
        data[index++] = (telemetry->group_voltages[i] >> 8) & 0xFF;
        data[index++] = telemetry->group_voltages[i] & 0xFF;
    }
    data[index++] = telemetry->balancing_active;
    data[index++] = telemetry->balancing_mask_1;
    data[index++] = telemetry->balancing_mask_2;
    data[index++] = (telemetry->charge_cycle_count >> 24) & 0xFF;
    data[index++] = (telemetry->charge_cycle_count >> 16) & 0xFF;
    data[index++] = (telemetry->charge_cycle_count >> 8) & 0xFF;
    data[index++] = telemetry->charge_cycle_count & 0xFF;
    data[index++] = (telemetry->total_charge_time >> 24) & 0xFF;
    data[index++] = (telemetry->total_charge_time >> 16) & 0xFF;
    data[index++] = (telemetry->total_charge_time >> 8) & 0xFF;
    data[index++] = telemetry->total_charge_time & 0xFF;
    data[index++] = (telemetry->total_discharge_time >> 24) & 0xFF;
    data[index++] = (telemetry->total_discharge_time >> 16) & 0xFF;
    data[index++] = (telemetry->total_discharge_time >> 8) & 0xFF;
    data[index++] = telemetry->total_discharge_time & 0xFF;
    data[index++] = (telemetry->total_operating_time >> 24) & 0xFF;
    data[index++] = (telemetry->total_operating_time >> 16) & 0xFF;
    data[index++] = (telemetry->total_operating_time >> 8) & 0xFF;
    data[index++] = telemetry->total_operating_time & 0xFF;
}

/**
  * @brief  Unpacks telemetry data from an SSP frame
  * @param  frame: Pointer to the SSP frame
  * @param  telemetry: Pointer to the telemetry data to fill
  * @retval None
  */
void SSP_UnpackTelemetry(SSP_FrameTypeDef *frame, SSP_TelemetryTypeDef *telemetry)
{
    uint8_t *data = frame->data;
    uint8_t index = 0;

    telemetry->mode = data[index++];
    telemetry->charge_enabled = data[index++];
    telemetry->discharge_enabled = data[index++];
    telemetry->charge_immediately = data[index++];
    telemetry->bms_online = data[index++];
    telemetry->error_flags = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4;
    telemetry->pack_voltage_1 = (data[index] << 8) | data[index + 1];
    index += 2;
    telemetry->pack_voltage_2 = (data[index] << 8) | data[index + 1];
    index += 2;
    telemetry->pack_current_1 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2;
    telemetry->pack_current_2 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2;
    telemetry->soc = data[index++];
    telemetry->soh = data[index++];
    telemetry->temp_1 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2;
    telemetry->temp_2 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2;
    telemetry->pcb_temp = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2;
    for (uint8_t i = 0; i < 3; i++) {
        telemetry->group_voltages[i] = (data[index] << 8) | data[index + 1];
        index += 2;
    }
    telemetry->balancing_active = data[index++];
    telemetry->balancing_mask_1 = data[index++];
    telemetry->balancing_mask_2 = data[index++];
    telemetry->charge_cycle_count = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4;
    telemetry->total_charge_time = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4;
    telemetry->total_discharge_time = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4;
    telemetry->total_operating_time = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
}

/**
  * @brief  Constructs an SSP frame from the given data
  * @param  frame: Pointer to the SSP frame structure
  * @param  buffer: Buffer to store the constructed frame
  * @param  frame_len: Pointer to store the frame length
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef SSP_ConstructFrame(SSP_FrameTypeDef *frame, uint8_t *buffer, uint16_t *frame_len)
{
    uint8_t index = 0;

    buffer[index++] = SSP_FLAG; // Start flag
    buffer[index++] = frame->dest;
    buffer[index++] = frame->src;
    buffer[index++] = frame->cmd_id;
    buffer[index++] = frame->data_len;

    for (uint8_t i = 0; i < frame->data_len; i++) {
        buffer[index++] = frame->data[i];
    }

    frame->crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4);
    buffer[index++] = (frame->crc >> 8) & 0xFF; // CRC_1 (MSB)
    buffer[index++] = frame->crc & 0xFF;        // CRC_0 (LSB)
    buffer[index++] = SSP_FLAG; // End flag

    *frame_len = index;
    return HAL_OK;
}

/**
  * @brief  Transmits an SSP frame over USART
  * @param  husart: Pointer to the USART handle
  * @param  buffer: Buffer containing the frame
  * @param  frame_len: Length of the frame
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef SSP_TransmitFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t frame_len)
{
    return HAL_USART_Transmit(husart, buffer, frame_len, HAL_MAX_DELAY);
}

/**
  * @brief  Receives an SSP frame over USART
  * @param  husart: Pointer to the USART handle
  * @param  buffer: Buffer to store the received frame
  * @param  buffer_len: Length of the buffer
  * @param  frame: Pointer to the SSP frame structure to fill
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef SSP_ReceiveFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame)
{
    uint16_t index = 0;
    uint8_t byte;

    // Wait for start flag
    while (1) {
        if (HAL_USART_Receive(husart, &byte, 1, 100) != HAL_OK) return HAL_TIMEOUT;
        if (byte == SSP_FLAG) break;
    }
    buffer[index++] = byte;

    // Read header
    while (index < SSP_HEADER_SIZE) {
        if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;
    }

    frame->dest = buffer[1];
    frame->src = buffer[2];
    frame->cmd_id = buffer[3];
    frame->data_len = buffer[4];

    if (frame->data_len > SSP_MAX_DATA_LEN) return HAL_ERROR;

    // Read data
    for (uint8_t i = 0; i < frame->data_len; i++) {
        if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;
        frame->data[i] = buffer[index - 1];
    }

    // Read CRC and end flag
    if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;
    if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;
    if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;

    if (buffer[index - 1] != SSP_FLAG) return HAL_ERROR;

    frame->crc = (buffer[index - 3] << 8) | buffer[index - 2];
    uint16_t calc_crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4);
    if (frame->crc != calc_crc) return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief  Requests the current satellite time using GTIME command
  * @param  husart: Pointer to the USART handle
  * @param  time: Pointer to the time structure to fill
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef SSP_RequestTime(USART_HandleTypeDef *husart, SSP_TimeTypeDef *time)
{
    SSP_FrameTypeDef frame = {0};
    uint16_t frame_len;

    // Construct the time request frame
    frame.dest = SSP_ADDR_OBC;
    frame.src = SSP_ADDR_EPS;
    frame.cmd_id = SSP_CMD_GTIME;
    frame.data_len = 0;

    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len);
    HAL_StatusTypeDef status = SSP_TransmitFrame(husart, ssp_tx_buffer, frame_len);
    if (status != HAL_OK) return status;

    // Receive the ACK/NACK response
    SSP_FrameTypeDef response = {0};
    status = SSP_ReceiveFrame(husart, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &response);
    if (status != HAL_OK) return status;

    if (response.dest != SSP_ADDR_EPS || (response.cmd_id != SSP_CMD_ACK && response.cmd_id != SSP_CMD_NACK)) {
        return HAL_ERROR;
    }

    if (response.cmd_id == SSP_CMD_NACK) {
        return HAL_ERROR; // OBC rejected the request
    }

    // Receive the time data in a separate frame
    status = SSP_ReceiveFrame(husart, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &response);
    if (status != HAL_OK) return status;

    if (response.dest != SSP_ADDR_EPS || response.cmd_id != (SSP_CMD_GTIME | SSP_FRAME_TYPE_REPLY) || response.data_len != 7) {
        return HAL_ERROR;
    }

    // Unpack the time data
    time->year = (response.data[0] << 8) | response.data[1];
    time->month = response.data[2];
    time->day = response.data[3];
    time->hour = response.data[4];
    time->minute = response.data[5];
    time->second = response.data[6];

    return HAL_OK;
}
