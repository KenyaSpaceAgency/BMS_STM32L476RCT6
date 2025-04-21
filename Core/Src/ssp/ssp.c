/*
 * ssp.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 *
 * @brief  Implements the Simple Serial Protocol (SSP) for communication between the
 *         Electrical Power System (EPS) and On-Board Computer (OBC) in a CubeSat.
 *         Handles packing/unpacking of telemetry data, frame construction, transmission,
 *         reception, and time synchronization over USART (RS485).
 * @note   - SSP is a lightweight protocol used in spacecraft for subsystem communication.
 *         - Frames consist of a start flag, header (dest, src, cmd_id, data_len), data,
 *           CRC-16, and end flag.
 *         - Used in the BMS to send telemetry (e.g., battery status) and receive commands
 *           (e.g., SON/SOF, KEN) from the OBC, as seen in `main.c`.
 * @context In a CubeSat, the EPS (implemented on an STM32 microcontroller) monitors
 *          and controls the battery system, communicating status and receiving commands
 *          via SSP over RS485. This file ensures reliable data exchange, critical for
 *          mission operations like power management and time synchronization.
 */

/* Include the SSP header for type definitions and constants */
#include "ssp.h"

/* Private variables ---------------------------------------------------------*/
/* @brief  Buffers for transmitting and receiving SSP frames.
 * @note   - `ssp_tx_buffer` holds the constructed frame for transmission.
 *         - `ssp_rx_buffer` holds the received frame for parsing.
 *         - Both are sized to `SSP_MAX_FRAME_LEN` (defined in `ssp.h`) to accommodate
 *           the maximum frame size (header + max data + CRC + flags).
 * @context These static buffers ensure efficient memory use on the STM32, avoiding
 *          dynamic allocation in a resource-constrained CubeSat environment.
 */
static uint8_t ssp_tx_buffer[SSP_MAX_FRAME_LEN];
static uint8_t ssp_rx_buffer[SSP_MAX_FRAME_LEN];

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Calculates CRC-16 for an SSP frame to ensure data integrity.
  * @param  data: Pointer to the data buffer (starting from dest, excluding start flag).
  * @param  len: Length of the data to calculate CRC over (dest + src + cmd_id + data_len + data).
  * @retval CRC-16 value as a 16-bit unsigned integer.
  * @note   - Uses the CRC-16-CCITT polynomial (0xA001, bit-reversed form of 0x1021).
  *         - Initial value is 0xFFFF, standard for CRC-16-CCITT.
  *         - Processes each byte bit-by-bit, XORing with the polynomial if the LSB is 1.
  * @context In CubeSat communication, CRC ensures data integrity over RS485, where noise
  *          or interference could corrupt frames. Used in `SSP_ConstructFrame` and
  *          `SSP_ReceiveFrame` to validate frames between EPS and OBC.
  * @debug  If CRC mismatches occur frequently, check for noise on the RS485 bus or
  *         incorrect buffer data alignment.
 */
static uint16_t SSP_CalculateCRC(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF; // Initialize CRC to standard initial value
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i]; // XOR current byte with CRC
        for (uint8_t j = 0; j < 8; j++) { // Process each bit of the byte
            if (crc & 0x0001) { // If LSB is 1
                crc >>= 1; // Shift right
                crc ^= 0xA001; // XOR with polynomial
            } else {
                crc >>= 1; // Shift right only
            }
        }
    }
    return crc;
}

/**
  * @brief  Packs BMS telemetry data into an SSP frame for transmission to the OBC.
  * @param  telemetry: Pointer to the telemetry structure containing BMS data.
  * @param  frame: Pointer to the SSP frame structure to fill with packed data.
  * @retval None (modifies the frame structure in-place).
  * @note   - Frame header: dest (OBC), src (EPS), cmd_id (GOSTM reply), data_len (43 bytes).
  *         - Data payload: 43 bytes, including mode, flags, voltages, currents, SOC, SOH,
  *           temperatures, balancing info, and timers.
  *         - Multi-byte fields (e.g., voltages, currents) are split into MSB/LSB for transmission.
  * @context Called by `SSP_SendStatus` in `main.c` to send BMS status (e.g., every 5 seconds).
  *          Ensures the OBC receives critical battery data for monitoring and decision-making.
  * @integration The STM32 uses this function to format telemetry data into a standardized
  *              SSP frame, enabling seamless communication with the OBC over RS485.
  * @debug  If the OBC receives incorrect data, verify the byte order (big-endian) and ensure
  *         `NUM_GROUPS_PER_IC` matches the expected number of cell groups (typically 4).
 */
void SSP_PackTelemetry(SSP_TelemetryTypeDef *telemetry, SSP_FrameTypeDef *frame)
{
    // Set frame header
    frame->dest = SSP_ADDR_OBC; // Destination: On-Board Computer
    frame->src = SSP_ADDR_EPS;  // Source: Electrical Power System (BMS)
    frame->cmd_id = SSP_CMD_GOSTM | SSP_FRAME_TYPE_REPLY; // Command: GOSTM reply
    frame->data_len = 43; // Fixed length of telemetry payload (in bytes)

    // Pointer to the frame's data field for packing
    uint8_t *data = frame->data;
    uint8_t index = 0;

    // Pack single-byte fields
    data[index++] = telemetry->mode;              // BMS operating mode
    data[index++] = telemetry->charge_enabled;    // Charging enabled flag
    data[index++] = telemetry->discharge_enabled; // Discharging enabled flag
    data[index++] = telemetry->charge_immediately; // Immediate charge flag
    data[index++] = telemetry->bms_online;        // BMS online status

    // Pack multi-byte fields (big-endian)
    data[index++] = (telemetry->error_flags >> 24) & 0xFF; // Error flags (4 bytes)
    data[index++] = (telemetry->error_flags >> 16) & 0xFF;
    data[index++] = (telemetry->error_flags >> 8) & 0xFF;
    data[index++] = telemetry->error_flags & 0xFF;
    data[index++] = (telemetry->pack_voltage_1 >> 8) & 0xFF; // Pack voltage 1 (2 bytes)
    data[index++] = telemetry->pack_voltage_1 & 0xFF;
    data[index++] = (telemetry->pack_voltage_2 >> 8) & 0xFF; // Pack voltage 2 (2 bytes)
    data[index++] = telemetry->pack_voltage_2 & 0xFF;
    data[index++] = (telemetry->pack_current_1 >> 8) & 0xFF; // Pack current 1 (2 bytes)
    data[index++] = telemetry->pack_current_1 & 0xFF;
    data[index++] = (telemetry->pack_current_2 >> 8) & 0xFF; // Pack current 2 (2 bytes)
    data[index++] = telemetry->pack_current_2 & 0xFF;
    data[index++] = telemetry->soc; // State of Charge (SOC, 1 byte)
    data[index++] = telemetry->soh; // State of Health (SOH, 1 byte)
    data[index++] = (telemetry->temp_1 >> 8) & 0xFF; // Battery temp 1 (2 bytes)
    data[index++] = telemetry->temp_1 & 0xFF;
    data[index++] = (telemetry->temp_2 >> 8) & 0xFF; // Battery temp 2 (2 bytes)
    data[index++] = telemetry->temp_2 & 0xFF;
    data[index++] = (telemetry->pcb_temp >> 8) & 0xFF; // PCB temp (2 bytes)
    data[index++] = telemetry->pcb_temp & 0xFF;

    // Pack cell group voltages (NUM_GROUPS_PER_IC, typically 4 cells)
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        data[index++] = (telemetry->group_voltages[i] >> 8) & 0xFF; // MSB
        data[index++] = telemetry->group_voltages[i] & 0xFF;        // LSB
    }

    // Pack balancing information
    data[index++] = telemetry->balancing_active; // Balancing active flag
    data[index++] = telemetry->balancing_mask_1; // Balancing mask for IC1
    data[index++] = telemetry->balancing_mask_2; // Balancing mask for IC2

    // Pack timers (4 bytes each)
    data[index++] = (telemetry->charge_cycle_count >> 24) & 0xFF; // Charge cycles
    data[index++] = (telemetry->charge_cycle_count >> 16) & 0xFF;
    data[index++] = (telemetry->charge_cycle_count >> 8) & 0xFF;
    data[index++] = telemetry->charge_cycle_count & 0xFF;
    data[index++] = (telemetry->total_charge_time >> 24) & 0xFF; // Total charge time
    data[index++] = (telemetry->total_charge_time >> 16) & 0xFF;
    data[index++] = (telemetry->total_charge_time >> 8) & 0xFF;
    data[index++] = telemetry->total_charge_time & 0xFF;
    data[index++] = (telemetry->total_discharge_time >> 24) & 0xFF; // Total discharge time
    data[index++] = (telemetry->total_discharge_time >> 16) & 0xFF;
    data[index++] = (telemetry->total_discharge_time >> 8) & 0xFF;
    data[index++] = telemetry->total_discharge_time & 0xFF;
    data[index++] = (telemetry->total_operating_time >> 24) & 0xFF; // Total operating time
    data[index++] = (telemetry->total_operating_time >> 16) & 0xFF;
    data[index++] = (telemetry->total_operating_time >> 8) & 0xFF;
    data[index++] = telemetry->total_operating_time & 0xFF;
}

/**
  * @brief  Unpacks telemetry data from a received SSP frame.
  * @param  frame: Pointer to the SSP frame containing the telemetry data.
  * @param  telemetry: Pointer to the telemetry structure to fill with unpacked data.
  * @retval None (modifies the telemetry structure in-place).
  * @note   - Reverses the packing process of `SSP_PackTelemetry`, extracting 43 bytes.
  *         - Multi-byte fields are reconstructed from MSB/LSB pairs (big-endian).
  *         - Assumes the frame has been validated (CRC checked) by `SSP_ReceiveFrame`.
  * @context Used when the EPS receives telemetry from another subsystem (though primarily
  *          the EPS sends telemetry). Ensures data consistency between packed and unpacked
  *          formats, critical for debugging and interoperability.
  * @integration Allows the STM32 to interpret telemetry data from the OBC or other subsystems,
  *              supporting bidirectional communication in the CubeSat.
  * @debug  If unpacked values are incorrect, ensure the frame’s data_len matches 43 bytes
  *         and verify byte order during unpacking.
 */
void SSP_UnpackTelemetry(SSP_FrameTypeDef *frame, SSP_TelemetryTypeDef *telemetry)
{
    // Pointer to the frame's data field for unpacking
    uint8_t *data = frame->data;
    uint8_t index = 0;

    // Unpack single-byte fields
    telemetry->mode = data[index++];              // BMS operating mode
    telemetry->charge_enabled = data[index++];    // Charging enabled flag
    telemetry->discharge_enabled = data[index++]; // Discharging enabled flag
    telemetry->charge_immediately = data[index++]; // Immediate charge flag
    telemetry->bms_online = data[index++];        // BMS online status

    // Unpack multi-byte fields (big-endian)
    telemetry->error_flags = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4; // Error flags (4 bytes)
    telemetry->pack_voltage_1 = (data[index] << 8) | data[index + 1];
    index += 2; // Pack voltage 1 (2 bytes)
    telemetry->pack_voltage_2 = (data[index] << 8) | data[index + 1];
    index += 2; // Pack voltage 2 (2 bytes)
    telemetry->pack_current_1 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2; // Pack current 1 (2 bytes)
    telemetry->pack_current_2 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2; // Pack current 2 (2 bytes)
    telemetry->soc = data[index++]; // State of Charge (SOC, 1 byte)
    telemetry->soh = data[index++]; // State of Health (SOH, 1 byte)
    telemetry->temp_1 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2; // Battery temp 1 (2 bytes)
    telemetry->temp_2 = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2; // Battery temp 2 (2 bytes)
    telemetry->pcb_temp = (int16_t)((data[index] << 8) | data[index + 1]);
    index += 2; // PCB temp (2 bytes)

    // Unpack cell group voltages
    for (uint8_t i = 0; i < NUM_GROUPS_PER_IC; i++) {
        telemetry->group_voltages[i] = (data[index] << 8) | data[index + 1];
        index += 2;
    }

    // Unpack balancing information
    telemetry->balancing_active = data[index++]; // Balancing active flag
    telemetry->balancing_mask_1 = data[index++]; // Balancing mask for IC1
    telemetry->balancing_mask_2 = data[index++]; // Balancing mask for IC2

    // Unpack timers (4 bytes each)
    telemetry->charge_cycle_count = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4; // Charge cycles
    telemetry->total_charge_time = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4; // Total charge time
    telemetry->total_discharge_time = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
    index += 4; // Total discharge time
    telemetry->total_operating_time = (data[index] << 24) | (data[index + 1] << 16) | (data[index + 2] << 8) | data[index + 3];
}

/**
  * @brief  Constructs an SSP frame from the given frame structure for transmission.
  * @param  frame: Pointer to the SSP frame structure containing header and data.
  * @param  buffer: Pointer to the buffer where the serialized frame will be stored.
  * @param  frame_len: Pointer to store the total length of the constructed frame.
  * @retval HAL_StatusTypeDef: HAL_OK on success, indicating the frame is ready for transmission.
  * @note   - Frame structure: [SSP_FLAG][Dest][Src][CmdID][DataLen][Data][CRC_MSB][CRC_LSB][SSP_FLAG].
  *         - CRC is calculated over Dest, Src, CmdID, DataLen, and Data (excluding flags).
  *         - Buffer must be at least `SSP_MAX_FRAME_LEN` to avoid overflow.
  * @context Used by functions like `SSP_SendStatus` and `SSP_RequestTime` in `main.c` to
  *          prepare frames for transmission to the OBC over RS485.
  * @integration The STM32 serializes the frame into a byte array, which is then transmitted
  *              via USART (RS485). The frame format ensures compatibility with the OBC’s SSP
  *              implementation, critical for CubeSat communication.
  * @debug  If the OBC rejects frames, verify the CRC calculation, ensure `SSP_FLAG` matches
  *         the expected value, and check that `data_len` matches the actual data length.
 */
HAL_StatusTypeDef SSP_ConstructFrame(SSP_FrameTypeDef *frame, uint8_t *buffer, uint16_t *frame_len)
{
    uint8_t index = 0;

    // Start flag: Marks the beginning of the frame
    buffer[index++] = SSP_FLAG;

    // Header: Destination, Source, Command ID, Data Length
    buffer[index++] = frame->dest;
    buffer[index++] = frame->src;
    buffer[index++] = frame->cmd_id;
    buffer[index++] = frame->data_len;

    // Copy data payload
    for (uint8_t i = 0; i < frame->data_len; i++) {
        buffer[index++] = frame->data[i];
    }

    // Calculate CRC over header and data (excluding start flag)
    frame->crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4);
    buffer[index++] = (frame->crc >> 8) & 0xFF; // CRC MSB
    buffer[index++] = frame->crc & 0xFF;        // CRC LSB

    // End flag: Marks the end of the frame
    buffer[index++] = SSP_FLAG;

    // Store the total frame length
    *frame_len = index;

    return HAL_OK;
}

/**
  * @brief  Transmits an SSP frame over USART (RS485) to the OBC.
  * @param  husart: Pointer to the USART handle (e.g., husart2 in `main.c`).
  * @param  buffer: Pointer to the serialized frame buffer (from `SSP_ConstructFrame`).
  * @param  frame_len: Length of the frame to transmit.
  * @retval HAL_StatusTypeDef: HAL_OK on successful transmission, else HAL_ERROR or HAL_TIMEOUT.
  * @note   - Uses `HAL_USART_Transmit` with a blocking timeout (`HAL_MAX_DELAY`).
  *         - Assumes the RS485 DE pin is set high before transmission and low after
  *           (handled in `main.c`, `SSP_SendStatus`).
  * @context Called after constructing a frame (e.g., in `SSP_SendStatus`) to send telemetry
  *          or command responses to the OBC. RS485 ensures reliable communication in the
  *          noisy CubeSat environment.
  * @integration The STM32 uses USART2 (configured in `main.c`) for half-duplex RS485
  *              communication. The DE pin control (via GPIO) ensures proper transmit/receive
  *              switching, critical for CubeSat bus communication.
  * @debug  If transmission fails, check USART2 configuration (baud rate 115200, 8N1),
  *         ensure the DE pin is correctly toggled, and verify the RS485 bus termination.
 */
HAL_StatusTypeDef SSP_TransmitFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t frame_len)
{
    // Transmit the entire frame using HAL USART driver
    return HAL_USART_Transmit(husart, buffer, frame_len, HAL_MAX_DELAY);
}

/**
  * @brief  Receives an SSP frame over USART (RS485) from the OBC.
  * @param  husart: Pointer to the USART handle (e.g., husart2 in `main.c`).
  * @param  buffer: Pointer to the buffer to store the received frame.
  * @param  buffer_len: Maximum length of the buffer (must be >= SSP_MAX_FRAME_LEN).
  * @param  frame: Pointer to the SSP frame structure to fill with parsed data.
  * @retval HAL_StatusTypeDef: HAL_OK on successful reception, HAL_TIMEOUT on timeout,
  *         HAL_ERROR on frame error (e.g., CRC mismatch, invalid flags).
  * @note   - Frame format: [SSP_FLAG][Dest][Src][CmdID][DataLen][Data][CRC_MSB][CRC_LSB][SSP_FLAG].
  *         - Uses a 100 ms timeout per byte to prevent indefinite blocking.
  *         - Validates frame structure, data length, and CRC before returning success.
  * @context Used in `SSP_ProcessReceivedFrame` and `SSP_RequestTime` in `main.c` to
  *          receive commands (e.g., SON/SOF) or time data from the OBC. Ensures reliable
  *          data reception in the CubeSat’s noisy environment.
  * @integration The STM32 uses USART2 in receive mode (DE pin low) to read incoming frames.
  *              The polling approach with timeout ensures the system remains responsive,
  *              critical for real-time CubeSat operations.
  * @debug  - If HAL_TIMEOUT occurs frequently, increase the timeout or check RS485 bus
  *           connectivity.
  *         - If HAL_ERROR occurs, verify CRC calculation, frame structure, and data length.
  *         - Use a logic analyzer to capture RS485 traffic for debugging frame issues.
 */
HAL_StatusTypeDef SSP_ReceiveFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame)
{
    uint16_t index = 0;
    uint8_t byte;

    // Wait for the start flag to begin frame reception
    while (1) {
        if (HAL_USART_Receive(husart, &byte, 1, 100) != HAL_OK) return HAL_TIMEOUT;
        if (byte == SSP_FLAG) break; // Start flag found
    }
    buffer[index++] = byte;

    // Read the header (Dest, Src, CmdID, DataLen)
    while (index < SSP_HEADER_SIZE) {
        if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;
    }

    // Parse header fields
    frame->dest = buffer[1];
    frame->src = buffer[2];
    frame->cmd_id = buffer[3];
    frame->data_len = buffer[4];

    // Validate data length to prevent buffer overflow
    if (frame->data_len > SSP_MAX_DATA_LEN) return HAL_ERROR;

    // Read the data payload
    for (uint8_t i = 0; i < frame->data_len; i++) {
        if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT;
        frame->data[i] = buffer[index - 1];
    }

    // Read CRC (2 bytes) and end flag
    if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; // CRC MSB
    if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; // CRC LSB
    if (HAL_USART_Receive(husart, &buffer[index++], 1, 100) != HAL_OK) return HAL_TIMEOUT; // End flag

    // Validate end flag
    if (buffer[index - 1] != SSP_FLAG) return HAL_ERROR;

    // Validate CRC
    frame->crc = (buffer[index - 3] << 8) | buffer[index - 2];
    uint16_t calc_crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4);
    if (frame->crc != calc_crc) return HAL_ERROR;

    return HAL_OK;
}

/**
  * @brief  Requests the current satellite time from the OBC using the GTIME command.
  * @param  husart: Pointer to the USART handle (e.g., husart2 in `main.c`).
  * @param  time: Pointer to the SSP_TimeTypeDef structure to fill with received time data.
  * @retval HAL_StatusTypeDef: HAL_OK on success, HAL_TIMEOUT on timeout, HAL_ERROR on
  *         protocol error (e.g., NACK, invalid response).
  * @note   - Sends a GTIME command frame to the OBC.
  *         - Expects an ACK/NACK response, followed by a time data frame if ACKed.
  *         - Time data: 7 bytes (year, month, day, hour, minute, second).
  * @context Used in `main.c` to synchronize the EPS’s RTC with the OBC every 60 seconds.
  *          Accurate timekeeping is critical for logging, scheduling, and coordinating
  *          CubeSat operations.
  * @integration The STM32 sends a request via USART2 (RS485) and processes the response,
  *              updating the RTC (`hrtc` in `main.c`). The two-frame exchange (request + response)
  *              ensures reliable communication, typical in spacecraft protocols.
  * @debug  - If HAL_TIMEOUT occurs, check RS485 bus connectivity or increase the timeout.
  *         - If HAL_ERROR occurs, log the response frame to verify cmd_id, dest, and data_len.
  *         - Ensure the OBC supports the GTIME command and responds with the correct format.
 */
HAL_StatusTypeDef SSP_RequestTime(USART_HandleTypeDef *husart, SSP_TimeTypeDef *time)
{
    SSP_FrameTypeDef frame = {0};
    uint16_t frame_len;

    // Construct the GTIME request frame
    frame.dest = SSP_ADDR_OBC; // Destination: On-Board Computer
    frame.src = SSP_ADDR_EPS;  // Source: Electrical Power System (BMS)
    frame.cmd_id = SSP_CMD_GTIME; // Command: Request satellite time
    frame.data_len = 0;        // No data payload for request

    // Serialize and transmit the request frame
    SSP_ConstructFrame(&frame, ssp_tx_buffer, &frame_len);
    HAL_StatusTypeDef status = SSP_TransmitFrame(husart, ssp_tx_buffer, frame_len);
    if (status != HAL_OK) return status;

    // Receive the ACK/NACK response from the OBC
    SSP_FrameTypeDef response = {0};
    status = SSP_ReceiveFrame(husart, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &response);
    if (status != HAL_OK) return status;

    // Validate the response: Must be addressed to EPS and be an ACK or NACK
    if (response.dest != SSP_ADDR_EPS || (response.cmd_id != SSP_CMD_ACK && response.cmd_id != SSP_CMD_NACK)) {
        return HAL_ERROR;
    }

    // If NACK, the OBC rejected the request
    if (response.cmd_id == SSP_CMD_NACK) {
        return HAL_ERROR;
    }

    // Receive the time data in a separate frame
    status = SSP_ReceiveFrame(husart, ssp_rx_buffer, SSP_MAX_FRAME_LEN, &response);
    if (status != HAL_OK) return status;

    // Validate the time response frame
    if (response.dest != SSP_ADDR_EPS || response.cmd_id != (SSP_CMD_GTIME | SSP_FRAME_TYPE_REPLY) || response.data_len != 7) {
        return HAL_ERROR;
    }

    // Unpack the time data (7 bytes)
    time->year = (response.data[0] << 8) | response.data[1]; // Year (2 bytes)
    time->month = response.data[2];                          // Month (1 byte)
    time->day = response.data[3];                            // Day (1 byte)
    time->hour = response.data[4];                           // Hour (1 byte)
    time->minute = response.data[5];                         // Minute (1 byte)
    time->second = response.data[6];                         // Second (1 byte)

    return HAL_OK;
}
