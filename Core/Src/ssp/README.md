
# AFDEVSAT Simple Serial Protocol (SSP) Report

**Date**: April 01, 2025  
**Reference Document**: AFDEVSAT.ICD.001.01, Page 64  

## 1. Introduction

The AFDEVSAT Cube satellite employs the Simple Serial Protocol (SSP) as its internal bus communication protocol to facilitate data exchange between subsystems. This report provides a detailed documentation of the SSP frame structure, its error-checking mechanism using Cyclic Redundancy Check (CRC), and an analysis of the CRC implementation within the provided `ssp.c` source code.

## 2. SSP Protocol Overview

### 2.1 Frame Structure

The SSP protocol defines a frame with a maximum size of 256 bytes, consisting of nine fields. The frame includes a variable-length data field, with a maximum data payload of 248 bytes after accounting for 8 bytes of overhead.

| **Field**  | **Size (Bytes)** | **Description**                          |
|------------|------------------|------------------------------------------|
| Flag       | 1                | Start/end frame identifier (0xC0)        |
| DEST       | 1                | Destination subsystem address            |
| SRC        | 1                | Source subsystem address                 |
| CMD ID     | 1                | Command identifier                       |
| D_Len      | 1                | Data field length (excluding header)     |
| Data       | Variable (0–248) | Actual subsystem data                    |
| CRC_0      | 1                | CRC least significant byte (LSB)         |
| CRC_1      | 1                | CRC most significant byte (MSB)          |
| Flag       | 1                | End frame identifier (0xC0)              |

- **Maximum Frame Size**: 256 bytes
- **Maximum Data Size**: 248 bytes (256 – 8 bytes overhead)

### 2.2 Cyclic Redundancy Check (CRC)

The SSP protocol uses a 16-bit CRC for error detection, calculated using the CRC-16-CCITT polynomial \( X^{16} + X^{12} + X^5 + 1 \) (0x1021). The CRC is split into two bytes:
- **CRC_0**: Least significant byte (LSB)
- **CRC_1**: Most significant byte (MSB)

The CRC is computed by both the sender and receiver over the frame’s header (excluding the start flag) and data fields to ensure data integrity during transmission.

## 3. CRC Implementation in Code

The provided `ssp.c` source code (created on March 29, 2025, by author "yomue") implements the SSP protocol, including CRC calculation and verification. This section details the key functions where the CRC is applied.

### 3.1 CRC Calculation: `SSP_CalculateCRC`

**Function Definition**:
```c
static uint16_t SSP_CalculateCRC(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

- **Purpose**: Computes the 16-bit CRC-16-CCITT value for a given data buffer.
- **Inputs**:
  - `data`: Pointer to the data buffer
  - `len`: Length of the data in bytes
- **Algorithm Details**:
  - Initial CRC value: 0xFFFF (standard for CRC-16-CCITT)
  - Polynomial: 0xA001 (bit-reversed form of 0x1021 for software efficiency)
  - Process: XOR each byte into the CRC, followed by 8 bit-wise operations
- **Output**: 16-bit CRC value

### 3.2 CRC Application in Frame Construction: `SSP_ConstructFrame`

**Function Definition**:
```c
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
```

- **Purpose**: Constructs an SSP frame and appends the calculated CRC.
- **CRC Application**:
  - **Scope**: Calculated over DEST, SRC, CMD_ID, D_Len, and Data fields (starting at `buffer[1]`, excluding the start flag).
  - **Length**: `frame->data_len + 4` (4 bytes for header fields).
  - **Storage**: 
    - MSB (CRC_1) stored as `(frame->crc >> 8) & 0xFF`
    - LSB (CRC_0) stored as `frame->crc & 0xFF`
- **Process**: The CRC is computed after the header and data are populated, then appended before the end flag.

### 3.3 CRC Verification in Frame Reception: `SSP_ReceiveFrame`

**Function Definition**:
```c
HAL_StatusTypeDef SSP_ReceiveFrame(USART_HandleTypeDef *husart, uint8_t *buffer, uint16_t buffer_len, SSP_FrameTypeDef *frame)
{
    // ... (frame reception logic)

    frame->crc = (buffer[index - 3] << 8) | buffer[index - 2]; // Received CRC
    uint16_t calc_crc = SSP_CalculateCRC(&buffer[1], frame->data_len + 4); // Recalculated CRC
    if (frame->crc != calc_crc) return HAL_ERROR;

    return HAL_OK;
}
```

- **Purpose**: Receives an SSP frame and verifies its integrity using CRC.
- **CRC Application**:
  - **Extraction**: Received CRC is reconstructed from:
    - MSB (CRC_1): `buffer[index - 3]`
    - LSB (CRC_0): `buffer[index - 2]`
  - **Verification**: Recalculated CRC over DEST, SRC, CMD_ID, D_Len, and Data (same scope as transmission).
  - **Check**: Compares received CRC (`frame->crc`) with recalculated CRC (`calc_crc`).
- **Error Handling**: Returns `HAL_ERROR` if CRCs do not match, indicating data corruption.

## 4. Summary

The SSP protocol for the AFDEVSAT Cube satellite ensures reliable communication between subsystems through a well-defined frame structure and CRC-16-CCITT error checking. The CRC is:
- **Calculated** using `SSP_CalculateCRC` with the polynomial 0xA001.
- **Applied** in `SSP_ConstructFrame` over the header and data fields, stored as two bytes (CRC_1, CRC_0).
- **Verified** in `SSP_ReceiveFrame` by recalculating and comparing with the received CRC.
