\### SSP Protocol Documentation

\#### Frame Structure

The SSP frame consists of 9 fields with a maximum total size of 256 bytes:

\- \*\*Total frame size\*\*: 256 bytes maximum

\- \*\*Maximum data size\*\*: 248 bytes (256 - 8 bytes of overhead)

| Field | Size (Bytes) | Description |

|----------|--------------|---------------------------------------------|

| Flag | 1 | Start/end frame identifier (0xC0) |

| DEST | 1 | Destination subsystem address |

| SRC | 1 | Source subsystem address |

| CMD ID | 1 | Command identifier |

| D\_Len | 1 | Data field length (excluding header) |

| Data | Variable | Actual subsystem data (max 248 bytes) |

| CRC\_0 | 1 | CRC least significant byte |

| CRC\_1 | 1 | CRC most significant byte |

| Flag | 1 | End frame identifier (0xC0) |

\#### CRC Specification

\- \*\*Algorithm\*\*: CRC-16-CCITT

\- \*\*Polynomial\*\*: \\( X^{16} + X^{12} + X^5 + 1 \\) (0x1021)

\- \*\*Implementation\*\*: 16-bit CRC split into two bytes:

\- \*\*CRC\_0\*\*: Least significant byte (LSB)

\- \*\*CRC\_1\*\*: Most significant byte (MSB)

\- \*\*Purpose\*\*: Ensures frame integrity during transmission

\### CRC Application in the Code

The CRC-16-CCITT calculation and verification are implemented in several key functions within the provided \`ssp.c\` code. Below, I’ll highlight where and how the CRC is applied.

\#### 1. CRC Calculation Function: \`SSP\_CalculateCRC\`

This function computes the CRC-16 value for a given data buffer.

\`\`\`c

static uint16\_t SSP\_CalculateCRC(uint8\_t \*data, uint16\_t len)

{

uint16\_t crc = 0xFFFF; // Initial value

for (uint16\_t i = 0; i < len; i++) {

crc ^= data\[i\]; // XOR byte into CRC

for (uint8\_t j = 0; j < 8; j++) {

if (crc & 0x0001) {

crc >>= 1; // Shift right

crc ^= 0xA001; // XOR with reversed polynomial (0x1021 mirrored)

} else {

crc >>= 1; // Shift right

}

}

}

return crc;

}

\`\`\`

\- \*\*Input\*\*: Pointer to data and its length

\- \*\*Polynomial\*\*: 0xA001 (bit-reversed form of 0x1021 for efficient software implementation)

\- \*\*Output\*\*: 16-bit CRC value

\- \*\*Note\*\*: The initial value is 0xFFFF, which is standard for CRC-16-CCITT.

\#### 2. CRC in Frame Construction: \`SSP\_ConstructFrame\`

The CRC is calculated and appended to the frame before transmission.

\`\`\`c

HAL\_StatusTypeDef SSP\_ConstructFrame(SSP\_FrameTypeDef \*frame, uint8\_t \*buffer, uint16\_t \*frame\_len)

{

uint8\_t index = 0;

buffer\[index++\] = SSP\_FLAG; // Start flag

buffer\[index++\] = frame->dest;

buffer\[index++\] = frame->src;

buffer\[index++\] = frame->cmd\_id;

buffer\[index++\] = frame->data\_len;

for (uint8\_t i = 0; i < frame->data\_len; i++) {

buffer\[index++\] = frame->data\[i\];

}

frame->crc = SSP\_CalculateCRC(&buffer\[1\], frame->data\_len + 4); // CRC over DEST, SRC, CMD\_ID, D\_Len, and Data

buffer\[index++\] = (frame->crc >> 8) & 0xFF; // CRC\_1 (MSB)

buffer\[index++\] = frame->crc & 0xFF; // CRC\_0 (LSB)

buffer\[index++\] = SSP\_FLAG; // End flag

\*frame\_len = index;

return HAL\_OK;

}

\`\`\`

\- \*\*CRC Scope\*\*: Calculated over the header (DEST, SRC, CMD\_ID, D\_Len) and Data fields, starting from \`buffer\[1\]\` (excluding the start flag).

\- \*\*Length\*\*: \`frame->data\_len + 4\` (4 bytes for DEST, SRC, CMD\_ID, D\_Len).

\- \*\*Storage\*\*: Split into MSB (CRC\_1) and LSB (CRC\_0) and appended before the end flag.

\#### 3. CRC Verification in Frame Reception: \`SSP\_ReceiveFrame\`

The receiver recalculates the CRC and compares it with the received CRC.

\`\`\`c

HAL\_StatusTypeDef SSP\_ReceiveFrame(USART\_HandleTypeDef \*husart, uint8\_t \*buffer, uint16\_t buffer\_len, SSP\_FrameTypeDef \*frame)

{

// ... (frame reception logic)

frame->crc = (buffer\[index - 3\] << 8) | buffer\[index - 2\]; // Received CRC

uint16\_t calc\_crc = SSP\_CalculateCRC(&buffer\[1\], frame->data\_len + 4); // Recalculated CRC

if (frame->crc != calc\_crc) return HAL\_ERROR; // CRC mismatch check

return HAL\_OK;

}

\`\`\`

\- \*\*CRC Extraction\*\*: Received CRC is reconstructed from \`buffer\[index - 3\]\` (MSB) and \`buffer\[index - 2\]\` (LSB).

\- \*\*Verification\*\*: Recalculated CRC over the same fields (DEST, SRC, CMD\_ID, D\_Len, Data) is compared with the received CRC.

\- \*\*Error Handling\*\*: Returns \`HAL\_ERROR\` if the CRCs don’t match, indicating corruption.

\### Summary of CRC Usage

\- \*\*Calculation\*\*: Performed in \`SSP\_CalculateCRC\` using CRC-16-CCITT with polynomial 0xA001.

\- \*\*Transmission\*\*: Applied in \`SSP\_ConstructFrame\` over header and data, stored as CRC\_1 (MSB) and CRC\_0 (LSB).

\- \*\*Reception\*\*: Verified in \`SSP\_ReceiveFrame\` by recalculating and comparing with the received CRC.

This implementation ensures data integrity across the satellite’s internal bus as specified in the AFDEVSAT SSP protocol.