CRC32 Calculation Documentation (crc32.c)
=========================================

Overview
--------

The crc32.c file contains a simple but powerful tool called CalculateCRC32, which helps ensure that data—like the firmware (software) for our satellite’s Battery Management System (BMS)—stays correct and safe. It’s like a “checksum” or a fingerprint that checks if information has been accidentally changed or damaged. This is super important in our project because we’re sending new software over a wire (RS485) to update the BMS, and we need to make sure it arrives perfectly.

### Purpose in the Code

*   **Simple Explanation**: Imagine sending a letter to a friend. You’d want to know if it got crumpled or lost words along the way. This tool makes a special code (a CRC32 number) from the data, and we check that code later to see if everything’s still right.
    
*   **Technical Purpose**:
    
    *   **Firmware Updates**: In Bootloader\_FirmwareUpdate, it verifies that the new software received matches what was sent, preventing corrupted updates that could break the satellite.
        
    *   **Data Integrity**: Could also be used elsewhere (e.g., in CC-CV charging logs or telemetry) to ensure critical data isn’t garbled.
        
    *   **Safety**: Ensures the BMS runs only correct, safe software, protecting the satellite’s battery.
        

Dependencies
------------

*   **crc32.h**: A header file that tells other parts of the code about CalculateCRC32.
    
*   **stdint.h**: Gives us exact number sizes (like uint32\_t for a 32-bit number), so the code works the same everywhere.