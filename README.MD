Below is a consolidated and polished version for the BMS (Battery Management System) project based on the STM32L476RCT6 microcontroller, incorporating all your notes and additional details from our discussions. This documentation is structured for clarity and completeness, suitable for use in STM32CubeIDE or as a project reference.


---


# BMS Project Documentation


## Project Overview


This project implements a Battery Management System (BMS) using the **STM32L476RCT6** microcontroller, configured via **STM32CubeIDE**. It leverages two **BQ76920** battery monitor ICs for cell monitoring and protection, targeting a battery pack with configurable parameters. The firmware is designed to ensure safe operation, efficient power management, and reliable communication, with support for firmware updates over RS485.


### Hardware Reference
- **Microcontroller**: STM32L476RCT6
- **Battery Monitor ICs**: 2x BQ76920
- **Schematic**: Refer to the provided schematic for pin assignments and connections.
- **Reference Image**:  

  ![STM32L476RCT6 pinout](/images/STM32L476RCT6.png)


---


## Features


- **Battery Monitoring**  
  - Monitors cell voltages, pack currents, and temperatures using dual BQ76920 ICs for redundancy.
  - Calculates State of Charge (SOC) and State of Health (SOH) using Coulomb counting and Kalman filtering.


- **Protection Mechanisms**  
  - Overcharge and over-discharge protection
  - Overcurrent (charge/discharge) and short-circuit protection
  - Overtemperature and undertemperature protection


- **Cell Balancing**  
  - Implements passive cell balancing to maintain uniform voltages across battery groups.


- **Temperature Control**  
  - Uses a PID controller to manage heaters, ensuring safe operating temperatures.


- **Error Handling**  
  - Logs errors to flash memory with timestamps.
  - Implements protective actions and fault recovery for critical conditions.


- **Telemetry**  
  - Logs battery data (voltages, currents, temperatures, SOC, SOH) to flash.
  - Communicates via RS485 using a custom Simple Serial Protocol (SSP).


- **Low-Power Modes**  
  - Supports sleep mode to minimize power consumption when idle.


- **Firmware Updates**  
  - Supports remote firmware updates over RS485 with checksum validation and fallback mechanisms.


---


## To-Do List Status


- [✓] **Operation Modes Implementation**: Charging, Discharging, Sleep, and Fault modes implemented in `Update_BMS_Mode`.
- [✓] **Battery Charging Control**: CC-CV charging algorithm implemented in `ChargeBattery`.
- [✓] **Error Flag Handling**: All BQ76920 status flags handled in `BQ76920_CheckStatus`.
- [✓] **Protective Actions**: Fault-specific actions added in `Update_BMS_Mode`.
- [✓] **Fault Recovery**: Recovery mechanisms implemented with timeouts and reset logic.
- [✓] **Battery Pack Configuration**: Configurable parameters defined in `BatteryConfig` struct.
- [✓] **Firmware Updates**: Fully implemented via RS485 with bootloader and CRC32 validation (see improvements below).


---


## Step-by-Step Configuration in STM32CubeIDE


### Step 1: Create a New Project
1. Open **STM32CubeIDE**.
2. Go to `File > New > STM32 Project`.
3. In the MCU Selector, search for `STM32L476RCT6`, select it, and click `Next`.
4. Name the project (e.g., `BMS_STM32L476RCT6`) and click `Finish`.
5. The `.ioc` file opens in STM32CubeMX for configuration.


### Step 2: Configure the Clock
1. **Enable Oscillators**:
   - Under `System Core > RCC`:
     - `HSE`: Crystal/Ceramic Resonator (PH0-OSC_IN, PH1-OSC_OUT).
     - `LSE`: Crystal/Ceramic Resonator (PC14-OSC32_IN, PC15-OSC32_OUT).
2. **Clock Tree**:
   - Go to `Clock Configuration`.
   - Set HSE to 8 MHz.
   - Configure PLL for 80 MHz system clock (e.g., PLLM = 1, PLLN = 20, PLLR = 2).
   - Use LSE (32.768 kHz) for RTC.


### Step 3: Configure Pins and Peripherals
#### 3.1 USART Configuration
- **USART1** (Debug/Logging):
  - PA9: USART1_TX
  - PA10: USART1_RX
- **USART2** (RS485):
  - PA2: USART2_TX
  - PA3: USART2_RX
  - PA1: GPIO_Output (RS4852_DE)


#### 3.2 I2C Configuration
- **I2C1** (BQ76920 IC1):
  - PB6: I2C1_SCL
  - PB7: I2C1_SDA
- **I2C2** (BQ76920 IC2):
  - (Not explicitly used in schematic; adjust as per hardware)
- **I2C3** (Temperature Sensors):
  - PC0: I2C3_SCL
  - PC1: I2C3_SDA


#### 3.3 GPIO Configuration
- **PA12**: GPIO_Input (ALERT2 from BQ76920 IC2)
- **PB5**: GPIO_Input (ALERT from BQ76920 IC1)
- **PC3**: GPIO_Output (LED DS3)
- **PC7**: GPIO_Output (BOOT2 for BQ76920 IC2)
- **PB4**: GPIO_Output (BOOT for BQ76920 IC1)


#### 3.4 PWM for Heaters
- **TIM4**:
  - PB8: TIM4_CH3 (HEATER2)
  - PB9: TIM4_CH4 (HEATER1)


#### 3.5 Debug Configuration (SWD)
- **PA13**: SWDIO
- **PA14**: SWCLK
- **PB3**: SWO (JTDO_TRACESWO)


#### 3.6 ADC Configuration
- **ADC1**:
  - Enable Temperature Sensor Channel (IN16).
  - Settings: Single-ended, 12-bit resolution, Clock Prescaler PCLK/4, Software Start.


#### 3.7 RTC Configuration
- Enable RTC with LSE as clock source.


### Step 4: Additional Configurations
- **Boot Pins**: BOOT0 (Pin 60) tied to ground via 10kΩ resistor for normal operation.
- **Power Supply**: 3.3V VDD, with decoupling capacitors as per schematic.
- **Generate Code**: Save the `.ioc` file and generate code in STM32CubeIDE.


---


## Flashing the Firmware


### Using SWD (Primary Method)
1. **Hardware Setup**:
   - Connect an ST-Link debugger to:
     - SWDIO (PA13)
     - SWCLK (PA14)
     - NRST (Pin 7)
     - VDD (3.3V)
     - GND
   - Power the board independently (e.g., via battery or external 3.3V).


2. **STM32CubeIDE Configuration**:
   - Go to `Run > Debug Configurations`.
   - Create an STM32 Cortex-M MCU Debugging profile.
   - Set Debugger to ST-Link (OpenOCD), Interface to SWD.
   - Select the binary (e.g., `BMS_STM32L476RCT6.elf`).
   - Click `Debug` to flash and optionally debug.


3. **Verification**:
   - LED (PC3) blinks for 5 seconds post-flashing, then turns off.
   - Monitor UART1 logs via a terminal (115200 baud).


### Using UART Bootloader (Alternative)
1. **Enter Bootloader Mode**:
   - Set BOOT0 (Pin 60) to 3.3V (requires hardware modification if not exposed).
   - Reset the MCU via NRST or power cycle.


2. **Connect UART**:
   - Use a USB-to-UART adapter (3.3V logic):
     - TX → PA10 (USART1_RX)
     - RX → PA9 (USART1_TX)
     - GND → Board GND


3. **Flash with STM32CubeProgrammer**:
   - Select UART interface, COM port, and 115200 baud.
   - Load the `.hex` or `.bin` file and start programming.
   - Reset BOOT0 to ground and restart the MCU.


---


## Firmware Architecture


### Key Files
- **main.c**: Main application logic, initialization, and infinite loop.
- **BQ76920.c/h**: BQ76920 driver for voltage, current, and status monitoring.
- **temperature.c/h**: Temperature sensor interface.
- **pid.c/h**: PID controller for heater management.
- **kalman_filter.c/h**: Kalman filter for SOC/SOH estimation.
- **ssp.c/h**: Simple Serial Protocol for RS485 communication.
- **crc32.c/h**: CRC32 implementation for firmware update validation.


### Memory Layout
- **Bootloader**: 0x08000000 - 0x08003FFF (16 KB)
- **Application**: 0x08004000 - 0x0807F7FF (238 KB)
- **Backup Image**: 0x08040000 - 0x0807F7FF (optional, 192 KB)
- **Log Storage**: 0x08080000 onwards
- **Flags**: 
  - Next Slot: 0x0807F800
  - Firmware Update Flag: 0x0807F810
  - App Validity Flag: 0x0807F820


### Main Loop Operations
1. Read BQ76920 data (voltages, currents).
2. Read temperatures (battery and PCB).
3. Check redundancy and BQ76920 status.
4. Update timers and SOC/SOH.
5. Balance cells and control heaters.
6. Update BMS mode and apply CC-CV charging.
7. Log data and send telemetry via RS485.


---


## Protection and Fault Handling


### Error Flags (main.h)
- `ERROR_OVERVOLTAGE`: Overvoltage detected
- `ERROR_UNDERVOLTAGE`: Undervoltage detected
- `ERROR_OCC`: Overcurrent charge
- `ERROR_OCD`: Overcurrent discharge
- `ERROR_SCD`: Short-circuit discharge
- `ERROR_OVERTEMP`: Overtemperature
- `ERROR_UNDERTEMP`: Undertemperature
- `ERROR_DISCREPANCY`: Redundancy mismatch
- `ERROR_DEVICE_XREADY`: BQ76920 not ready
- `ERROR_OVRD_ALERT`: BQ76920 alert condition


### Protective Actions (Update_BMS_Mode)
- **Overvoltage**: Disable charging, allow discharging.
- **Undervoltage**: Disable discharging, allow charging.
- **Overcurrent (OCC/OCD)**: Disable respective operation, wait for cooldown.
- **Short-Circuit**: Disable both, reset after 30s if unresolved.
- **Temperature**: Disable operations based on severity, use PID for recovery.
- **Discrepancy/Device Issues**: Disable operations, attempt reinitialization.


### Fault Recovery
- **Timeouts**: 
  - General faults: 30s
  - Temperature faults: 60s
  - Cooldown: 10s
  - Recovery delay: 5s
- **Recovery**: Clears flags when conditions normalize (e.g., voltage, current, temperature thresholds met).
- **Reset**: Triggers `HAL_NVIC_SystemReset()` for persistent critical faults.


---


## Firmware Updates


### Implementation
- **Interface**: RS485 via USART2.
- **Protocol**: SSP with custom command `SSP_CMD_FIRMWARE_UPDATE` (0x40).
- **Process**:
  1. `Bootloader_Check` reads flag at `FIRMWARE_UPDATE_FLAG_ADDR`.
  2. If set (0xDEADBEEF), `Bootloader_FirmwareUpdate` erases app region and receives new firmware.
  3. Packets validated with CRC32; system reboots on success.
- **Trigger**: SSP command `SSP_CMD_SFP` sets the update flag.


### Improvements
1. **Checksum**: Upgraded from XOR to CRC32 for robust validation.
2. **Fallback**: Added backup image support at `BACKUP_START_ADDR` with validity check.
3. **Testing**: Recommended scenarios include partial updates, corrupted data, and timeouts.


---


## Notes
- **Hardware**: Verify pin assignments against your schematic, especially for I2C2 and unused pins.
- **Timeouts**: Adjust fault recovery timeouts based on battery specs and application needs.
- **Testing**: Thoroughly test firmware updates and fault recovery in a controlled environment.


---




