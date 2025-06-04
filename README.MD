# BMS Project Documentation

## Project Overview

This project implements a Battery Management System (BMS) for a CubeSat using the **STM32L476RCT6** microcontroller, configured via **STM32CubeIDE**. It manages a 4S2P lithium-ion battery pack (4 cells in series, 2 in parallel per group) using two **BQ76920** battery monitor ICs for redundancy. The firmware ensures safe operation, efficient power management, and reliable communication with the On-Board Computer (OBC) via the Simple Serial Protocol (SSP) over RS485. Key features include battery monitoring, protection, temperature control, and firmware updates.

### Hardware Reference
- **Microcontroller**: STM32L476RCT6
- **Battery Monitor ICs**: 2x BQ76920 (I2C1 and I2C2 for redundancy)
- **Temperature Sensors**: 2x TMP100 (I2C1 address 0x48 for NTC-1, I2C2 address 0x49 for NTC-2)
- **Heater Control**: 2x TPS22810 load switches (controlled via GPIO PB9 for Heater 1, PB8 for Heater 2)
- **Schematic**: Refer to the provided schematic for pin assignments and connections.
- **Reference Image**:  

  ![STM32L476RCT6 pinout](/images/STM32L476RCT6.png)

---

## Features

- **Battery Monitoring**
  - Monitors cell voltages, pack currents, and temperatures using dual BQ76920 ICs for redundancy.
  - Calculates State of Charge (SOC) and State of Health (SOH) using Coulomb counting and Kalman filtering.

- **Protection Mechanisms**
  - Overcharge (4.2V), over-discharge (2.8V), overcurrent (5000 mA), and short-circuit protection.
  - Overtemperature (60°C) and undertemperature (-20°C) protection.

- **Cell Balancing**
  - Implements passive cell balancing to maintain uniform voltages across battery groups.

- **Temperature Control**
  - Uses a PID controller to manage two heaters via TPS22810 load switches (on/off control).
  - Maintains battery temperature between 15°C and 35°C, with a safety override at 60°C.
  - Monitors PCB temperature using the STM32’s internal sensor.

- **Error Handling**
  - Logs errors to flash memory with timestamps.
  - Implements protective actions and fault recovery for critical conditions.

- **Telemetry**
  - Logs battery data (voltages, currents, temperatures, SOC, SOH) to flash.
  - Communicates with the OBC via RS485 using SSP (115200 baud).

- **Low-Power Modes**
  - Supports sleep mode to minimize power consumption when idle.

- **Firmware Updates**
  - Supports remote firmware updates over RS485 with CRC16 validation and fallback mechanisms.

---

## To-Do List Status

- [✓] **Operation Modes Implementation**: Charging, Discharging, Sleep, and Fault modes implemented in `Update_BMS_Mode`.
- [✓] **Battery Charging Control**: CC-CV charging algorithm implemented in `ChargeBattery`.
- [✓] **Error Flag Handling**: All BQ76920 status flags handled in `BQ76920_CheckStatus`.
- [✓] **Protective Actions**: Fault-specific actions added in `Update_BMS_Mode`.
- [✓] **Fault Recovery**: Recovery mechanisms implemented with timeouts and reset logic.
- [✓] **Battery Pack Configuration**: Configurable parameters defined in `BatteryConfig` struct.
- [✓] **Firmware Updates**: Fully implemented via RS485 with bootloader and CRC16 validation.
- [✓] **Heater Control Update**: Replaced PWM control with direct GPIO on/off control via TPS22810 switches.
- [✓] **Code Consolidation**: Merged `power_switch.c` and `power_switch.h` into `pid.c` and `pid.h`.

---

## Step-by-Step Configuration in STM32CubeIDE
---------------------------------------------

This section provides a detailed guide to configure the STM32L476RCT6 microcontroller for the AFDEVSAT Battery Management System (BMS) using STM32CubeIDE. Each step is justified based on the requirements outlined in the Interface Control Document (ICD) (AFDEVSAT\_ICD\_EgSA Release-V1.1.docx.pdf) and the schematics (EPS\_BMS1.pdf). The goal is to ensure that users can replicate the setup accurately, producing the expected pin assignments and functionality.

### Step 1: Create a New Project

1.  **Launch STM32CubeIDE**:
    
    *   Open STM32CubeIDE on your computer. This is the integrated development environment (IDE) provided by STMicroelectronics for STM32 development, which includes STM32CubeMX for graphical configuration.
        
2.  **Create a New Project**:
    
    *   Go to File > New > STM32 Project.
        
    *   In the **MCU Selector**, search for STM32L476RCT6. This microcontroller is specified in the project overview as the target device for the BMS.
        
    *   Select STM32L476RCT6, click Next, and name the project (e.g., BMS\_STM32L476RCT6).
        
    *   Click Finish. This generates a new project, and the .ioc file opens in STM32CubeMX, allowing graphical configuration of the MCU.
        
3.  **Understand the Interface**:
    
    *   STM32CubeMX has two main views: **Pinout & Configuration** (for setting up peripherals and pins) and **Clock Configuration** (for configuring the clock tree). You’ll use both views to set up the system.
        

### Step 2: Configure the Clock (RCC Settings)

The clock configuration is critical for ensuring the MCU operates at the correct frequency, meeting timing requirements for communication protocols and peripheral operation. The settings are derived from the ICD and hardware constraints.

#### 2.1 Enable Oscillators

*   **Navigate to RCC**:
    
    *   In STM32CubeMX, go to the **Pinout & Configuration** tab, then under System Core, select RCC.
        
*   **Configure HSE (High-Speed External Oscillator)**:
    
    *   Set HSE to Crystal/Ceramic Resonator.
        
    *   **Justification**:
        
        *   The STM32L476RCT6 schematic (EPS\_BMS1.pdf) typically uses an 8 MHz external crystal connected to pins PH0-OSC\_IN (Pin 2) and PH1-OSC\_OUT (Pin 3), as per the STM32L476RCT6 datasheet recommendations for stable clock sources.
            
        *   The ICD does not specify a system clock frequency, but an 8 MHz HSE is a common choice for low-power applications like CubeSats, balancing performance and power consumption.
            
        *   The HSE provides a stable clock source for the system clock, which is necessary for accurate timing in RS485 communication (115200 baud, as specified in ICD Section 2.4, page 18).
            
*   **Configure LSE (Low-Speed External Oscillator)**:
    
    *   Set LSE to Crystal/Ceramic Resonator.
        
    *   **Justification**:
        
        *   The LSE uses a 32.768 kHz crystal connected to pins PC14-OSC32\_IN (Pin 45) and PC15-OSC32\_OUT (Pin 46), as per typical STM32 designs and the schematic (EPS\_BMS1.pdf).
            
        *   The LSE is required for the Real-Time Clock (RTC), which is used for timestamping logs and synchronizing time with the OBC (ICD Section 6.3.2.4, STIME command, page 69). The ICD specifies time synchronization every 60 seconds (page 101), necessitating an accurate RTC clock source.
            

#### 2.2 Configure the Clock Tree

*   **Navigate to Clock Configuration**:
    
    *   Switch to the **Clock Configuration** tab in STM32CubeMX to view and configure the clock tree.
        
*   **Set HSE to 8 MHz**:
    
    *   In the clock tree, set the HSE frequency to 8 MHz (manually enter 8 in the HSE box).
        
    *   **Justification**:
        
        *   The 8 MHz frequency aligns with the typical external crystal used in STM32 designs and assumed in the schematic (EPS\_BMS1.pdf).
            
        *   This frequency ensures sufficient performance for the BMS tasks (e.g., RS485 communication, I2C, ADC sampling) while keeping power consumption low, which is critical for a CubeSat application (ICD Section 3, Power Budget, pages 22-51).
            
*   **Configure System Clock to 8 MHz**:
    
    *   Set SYSCLKSource to HSE (select HSE as the system clock source).
        
    *   Set the PLL to Disabled (set PLL State to RCC\_PLL\_NONE).
        
    *   **Justification**:
        
        *   The ICD does not specify a high-performance requirement for the BMS (e.g., no high-speed processing for imaging or complex algorithms). An 8 MHz system clock is sufficient for:
            
            *   RS485 communication at 115200 baud (ICD Section 2.4, page 18: maximum command/response time of 70 ms, which an 8 MHz clock can handle).
                
            *   I2C communication at 400 kHz (ICD does not specify, but 400 kHz is used for BQ76920 and TMP100 sensors).
                
            *   ADC sampling for the internal temperature sensor (12-bit resolution, low sampling rate).
                
        *   Disabling the PLL reduces power consumption, aligning with the CubeSat’s power budget constraints (ICD Section 3, pages 22-51). The EPS board itself consumes only 0.5W at 5V and 0.3W at 3.3V (ICD Table 2, page 24), indicating a low-power design.
            
        *   The STM32L476RCT6 can operate at 8 MHz without PLL, providing a straightforward clock setup that avoids additional jitter or power overhead.
            
*   **Set AHB, APB1, and APB2 Prescalers**:
    
    *   Set AHBCLKDivider to 1 (HCLK = 8 MHz).
        
    *   Set APB1CLKDivider to 1 (PCLK1 = 8 MHz).
        
    *   Set APB2CLKDivider to 1 (PCLK2 = 8 MHz).
        
    *   **Justification**:
        
        *   No division is needed since the system clock is already at 8 MHz, which is sufficient for all peripherals.
            
        *   The RS485 communication at 115200 baud requires precise UART timing, and an 8 MHz PCLK ensures accurate baud rate generation (error < 1%, as per STM32L476RCT6 datasheet).
            
        *   The ADC clock (derived from PCLK2) needs to be within the recommended range (0.5 MHz to 48 MHz, datasheet Section 6.3.20). With PCLK2 at 8 MHz and an ADC prescaler of 4 (configured later), the ADC clock is 2 MHz, which is suitable for sampling the internal temperature sensor at a low rate.
            
        *   I2C at 400 kHz can operate with PCLK1 at 8 MHz, meeting timing requirements (I2C timing parameter 0x00201D2B used in main.c supports 400 kHz at 8 MHz PCLK1).
            
*   **Set RTC Clock Source**:
    
    *   Ensure the RTC clock source is set to LSE (32.768 kHz).
        
    *   **Justification**:
        
        *   The RTC requires a precise 32.768 kHz clock for accurate timekeeping, as specified for timestamping logs (Log\_Error in main.c) and time synchronization with the OBC (ICD Section 6.3.2.4, STIME, and page 101).
            
*   **Verify Clock Settings**:
    
    *   After configuration, the clock tree should show:
        
        *   SYSCLK: 8 MHz (from HSE).
            
        *   HCLK: 8 MHz (AHB clock for CPU and memory).
            
        *   PCLK1: 8 MHz (APB1 clock for peripherals like I2C, USART2).
            
        *   PCLK2: 8 MHz (APB2 clock for peripherals like ADC, USART1).
            
        *   RTC Clock: 32.768 kHz (from LSE).
            
    *   **Justification**:
        
        *   These settings ensure all peripherals operate within their required frequency ranges while minimizing power consumption, aligning with the CubeSat’s power budget (ICD Section 3).
            

### Step 3: Configure Pins and Peripherals

This step configures the STM32L476RCT6 pins and peripherals based on the ICD and schematic requirements. Each configuration is justified to match the AFDEVSAT BMS functionality.

#### 3.1 USART Configuration

The BMS uses two USART interfaces for RS485 communication, as specified in the ICD.

*   **USART1 (Debug/Logging over RS485)**:
    
    *   **Pin Assignment**:
        
        *   PA9 (Pin 32): USART1\_TX
            
        *   PA10 (Pin 33): USART1\_RX
            
    *   **Settings**:
        
        *   Baud Rate: 115200
            
        *   Word Length: 8 bits
            
        *   Parity: None
            
        *   Stop Bits: 1 (8N1 configuration)
            
    *   **Justification**:
        
        *   The ICD Section 2.4 (page 18) specifies that the RS485 bus operates at 115200 bps, 8 bits, 1 start bit, and 1 stop bit (8N1).
            
        *   main.c uses USART1 (huart1) for logging (Log\_Read\_All, page 43), sending logs to the OBC over RS485.
            
        *   The schematic (EPS\_BMS1.pdf, page 3, "RS485 Transceiver") shows an RS485 transceiver connected to PA9 (TX) and PA10 (RX), consistent with the pin assignments in main.c (main.h, page 13).
            
*   **USART2 (Communication with OBC over RS485)**:
    
    *   **Pin Assignment**:
        
        *   PA2 (Pin 17): USART2\_TX
            
        *   PA3 (Pin 18): USART2\_RX
            
        *   PA1 (Pin 16): GPIO\_Output (named RS4852\_DE, driver enable for RS485 transceiver)
            
    *   **Settings**:
        
        *   Baud Rate: 115200
            
        *   Word Length: 8 bits
            
        *   Parity: None
            
        *   Stop Bits: 1 (8N1)
            
    *   **Justification**:
        
        *   The ICD Section 2.4 (page 18) confirms the RS485 bus settings (115200 baud, 8N1), and Section 2.3 (page 16) specifies two RS485 interfaces: RS485\_1 (pins 18, 20) and RS485\_2 (pins 29, 31).
            
        *   main.c uses USART2 (husart2) for SSP communication with the OBC (SSP\_SendStatus, SSP\_ProcessReceivedFrame, pages 48-50), matching the ICD’s use of RS485\_2 for OBC communication.
            
        *   The schematic (EPS\_BMS1.pdf, page 3) shows an RS485 transceiver connected to PA2 (TX), PA3 (RX), and PA1 (DE pin), aligning with main.h (page 13) definitions (RS4852\_DE\_Pin, USART2\_TX\_Pin, USART2\_RX\_Pin).
            
        *   PA1 (RS4852\_DE) is configured as a GPIO output to control the RS485 transceiver’s driver enable, toggling between transmit and receive modes (set high for TX, low for RX, as in SSP\_SendStatus, page 48).
            

#### 3.2 I2C Configuration

The BMS uses I2C to communicate with BQ76920 ICs and TMP100 temperature sensors for battery monitoring and temperature control.

*   **I2C1 (BQ76920 IC1 and TMP100 NTC-1)**:
    
    *   **Pin Assignment**:
        
        *   PB6 (Pin 29): I2C1\_SCL
            
        *   PB7 (Pin 30): I2C1\_SDA
            
    *   **Settings**:
        
        *   Speed: 400 kHz
            
        *   Addressing Mode: 7-bit
            
        *   Analog Filter: Enabled
            
        *   Digital Filter: Disabled
            
    *   **Justification**:
        
        *   The ICD does not specify I2C speed, but 400 kHz is a standard choice for BQ76920 ICs (as per BQ76920 datasheet, supports up to 400 kHz) and TMP100 sensors (TMP100 datasheet, max 400 kHz).
            
        *   main.c uses I2C1 (hi2c1) to interface with BQ76920 IC1 (address 0x08, assumed) and TMP100 NTC-1 (address 0x48, ICD page 89), as seen in BQ76920\_ReadVoltages and Temperature\_Read calls (page 43).
            
        *   The schematic (EPS\_BMS1.pdf) does not explicitly show I2C connections, but PB6 and PB7 are standard I2C1 pins on the STM32L476RCT6, matching main.h definitions (page 13).
            
        *   Analog filter is enabled to improve noise immunity in the noisy space environment, while the digital filter is disabled to avoid unnecessary signal delay.
            
*   **I2C2 (BQ76920 IC2 and TMP100 NTC-2)**:
    
    *   **Pin Assignment**:
        
        *   PB10 (Pin 35): I2C2\_SCL
            
        *   PB11 (Pin 36): I2C2\_SDA
            
    *   **Settings**:
        
        *   Speed: 400 kHz
            
        *   Addressing Mode: 7-bit
            
        *   Analog Filter: Enabled
            
        *   Digital Filter: Disabled
            
    *   **Justification**:
        
        *   I2C2 is used for redundancy with BQ76920 IC2 (address 0x09, assumed) and TMP100 NTC-2 (address 0x49, ICD page 89), as per main.c (BQ76920\_ReadVoltages and Temperature\_Read, page 43).
            
        *   The schematic (EPS\_BMS1.pdf) does not specify I2C2 pins, so PB10 and PB11 are chosen as they are default I2C2 pins on the STM32L476RCT6 and available (not conflicting with other peripherals in main.h).
            
        *   Settings match I2C1 for consistency and compatibility with BQ76920 and TMP100 requirements.
            
*   **I2C3 (Unused but Configured for Potential Expansion)**:
    
    *   **Pin Assignment**:
        
        *   PC0 (Pin 41): I2C3\_SCL
            
        *   PC1 (Pin 42): I2C3\_SDA
            
    *   **Settings**:
        
        *   Speed: 400 kHz
            
        *   Addressing Mode: 7-bit
            
        *   Analog Filter: Enabled
            
        *   Digital Filter: Disabled
            
    *   **Justification**:
        
        *   I2C3 (hi2c3) is configured in main.c (page 41) but not used in the current firmware. It’s retained for potential future expansion (e.g., additional sensors).
            
        *   PC0 and PC1 are default I2C3 pins and do not conflict with other assignments in main.h (page 13).
            

#### 3.3 GPIO Configuration

GPIO pins are configured for status indication, BQ76920 control, RS485 driver enable, and heater control.

<<<<<<< HEAD
#### 3.4 PWM for Heaters(THIS IS WRONG I NEED TO WORK ON THIS)
- **TIM4**:
  - PB8: TIM4_CH3 (HEATER2)
  - PB9: TIM4_CH4 (HEATER1)
=======
*   **PC3 (Status LED)**:
    
    *   **Configuration**: GPIO\_Output (named LED).
        
    *   **Justification**:
        
        *   Used for status indication (blinking for 5 seconds at startup, main.c page 43).
            
        *   Defined in main.h as LED\_Pin (page 13), matching the schematic (EPS\_BMS1.pdf, not explicitly shown but assumed as DS3).
            
        *   Set as push-pull output with no pull-up/pull-down (GPIO\_NOPULL) and low speed (GPIO\_SPEED\_FREQ\_LOW) to minimize power consumption (ICD power budget, page 24).
            
*   **PA1 (RS485 Driver Enable)**:
    
    *   **Configuration**: GPIO\_Output (named RS4852\_DE).
        
    *   **Justification**:
        
        *   Controls the RS485 transceiver’s driver enable for USART2, toggling between transmit and receive modes (ICD Section 2.4, page 18; main.c page 48).
            
        *   Matches main.h definition (RS4852\_DE\_Pin, page 13) and schematic (EPS\_BMS1.pdf, page 3).
            
*   **PC7 (BOOT2 for BQ76920 IC2)**:
    
    *   **Configuration**: GPIO\_Input (named BOOT2).
        
    *   **Justification**:
        
        *   Defined in main.h (page 13) but unused in the current firmware (main.c does not reference it).
            
        *   Originally assumed to control a power line, but clarified as unused for heaters (now controlled by PB8, PB9). Configured as input for potential future use (e.g., BQ76920 IC2 boot control).
            
*   **PA12 (ALERT2 from BQ76920 IC2)**:
    
    *   **Configuration**: GPIO\_Input (named ALERT2).
        
    *   **Justification**:
        
        *   Connected to the ALERT pin of BQ76920 IC2 for fault detection (main.c, BQ76920\_CheckStatus, page 43).
            
        *   Matches main.h definition (ALERT2\_Pin, page 13) and schematic (assumed, as EPS\_BMS1.pdf does not show specific connections).
            
*   **PB4 (BOOT for BQ76920 IC1)**:
    
    *   **Configuration**: GPIO\_Input (named BOOT).
        
    *   **Justification**:
        
        *   Similar to PC7, defined in main.h (page 13) but unused in the current firmware. Configured as input for potential future use.
            
*   **PB5 (ALERT from BQ76920 IC1)**:
    
    *   **Configuration**: GPIO\_Input (named ALERT).
        
    *   **Justification**:
        
        *   Connected to the ALERT pin of BQ76920 IC1 for fault detection (main.c, BQ76920\_CheckStatus, page 43).
            
        *   Matches main.h definition (ALERT\_Pin, page 13) and schematic (assumed).
            
*   **PB8 (Heater 2 Control)**:
    
    *   **Configuration**: GPIO\_Output (named HEATER2).
        
    *   **Justification**:
        
        *   Controls the TPS22810 load switch for Heater 2 (on/off control, no PWM, as updated in pid.c).
            
        *   Matches main.h definition (HEATER2\_Pin, page 13) and schematic (EPS\_BMS1.pdf, page 6, "Heater Control Circuit").
            
        *   Set as push-pull output with no pull-up/pull-down and low speed to minimize power consumption (ICD Table 2, heaters consume 9.6W at 12V, page 25).
            
*   **PB9 (Heater 1 Control)**:
    
    *   **Configuration**: GPIO\_Output (named HEATER1).
        
    *   **Justification**:
        
        *   Controls the TPS22810 load switch for Heater 1 (on/off control, no PWM).
            
        *   Matches main.h definition (HEATER1\_Pin, page 13) and schematic (EPS\_BMS1.pdf, page 6).
            
        *   Same settings as PB8 for consistency.
            
*   **PE11 (Charging Circuit Enable)**:
    
    *   **Configuration**: GPIO\_Output (unnamed in main.h but referenced in main.c).
        
    *   **Justification**:
        
        *   Controls the charging circuit enable for CC-CV charging (ChargeBattery, main.c page 50).
            
        *   Not explicitly named in main.h but used as GPIOE Pin 11 in main.c. Configure as push-pull output with no pull-up/pull-down and low speed.
            
>>>>>>> fa1221c (Refactored heater control: removed PWM, added GPIO power switch control)

#### 3.4 Heater Control (No PWM Required)

*   **PB8 (HEATER2)**: Controls Heater 2 via TPS22810 (on/off, no PWM).
    
*   **PB9 (HEATER1)**: Controls Heater 1 via TPS22810 (on/off, no PWM).
    
*   **Note**: Remove TIM4 configuration for PWM (previously used for heaters) from STM32CubeMX unless used elsewhere.
    
*   **Justification**:
    
    *   The ICD (page 16, Table 1) specifies a switchable "Heater Supply" power line at 12V, 1000 mA (9.6W, page 25), controlled by the EPS.
        
    *   The firmware update removed PWM control (TIM4 channels 3 and 4) in favor of direct GPIO control via TPS22810 switches (pid.c), reducing complexity and CPU overhead.
        
    *   The schematic (EPS\_BMS1.pdf, page 6) confirms the heater control circuit uses TPS22810 switches, controlled by PB8 and PB9, aligning with main.h and pid.c.
        

#### 3.5 Debug Configuration (SWD)

*   **Pin Assignment**:
    
    *   PA13 (Pin 49): SWDIO (debug interface).
        
    *   PA14 (Pin 50): SWCLK (debug interface).
        
    *   PB3 (Pin 31): SWO (JTDO\_TRACESWO, for trace output).
        
*   **Justification**:
    
    *   These are the default SWD pins for the STM32L476RCT6, used for debugging and flashing the firmware (ICD Section "Flashing the Firmware", page 104).
        
    *   Matches main.h definitions (page 13) and schematic (EPS\_BMS1.pdf, not explicitly shown but standard for STM32).
        

#### 3.6 ADC Configuration

The BMS uses the ADC to read the STM32’s internal temperature sensor for PCB temperature monitoring.

*   **ADC1**:
    
    *   **Channel**:
        
        *   Enable Internal Temperature Sensor Channel (ADC\_CHANNEL\_16, IN16).
            
    *   **Settings**:
        
        *   Mode: Single-ended
            
        *   Resolution: 12-bit
            
        *   Clock Prescaler: PCLK/4 (ADC clock = 8 MHz )
            
        *   Conversion Trigger: Software Start
            
            
    *   **Justification**:
        
        *   The internal temperature sensor is used to monitor PCB temperature (Read\_Internal\_Temperature, main.c page 48), protecting electronics (ICD Section 2.3, overtemperature threshold 60°C, page 15).
            
        *   12-bit resolution provides sufficient accuracy for temperature measurement (datasheet Section 6.3.20, temperature sensor accuracy ±2°C).
            
        *   ADC clock at 2 MHz (from PCLK2 8 MHz) is within the recommended range (0.5 MHz to 48 MHz, datasheet), ensuring reliable sampling.
            
        *   Software start allows on-demand sampling in the main loop (page 43), and 2.5 cycles sampling time ensures accurate readings for the slow-changing temperature signal.
            

#### 3.7 RTC Configuration

The RTC is used for timestamping logs and time synchronization with the OBC.

*   **Configuration**:
    
    *   Enable RTC with LSE as the clock source.
        
    *   Settings:
        
        *   Hour Format: 24-hour
            
        *   Asynchronous Prescaler: 127
            
        *   Synchronous Prescaler: 255
            
*   **Justification**:
    
    *   The LSE (32.768 kHz) provides the RTC clock, as configured in Step 2.
        
    *   Asynchronous prescaler 127 and synchronous prescaler 255 divide the 32.768 kHz clock to produce a 1 Hz tick (32.768 kHz / (128 \* 256) = 1 Hz), standard for RTC operation.
        
    *   The RTC is used for timestamping logs (Log\_Error, main.c page 41) and synchronizing time with the OBC every 60 seconds (ICD page 101, STIME command, page 69).
        

### Step 4: Additional Configurations

These steps ensure the hardware setup aligns with the software configuration.

*   **Boot Pins**:
    
    *   **BOOT0 (Pin 60)**: Tie to ground via a 10kΩ resistor for normal operation (or expose for UART bootloader access).
        
    *   **Justification**:
        
        *   BOOT0 low ensures the MCU boots from flash memory (normal operation mode), as per the STM32L476RCT6 datasheet (Section 2.5, Boot Modes).
            
        *   Exposing BOOT0 allows entry into the UART bootloader for firmware updates if SWD is unavailable (ICD Section "Flashing the Firmware", page 104).
            
*   **Power Supply**:
    
    *   **VDD**: 3.3V, with decoupling capacitors (e.g., 100 nF near each VDD pin, 10 µF bulk).
        
    *   **VDDA**: 3.3V, with a 1 µF decoupling capacitor for ADC.
        
    *   **Justification**:
        
        *   The STM32L476RCT6 operates at 3.3V (datasheet Section 3.3, Supply Voltage), matching the ICD’s 3.3V logic levels for GPIO and serial interfaces (ICD Section 2.3, page 17).
            
        *   Decoupling capacitors are required for stable operation (datasheet Section 6.1.2, Power Supply Scheme), especially in a noisy space environment.
            
        *   VDDA requires a dedicated 1 µF capacitor for ADC accuracy (datasheet Section 6.3.20), as the BMS uses ADC1 for temperature monitoring.
            
*   **Generate Code**:
    
    *   Save the .ioc file and click Generate Code in STM32CubeIDE.
        
    *   **Justification**:
        
        *   This step generates the initialization code (main.c, main.h, etc.) based on the configurations, ready for firmware integration.
            

### Verification of Configuration

After completing the steps above, verify the following in STM32CubeIDE:

*   **Pinout View**:
    
    *   Ensure all pins match main.h definitions (page 13):
        
        *   USART1: PA9 (TX), PA10 (RX)
            
        *   USART2: PA2 (TX), PA3 (RX), PA1 (RS4852\_DE)
            
        *   I2C1: PB6 (SCL), PB7 (SDA)
            
        *   I2C2: PB10 (SCL), PB11 (SDA)
            
        *   I2C3: PC0 (SCL), PC1 (SDA)
            
        *   GPIOs: PC3 (LED), PC7 (BOOT2), PA12 (ALERT2), PB4 (BOOT), PB5 (ALERT), PB8 (HEATER2), PB9 (HEATER1), PE11 (charging enable)
            
        *   SWD: PA13 (SWDIO), PA14 (SWCLK), PB3 (SWO)
            
*   **Clock Tree**:
    
    *   SYSCLK, HCLK, PCLK1, PCLK2: 8 MHz
        
    *   RTC Clock: 32.768 kHz
        
    *   ADC Clock: 8 MHz (PCLK2 / 4)
        
*   **Generated Code**:
    
    *   Open main.c and confirm that peripheral initialization functions (MX\_USART1\_UART\_Init, MX\_USART2\_Init, MX\_I2C1\_Init, etc.) match the settings above.
        
    *   Ensure MX\_GPIO\_Init configures all GPIO pins as specified (page 52).

## Flashing the Firmware

### Using SWD (Primary Method)
1. **Hardware Setup**:
   - Connect an ST-Link debugger to:
     - SWDIO (PA13, Pin 49)
     - SWCLK (PA14, Pin 50)
     - NRST (Pin 7)
     - VDD (3.3V, e.g., Pin 55)
     - GND (e.g., Pin 54)
   - Power the board independently (e.g., via battery or external 3.3V supply).

2. **STM32CubeIDE Configuration**:
   - Go to `Run > Debug Configurations`.
   - Create a new `STM32 Cortex-M MCU Debugging` profile.
   - Set Debugger to ST-Link (OpenOCD), Interface to SWD.
   - Select the binary (e.g., `BMS_STM32L476RCT6.elf`).
   - Click `Debug` to flash and optionally debug.

3. **Verification**:
   - LED (PC3) blinks for 5 seconds post-flashing (200 ms interval), then turns off.
   - Monitor UART1 logs via a terminal (115200 baud, 8N1) for system messages (e.g., "Booting to application").

### Using UART Bootloader (Alternative)
1. **Enter Bootloader Mode**:
   - Set BOOT0 (Pin 60) to 3.3V (requires hardware modification if not exposed).
   - Reset the MCU via NRST (Pin 7) or power cycle.

2. **Connect UART**:
   - Use a USB-to-UART adapter (3.3V logic):
     - TX → PA10 (USART1_RX, Pin 33)
     - RX → PA9 (USART1_TX, Pin 32)
     - GND → Board GND (e.g., Pin 54)

3. **Flash with STM32CubeProgrammer**:
   - Select UART interface, COM port, and 115200 baud.
   - Load the `.hex` or `.bin` file and start programming.
   - Reset BOOT0 to ground and restart the MCU.

4. **Verification**:
   - Same as SWD method: LED blinking and UART1 logs.

---

## Firmware Architecture

### Key Files
- **main.c**: Main application logic, peripheral initialization, and infinite loop for periodic tasks.
- **main.h**: Definitions for GPIO pins, error flags, battery configuration, and constants (e.g., flash addresses, log sizes).
- **BQ76920.c/h**: Driver for BQ76920 ICs, handling voltage, current, and status monitoring, protection configuration, and cell balancing.
- **temperature.c/h**: Interface for TMP100 temperature sensors on I2C1 (NTC-1) and I2C2 (NTC-2).
- **pid.c/h**: PID controller for heater management, now includes TPS22810 load switch control (previously in `power_switch.c/h`).
- **kalman_filter.c/h**: Kalman filter implementation for SOC and SOH estimation.
- **ssp.c/h**: Simple Serial Protocol (SSP) for RS485 communication with the OBC.
- **crc16.c/h**: CRC16 implementation for firmware update validation (assumed, as `CalculateCRC16` is referenced).

### Memory Layout
- **Bootloader**: 0x08000000 - 0x08003FFF (16 KB)
- **Application**: 0x08004000 - 0x0807F7FF (238 KB)
- **Backup Image**: 0x08040000 - 0x0807F7FF (optional, 192 KB)
- **Log Storage**: 0x08080000 onwards (size depends on `NUM_LOG_ENTRIES` and `LOG_ENTRY_SIZE`)
- **Flags**:
  - Next Slot: `NEXT_SLOT_ADDR` (0x0807F800)
  - Firmware Update Flag: `FIRMWARE_UPDATE_FLAG_ADDR` (0x0807F810)
  - App Validity Flag: `APP_VALIDITY_FLAG_ADDR` (0x0807F820)

### Main Loop Operations
The main loop in `main.c` runs every `LOOP_TIME` (assumed 0.1s, defined in `main.h`) and performs:
1. Read BQ76920 data (cell voltages, pack currents) from both ICs.
2. Read battery temperatures (NTC-1, NTC-2) and PCB temperature (internal sensor).
3. Check redundancy between BQ76920 ICs and detect faults (e.g., overvoltage, overcurrent).
4. Update timers (charge, discharge, operating time) and calculate SOC/SOH.
5. Perform cell balancing if needed.
6. Control heaters using PID based on the lowest battery temperature.
7. Update BMS mode (Charging, Discharging, Sleep, Fault) and apply CC-CV charging if in charging mode.
8. Log system state (voltages, currents, temperatures, SOC, SOH) to flash.
9. Send telemetry to the OBC every 5 seconds via RS485.
10. Synchronize RTC with the OBC every 60 seconds.
11. Process incoming SSP commands from the OBC (e.g., SON/SOF, firmware update requests).

---

## Protection and Fault Handling

### Error Flags (`main.h`)
- `ERROR_OVERVOLTAGE`: Cell voltage > 4.2V
- `ERROR_UNDERVOLTAGE`: Cell voltage < 2.8V
- `ERROR_OCC`: Overcurrent during charging (> 5000 mA)
- `ERROR_OCD`: Overcurrent during discharging (> 5000 mA)
- `ERROR_SCD`: Short-circuit detected
- `ERROR_OVERTEMP`: Temperature > 60°C (battery or PCB)
- `ERROR_UNDERTEMP`: Temperature < -20°C (battery)
- `ERROR_DISCREPANCY`: Redundancy mismatch between BQ76920 ICs
- `ERROR_DEVICE_XREADY`: BQ76920 initialization failure
- `ERROR_OVRD_ALERT`: General alert from BQ76920

### Protective Actions (`Update_BMS_Mode`)
- **Overvoltage**: Disable charging, allow discharging.
- **Undervoltage**: Disable discharging, allow charging.
- **Overcurrent (OCC/OCD)**: Disable respective operation, wait for cooldown (10s).
- **Short-Circuit**: Disable both charging and discharging, reset after 30s if unresolved.
- **Overtemperature**: Disable both, wait for temperature to drop below 50°C (10°C hysteresis).
- **Undertemperature**: Disable charging, allow discharging, wait for temperature > -10°C.
- **Discrepancy/Device Issues**: Disable both, attempt reinitialization after 5s, reset after 3 attempts.

### Fault Recovery
- **Timeouts**:
  - General faults: 30s
  - Temperature faults: 60s
  - Cooldown period: 10s
  - Recovery delay: 5s
- **Recovery**: Clears fault flags when conditions normalize (e.g., voltage within range, temperature safe).
- **System Reset**: Triggers `HAL_NVIC_SystemReset()` for persistent critical faults.

---

## Heater Control Implementation

### Overview
The firmware controls two heaters to maintain battery temperature within a safe operating range (15°C to 35°C). Previously, heaters were controlled using PWM on TIM4 (channels 3 and 4), but this has been updated to direct on/off control via TPS22810 load switches, simplifying the design and reducing CPU overhead.

### Hardware Details
- **Heater 1**: Controlled by a TPS22810 load switch, enabled/disabled via GPIO PB9 (`HEATER1_Pin`).
- **Heater 2**: Controlled by a TPS22810 load switch, enabled/disabled via GPIO PB8 (`HEATER2_Pin`).
- **TPS22810 Configuration**:
  - EN/UVLO pins connected to PB9 (Heater 1) and PB8 (Heater 2), active high.
  - CT pins: Use 4700 pF capacitors for a 957 µs rise time at 12 V to limit inrush current (220 mA for a 22 µF load, per `tps22810_3.pdf` Section 10.5.2.3).
  - QOD pins: Tie to VOUT for 500 Ω discharge when disabled (per datasheet Section 9.3.3).
  - VIN: Connected to battery pack (or regulated 12 V), with a 1 µF ceramic capacitor.
  - VOUT: Connected to heater circuit, with an optional 22 µF capacitor if needed.

### Software Implementation
- **File**: Integrated into `pid.c` and `pid.h` (previously separate in `power_switch.c/h`).
- **Control Logic**:
  - `PID_Control` uses a PID algorithm to determine heater state:
    - Turns heaters on if PID output > 0 AND temperature < 15°C (error > 10, since `TARGET_TEMP` = 25°C).
    - Turns heaters off if PID output <= 0 OR temperature > 35°C (error < -10).
    - Maintains current state if conditions are not met (e.g., temperature between 15°C and 35°C).
  - Safety override: Disables heaters if temperature exceeds `TEMP_UPPER_LIMIT` (60°C).
- **GPIO Control**:
  - PB9 (`HEATER1_Pin`) and PB8 (`HEATER2_Pin`) are toggled high/low to enable/disable the TPS22810 switches.
  - Initialization in `PID_Init` ensures heaters are off at startup.
- **Logging**:
  - Logs heater state changes with temperature, PID output, and error for debugging.

### Verification
- **Expected Output**:
  - Below 15°C: Both heaters turn on (PB9 and PB8 high), logged as "Heaters enabled, Temp: <value>C, PID Output: <value>, Error: <value>".
  - Above 35°C: Both heaters turn off (PB9 and PB8 low), logged as "Heaters disabled, Temp: <value>C, PID Output: <value>, Error: <value>".
  - Above 60°C: Heaters disabled with log "Heaters disabled, Temp: <value>C".
- **Testing**:
  - Use a temperature sensor emulator or controlled environment to vary temperature.
  - Monitor PB9 and PB8 with a multimeter or logic analyzer to confirm toggling.
  - Check UART1 logs for heater state changes.

---

## Firmware Updates

### Implementation
- **Interface**: RS485 via USART2 (PA2, PA3, PA1 as DE pin).
- **Protocol**: SSP with custom command `SSP_CMD_FIRMWARE_UPDATE` (assumed 0x40).
- **Process**:
  1. `Bootloader_Check` reads flag at `FIRMWARE_UPDATE_FLAG_ADDR` (0x0807F810).
  2. If set (0xDEADBEEF), `Bootloader_FirmwareUpdate` erases app region and receives new firmware.
  3. First packet contains firmware size (4 bytes); subsequent packets are up to 128 bytes each.
  4. Packets are validated with CRC16 (`CalculateCRC16`); system reboots on success.
- **Trigger**: SSP command `SSP_CMD_SFP` (param_id 0x02) sets the update flag and reboots.
- **Fallback**: If the main application (0x08004000) is invalid, falls back to backup at `BACKUP_START_ADDR`.

### Improvements
1. **Checksum**: Uses CRC16 for packet validation, ensuring data integrity.
2. **Fallback**: Supports backup image with validity check (`IsApplicationValid`).
3. **Timeout**: Reboots after 10s (`FIRMWARE_UPDATE_TIMEOUT`) if no packets are received.
4. **Testing**: Recommended scenarios include partial updates, corrupted data, and timeouts.

---

## Notes
- **Hardware**:
  - Verify pin assignments against your schematic, especially for I2C2 (BQ76920 IC2) if not explicitly used.
  - Ensure TPS22810 CT capacitors (4700 pF) and QOD connections (tied to VOUT) are implemented as specified.
  - BOOT_Pin (PB4) and BOOT2_Pin (PC7) are configured as inputs but unused; consider repurposing or removing.
- **Timeouts**:
  - Adjust fault recovery timeouts based on battery specifications and application needs.
- **Testing**:
  - Thoroughly test heater control by varying temperature (e.g., 10°C to 40°C) and monitoring GPIO states (PB9, PB8).
  - Test firmware updates with simulated failures (e.g., disconnect during transfer) to verify fallback behavior.
- **Power Lines (SON/SOF Commands)**:
  - The `power_lines` array (controlled by SSP commands SON/SOF) is currently updated but not linked to hardware. Implement control for additional TPS22810 switches (e.g., using BOOT_Pin and BOOT2_Pin) if needed.

---

## Expected Output After Flashing

1. **LED Behavior**:
   - PC3 (LED) blinks at 200 ms intervals for 5 seconds post-flashing, then turns off, indicating successful startup.

2. **UART1 Logs** (115200 baud, 8N1):
   - Initial messages:
     ```
     Booting to application
     Time synchronized with OBC
     ```
   - Periodic logs every `LOOP_TIME` (e.g., 0.1s):
     ```
     Time: <tick> | Cell1: <voltage>mV Cell2: <voltage>mV Cell3: <voltage>mV Cell4: <voltage>mV I1: <current>mA I2: <current>mA T1: <temp>C T2: <temp>C PCB: <temp>C SOC: <value>% SOH: <value>%
     ```
   - Heater control logs:
     ```
     Heaters enabled, Temp: 14C, PID Output: 110.00, Error: 11.00
     Heaters disabled, Temp: 36C, PID Output: -110.00, Error: -11.00
     ```
   - Telemetry sent every 5 seconds (logged if monitoring SSP communication on USART2):
     - Includes pack voltage, current, SOC, SOH, temperatures, balancing status, etc.

3. **Heater Control**:
   - PB9 (Heater 1) and PB8 (Heater 2) toggle high (3.3V) when temperature < 15°C and PID output > 0.
   - Both pins go low (0V) when temperature > 35°C or PID output <= 0, or if temperature > 60°C (safety override).

4. **Fault Conditions** (if triggered):
   - Example log for overtemperature:
     ```
     Overtemperature detected
     Protective action: Disabled charging and discharging due to overtemperature
     ```

---

## Potential Improvements
- **Dynamic Configuration**: Load `BatteryConfig` from EEPROM to support different battery types.
- **Watchdog Timer**: Add a hardware watchdog to recover from software hangs.
- **Interrupt-Driven UART**: Use UART interrupts for SSP communication to reduce polling overhead.
- **Power Optimization**: Implement low-power modes for `MODE_SLEEP` to extend battery life.
- **Power Lines Control**: Implement hardware control for `power_lines` array (e.g., additional TPS22810 switches).




