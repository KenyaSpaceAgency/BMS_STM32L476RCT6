
# BMS_Project

## Project Overview
This project is based on the STM32L476RCT6 microcontroller and is configured using STM32CubeIDE. The project includes configurations for clocks, pins, peripherals, and additional settings as per the provided schematic.



## Features
- **Battery Monitoring**: Utilizes the BQ76920 IC to monitor cell voltages, currents, and temperatures.
- **Protection Mechanisms**:
  - Overcharge and Over-discharge Protection
  - Overcurrent and Short-Circuit Protection
  - Overtemperature and Undertemperature Protection
- **Cell Balancing**: Implements cell balancing to maintain uniform cell voltages.
- **Temperature Control**: Uses PID control to manage heaters and ensure safe operating temperatures.
- **Error Handling**: Logs errors and handles critical faults to maintain system reliability.
- **Telemetry**: Logs battery data (voltages, currents, temperatures, SOC, SOH) and communicates via RS485.
- **Low-Power Modes**: Implements sleep modes to optimize power consumption when idle.

## To-Do List
 [✓] **Operation Modes Implementation**: Develop distinct modes for charging, discharging, sleep, and fault.
 [✓] **Battery Charging Control**: Implement a CC-CV charging algorithm and hardware control for enabling/disabling the charger.
 [✓] **Error Flag Handling**: Fully implement handling of all status flags from the BQ76920.
 [✓] **Protective Actions**: Add logic to take protective actions based on detected faults.
 [✓] **Fault Recovery**: Implement mechanisms for fault recovery and reset.
 [✓] **Battery Pack Configuration**: Support configurable battery parameters.
 [ ] **Firmware Updates**: Support firmware updates via UART or RS485.



STM32 Microcontroller Reference
-------------------------------

Below is an image of the STM32L476RCT6 for reference:


![STM32L476RCT6 pinout](/images/STM32L476RCT6.png)

## Step-by-Step Configuration

### Step 1: Create a New Project in STM32CubeIDE
1. **Open STM32CubeIDE**:
   - Launch STM32CubeIDE on your computer.
2. **Create a New Project**:
   - Go to `File > New > STM32 Project`.
   - In the "MCU/MPU Selector" window, search for `STM32L476RCT6`.
   - Select `STM32L476RCT6` from the list and click `Next`.
   - Name your project (e.g., `BMS_Project`) and click `Finish`.
3. **Initialize Peripherals**:
   - STM32CubeIDE will open the `.ioc` file in the Device Configuration Tool (STM32CubeMX).

### Step 2: Configure the Clock
1. **Enable HSE (High-Speed External) Clock**
   - In the `.ioc` file, go to the `Pinout & Configuration` tab.
   - Under `System Core > RCC`, set the following:
     - `HSE`: Crystal/Ceramic Resonator (PH0 and PH1).
     - `LSE`: Crystal/Ceramic Resonator (PC14 and PC15).
2. **Configure the Clock Tree**:
   - Go to the `Clock Configuration` tab.
   - Set the HSE frequency (e.g., 8 MHz).
   - Configure the PLL to achieve the desired system clock (e.g., 80 MHz):
     - Set HSE as the PLL source.
     - Adjust the PLL settings (e.g., PLLM = 1, PLLN = 20, PLLR = 2).
   - Ensure the LSE (32.768 kHz) is selected for the RTC.

### Step 3: Configure the Pins and Peripherals
#### 3.1. USART Configuration
1. **PA1 (RS4852 DE)**:
   - Set to `GPIO_Output`.
2. **PA2 (RS4852 TX) and PA3 (RS4852 RX)**:
   - Configure for `USART2`.
3. **PA9 (USART1_TX) and PA10 (USART1_RX)**:
   - Configure for `USART1`.

#### 3.2. I2C Configuration
1. **PC0 (I2C3_SCL) and PC1 (I2C3_SDA)**:
   - Configure for `I2C3`.
2. **PB6 (I2C1_SCL) and PB7 (I2C1_SDA)**:
   - Configure for `I2C1`.

#### 3.3. GPIO Configuration
1. **PA12 (ALERT2)**:
   - Set to `GPIO_Input`.
2. **PB5 (ALERT)**:
   - Set to `GPIO_Input`.
3. **PC3 (LED)**:
   - Set to `GPIO_Output`.
4. **PC7 (D1/BOOT2)**:
   - Set to `GPIO_Input`.
5. **PB4 (NJRST/D2/BOOT)**:
   - Set to `GPIO_Input`.
6. **PB3 (JTDO_TRACESWO/SWO)**:
   - Used for SWO debugging.

#### 3.4. PWM for Heaters
1. **PB8 (HEATER2) and PB9 (HEATER1)**:
   - Configure for PWM using `TIM4`.

#### 3.5. Debug Configuration (SWD)
1. **PA13 (SWDIO) and PA14 (SWCLK)**:
   - Enable SWD.
2. **PB3 (SWO)**:
   - Enable SWO.

#### 3.6. Oscillator Pins
1. **PH0 (OSC_IN) and PH1 (OSC_OUT)**:
   - Configured for HSE.
2. **PC14 (OSC32_IN) and PC15 (OSC32_OUT)**:
   - Configured for LSE.

#### 3.7. RTC Configuration
1. **Enable RTC**:
   - Set the clock source to LSE.

### Step 4: Additional Configurations
1. **Boot Pins**:
   - Ensure hardware has appropriate pull-down resistors.
2. **Power Supply**:
   - Ensure hardware provides correct voltage (e.g., 3.3V).
3. **Capacitors**:
   - Ensure decoupling capacitors are present in hardware design.

### Step 5: Generate Code
1. **Save the Configuration**:
   - Click the `Save` button (or press `Ctrl+S`) in the `.ioc` file.
   - STM32CubeIDE will prompt you to generate code. Click `Yes`.
2. **Review Generated Code**:
   - The generated code will include initialization functions for all configured peripherals in `main.c` and other files (e.g., `stm32l4xx_hal_msp.c`).


### **Steps to Enable the Temperature Sensor Channel in STM32CubeMX**

Here’s how to properly enable the temperature sensor channel in STM32CubeMX:

1.  **Open Your .ioc File in STM32CubeMX**:
    
    *   Launch STM32CubeMX and open your .ioc file.
        
2.  **Navigate to ADC1 Configuration**:
    
    *   In the **Pinout & Configuration** tab, go to the **Analog** category in the left-hand panel.
        
    *   Select **ADC1**.
        
3.  **Enable the Temperature Sensor Channel**:
    
    *   In the ADC1 configuration pane, you’ll see a section labeled **Parameter Settings**.
        
    *   Look for a checkbox labeled **"Temperature Sensor Channel"** (it might also be called **"Enable Temperature Sensor"** depending on the STM32CubeMX version).
        
    *   **Check this box** to enable the internal temperature sensor.
        
4.  **Select IN16 (Temperature Sensor Channel)**:
    
    *   Under the **ADC1** configuration, you’ll see a list of channels (e.g., IN0, IN1, ..., IN16).
        
    *   Find **IN16** (which corresponds to the temperature sensor).
        
    *   Check the box next to **IN16** to enable it for ADC conversions.
        
    *   When you check IN16, STM32CubeMX might automatically enable the "Temperature Sensor Channel" if it wasn’t already checked. If not, ensure the checkbox is selected as mentioned above.
        
5.  **Set the Mode to Single-Ended**:
    
    *   For the temperature sensor channel (IN16), ensure the mode is set to **Single-ended**. The STM32L4 temperature sensor is a single-ended input, meaning it measures the voltage relative to ground (not differentially). This matches your code:cCollapseWrapCopysConfig.SingleDiff = ADC\_SINGLE\_ENDED;
        
    *   In STM32CubeMX, this is typically the default setting for the temperature sensor channel, but you can confirm it under the channel configuration settings.
        
6.  **Configure Other ADC Parameters**:
    
    *   As mentioned in the previous response, configure the ADC parameters to match your code:
        
        *   **Clock Prescaler**: PCLK/4 (matches ADC\_CLOCKPRESCALER\_PCLK\_DIV4).
            
        *   **Resolution**: 12-bit (matches ADC\_RESOLUTION\_12B).
            
        *   **Data Alignment**: Right (matches ADC\_DATAALIGN\_RIGHT).
            
        *   **Scan Conversion Mode**: Disabled (matches DISABLE).
            
        *   **End of Conversion Selection**: End of Single Conversion (matches ADC\_EOC\_SINGLE\_CONV).
            
        *   **Continuous Conversion Mode**: Disabled (matches DISABLE).
            
        *   **Number of Conversions**: 1 (matches your code).
            
        *   **External Trigger Conversion Source**: Software Start (matches ADC\_SOFTWARE\_START).
            
        *   **External Trigger Conversion Edge**: None (matches ADC\_EXTERNALTRIG\_EDGE\_NONE).
            
        *   **Overrun Behavior**: Data Overwritten (matches ADC\_OVR\_DATA\_OVERWRITTEN).
            
7.  **Generate Code**:
    
    *   Once you’ve configured ADC1 and enabled the temperature sensor channel, go to **Project Manager** > **Code Generator**.
        
    *   Ensure **Generate peripheral initialization as a pair of '.c/.h' files per peripheral** is checked.
        
    *   Click **Generate Code** to regenerate the project files.
    
## Notes
- Ensure all configurations match your schematic and requirements.
- Resolve any conflicts or issues as suggested by STM32CubeIDE.


### **Step 1: Understand the Flashing Process for STM32L476RCT6**

The STM32L476RCT6 can be programmed using the following methods:

1.  **SWD (Serial Wire Debug)**: This is the most common method for STM32 MCUs. It requires two pins: SWDIO (Serial Wire Debug Input/Output) and SWCLK (Serial Wire Clock), along with power (VDD) and ground (GND).
    
2.  **JTAG**: A more complex interface that uses more pins (e.g., TDI, TDO, TMS, TCK, TRST). It’s less commonly used for STM32 programming compared to SWD.
    
3.  **Bootloader (via UART, USB, etc.)**: The STM32L476RCT6 has a built-in bootloader that can be accessed via UART, USB, or other interfaces, but this requires specific hardware setup and bootloader activation (e.g., by setting BOOT0 and BOOT1 pins).
    

Given that you’re using STM32CubeIDE, the most straightforward method is to use **SWD** with a debugger/programmer like the ST-Link (commonly integrated into STM32 Nucleo or Discovery boards, or available as a standalone ST-Link/V2 or ST-Link/V3).

### **Step 2: Identify the Programming Pins in the Schematic**

Let’s examine the schematic for the STM32L476RCT6 (labeled as U1) to identify the pins used for programming and debugging. The relevant section is labeled "MCU," and the schematic includes a "Communication\_SchDoc" block that shows connections for programming.

#### **Relevant Pins for SWD**:

*   **SWDIO (PA13)**: Serial Wire Debug Input/Output.
    
*   **SWCLK (PA14)**: Serial Wire Clock.
    
*   **NRST (Pin 7)**: Reset pin, used to reset the MCU during programming/debugging.
    
*   **VDD and GND**: Power and ground connections for the MCU and debugger.
    

#### **Schematic Analysis**:

*   **PA13 (SWDIO)**: Pin 46 on the STM32L476RCT6 is labeled PA13(JTMS-SWDIO). This pin is connected to the "Communication\_SchDoc" block with the label SWDIO. This confirms that PA13 is exposed for SWD programming.
    
*   **PA14 (SWCLK)**: Pin 49 on the STM32L476RCT6 is labeled PA14(JTCK-SWCLK). This pin is connected to the "Communication\_SchDoc" block with the label SWCLK. This confirms that PA14 is exposed for SWD programming.
    
*   **NRST (Pin 7)**: The reset pin is labeled NRST and is connected to a pull-up resistor (R4, 10kΩ) to 3V3, with a capacitor (C6, 1µF) to ground. It’s also connected to the "Communication\_SchDoc" block with the label NRST. This pin is exposed for programming/debugging.
    
*   **VDD and GND**:
    
    *   VDD pins (e.g., Pin 48: VDDUSB, Pin 19: VDD, Pin 32: VDD, Pin 64: VDD) are connected to the 3V3 supply.
        
    *   VSS pins (e.g., Pin 47, Pin 18, Pin 31, Pin 63) are connected to ground.
        
    *   These power and ground connections are necessary for the debugger to power the MCU during programming.
        

#### **Additional Pins**:

*   **BOOT0 (Pin 60)**: This pin is labeled BOOT0 and is connected to ground via a resistor (R3, 10kΩ). The BOOT0 pin determines the boot mode of the MCU:
    
    *   BOOT0 = 0 (grounded): Boot from main flash memory (normal operation).
        
    *   BOOT0 = 1: Boot from system memory (bootloader mode, e.g., for UART flashing).
        
    *   In the schematic, BOOT0 is tied to ground, which is correct for booting from flash memory after programming. However, if you want to use the bootloader method (e.g., UART flashing), you’d need to pull **expose** this pin (e.g., via a jumper or switch) to set BOOT0 to 1.
        

#### **Conclusion**:

*   The SWD pins (SWDIO, SWCLK, NRST) are **exposed** in the "Communication\_SchDoc" block, which suggests that these signals are routed to a connector or header on the board for programming/debugging. The schematic doesn’t show the connector itself, but the labels SWDIO, SWCLK, and NRST indicate that these signals are available for external connection.
    
*   The VDD and VSS pins are connected to the 3V3 power supply and ground, respectively, which are also necessary for the debugger.
    

### **Step 3: Flashing the Code Using SWD**

Since the SWD pins are exposed, you can flash the code using an ST-Link debugger. Here’s how to do it in STM32CubeIDE:

#### **1\. Connect the ST-Link to the MCU**

*   **Hardware Setup**:
    
    *   Use an ST-Link debugger (e.g., ST-Link/V2, ST-Link/V3, or the ST-Link on an STM32 Nucleo/Discovery board).
        
    *   Connect the ST-Link to your BMS board via the SWD pins:
        
        *   **SWDIO**: Connect to the SWDIO pin (PA13) on the BMS board.
            
        *   **SWCLK**: Connect to the SWCLK pin (PA14) on the BMS board.
            
        *   **NRST**: Connect to the NRST pin on the BMS board.
            
        *   **VDD**: Connect to the 3V3 supply on the BMS board (e.g., from one of the VDD pins).
            
        *   **GND**: Connect to the ground on the BMS board (e.g., from one of the VSS pins).
            
    *   Typically, these connections are made via a **programming header** on the board (e.g., a 10-pin or 20-pin JTAG/SWD connector). If your board doesn’t have a dedicated header, you may need to solder wires to the exposed pins or pads labeled SWDIO, SWCLK, NRST, 3V3, and GND in the "Communication\_SchDoc" block.
        
*   **Power the Board**:
    
    *   Ensure the BMS board is powered (e.g., via the battery pack or an external 3V3 supply). The ST-Link can sometimes power the board via the VDD pin, but it’s safer to power the board independently to avoid voltage mismatches.
        

#### **2\. Configure STM32CubeIDE for Flashing**

*   **Open Your Project**:
    
    *   Open your BMS project in STM32CubeIDE.
        
*   **Set Up the Debug Configuration**:
    
    *   Go to **Run** > **Debug Configurations**.
        
    *   Create a new **STM32 Cortex-M MCU Debugging** configuration (or edit an existing one).
        
    *   In the **Debugger** tab:
        
        *   **Debug Probe**: Select ST-Link (OpenOCD).
            
        *   **Interface**: Select SWD.
            
        *   **Scan**: Click the "Scan" button to detect the ST-Link and the MCU. If the ST-Link is connected properly and the MCU is powered, STM32CubeIDE should detect the STM32L476RCT6.
            
    *   In the **Startup** tab:
        
        *   Ensure the correct binary (e.g., BMS\_STM32L476RCT6.elf) is selected for loading.
            
        *   Check the box for **Enable Debug Mode** if you want to debug the code after flashing.
            
    *   Apply and close the configuration.
        
*   **Flash the Code**:
    
    *   Click the **Debug** button (or press F11) in STM32CubeIDE.
        
    *   STM32CubeIDE will:
        
        *   Connect to the ST-Link.
            
        *   Reset the MCU via the NRST pin.
            
        *   Flash the code to the MCU’s flash memory using the SWD interface (PA13 and PA14).
            
        *   If debugging is enabled, it will halt the MCU at the start of main() and allow you to step through the code.
            
    *   If you only want to flash without debugging, you can use **Run** > **Run** instead, which will flash the code and start execution immediately.
        

#### **3\. Verify the Flashing**

*   After flashing, the MCU should start executing the code from flash memory (since BOOT0 is tied to ground).
    
*   Use the debugging features in STM32CubeIDE to verify that the program is running:
    
    *   Set breakpoints in your code (e.g., in main() or Log\_Error()).
        
    *   Check the values of variables like group\_voltages\_1, pack\_current\_1, temperature\_1, etc., to ensure the BMS is functioning as expected.
        
*   If you have a serial terminal (e.g., connected to UART1 via USART1\_TX and USART1\_RX), you can monitor the logs output by Log\_Read\_All() to confirm that the system is logging data correctly.
    

### **Step 4: Alternative Flashing Method (Bootloader via UART)**

If you cannot use SWD (e.g., the SWD pins are not easily accessible), you can use the STM32’s built-in bootloader via UART. The schematic shows USART1\_TX (PA9) and USART1\_RX (PA10) connected to the "Communication\_SchDoc" block, which suggests these pins are exposed for UART communication.

#### **1\. Enter Bootloader Mode**

*   **Set BOOT0 to 1**:
    
    *   The BOOT0 pin (Pin 60) is currently tied to ground via R3 (10kΩ). To enter bootloader mode, you need to set BOOT0 to 1 (3V3).
        
    *   If BOOT0 is not exposed (e.g., via a jumper or switch), you’ll need to temporarily disconnect R3 from ground and connect BOOT0 to 3V3. This might require soldering or using a jumper if your board has a BOOT0 header.
        
*   **Reset the MCU**:
    
    *   Toggle the NRST pin (e.g., by shorting it to ground briefly) or power cycle the board to reset the MCU.
        
    *   With BOOT0 set to 1, the MCU will enter bootloader mode and start the built-in bootloader.
        

#### **2\. Connect to UART**

*   Connect a USB-to-UART adapter (e.g., FTDI module, CP2102, or CH340) to the UART pins:
    
    *   **TX (Adapter)** → USART1\_RX (PA10)
        
    *   **RX (Adapter)** → USART1\_TX (PA9)
        
    *   **GND (Adapter)** → Ground on the BMS board
        
    *   **3V3 (Adapter)** → 3V3 on the BMS board (optional, if the board is not powered)
        
*   Ensure the UART adapter is set to 3.3V logic levels (not 5V) to match the STM32L476RCT6’s voltage.
    
    #### **SWD Interface**:

*   The SWD interface (SWD1) is correctly set up in the schematic and does not require software configuration in the application code. The STM32L476RCT6’s SWD pins (PA13, PA14, NRST) are connected to SWDIO, SWCLK, and RST, respectively, which is standard for debugging.
    

#### **LED Circuits**:

*   **DS4**:
    
    *   Powered directly by 3V3, so no software control is needed. The LED will light up as long as the 3.3V supply is active.
        
*   **DS3**:
    
    *   Controlled by PC3, which is configured as a GPIO output in MX\_GPIO\_Init.
        
    *   The LED blinks rapidly for 5 seconds after startup to indicate the application has started (post-flashing), then turns off.
        
    *   In Error\_Handler, the LED blinks rapidly to indicate a failure, aiding debugging.
        

#### **UART Interface**:

*   The UART interface (USART1\_TX, USART1\_RX) is correctly configured in MX\_USART1\_UART\_Init and used for logging, which aligns with the schematic.
    
*   **Debugging Purpose of the LED**:
    
    *   The LED (DS3) is used to indicate when the MCU has been flashed or updated:
        
        *   Blinks rapidly (200 ms on/off) for 5 seconds after startup to signal that the application has started, which indirectly indicates a potential flashing operation.
            
        *   Turns off after the initial 5-second period to indicate normal operation.
            
        *   Blinks rapidly in Error\_Handler to indicate a failure, aiding debugging.
            
*   **Code Updates**:
    
    *   Reinstated the LED configuration in MX\_GPIO\_Init for PC3.
        
    *   Added startup blinking logic in main to indicate post-flashing startup.
        
    *   Added rapid blinking in Error\_Handler for error indication.
        
    *   Removed the LED toggle in the main loop, as the LED is specifically for flashing/debugging purposes.
        
*   **Alignment with Schematic**:
    
    *   The LED control logic now aligns with the schematic, where PC3 controls DS3 through R61 (1 kΩ) to ground.
        
    *   The SWD interface is correctly set up for flashing and debugging.
        

This implementation provides a clear visual indication of the MCU’s state after flashing, which is useful for debugging during development.


1.  **Integrated BQ76920\_CheckStatus**:
    
    *   Added a call to BQ76920\_CheckStatus in the main loop (Step 5) to check the SYS\_STAT register of both BQ76920 ICs and update error\_flags.
        
    *   This replaces the manual checks for overvoltage, undervoltage, and overcurrent previously done in Steps 5 and 6.
        
2.  **Removed Redundant Checks**:
    
    *   Removed BQ76920\_CheckProtection and BQ76920\_CheckOvercurrent calls from the main loop, as BQ76920\_CheckStatus now handles these via the SYS\_STAT register.
        
3.  **Updated Update\_BMS\_Mode**:
    
    *   Modified Update\_BMS\_Mode to check all possible error flags (ERROR\_OVERVOLTAGE, ERROR\_UNDERVOLTAGE, ERROR\_OCC, ERROR\_OCD, ERROR\_SCD, ERROR\_OVERTEMP, ERROR\_UNDERTEMP, ERROR\_DISCREPANCY, ERROR\_DEVICE\_XREADY, ERROR\_OVRD\_ALERT).
        
    *   If any fault is detected, the BMS enters MODE\_FAULT, disables charging and discharging, and logs the specific fault.
        
4.  **Error Flag Handling**:
    
    *   Ensured that all status flags from the BQ76920 are mapped to error\_flags and handled appropriately.
        
    *   The BQ76920\_CheckStatus function logs each fault and clears the flags in SYS\_STAT after handling.
        
5.  **Main Loop Structure**:
    
    *   The main loop now relies on BQ76920\_CheckStatus for BQ76920 fault detection, followed by temperature checks and other BMS operations.
        
    *   The loop maintains its original structure but is streamlined by removing redundant fault checks.


This implementation fully handles all status flags from the BQ76920, aligns with the datasheet, and ensures the BMS responds appropriately to faults.


#### ** Flash Using STM32CubeProgrammer**

*   **Install STM32CubeProgrammer**:
    
    *   Download and install STM32CubeProgrammer from STMicroelectronics’ website.
        
*   **Connect to the MCU**:
    
    *   Open STM32CubeProgrammer.
        
    *   Select **UART** as the interface.
        
    *   Choose the correct COM port for your USB-to-UART adapter.
        
    *   Set the baud rate to 115200 (or try different rates if it doesn’t connect).
        
    *   Click **Connect**.
        
*   **Flash the Code**:
    
    *   In the **Download** tab, select your binary file (BMS\_STM32L476RCT6.hex or .bin, which you can generate from STM32CubeIDE).
        
    *   Click **Start Programming** to flash the code to the MCU’s flash memory.
        
*   **Exit Bootloader Mode**:
    
    *   Set BOOT0 back to 0 (ground) by reconnecting R3 or removing the jumper.
        
    *   Reset the MCU (via NRST or power cycle) to start executing the flashed code.


---


1.  The error\_flags variable currently tracks the following faults (as defined in main.h):
    
2.  ERROR\_OVERVOLTAGE
    
3.  : Overvoltage fault (from BQ76920
    
4.  SYS\_STAT
    
5.  ).
    
6.  ERROR\_UNDERVOLTAGE: Undervoltage fault (from BQ76920 SYS\_STAT).
    
7.  ERROR\_OCC: Overcurrent charge fault (from BQ76920 SYS\_STAT).
    
8.  ERROR\_OCD: Overcurrent discharge fault (from BQ76920 SYS\_STAT).
    
9.  ERROR\_SCD: Short-circuit discharge fault (from BQ76920 SYS\_STAT).
    
10.  ERROR\_OVERTEMP: Overtemperature fault (from external temperature sensors).
    
11.  ERROR\_UNDERTEMP: Undertemperature fault (from external temperature sensors).
    
12.  ERROR\_DISCREPANCY: Redundancy discrepancy between the two BQ76920 ICs.
    
13.  ERROR\_DEVICE\_XREADY: BQ76920 device not ready (from BQ76920 SYS\_STAT).
    
14.  ERROR\_OVRD\_ALERT: BQ76920 alert condition (from BQ76920 SYS\_STAT).
    
15.  Protective Actions for Each Fault:
    
16.  **Overvoltage (ERROR\_OVERVOLTAGE)**:
    

18.  Disable charging to prevent further voltage increase.
    
19.  Allow discharging to reduce voltage.
    
20.  Log the fault and enter MODE\_FAULT if the condition persists.
    
21.  **Undervoltage (ERROR\_UNDERVOLTAGE)**:
    
    *   Disable discharging to prevent deep discharge.
        
    *   Allow charging to recover the battery.
        
    *   Log the fault and enter MODE\_FAULT if the condition persists.
        
22.  **Overcurrent Charge (ERROR\_OCC)**:
    
    *   Disable charging to prevent damage.
        
    *   Allow discharging.
        
    *   Log the fault and enter MODE\_FAULT.
        
23.  **Overcurrent Discharge (ERROR\_OCD)**:
    
    *   Disable discharging to prevent damage.
        
    *   Allow charging.
        
    *   Log the fault and enter MODE\_FAULT.
        
24.  **Short-Circuit Discharge (ERROR\_SCD)**:
    
    *   Disable both charging and discharging immediately.
        
    *   Log the fault and enter MODE\_FAULT.
        
    *   Consider a system reset or permanent shutdown if the fault persists.
        
25.  **Overtemperature (ERROR\_OVERTEMP)**:
    
    *   Disable both charging and discharging to prevent thermal runaway.
        
    *   Log the fault and enter MODE\_FAULT.
        
    *   Activate cooling mechanisms (e.g., increase heater PID control to cool down).
        
26.  **Undertemperature (ERROR\_UNDERTEMP)**:
    
    *   Disable charging to prevent lithium plating.
        
    *   Allow discharging if safe.
        
    *   Log the fault and enter MODE\_FAULT.
        
    *   Activate heating mechanisms (already handled by PID\_Control).
        
27.  **Redundancy Discrepancy (ERROR\_DISCREPANCY)**:
    
    *   Disable both charging and discharging due to unreliable measurements.
        
    *   Log the fault and enter MODE\_FAULT.
        
    *   Consider a system reset to reinitialize the BQ76920 ICs.
        
28.  **Device Not Ready (ERROR\_DEVICE\_XREADY)**:
    
    *   Disable both charging and discharging due to unreliable device state.
        
    *   Log the fault and enter MODE\_FAULT.
        
    *   Attempt to reinitialize the BQ76920 ICs.
        
29.  **Override Alert (ERROR\_OVRD\_ALERT)**:
    
    *   Disable both charging and discharging as a precaution.
        
    *   Log the fault and enter MODE\_FAULT.
        
    *   Investigate the cause (e.g., check ALERT pin status or other conditions).
        

#### General Protective Actions:

*   **Enter MODE\_FAULT**: When a fault is detected, the BMS should enter MODE\_FAULT to indicate an unsafe state.
    
*   **Disable Charging/Discharging**: Use BQ76920\_SetChargeEnable to control the CHG\_ON and DSG\_ON bits in the SYS\_CTRL2 register of the BQ76920.
    
*   **Log Faults**: Already handled by BQ76920\_CheckStatus and other checks.
    
*   **Recovery Attempt**: For some faults (e.g., ERROR\_DEVICE\_XREADY), attempt recovery by reinitializing the BQ76920 ICs.
    
*   **System Reset**: For critical faults (e.g., ERROR\_SCD, ERROR\_DISCREPANCY), consider triggering a system reset after a delay if the fault persists.
    

1.  **Added Protective Actions for Each Fault**:
    
    *   **Overvoltage**: Disables charging, allows discharging.
        
    *   **Undervoltage**: Disables discharging, allows charging.
        
    *   **Overcurrent Charge (OCC)**: Disables charging, allows discharging.
        
    *   **Overcurrent Discharge (OCD)**: Disables discharging, allows charging.
        
    *   **Short-Circuit Discharge (SCD)**: Disables both charging and discharging, triggers a system reset after 10 seconds if the fault persists.
        
    *   **Overtemperature**: Disables both charging and discharging, relies on PID\_Control for cooling.
        
    *   **Undertemperature**: Disables charging, allows discharging, relies on PID\_Control for heating.
        
    *   **Redundancy Discrepancy**: Disables both charging and discharging, triggers a system reset after 10 seconds if the fault persists.
        
    *   **Device Not Ready (DEVICE\_XREADY)**: Disables both charging and discharging, attempts to reinitialize the BQ76920 ICs after 5 seconds.
        
    *   **Override Alert (OVRD\_ALERT)**: Disables both charging and discharging as a precaution.
        
2.  **Fault Persistence Tracking**:
    
    *   Added fault\_start\_time and in\_fault\_mode static variables to track how long a fault has been active.
        
    *   Used this to implement delayed actions like system reset for critical faults (ERROR\_SCD, ERROR\_DISCREPANCY) and recovery attempts for ERROR\_DEVICE\_XREADY.
        
3.  **Enhanced Logging**:
    
    *   Added detailed logging for each protective action to indicate what action was taken and why.
        
4.  **Integration with BQ76920**:
    
    *   Used BQ76920\_SetChargeEnable to control the CHG\_ON and DSG\_ON bits in the SYS\_CTRL2 register of the BQ76920 ICs based on the charge\_enabled and discharge\_enabled flags.
        
5.  **Recovery Mechanism**:
    
    *   For ERROR\_DEVICE\_XREADY, the BMS attempts to reinitialize the BQ76920 ICs after 5 seconds and clears the fault flag to allow recovery.
        
6.  **System Reset for Critical Faults**:
    
    *   For ERROR\_SCD and ERROR\_DISCREPANCY, the BMS triggers a system reset after 10 seconds if the fault persists, using HAL\_NVIC\_SystemReset().
        

### Additional Notes

*   **PID Control for Temperature**: The PID\_Control function in the main loop already handles heating/cooling based on temperature. This complements the protective actions for ERROR\_OVERTEMP and ERROR\_UNDERTEMP.
    
*   **System Reset**: The system reset for critical faults (ERROR\_SCD, ERROR\_DISCREPANCY) is a last resort. You may want to adjust the timeout (currently 10 seconds) or add additional recovery mechanisms depending on your application requirements.
    
*   **Recovery for ERROR\_DEVICE\_XREADY**: The reinitialization attempt for ERROR\_DEVICE\_XREADY may need additional error handling (e.g., limiting the number of retries) to prevent infinite loops in case the BQ76920 ICs cannot be reinitialized.



### Define Fault Recovery Strategies

The error\_flags variable tracks the following faults (as defined in main.h):

*   ERROR\_OVERVOLTAGE: Overvoltage fault (from BQ76920 SYS\_STAT).
    
*   ERROR\_UNDERVOLTAGE: Undervoltage fault (from BQ76920 SYS\_STAT).
    
*   ERROR\_OCC: Overcurrent charge fault (from BQ76920 SYS\_STAT).
    
*   ERROR\_OCD: Overcurrent discharge fault (from BQ76920 SYS\_STAT).
    
*   ERROR\_SCD: Short-circuit discharge fault (from BQ76920 SYS\_STAT).
    
*   ERROR\_OVERTEMP: Overtemperature fault (from external temperature sensors).
    
*   ERROR\_UNDERTEMP: Undertemperature fault (from external temperature sensors).
    
*   ERROR\_DISCREPANCY: Redundancy discrepancy between the two BQ76920 ICs.
    
*   ERROR\_DEVICE\_XREADY: BQ76920 device not ready (from BQ76920 SYS\_STAT).
    
*   ERROR\_OVRD\_ALERT: BQ76920 alert condition (from BQ76920 SYS\_STAT).
    

#### **Recovery Strategies for Each Fault**:

1.  **Overvoltage (ERROR\_OVERVOLTAGE)**:
    
    *   **Recovery**: Allow discharging to reduce voltage. Monitor cell voltages (group\_voltages\_1 and group\_voltages\_2) and clear the fault if all voltages drop below the threshold (OV\_THRESHOLD).
        
    *   **Reset**: If the fault persists for too long (e.g., 30 seconds), trigger a system reset.
        
2.  **Undervoltage (ERROR\_UNDERVOLTAGE)**:
    
    *   **Recovery**: Allow charging to increase voltage. Monitor cell voltages and clear the fault if all voltages rise above the threshold (UV\_THRESHOLD).
        
    *   **Reset**: If the fault persists for too long (e.g., 30 seconds), trigger a system reset.
        
3.  **Overcurrent Charge (ERROR\_OCC)**:
    
    *   **Recovery**: Disable charging and wait for a cooldown period (e.g., 10 seconds). Check the current (pack\_current\_1 and pack\_current\_2) and clear the fault if the current drops below a safe threshold.
        
    *   **Reset**: If the fault persists after the cooldown period, trigger a system reset.
        
4.  **Overcurrent Discharge (ERROR\_OCD)**:
    
    *   **Recovery**: Disable discharging and wait for a cooldown period (e.g., 10 seconds). Check the current and clear the fault if the current drops below a safe threshold.
        
    *   **Reset**: If the fault persists after the cooldown period, trigger a system reset.
        
5.  **Short-Circuit Discharge (ERROR\_SCD)**:
    
    *   **Recovery**: This is a critical fault. Disable both charging and discharging. Wait for a longer period (e.g., 30 seconds) and check if the fault clears in SYS\_STAT. If it clears, attempt to resume normal operation.
        
    *   **Reset**: If the fault persists, trigger a system reset.
        
6.  **Overtemperature (ERROR\_OVERTEMP)**:
    
    *   **Recovery**: Disable both charging and discharging. Monitor temperatures (temperature\_1, temperature\_2, pcb\_temperature) and clear the fault if all temperatures drop below a safe threshold (e.g., 50°C).
        
    *   **Reset**: If the fault persists for too long (e.g., 60 seconds), trigger a system reset.
        
7.  **Undertemperature (ERROR\_UNDERTEMP)**:
    
    *   **Recovery**: Disable charging. Monitor temperatures and clear the fault if all temperatures rise above a safe threshold (e.g., -10°C).
        
    *   **Reset**: If the fault persists for too long (e.g., 60 seconds), trigger a system reset.
        
8.  **Redundancy Discrepancy (ERROR\_DISCREPANCY)**:
    
    *   **Recovery**: Disable both charging and discharging. Attempt to reinitialize the BQ76920 ICs and recheck redundancy. Clear the fault if the discrepancy is resolved.
        
    *   **Reset**: If the fault persists after reinitialization, trigger a system reset.
        
9.  **Device Not Ready (ERROR\_DEVICE\_XREADY)**:
    
    *   **Recovery**: Attempt to reinitialize the BQ76920 ICs after a delay (e.g., 5 seconds). Clear the fault if reinitialization succeeds.
        
    *   **Reset**: If reinitialization fails after multiple attempts (e.g., 3 attempts), trigger a system reset.
        
10.  **Override Alert (ERROR\_OVRD\_ALERT)**:
    
    *   **Recovery**: Disable both charging and discharging. Wait for a period (e.g., 10 seconds) and check if the fault clears in SYS\_STAT. If it clears, attempt to resume normal operation.
        
    *   **Reset**: If the fault persists, trigger a system reset.
        

#### **General Recovery and Reset Mechanism**:

*   **Fault Persistence Tracking**: Use a timer to track how long each fault has been active. If a fault persists beyond a threshold, escalate to a system reset.
    
*   **Recovery Attempts**: For faults like ERROR\_DEVICE\_XREADY and ERROR\_DISCREPANCY, attempt recovery by reinitializing hardware components.
    
*   **System Reset**: Use HAL\_NVIC\_SystemReset() to reset the system for unrecoverable faults.
    
*   **Fault Clearing**: Clear error\_flags bits when recovery conditions are met, allowing the BMS to return to normal operation.
    

1.  **Fault Recovery Logic**:
    
    *   **Overvoltage**: Monitors cell voltages and clears the fault if they drop below OV\_THRESHOLD.
        
    *   **Undervoltage**: Monitors cell voltages and clears the fault if they rise above UV\_THRESHOLD.
        
    *   **Overcurrent Charge (OCC)**: Waits for a cooldown period and clears the fault if no charging current is detected.
        
    *   **Overcurrent Discharge (OCD)**: Waits for a cooldown period and clears the fault if no discharging current is detected.
        
    *   **Short-Circuit Discharge (SCD)**: Waits and checks if the fault clears in SYS\_STAT.
        
    *   **Overtemperature**: Clears the fault if temperatures drop below 50°C.
        
    *   **Undertemperature**: Clears the fault if temperatures rise above -10°C.
        
    *   **Redundancy Discrepancy**: Attempts to reinitialize BQ76920 ICs and rechecks redundancy.
        
    *   **Device Not Ready (DEVICE\_XREADY)**: Attempts to reinitialize BQ76920 ICs with a limited number of retries.
        
    *   **Override Alert (OVRD\_ALERT)**: Waits and checks if the fault clears in SYS\_STAT.
        
2.  **Reset Mechanism**:
    
    *   Added timeouts for each fault type (FAULT\_TIMEOUT, TEMP\_FAULT\_TIMEOUT, COOLDOWN\_PERIOD).
        
    *   Triggers a system reset using HAL\_NVIC\_SystemReset() if faults persist beyond their respective timeouts.
        
    *   For ERROR\_DEVICE\_XREADY, limits the number of recovery attempts to MAX\_RECOVERY\_ATTEMPTS before resetting.
        
3.  **Fault Persistence Tracking**:
    
    *   Used fault\_start\_time and in\_fault\_mode to track how long a fault has been active.
        
    *   Added recovery\_attempts to limit the number of recovery attempts for ERROR\_DEVICE\_XREADY.
        
4.  **Fixed Typo in Log\_Error**:
    
    *   Corrected ×tamp to timestamp in the memcpy call within Log\_Error.
        

### Additional Notes

*   **Timeouts**: The timeout values (FAULT\_TIMEOUT, TEMP\_FAULT\_TIMEOUT, COOLDOWN\_PERIOD, RECOVERY\_DELAY) are set to reasonable defaults but can be adjusted based on your application requirements.
    
*   **Recovery Attempts**: The MAX\_RECOVERY\_ATTEMPTS for ERROR\_DEVICE\_XREADY is set to 3. You can adjust this value if needed.
    
*   **System Reset**: The system reset is a last resort. In a production system, you might want to add additional safeguards (e.g., saving state to flash before resetting) or alternative recovery mechanisms.
    
*   **Temperature Thresholds**: The recovery thresholds for ERROR\_OVERTEMP (50°C) and ERROR\_UNDERTEMP (-10°C) are conservative. Adjust these based on your battery specifications.


### Analysis of Firmware Update Support

The requirement is to support firmware updates via UART or RS485. The current code uses RS485 (via husart2) for communication, and the firmware update functionality is implemented using this interface. Let's break down the implementation to confirm if it meets the requirement:

#### 1\. **Bootloader Logic**

*   **Bootloader Check**:
    
    *   The Bootloader\_Check function in main.c checks a firmware update flag stored in flash memory (FIRMWARE\_UPDATE\_FLAG\_ADDR) to determine if the system should enter firmware update mode.
        
    *   If the flag is set to 0xDEADBEEF, the system enters firmware update mode; otherwise, it boots to the application.
        
    *   The flag is cleared after entering firmware update mode to prevent repeated updates.
        
*   **Jump to Application**:
    
    *   The JumpToApplication function jumps to the application code at APP\_START\_ADDR by setting the stack pointer and calling the reset handler.
        

#### 2\. **Firmware Update Process**

*   **Firmware Update Function**:
    
    *   The Bootloader\_FirmwareUpdate function handles the firmware update process over RS485.
        
    *   It erases the application flash region (APP\_START\_ADDR to APP\_END\_ADDR) and waits for firmware packets via RS485.
        
    *   The first packet contains the expected firmware size, and subsequent packets contain the firmware data.
        
    *   Each packet is written to flash, and a simple XOR checksum is calculated for validation.
        
    *   Once all data is received, the checksum is verified, and the system reboots if successful.
        
*   **Communication Protocol**:
    
    *   The firmware update uses the SSP (Simple Serial Protocol) framework over RS485.
        
    *   A custom command SSP\_CMD\_FIRMWARE\_UPDATE (defined as 0x40) is used to initiate and manage the firmware update process.
        
    *   The SSP\_ProcessReceivedFrame function in main.c handles the SSP\_CMD\_SFP command to set the firmware update flag and trigger a reboot into firmware update mode.
        

#### 3\. **Flash Memory Layout**

*   The flash memory layout is defined in main.h:
    
    *   Bootloader: 0x08000000 to 0x08003FFF (16 KB).
        
    *   Application: 0x08004000 to 0x0807F7FF (238 KB).
        
    *   Firmware update flag: 0x0807F810 (within the same page as NEXT\_SLOT\_ADDR).
        

#### 4\. **Security and Reliability**

*   A simple XOR checksum is used to validate the firmware data.
    
*   A timeout mechanism (FIRMWARE\_UPDATE\_TIMEOUT) ensures the system reboots if the update process stalls.
    
*   The firmware update flag is cleared after entering update mode to prevent infinite loops.
    

#### 5\. **Integration with Main Application**

*   The main application checks for the firmware update flag at startup and enters the update mode if necessary.
    
*   The SSP\_ProcessReceivedFrame function allows the OBC to trigger a firmware update by setting the update flag via the SSP\_CMD\_SFP command.