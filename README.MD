
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
 [ ] **Battery Charging Control**: Implement a CC-CV charging algorithm and hardware control for enabling/disabling the charger.
 [ ] **MPPT Integration**: Integrate with an MPPT controller to optimize power input from solar panels.
 [ ] **Error Flag Handling**: Fully implement handling of all status flags from the BQ76920.
 [ ] **Protective Actions**: Add logic to take protective actions based on detected faults.
 [ ] **Battery Pack Authentication**: Implement security features to authenticate battery packs.
 [ ] **User Interface**: Develop a communication protocol and user interface for monitoring and control.
 [ ] **Fault Recovery**: Implement mechanisms for fault recovery and reset.
 [ ] **Battery Pack Configuration**: Support configurable battery parameters.
 [ ] **Safety Compliance**: Ensure the firmware meets relevant safety standards.
 [ ] **Testing and Validation**: Add test modes and unit tests for critical functions.
 [ ] **Data Logging Enhancements**: Improve logging detail and retrieval mechanisms.
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
    

#### **3\. Flash Using STM32CubeProgrammer**

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
