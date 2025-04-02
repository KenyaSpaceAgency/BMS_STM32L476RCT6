
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
- **Operation Modes Implementation**: Develop distinct modes for charging, discharging, sleep, and fault.
- **Battery Charging Control**: Implement a CC-CV charging algorithm and hardware control for enabling/disabling the charger.
- **MPPT Integration**: Integrate with an MPPT controller to optimize power input from solar panels.
- **Error Flag Handling**: Fully implement handling of all status flags from the BQ76920.
- **Protective Actions**: Add logic to take protective actions based on detected faults.
- **Battery Pack Authentication**: Implement security features to authenticate battery packs.
- **User Interface**: Develop a communication protocol and user interface for monitoring and control.
- **Fault Recovery**: Implement mechanisms for fault recovery and reset.
- **Battery Pack Configuration**: Support configurable battery parameters.
- **Safety Compliance**: Ensure the firmware meets relevant safety standards.
- **Testing and Validation**: Add test modes and unit tests for critical functions.
- **Data Logging Enhancements**: Improve logging detail and retrieval mechanisms.
- **Firmware Updates**: Support firmware updates via UART or RS485.



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

---
