### **Overview of the EPS Components**

The EPS (Electrical Power System) for the AFDEVSAT satellite is divided into three main parts, as indicated by the schematics:

1.  **EPS\_BMS.pdf**: This schematic focuses on the **Battery Management System (BMS)**, which manages the battery pack, monitors its health, and controls charging and discharging.
    
2.  **EPS\_PDM.pdf**: This schematic covers the **Power Distribution Module (PDM)**, which handles power distribution to various subsystems in the satellite.
    
3.  **EPS\_Solar.pdf**: This schematic details the **solar power system**, including the solar arrays and associated power management components.
    

Our task is to determine which of these components is responsible for the **CC-CV charging algorithm** and **hardware control for enabling/disabling the charger**.

### **Step 1: Analyze Each Schematic**

#### **1\. EPS\_BMS.pdf (Battery Management System)**

*   **Key Components**:
    
    *   **Microcontroller**: STM32L476RCT6 (U1) – This is the main controller for the BMS, running the firmware that manages the battery.
        
    *   **Battery Monitoring ICs**: BQ7692003PWR (IC2, IC4) – These are the BQ76920 chips used to monitor the battery’s voltage, current, and temperature.
    
    **EPS\_BMS (Battery Management System)**:

*   **Capabilities**:
    
    *   The BMS uses the STM32L476RCT6 microcontroller to run firmware, which includes your ChargeBattery function implementing the CC-CV algorithm.
        
    *   It monitors the battery via the BQ76920 chips (voltage, current, temperature) and can control charging using MOSFETs (e.g., Q6, Q12, Q13, Q14).
        
    *   It has GPIO pins (e.g., PA6 in your implementation) to enable/disable the charger.
    
    #### **Interaction Between EPS\_Solar and EPS\_BMS**

*   **Power Flow**:
    
    1.  The **EPS\_Solar** module (with the LT3652HV) takes power from the solar panels, performs MPPT to maximize power, and can charge the battery using its built-in CC-CV algorithm.
        
    2.  The **EPS\_BMS** module is connected to the battery (via BAT-, BAT+) and monitors its state using the BQ76920 chips.
        
    3.  The **EPS\_PDM** takes power from the EPS (likely from a power bus) and distributes it to subsystems.
    
   ### **Step 1: Reanalyze the Schematic (EPS\_BMS.pdf)**

The schematic page from **EPS\_BMS.pdf** provides the foundation for understanding the BMS’s role in charging and discharging. Let’s break down the key components and connections, incorporating the new insight about LOAD+ and LOAD-.

#### **Key Components and Connections**

1.  **Battery Pack (PACK+ and PACK-)**:
    
    *   The battery pack is connected via the PACK+ and PACK- terminals on the right side of the schematic.
        
    *   It consists of **3 cells in series** (indicated by labels 1S, 2S, 3S on the BQ76920 IC, IC2).
        
    *   Capacitors (C18, C17, C30, 4.7nF) are likely filtering capacitors for the BQ76920’s voltage sensing pins (VC1, VC2, VC3), not balancing capacitors (balancing resistors and MOSFETs are likely on another page).
        
    *   The total pack voltage is monitored at PACK+ (positive terminal) and PACK- (negative terminal).
        
2.  **BQ76920 Battery Monitoring IC (IC2)**:
    
    *   The BQ76920 (IC2) monitors individual cell voltages via pins VC1, VC2, VC3, etc.
        
    *   It communicates with the STM32L476RCT6 microcontroller via I2C (I2C1\_SCL, I2C1\_SDA).
        
    *   It has pins like BOOT, ALERT, and CHG/DSG for controlling charging and discharging MOSFETs.
        
    *   The REGSRC pin is connected to PACK+ through a resistor (R32, 1kΩ) and capacitor (C18, 4.7nF), providing power to the IC.
        
3.  **Charging and Discharging MOSFETs (Q6, Q12)**:
    
    *   **Q6 (CSD19536KTT)**: N-channel MOSFET labeled as the charge MOSFET, connected to the BQ76920’s CHG pin via R34 (5.1kΩ).
        
    *   **Q12 (CSD19536KTT)**: N-channel MOSFET labeled as the discharge MOSFET, connected to the BQ76920’s DSG pin via R35 (5.1kΩ).
        
    *   **Back-to-Back Configuration**:
        
        *   Drains of Q6 and Q12 are connected together.
            
        *   Source of Q6 connects to PACK- (via shunt resistor R54).
            
        *   Source of Q12 connects to LOAD-.
            
        *   This configuration allows the BQ76920 to control the direction of current flow:
            
            *   **Charging Path**: When Q6 is on (CHG high), current flows from LOAD- to PACK-, charging the battery.
                
            *   **Discharging Path**: When Q12 is on (DSG high), current flows from PACK- to LOAD-, supplying the load.
                
        *   When both MOSFETs are off, current is blocked in both directions, providing full control.
    
  
 4. *   **Shunt Resistor (R54)**:
    
    *   R54 (0.005Ω, per the BOM) is connected between PACK- and the source of Q6.
        
    *   The BQ76920 uses its SRP and SRN pins to measure the current flowing through R54, which is critical for the CC-CV algorithm (monitoring charging current to switch from CC to CV phase).
        
5.  **Diode (D19, S3B-13-F)**:
    
    *   D19 in parallel with the MOSFETs.
        
    *   It allows current to flow from the battery to the load if the MOSFETs are off (e.g., during a fault), but prevents reverse current from the load to the battery, enhancing safety. 
   
6. **Load and Charger Connections (LOAD+ and LOAD-)**:

	*   **LOAD+** is directly connected to PACK+, meaning the positive terminal of the battery is always connected to the external system.
	    
	*   **LOAD-** is connected to PACK- through the shunt resistor (R54) and MOSFETs (Q6, Q12), allowing the BMS to control current flow.
	    
	*   **New Insight**:
	    
	    *   LOAD+ and LOAD- connect the BMS to a **power bus**, which is powered by the LT3652HV (MPPT) in the EPS\_Solar module when solar power is available.
	        
	    *   When solar power is unavailable, the battery supplies power to the power bus via LOAD+ and LOAD-, and the PDM draws power from the bus to supply subsystems.
   
   

   
   
   
   
    Solar Panels | v\[EPS\_Solar: LT3652HV\] ----> Power Bus (e.g., 12V) | | | v | \[EPS\_PDM\] ----> Subsystems (3.3V, 5V, 12V) v\[EPS\_BMS: LOAD+, LOAD-\] ----> Battery (PACK+, PACK-) |Controls charging via Q6
    
    Solar Panels (inactive) | v\[EPS\_Solar: LT3652HV\] (inactive) | v\[EPS\_BMS: LOAD+, LOAD-\] <---- Battery (PACK+, PACK-) | | | v | \[EPS\_PDM\] ----> Subsystems (3.3V, 5V, 12V) vPower Bus (powered by battery) |Controls discharging via Q12
    
    #### **LOAD+ and LOAD- as Interface to the BMS from the MPPT**

*   **Correct**: LOAD+ and LOAD- are indeed the interface to the BMS from the MPPT, but indirectly:
    
    *   The MPPT (LT3652HV in EPS\_Solar) supplies power to a power bus.
        
    *   The BMS connects to this bus via LOAD+ and LOAD- to receive power for charging.
        
    *   The BMS controls the charging process using the BQ76920 and Q6 (charge MOSFET), implementing the CC-CV algorithm in firmware (ChargeBattery).
    
    #### **When There’s No Solar Power, LOAD+ and LOAD- Provide Power to the System**

*   **Correct**: When there’s no solar power, the BMS allows the battery to supply power to the system:
    
    *   The BQ76920 turns on Q12 (discharge MOSFET), enabling current to flow from the battery to the power bus via LOAD+ and LOAD-.
        
    *   The PDM draws power from the bus to supply the satellite’s subsystems.
        

#### **Nuances and Clarifications**

*   **Power Bus Role**: The LOAD+ and LOAD- terminals don’t connect directly to the MPPT or the PDM; they connect to a power bus that serves as the central power distribution network. The MPPT (LT3652HV) supplies power to the bus during charging, and the battery supplies power to the bus (via LOAD+ and LOAD-) during discharging.
    
*   **System Power Supply**:
    
    *   When solar power is available, the LT3652HV powers the bus, and the BMS uses this power to charge the battery. The PDM also draws power from the bus to supply subsystems.
        
    *   When solar power is unavailable, the battery powers the bus via LOAD+ and LOAD-, and the PDM continues to draw power from the bus to supply subsystems.
        
*   **BMS Control**: The BMS controls the direction of power flow (charging or discharging) using the BQ76920 and MOSFETs (Q6, Q12), ensuring the battery is managed safely in both scenarios.


1.  **Other Components**:
    
    *   **DS1 (LTST-C170KGKT)**: A green LED connected to the ALERT pin of the BQ76920, likely for status or fault indication.
        
    *   **Capacitors (C24, C18, etc.)**: Filtering capacitors for the BQ76920 and other circuits.
        
    *   **Resistors (R34, R35, etc.)**: Pull-down and current-limiting resistors for the MOSFET gates and other signals.
        

#### **Charging Path and Control**

*   **Charging Path**:
    
    *   Power comes from the power bus (supplied by the LT3652HV in EPS\_Solar) via LOAD+ and LOAD-.
        
    *   Current flows from LOAD+ to PACK+ (directly connected), through the battery, and back through PACK- to LOAD-.
        
    *   The BQ76920 turns on Q6 (charge MOSFET) via the CHG pin, allowing current to flow into the battery.
        
*   **Discharging Path**:
    
    *   Current flows from PACK+ to LOAD+, through the power bus, and back through LOAD- to PACK-.
        
    *   The BQ76920 turns on Q12 (discharge MOSFET) via the DSG pin, supplying power to the bus.
        
*   **Hardware Control**:
    
    *   The BQ76920 directly controls Q6 and Q12 using its CHG and DSG pins.
        
    *   The STM32L476RCT6 firmware communicates with the BQ76920 over I2C and commands it to enable/disable charging by setting registers (e.g., SYS\_CTRL2 for CHG\_ON and DSG\_ON).
        

#### **CC-CV Charging in the Schematic**

*   **Current Monitoring**: The BQ76920 measures current using the shunt resistor (R54) via SRP and SRN pins.
    
*   **Voltage Monitoring**: The BQ76920 measures cell voltages (VC1, VC2, VC3) and total pack voltage.
    
*   **Temperature Monitoring**: The BQ76920 monitors temperature via the NTC thermistor (R53).
    
*   **Control**: The STM32L476RCT6 firmware implements the CC-CV algorithm, using data from the BQ76920 to:
    
    *   Monitor voltage and current.
        
    *   Switch from CC to CV phase.
        
    *   Command the BQ76920 to enable/disable charging.
        

### **Step 2: Role of NTR3C21NZT3G MOSFETs (Q1, Q2, Q3, Q4, Q7, Q8, Q9, Q10)**

#### **NTR3C21NZT3G Overview**

*   **Type**: N-channel MOSFET.
    
*   **Specifications**:
    
    *   **Voltage Rating (Vds)**: 20V.
        
    *   **Current Rating (Id)**: 3.6A (continuous).
        
    *   **Rds(on)**: 21 mΩ (typical) at Vgs = 4.5V, making it a low-resistance switch.
        
    *   **Package**: SOT-23.
        
    *   **Gate Threshold Voltage (Vgs(th))**: 0.5V to 1.2V, suitable for logic-level control.
        
*   **Quantity**: 8 instances (Q1, Q2, Q3, Q4, Q7, Q8, Q9, Q10).
    
*   **Not in Main Path**: These are distinct from the high-current CSD19536KTT MOSFETs (Q6, Q12, Q13, Q14), which handle the main charge/discharge paths.
    

#### **Schematic Context**

*   The provided schematic page focuses on the main charge/discharge path and does not show Q1, Q2, Q3, Q4, Q7, Q8, Q9, or Q10.
    
*   These MOSFETs are likely on another page of the EPS\_BMS.pdf (e.g., a page dedicated to cell balancing or auxiliary switching), as indicated by the BOM.
    

#### **Likely Role of NTR3C21NZT3G MOSFETs**

1.  **Cell Balancing**:
    
    *   **Hypothesis**: The primary role of Q1, Q2, Q3, Q4, Q7, Q8, Q9, and Q10 is **cell balancing**.
        
    *   **Reasoning**:
        
        *   The BQ76920 supports **passive cell balancing**, dissipating excess charge from a cell through a resistor to balance cell voltages.
            
        *   The BQ76920 has balancing pins (CB1, CB2, CB3, etc.) that connect to external MOSFETs and resistors for each cell.
            
        *   With 3 cells in series, the BQ76920 can balance each cell individually. Each cell typically requires 1 or 2 MOSFETs for balancing:
            
            *   **Single MOSFET per Cell**: One MOSFET (e.g., Q1) connects a balancing resistor (e.g., 100Ω) across the cell terminals.
                
            *   **Dual MOSFET per Cell**: Two MOSFETs (e.g., Q1 and Q2) for redundancy or higher current handling.
                
        *   **8 MOSFETs for 3 Cells**:
            
            *   3 cells × 2 MOSFETs per cell = 6 MOSFETs (e.g., Q1 and Q2 for Cell 1, Q3 and Q4 for Cell 2, Q7 and Q8 for Cell 3).
                
            *   The remaining 2 MOSFETs (Q9, Q10) might be used for other purposes (see below).
                
    *   **Configuration**:
        
        *   The BQ76920 turns on a MOSFET (via a CBx pin) to discharge the cell with the highest voltage, balancing it with the others.
            
        *   Example: If Cell 1 is at 4.2V and Cell 2 is at 4.0V, the BQ76920 turns on Q1 (or Q1 and Q2) to discharge Cell 1, allowing Cell 2 to catch up.
            
2.  **Auxiliary Switching**:
    
    *   **Hypothesis**: The remaining NTR3C21NZT3G MOSFETs (e.g., Q9, Q10) might be used for auxiliary switching, such as controlling power to peripherals (e.g., heaters, sensors).
        
    *   **Reasoning**:
        
        *   The schematic includes HEATER1, HEATER2, NTC-1, and NTC-2 connections, indicating the BMS controls heaters.
            
        *   The STM32L476RCT6 uses TIM4 for PWM control of the heaters (as seen in your code), but additional MOSFETs might be needed to switch power to the heaters or other components.
            
        *   The NTR3C21NZT3G’s 3.6A rating is suitable for switching moderate loads like heaters or sensors.
            
3.  **Not for Main Charge/Discharge Path**:
    
    *   The NTR3C21NZT3G MOSFETs are **not** used for the main charge/discharge path:
        
        *   Their 3.6A rating is too low for the main battery current, which is handled by the CSD19536KTT MOSFETs (200A rating).
            
        *   The schematic explicitly shows Q6 and Q12 as the charge and discharge MOSFETs, controlled by the BQ76920’s CHG and DSG pins.
            

#### **Role in CC-CV Charging and Hardware Control**

*   **CC-CV Charging Algorithm**:
    
    *   **Direct Role**: The NTR3C21NZT3G MOSFETs are **not directly involved** in the CC-CV charging algorithm:
        
        *   The CC-CV algorithm is implemented in the STM32L476RCT6 firmware (ChargeBattery), which monitors voltage and current via the BQ76920 and controls the charging process.
            
        *   The main charge path is controlled by Q6 (charge MOSFET), managed by the BQ76920’s CHG pin.
            
    *   **Indirect Role (Cell Balancing)**:
        
        *   If used for cell balancing, these MOSFETs support the CC-CV process indirectly:
            
            *   During charging, the BQ76920 (or firmware) activates balancing to ensure all cells reach the same voltage, preventing overcharging.
                
            *   Example: If Cell 1 reaches 4.2V (CV threshold) before Cell 2, the BQ76920 turns on Q1 (or Q1 and Q2) to discharge Cell 1, allowing Cell 2 to catch up.
                
        *   This ensures the CC-CV algorithm charges the battery pack safely, but the MOSFETs are not part of the main charging path.
            
*   **Hardware Control for Enabling/Disabling the Charger**:
    
    *   **Direct Role**: The NTR3C21NZT3G MOSFETs are **not directly involved** in enabling/disabling the charger:
        
        *   The BQ76920 enables/disables charging by controlling Q6 via the CHG pin, commanded by the firmware using BQ76920\_SetChargeEnable.
            
    *   **Indirect Role (Auxiliary Switching)**:
        
        *   If Q9 and Q10 are used for auxiliary switching (e.g., controlling power to heaters), they indirectly affect charging:
            
            *   Heaters maintain battery temperature during charging (via PID\_Control), and these MOSFETs might switch heater power.
                
            *   However, this is not directly related to enabling/disabling the charger.
                

#### **Summary of NTR3C21NZT3G Role**

*   **Primary Function**: Likely used for **cell balancing** (e.g., Q1–Q8 for 3 cells, 2 MOSFETs per cell for redundancy), with Q9 and Q10 possibly for auxiliary switching (e.g., heaters).
    
*   **Relation to CC-CV**: Indirectly supports CC-CV by enabling cell balancing, ensuring even charging across cells.
    
*   **Relation to Hardware Control**: Not directly involved in enabling/disabling the charger, which is handled by the BQ76920 and Q6.
    

### **Step 3: Revisit the CC-CV Charging Responsibility**

#### **Previous Conclusion**

The **EPS\_BMS** is responsible for the CC-CV charging algorithm and hardware control for enabling/disabling the charger, based on:

*   **EPS\_Solar.pdf**: The LT3652HV performs MPPT and provides regulated power outputs (3.3V, 5V, 12V) to a power bus, not directly charging the battery.
    
*   **EPS\_BMS.pdf**: The BMS controls the battery and charging path via the BQ76920 and MOSFETs (Q6, Q12).
    
*   **Firmware**: The ChargeBattery function implements the CC-CV algorithm in the BMS.
    

#### **New Insights from LOAD+ and LOAD-**

*   **LOAD+ and LOAD- Role**:
    
    *   These terminals connect the BMS to a power bus, which is powered by the LT3652HV (MPPT) when solar power is available.
        
    *   During charging, the BMS draws power from the bus via LOAD+ and LOAD- to charge the battery.
        
    *   When solar power is unavailable, the BMS supplies power to the bus via LOAD+ and LOAD-, and the PDM draws power from the bus for subsystems.
        
*   **Impact on CC-CV**:
    
    *   The BMS remains the central controller for charging, as it directly manages the battery and controls the charging path (via Q6).
        
    *   The LT3652HV provides power to the bus but does not directly charge the battery, leaving the CC-CV algorithm to the BMS.
        

#### **Reassessing the MPPT’s Role**

*   **EPS\_Solar (LT3652HV)**:
    
    *   The LT3652HV handles MPPT and supplies power to the power bus (e.g., 12V).
        
    *   It could perform CC-CV charging if directly connected to the battery, but the schematic shows the battery connected to the BMS, and the BMS controls the charging path.
        
*   **MPPT’s Role**:
    
    *   The LT3652HV optimizes solar power and provides it to the bus.
        
    *   The BMS uses this power (via LOAD+ and LOAD-) to charge the battery, applying the CC-CV algorithm.
        

#### **Updated Conclusion**

The **EPS\_BMS** is responsible for the CC-CV charging algorithm and hardware control for enabling/disabling the charger:

*   **Charging Path Control**: The BMS controls the charging path using the BQ76920 and Q6 (charge MOSFET). The CHG pin enables/disables charging, commanded by the STM32L476RCT6 firmware via I2C.
    
*   **CC-CV Algorithm**: The BMS implements the CC-CV algorithm in firmware, using data from the BQ76920 (voltage, current, temperature).
    
*   **MPPT’s Role**: The LT3652HV provides power to the bus, but the BMS manages the charging process.
    

#### **Why the BMS Handles CC-CV, Not the MPPT**

*   **BMS’s Role**:
    
    *   Directly connected to the battery, with real-time data on its state.
        
    *   Controls the charging path using MOSFETs, enabling dynamic adjustments.
        
    *   Integrates CC-CV with other functions (e.g., balancing, fault handling).
        
*   **MPPT’s Role**:
    
    *   Focuses on MPPT to maximize solar power, supplying the bus.
        
    *   Does not directly charge the battery in this system; the BMS handles CC-CV.
