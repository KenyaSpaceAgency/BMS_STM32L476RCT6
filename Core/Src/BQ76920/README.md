BQ76920 Battery Management System (BMS) Driver Documentation
============================================================

Overview
This C-based driver controls the **BQ76920**, a chip that watches over the battery in the **AFDEVSAT satellite’s Electrical Power System (EPS)**. It’s part of the **EPS\_BMS** (Battery Management System), one of three main parts of the satellite’s power setup, alongside the **EPS\_Solar** (solar panels) and **EPS\_PDM** (power distribution). The BQ76920 driver uses **I2C** (a way for chips to talk over wires) to do these jobs:

*   **Wakes up the chip** to get it ready.
    
*   **Checks battery voltages** to see how full each part is.
    
*   **Measures current** to track energy flow.
    
*   **Balances the battery** so all parts stay even.
    
*   **Spots problems** like too much or too little power.
    
*   **Controls charging** with a special CC-CV (Constant Current-Constant Voltage) method.
    

The driver runs on an **STM32L476RCT6 microcontroller** (a tiny computer brain) using the **STM32 HAL library** (helper tools) and works with two BQ76920 chips for extra safety.

### EPS Context

The EPS has three parts:

1.  **EPS\_BMS**: Manages the battery (this driver’s home).
    
2.  **EPS\_Solar**: Uses solar panels and an LT3652HV chip to make power from sunlight.
    
3.  **EPS\_PDM**: Shares power with the satellite’s other systems.
    

The **EPS\_BMS** is the boss of charging the battery, using this driver to run the CC-CV charging plan and turn the charger on or off.

Dependencies
------------

To work, this driver needs:

*   **STM32 HAL Library**: Tools to help the microcontroller talk to the BQ76920 over I2C.
    
*   **BQ76920 Chips**: Two of these monitor the battery (IC2 and IC4 in the schematic).
    
*   **I2C Wires**: Connect the microcontroller to the chips (like a phone line).
    
*   **STM32L476RCT6**: The brain running the driver.
    

Register Definitions
--------------------

The BQ76920 keeps info in little “notebooks” called **registers**. Here’s what we use:

## Register Definitions

The BQ76920 keeps information in little “notebooks” called registers. Here’s what we use:

| Register Name   | Address | Description                                      |
|-----------------|---------|--------------------------------------------------|
| SYS_STAT_REG    | 0x00    | Shows if something’s wrong (like a warning light).|
| VC1_HI_REG      | 0x0C    | Holds the power levels of battery parts (cells). |
| CC_HI_REG       | 0x32    | Tracks how much energy flows in or out.          |
| CELLBAL1_REG    | 0x01    | Controls which battery parts to balance.         |
| SYS_CTRL2_REG   | 0x05    | Turns charging or power-giving on/off.           |

## Pin Description (EPS_BMS Schematic)

Here’s how the BQ76920 (IC2) is wired in the EPS_BMS schematic:

| Pin Number | Pin Name | Description                                                                 |
|------------|----------|-----------------------------------------------------------------------------|
| 1          | DSG      | Controls the discharge switch (Q12) to let the battery power the satellite. |
| 2          | CHG      | Controls the charge switch (Q6) to let power into the battery.              |
| 3          | VSS      | Ground pin, the “zero” point for electricity (tied to PACK- via R54).       |
| 4          | SDA      | Data wire to talk to the microcontroller (I2C1_SDA).                        |
| 5          | SCL      | Clock wire to keep talking in sync (I2C1_SCL).                              |
| 6          | TS1      | Checks battery temperature with a sensor (connected to NTC via R53).        |
| 7          | CAP1     | Might steady power with a capacitor (not key for charging here).            |
| 8          | REGOUT   | Makes 3.3V power for small parts (like a mini power supply).                |
| 9          | REGSRC   | Gets power from the battery (PACK+ via R32) to run the chip.                |
| 10         | BAT      | Measures the total battery voltage (connected to PACK+).                    |
| 11         | BOOT     | Wakes the chip up (connected to PB4 via a circuit).                         |
| 12         | VC5      | Not used (only 3 cells here).                                               |
| 13         | VC4      | Not used (only 3 cells).                                                    |
| 14         | VC3      | Measures the third cell’s voltage (3S).                                     |
| 15         | VC2      | Measures the second cell’s voltage (2S).                                    |
| 16         | VC1      | Measures the first cell’s voltage (1S).                                     |
| 17         | VC0      | Bottom of the battery (ground, tied to PACK-).                              |
| 18         | SRP      | One side of a tiny resistor (R54) to measure current.                       |
| 19         | SRN      | Other side of R54 for current measurement.                                  |
| 20         | ALERT    | Warns if something’s wrong (lights DS1 and signals PB5).                    |

Pin Description (EPS\_BMS Schematic)
------------------------------------

Here’s how the BQ76920 (IC2) is wired in the **EPS\_BMS** schematic:

**Pin NumberPin NameDescription**1DSGControls the discharge switch (Q12) to let the battery power the satellite.2CHGControls the charge switch (Q6) to let power into the battery.3VSSGround pin, the “zero” point for electricity (tied to PACK- via R54).4SDAData wire to talk to the microcontroller (I2C1\_SDA).5SCLClock wire to keep talking in sync (I2C1\_SCL).6TS1Checks battery temperature with a sensor (connected to NTC via R53).7CAP1Might steady power with a capacitor (not key for charging here).8REGOUTMakes 3.3V power for small parts (like a mini power supply).9REGSRCGets power from the battery (PACK+ via R32) to run the chip.10BATMeasures the total battery voltage (connected to PACK+).11BOOTWakes the chip up (connected to PB4 via a circuit).12VC5Not used (only 3 cells here).13VC4Not used (only 3 cells).14VC3Measures the third cell’s voltage (3S).15VC2Measures the second cell’s voltage (2S).16VC1Measures the first cell’s voltage (1S).17VC0Bottom of the battery (ground, tied to PACK-).18SRPOne side of a tiny resistor (R54) to measure current.19SRNOther side of R54 for current measurement.20ALERTWarns if something’s wrong (lights DS1 and signals PB5).

### Key Connections

*   **PACK+ & PACK-**: The battery’s positive and negative ends.
    
*   **LOAD+ & LOAD-**: Connect to a power bus (shared with EPS\_Solar and EPS\_PDM).
    
*   **Q6 (CHG MOSFET)**: Turns charging on/off.
    
*   **Q12 (DSG MOSFET)**: Turns power-giving on/off.
    
*   **R54 (Shunt)**: Measures energy flow for the CC-CV plan.


CC-CV Charging in EPS\_BMS
--------------------------

### Why EPS\_BMS Handles CC-CV

*   **Simple**: The BMS is like the battery’s babysitter—it decides how fast to fill it with power (CC) and when to slow down (CV) to keep it safe. The solar part (EPS\_Solar) just gives power to a big “power highway” (bus), and the BMS uses it smartly.
    
*   **Tech**:
    
    *   The **LT3652HV** in EPS\_Solar does MPPT (gets the most from solar panels) and powers a bus (e.g., 12V).
        
    *   The **EPS\_BMS** connects to this bus via LOAD+/LOAD-, using the BQ76920 and Q6 to control charging.
        
    *   The STM32 firmware’s ChargeBattery function runs the CC-CV plan, watching voltage/current from the BQ76920 and switching Q6 on/off.
        

### How It Works

1.  **Power Flow**:
    
    *   **Charging**: Solar power → LT3652HV → Power Bus → LOAD+/LOAD- → Battery (PACK+/PACK- via Q6).
        
    *   **Discharging**: Battery → LOAD+/LOAD- → Power Bus → EPS\_PDM → Satellite systems (via Q12).
        
2.  **CC-CV Steps**:
    
    *   **Constant Current (CC)**: Fills the battery fast until it’s nearly full (e.g., 4.2V per cell).
        
    *   **Constant Voltage (CV)**: Slows down to keep it at 4.2V until fully charged.
        
3.  **Control**: The BQ76920 turns Q6 on/off via CHG pin, guided by the firmware.
    

NTR3C21NZT3G MOSFETs (Q1-Q4, Q7-Q10)
------------------------------------

*   **Simple**: These are tiny switches (8 of them) that help even out the battery parts—they’re not for the main charging but help keep things balanced.
    
*   **Tech**:
    
    *   Controlled by BQ76920 CBx pins to drain extra power from high cells.
        
    *   Not in the main charge/discharge path (that’s Q6/Q12, bigger CSD19536KTT MOSFETs).
        
    *   Possible extras (e.g., Q9/Q10) might switch heaters or sensors.
--------

Imagine you have a bunch of batteries powering something important, like a toy robot or an electric scooter. These batteries need someone to watch over them to make sure they don’t get too full (overcharged), too empty (drained), or too hot. That’s where the **BQ76920** chip comes in—it’s like a tiny babysitter for batteries! This project is a set of instructions (called a "driver") written in the C language that tells the **STM32 microcontroller** (a small computer brain) how to talk to the BQ76920 chip using a special wire called **I2C**.

Here’s what this driver can do:

*   **Wake up the BQ76920** and make sure it’s ready to work.
    
*   **Measure how full each battery cell is** (in volts).
    
*   **Check how much power is flowing** (in amps).
    
*   **Balance the batteries** so they all have about the same amount of energy.
    
*   **Watch for trouble** like batteries getting too full or too empty.
    

This driver is designed to work with the STM32 microcontroller, which is popular in things like robots, drones, and electric cars.

Dependencies
------------

To use this driver, you need:

*   **STM32 HAL Library**: This is like a helper kit that makes it easy for the STM32 to talk to other parts, like the BQ76920 chip over I2C.
    
*   **BQ76920 Chip**: The actual babysitter chip connected to your batteries.
    
*   **I2C Wires**: A way for the STM32 and BQ76920 to chat (like a phone line).
    

Register Definitions
--------------------

The BQ76920 chip has little "notebooks" inside it called **registers**. Each notebook has a number (address) and holds specific information. Here are the ones we use:

**Register NameAddressWhat It Does**SYS\_STAT\_REG0x00Tells us if something’s wrong (like a warning light).VC1\_HI\_REG0x0CHolds the voltage readings for the battery cells.CC\_HI\_REG0x32Shows how much current (power flow) is happening.CELLBAL1\_REG0x01Controls which batteries need balancing.SYS\_CTRL2\_REG0x05Lets us turn charging or discharging on/off.

Functions and Technical Details
-------------------------------

Here’s a breakdown of each function in simple terms, followed by the updated code with comments your mum could understand!

### 1\. BQ76920\_Init

*   **What It Does**: Says "hello" to the BQ76920 chip to make sure it’s awake and ready to help.
    
*   **How**: Sends a message over I2C to check the status notebook (SYS\_STAT\_REG).
    
*   **Returns**: "All good" (HAL\_OK) if it works, or an error if the chip doesn’t answer.
    

### 2\. BQ76920\_ReadVoltages

*   **What It Does**: Asks the BQ76920 how full each battery cell is and writes it down.
    
*   **How**: Reads 6 bytes starting from VC1\_HI\_REG, turns them into millivolts (mV), and saves them in a list.
    
*   **Returns**: "All good" if it worked, or an error if the chip didn’t reply.
    

### 3\. BQ76920\_ReadCurrent

*   **What It Does**: Checks how much power is flowing in or out of the batteries.
    
*   **How**: Reads 2 bytes from CC\_HI\_REG and turns them into milliamps (mA).
    
*   **Returns**: "All good" if successful, or an error if not.
    

### 4\. BQ76920\_BalanceCells

*   **What It Does**: Makes sure all batteries have about the same energy by draining the fuller ones a bit.
    
*   **How**: Finds the emptiest battery, then tells the chip to balance any battery 50 mV fuller by writing to CELLBAL1\_REG.
    
*   **Returns**: "All good" if it worked, or an error if not.
    

### 5\. BQ76920\_CheckProtection

*   **What It Does**: Looks at the battery voltages to see if any are too high or too low.
    
*   **How**: Compares each voltage to safety limits (OV\_THRESHOLD and UV\_THRESHOLD) and raises a flag if there’s trouble.
    
*   **Returns**: Nothing directly, but sets flags to warn us.
    

### Additional Functions (Added for Completeness)

*   **BQ76920\_CheckStatus**: Reads all the warning lights from the chip and logs them.
    
*   **BQ76920\_SetChargeEnable**: Turns charging or discharging on/off like a switch.

Key Features & Benefits
-----------------------

*   ✔️ **Easy Talking**: Uses I2C to chat with the chip quickly.
    
*   ✔️ **Accurate Checks**: Measures volts and amps precisely.
    
*   ✔️ **Balancing Act**: Keeps all batteries evened out.
    
*   ✔️ **Safety First**: Watches for trouble like overfull or empty batteries.
    
*   ✔️ **Small & Simple**: Works well in tiny gadgets.



