PID Controller for Heater System
================================

Overview
--------

This project implements a Proportional-Integral-Derivative (PID) controller for a heating system using PWM (Pulse Width Modulation). The PID controller adjusts the power supplied to the heaters based on the difference between the current temperature and a target temperature. The implementation ensures temperature regulation while preventing overshooting or oscillations.
The pid.c file gives us a **PID Controller**—a smart system written in C that keeps our satellite’s battery at just the right temperature by controlling heaters. It’s like a thermostat at home: if it’s too cold, it turns the heat up; if it’s too hot, it turns it down. In our Battery Management System (BMS), it uses **PWM** (Pulse Width Modulation) to adjust how much power goes to the heaters, keeping the battery safe and cozy in space.
### What is a PID Controller?

*   **Simple Explanation**: Imagine you’re steering a toy boat to a spot on a pond. If it’s off course, you turn the rudder a bit (Proportional), keep adjusting if it’s been off for a while (Integral), and tweak it based on how fast it’s veering (Derivative). The PID Controller does this with numbers to keep the temperature perfect.
    
*   **Technical Definition**: A PID Controller is a feedback mechanism that calculates an output (e.g., heater power) based on the **error** (difference between desired and actual temperature) using three parts:
    
    *   **Proportional (P)**: Adjusts based on the current error.
        
    *   **Integral (I)**: Fixes past errors that add up over time.
        
    *   **Derivative (D)**: Predicts future errors by looking at how fast the error changes.
        

### Purpose in the Code

*   **Simple**: It keeps the battery warm enough to work well in the cold of space, but not too hot to get hurt. It turns heaters on just the right amount.
    
*   **Technical Purpose**:
    
    *   **Temperature Regulation**: In PID\_Control, it adjusts heater power (via PWM on TIM4) to match the current temperature (temp) to TARGET\_TEMP (25°C).
        
    *   **Safety**: Shuts off heaters if the temperature exceeds TEMP\_UPPER\_LIMIT (60°C), protecting the battery.
        
    *   **Logging**: Tracks big changes in heater power for debugging via Log\_Error.
        

### Why It’s Important for This Project

*   **Simple**: Space is freezing (-20°C or lower), and our battery doesn’t like being too cold or too hot. The PID Controller keeps it happy so the satellite can keep working.
    
*   **Technical Reasons**:
    
    *   **Battery Performance**: Lithium-ion batteries (like ours) work best between -20°C and 60°C. Too cold slows them down; too hot can damage them.
        
    *   **Safety**: Prevents overheating (risking thermal runaway) or underheating (reducing capacity), critical in the EPS\_BMS.
        
    *   **Stability**: Smoothly adjusts heat to avoid wild swings, ensuring steady power for the satellite

Files
-----

*   **pid.c**: Contains the implementation of the PID control logic.
    
*   **pid.h**: Header file containing PID-related definitions.
    

Functionality
-------------

### 1\. PID Initialization (PID\_Init)

This function initializes the PID controller by resetting:

*   The integral term to zero.
    
*   The previous error term to zero.
    
*   The last duty cycle to zero.
    

### 2\. PID Control (PID\_Control)

This function is responsible for:

*   Measuring the error between the current temperature (temp) and the target temperature (TARGET\_TEMP).
    
*   output = (KP \* error) + (KI \* integral) + (KD \* derivative)where:
    
    *   KP is the proportional gain.
        
    *   KI is the integral gain.
        
    *   KD is the derivative gain.
        
*   Limiting the output between 0% and 100% to prevent excessive heating.
    
*   Converting the output to a PWM duty cycle (0-1000 range) and applying it to both heaters.
    
*   Logging significant duty cycle changes (>50 PWM units) for debugging purposes.
    

### 3\. Over-temperature Protection

If the measured temperature exceeds TEMP\_UPPER\_LIMIT, the system:

*   Turns off both heaters.
    
*   Resets the integral term to prevent wind-up.
    


```plaintext
[Target Temp: 25°C] --> [Compare] --> [Error] --> [PID Math] --> [Heater Power]
   |                     ^              |              |                |
   |                     |              v              v                v
[Actual Temp] <--------- [Measure] <--------------------------------- [Apply Heat]  

```

#### 1\. Measure Error

*   **Simple**: “Is it 25°C yet? If not, how far off are we?”
    
*   **Tech**: error = TARGET\_TEMP - temp (e.g., 25 - 20 = 5°C too cold).
    

#### 2\. PID Calculation

*   **Proportional (P)**: “Turn up the heat a bit because it’s 5°C off.” (KP \* error)
    
*   **Integral (I)**: “It’s been cold for a while, so add more heat.” (KI \* integral)
    
*   **Derivative (D)**: “It’s warming up fast, so ease off a little.” (KD \* derivative)
    
*   **Tech**: output = KP \* error + KI \* integral + KD \* derivative (e.g., 10 \* 5 + 0.1 \* 5 + 1 \* 0 = 50%).
    

#### 3\. Apply Heat

*   **Simple**: “Set the heaters to 50% power to warm it up.”
    
*   **Tech**: Converts output (0-100%) to PWM duty\_cycle (0-1000) and sets TIM4 channels.
    

#### 4\. Safety Check

*   **Simple**: “Too hot? Turn everything off!”
    
*   **Tech**: If temp >= 60°C, sets duty cycle to 0 and resets integral.
    

Role in the BMS Project
-----------------------

### Where It’s Used

*   **In main.c**:
    
    *   Called in the main loop (every 100 ms) as PID\_Control(lowest\_temp) to adjust heaters based on the coldest battery temperature (temperature\_1 or temperature\_2).
        
    *   Works with htim4 (TIM4) to send PWM signals to HEATER1 (PB9) and HEATER2 (PB8).
        

### Example in Action

*   **Simple**: If the battery’s at 20°C (too cold), the PID says, “Heat it up 50%!” Next time, if it’s 24°C, it might say, “Just 10% now,” until it’s 25°C.
    
*   **Tech**:
    
    *   temp = 20°C, error = 25 - 20 = 5, integral = 0.5 (after one loop), derivative = 0 (first time).
        
    *   output = 10 \* 5 + 0.1 \* 0.5 + 1 \* 0 = 50.5%, duty\_cycle = 505.
        

Why It’s Important
------------------

*   **Simple**: Space is super cold, and our battery needs to stay warm to work. This keeps it at 25°C without cooking it, so the satellite doesn’t freeze or overheat.
    
*   **Technical**:
    
    *   **Temperature Range**: Lithium-ion batteries need -20°C to 60°C. Below -20°C, they lose power; above 60°C, they can get damaged.
        
    *   **Stability**: PID prevents wild temperature swings, keeping the battery steady.
        
    *   **Safety**: Shuts off heaters above 60°C, avoiding thermal runaway (a battery fire risk).

# Heater Control Process

## Flowchart

```plaintext
[Error] --> [PID] --> [Heater Power %]
  +5°C  --> 50%   --> Warm Up
  +1°C  --> 10%   --> Fine Tune
   0°C  -->  0%   --> Hold Steady
  >60°C -->  0%   --> Safety Off
```



Dependencies
------------

*   **Timer (PWM Control):** Uses TIM4 PWM channels TIM\_CHANNEL\_3 and TIM\_CHANNEL\_4 to control heater power.
    
*   **Logging (Log\_Error):** Captures important system events.
    

Constants and Macros
--------------------

ConstantDescriptionTARGET\_TEMPDesired temperature setpoint (°C)TEMP\_UPPER\_LIMITMaximum allowable temperature (°C) before safety shutoffKP, KI, KDPID tuning parametersDTSampling interval (time step in seconds)

Usage
-----

1.  Call PID\_Init() at the beginning of the program.
    
2.  Continuously monitor the temperature and call PID\_Control(temp) with the latest temperature measurement.
    
3.  Adjust KP, KI, and KD values as needed for optimal performance.
    

Notes
-----

*   **Integral Wind-up Prevention:** The integral term resets when heaters are disabled due to high temperature.
    
*   **Derivative Noise Handling:** The derivative component is computed using discrete differentiation.
    
*   **Duty Cycle Logging:** To track performance, significant changes in heater power output are logged.
    

Future Improvements
-------------------

1.  **Smart Tuning**: Adjust KP, KI, KD automatically for perfect control.
    
    *   **Tech**: Add adaptive PID algorithms.
        
2.  **Smoother Temps**: Filter noisy temperature readings.
    
    *   **Tech**: Use a moving average or Kalman Filter on temp.
        
3.  **Better Logs**: Add timestamps to heater logs.
    
    *   **Tech**: Include RTC time in Log\_Error.

This PID controller ensures efficient and stable heating control while protecting against overheating conditions. 🚀