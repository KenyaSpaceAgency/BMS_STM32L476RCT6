PID Controller for Heater System
================================

Overview
--------

This project implements a Proportional-Integral-Derivative (PID) controller for a heating system using PWM (Pulse Width Modulation). The PID controller adjusts the power supplied to the heaters based on the difference between the current temperature and a target temperature. The implementation ensures temperature regulation while preventing overshooting or oscillations.

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

*   Implement an adaptive PID tuning algorithm.
    
*   Introduce a temperature filtering mechanism to reduce noise.
    
*   Improve logging with timestamped events.
    

This PID controller ensures efficient and stable heating control while protecting against overheating conditions. 🚀