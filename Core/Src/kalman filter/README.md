Kalman Filter Implementation Documentation (kalman\_filter.c)
=============================================================

Overview
--------

The kalman\_filter.c file gives us a clever tool called a **Kalman Filter**, written in C, to guess how much charge is left in our satellite’s battery (called **State of Charge**, or SOC) and how healthy it is (called **State of Health**, or SOH). It’s like a super-smart guesser that takes noisy, wobbly measurements—like how much power the battery has—and smooths them out to give us a better, more trustworthy number.

### What is a Kalman Filter?

*   **Simple Explanation**: Imagine you’re trying to guess how much water is in a bucket, but your measuring cup is shaky and sometimes spills a bit. The Kalman Filter is like a friend who watches your shaky cup, remembers past measurements, and uses a bit of math magic to say, “I think this is how much water is really there!” It mixes what it already knows with your new (imperfect) measurement to get a great guess.
    
*   **Technical Definition**: The Kalman Filter is a mathematical algorithm that estimates the true state of a system (like SOC or SOH) by combining a prediction (based on a model) with noisy measurements. It does this in two steps: **predicting** what the state might be next, and **updating** that guess with new data, all while balancing how much to trust the prediction versus the measurement.
    

How It Works: Step-by-Step with Diagrams
----------------------------------------

### Conceptual Diagram: Kalman Filter Process

Imagine a timeline with two repeating steps:

\[Start\] --> \[Prediction\] --> \[Update\] --> \[New Guess\] --> \[Repeat\]

+---------+ | Start | (Oval) +---------+ | V+-------------+| Prediction | (Rectangle)+-------------+ | V+-------------+| Update | (Rectangle)+-------------+ | V+-------------+| New Guess | (Rectangle)+-------------+ | |<-----------+ | "Repeat" | V |+-------------+ || Prediction |----++-------------+

### Purpose in the Code

*   **Simple**: It helps our satellite’s battery system figure out how full and healthy the battery is, even if our sensors aren’t perfect. This keeps the battery safe and working well in space!
    
*   **Technical Purpose**:
    
    *   **SOC Estimation**: In Update\_SOC\_SOH (in main.c), it refines the SOC (%) using noisy coulomb counting data from the BQ76920 chips, ensuring accurate battery charge tracking.
        
    *   **SOH Estimation**: It also smooths out SOH (%) estimates, which show how much life the battery has left, based on its capacity over time.
        
    *   **Reliability**: By filtering noise, it prevents bad decisions (e.g., overcharging) that could harm the battery or satellite.
        






### Why It’s Important for This Project

*   **Simple**: In space, we can’t check the battery with our hands, and sensors get wobbly signals. The Kalman Filter makes sure we trust the right numbers to keep the satellite powered up safely.
    
*   **Technical Reasons**:
    
    *   **Noisy Measurements**: The BQ76920 gives us voltage and current readings, but they wiggle a bit (noise). The Kalman Filter cleans this up for SOC/SOH.
        
    *   **Battery Safety**: Accurate SOC prevents overcharging or deep discharging, which could damage the battery or cut power to the satellite.
        
    *   **Efficiency**: Smooth SOH estimates help us plan the satellite’s mission life, avoiding surprises in space.
        
    *   **Real-Time**: It’s fast and light, perfect for the STM32L476RCT6 microcontroller in our EPS\_BMS.
        

Dependencies
------------

*   **kalman\_filter.h**: The instruction sheet telling other code parts about the Kalman Filter’s tools and structure.