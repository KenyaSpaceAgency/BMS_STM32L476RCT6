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


```
+---------+
|  Start  | 
+---------+
	|
	|<-----------+
    |    "Repeat"|
    V            |
+-------------+  |
| Prediction  |  |
+-------------+  |
    |            |
    V            |
+-------------+  |
|   Update    |  |
+-------------+  |
    |            |
    V            |
+-------------+  |
|  New Guess  |  |
+-------------+  |
    |            |
    |            |
    | "Repeat"   |
    V            |
+-------------+  |
| Prediction  |--+
+-------------+
```
#### 1\. Prediction Step

### Key Symbols
| Symbol | Meaning                          |
|--------|----------------------------------|
| `x̂`    | State estimate                   |
| `P`    | Error covariance matrix          |
| `F`    | State transition model           |
| `Q`    | Process noise covariance         |
| `H`    | Observation model                |
| `R`    | Measurement noise covariance     |
| `K`    | Kalman Gain                      |
| `z`    | Measurement vector               |



\[Old Guess: 50%\] --> \[Add Wiggle: +0.5\] --> \[New Guess: 50%, More Unsure\] (Variance: 1) (Process Noise) (Variance: 1.5)

*   **Simple**: “Based on what I know, I think the battery’s charge will stay about the same, but I’ll add a little wiggle because things might shift a bit.”
    
*   **Tech**: Increases variance by process\_noise (Q) to account for uncertainty in the battery’s behavior (e.g., coulomb counting errors).

#### 2\. Update Step



\[Guess: 50%, Var: 1.5\] + \[Shaky Reading: 49%\] --> \[Kalman Gain: 0.6\] --> \[New Guess: 49.4%, Var: 0.6\] (Smoother, More Sure)




*   **Simple**: “Here’s a new shaky reading (say 49%). I’ll mix it with my guess to get a better one, trusting the reading more if my guess is wobbly.”
    
*   **Tech**:
    
    *   Calculates kalman\_gain (0 to 1) based on variance vs. measurement\_noise (R).
        
    *   Adjusts state toward the measurement, weighted by kalman\_gain.
        
    *   Shrinks variance since we’re more certain now.    
   

\[Old Guess: 50%\] --> \[Add Wiggle: +0.5\] --> \[New Guess: 50%, More Unsure\] (Variance: 1) (Process Noise) (Variance: 1.5)

### Purpose in the Code

*   **Simple**: It helps our satellite’s battery system figure out how full and healthy the battery is, even if our sensors aren’t perfect. This keeps the battery safe and working well in space!
    
*   **Technical Purpose**:
    
    *   **SOC Estimation**: In Update\_SOC\_SOH (in main.c), it refines the SOC (%) using noisy coulomb counting data from the BQ76920 chips, ensuring accurate battery charge tracking.
        
    *   **SOH Estimation**: It also smooths out SOH (%) estimates, which show how much life the battery has left, based on its capacity over time.
        
    *   **Reliability**: By filtering noise, it prevents bad decisions (e.g., overcharging) that could harm the battery or satellite.
        

Role in the BMS Project
-----------------------

### Where It’s Used

*   **In main.c**:
    
    *   Two Kalman Filters (soc\_kf and soh\_kf) are initialized in main() with starting guesses (e.g., 50% SOC, 100% SOH).
        
    *   Update\_SOC\_SOH calls KalmanFilter\_Update with coulomb counting data to refine SOC and SOH every loop (100 ms).
        

### Example in Action

*   **Simple**: Suppose our sensor says the battery’s at 48%, but it’s shaky. The filter might guess 49% last time. It mixes them and says, “I think it’s 48.6%,” smoothing out the wobble.
    
*   **Tech**:
    
    *   Input: measurement = 48%, state = 49%, variance = 1, process\_noise = 0.01, measurement\_noise = 1.
        
    *   Prediction: variance = 1.01.
        
    *   Update: kalman\_gain = 1.01 / (1.01 + 1) ≈ 0.5, state = 49 + 0.5 \* (48 - 49) = 48.5, variance = 1.01 \* (1 - 0.5) ≈ 0.505.
        

Why It’s Important
------------------

*   **Simple**: Space is tricky—sensors get noisy, and we can’t fix things up there. This filter keeps our battery guesses spot-on so the satellite stays powered and safe.
    
*   **Technical**:
    
    *   **Noise Reduction**: Cleans up BQ76920 data for reliable SOC/SOH.
        
    *   **Battery Management**: Prevents overcharging (SOC too high) or deep discharge (SOC too low), critical for lithium-ion batteries.
        
    *   **Mission Success**: Accurate SOH helps predict battery life, ensuring the satellite lasts its whole mission.




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