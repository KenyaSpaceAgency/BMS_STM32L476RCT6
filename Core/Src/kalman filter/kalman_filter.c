/*
 * kalman_filter.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 *
 * @brief  Implements a Kalman Filter to make smart guesses about the battery’s
 *         State of Charge (SOC) and State of Health (SOH) in our satellite’s
 *         Battery Management System (BMS). It smooths out noisy sensor readings
 *         to keep the battery safe and reliable in space!
 * @note   Uses a simple 1D Kalman Filter, assuming a linear system with constant
 *         process and measurement noise. Perfect for estimating scalar values
 *         like SOC (%) or SOH (%).
 */

#include "kalman_filter.h"

/**
  * @brief  Sets up the Kalman Filter with starting info
  * @param  kf: Pointer to the Kalman Filter structure (our "toolbox")
  * @param  initial_state: Initial guess for the battery’s value (e.g., 50% SOC)
  * @param  initial_variance: How uncertain we are about the initial guess
  *                          (larger = less confident)
  * @param  process_noise: Expected variability in the battery’s behavior (Q)
  *                        (how much it might change unexpectedly)
  * @param  measurement_noise: Expected noise in sensor readings (R)
  *                            (how shaky our measurements are)
  * @retval None (just initializes the filter)
  * @note   Typical values:
  *         - initial_state: 50% for SOC, 100% for SOH
  *         - initial_variance: 1.0 (moderate confidence)
  *         - process_noise: 0.01 (small changes in battery state)
  *         - measurement_noise: 1.0 (sensor noise level)
  * @reason Initializes the filter with a starting point and uncertainty estimates,
  *         allowing it to refine guesses as new measurements arrive. Critical for
  *         accurate SOC/SOH tracking despite noisy sensors.
  */
void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise)
{
    kf->state = initial_state;              // Set the initial guess (e.g., 50% SOC)
    kf->variance = initial_variance;        // Set how much we trust the guess
                                            // (small = confident, large = unsure)
    kf->process_noise = process_noise;      // Set expected battery variability (Q)
    kf->measurement_noise = measurement_noise; // Set expected sensor noise (R)
}

/**
  * @brief  Updates the Kalman Filter with a new measurement to make a better guess
  * @param  kf: Pointer to the Kalman Filter structure with current state
  * @param  measurement: New sensor reading (e.g., raw SOC from coulomb counting)
  * @retval float: Updated, smoother estimate of the battery state (e.g., refined SOC %)
  * @note   The Kalman Filter works in two steps:
  *         1. Predict: Guess the next state and increase uncertainty.
  *         2. Update: Blend the new measurement with the prediction based on trust.
  * @reason Combines noisy sensor data with our best guess to produce a more accurate
  *         estimate, reducing errors in SOC/SOH calculations for safe battery management.
  */
float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    // Step 1: Predict the Next State
    // Assume the state doesn’t change much, but add uncertainty due to possible
    // battery variations (process noise)
    kf->variance += kf->process_noise;

    // Step 2: Update with New Measurement
    // Calculate Kalman Gain: How much to trust the new measurement vs. our prediction
    // - If variance is high (we’re unsure), trust the measurement more
    // - If measurement_noise is high (sensors are shaky), trust the prediction more
    float kalman_gain = kf->variance / (kf->variance + kf->measurement_noise);

    // Update the state: Blend the prediction with the measurement
    // - Adjust state based on the difference between measurement and prediction
    // - Kalman Gain scales the adjustment (0 = ignore measurement, 1 = fully trust it)
    kf->state += kalman_gain * (measurement - kf->state);

    // Update variance: We’re more confident now, so reduce uncertainty
    // - Variance shrinks as we incorporate the measurement
    kf->variance *= (1.0f - kalman_gain);

    // Return the refined state estimate
    return kf->state;
}
