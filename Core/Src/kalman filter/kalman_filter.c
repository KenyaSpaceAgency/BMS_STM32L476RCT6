/*
 * kalman_filter.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "kalman_filter.h"

/**
  * @brief  Initializes the Kalman Filter with initial conditions
  * @param  kf: Pointer to the Kalman Filter structure
  * @param  initial_state: Initial estimate of the state (e.g., SOC in %)
  * @param  initial_variance: Initial uncertainty in the state estimate
  * @param  process_noise: Process noise covariance (Q)
  * @param  measurement_noise: Measurement noise covariance (R)
  * @retval None
  */
void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise)
{
    kf->state = initial_state;              // Set the initial state estimate (e.g., 50% for SOC)
    kf->variance = initial_variance;        // Set the initial uncertainty (variance)
    kf->process_noise = process_noise;      // Set the process noise covariance (Q)
    kf->measurement_noise = measurement_noise; // Set the measurement noise covariance (R)
}

/**
  * @brief  Updates the Kalman Filter with a new measurement
  * @param  kf: Pointer to the Kalman Filter structure
  * @param  measurement: New measurement value (e.g., SOC in %)
  * @retval Updated state estimate
  */
float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    // Prediction Step: Update the variance by adding process noise
    // This accounts for uncertainty in the system model (e.g., coulomb counting errors)
    kf->variance += kf->process_noise;

    // Update (Correction) Step:
    // 1. Compute the Kalman Gain: Determines how much to trust the new measurement
    float kalman_gain = kf->variance / (kf->variance + kf->measurement_noise);

    // 2. Update the state estimate: Adjust the state based on the measurement residual
    kf->state += kalman_gain * (measurement - kf->state);

    // 3. Update the variance: Reduce uncertainty based on the Kalman Gain
    kf->variance *= (1.0f - kalman_gain);

    // Return the updated state estimate
    return kf->state;
}
