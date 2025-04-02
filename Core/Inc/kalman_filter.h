/*
 * kalman_filter.h
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

/**
  * @brief Structure to hold Kalman Filter parameters and state
  */
typedef struct {
    float state;              // Estimated state (e.g., SOC, SOH)
    float variance;           // Estimate uncertainty (covariance)
    float process_noise;      // Process noise covariance (Q)
    float measurement_noise;  // Measurement noise covariance (R)
} KalmanFilter;

/**
  * @brief  Initializes the Kalman Filter with initial conditions
  * @param  kf: Pointer to the Kalman Filter structure
  * @param  initial_state: Initial estimate of the state (e.g., SOC in %)
  * @param  initial_variance: Initial uncertainty in the state estimate
  * @param  process_noise: Process noise covariance (Q)
  * @param  measurement_noise: Measurement noise covariance (R)
  * @retval None
  */
void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise);

/**
  * @brief  Updates the Kalman Filter with a new measurement
  * @param  kf: Pointer to the Kalman Filter structure
  * @param  measurement: New measurement value (e.g., SOC in %)
  * @retval Updated state estimate
  */
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#endif /* KALMAN_FILTER_H_ */
