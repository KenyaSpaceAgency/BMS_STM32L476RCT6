/*
 * kalman_filter.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 */

#include "kalman_filter.h"

/**
  * @brief  Initializes the Kalman Filter
  * @param  kf: Pointer to the Kalman Filter structure
  * @param  initial_state: Initial state estimate (e.g., SOC or SOH)
  * @param  initial_variance: Initial variance estimate
  * @param  process_noise: Process noise (Q)
  * @param  measurement_noise: Measurement noise (R)
  * @retval None
  */
void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise)
{
    kf->state = initial_state;
    kf->variance = initial_variance;
    kf->process_noise = process_noise;
    kf->measurement_noise = measurement_noise;
}

/**
  * @brief  Updates the Kalman Filter with a new measurement
  * @param  kf: Pointer to the Kalman Filter structure
  * @param  measurement: New measurement value
  * @retval None
  */
void KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    // Predict
    kf->variance += kf->process_noise;

    // Update
    float kalman_gain = kf->variance / (kf->variance + kf->measurement_noise);
    kf->state += kalman_gain * (measurement - kf->state);
    kf->variance *= (1.0f - kalman_gain);
}
