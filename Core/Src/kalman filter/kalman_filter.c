/*
 * kalman_filter.c
 *
 *  Created on: Mar 29, 2025
 *      Author: yomue
 *
 * This file is like a smart guesser for our satellite’s battery. It helps figure out
 * how much power (SOC) and life (SOH) the battery has, even if our measurements are
 * a bit shaky. It’s super important for keeping the battery safe in space!
 */

#include "kalman_filter.h"

/**
  * @brief  Sets up the Kalman Filter with starting info
  * @param  kf: The filter’s toolbox we’re filling up
  * @param  initial_state: Our first guess at the battery’s charge (e.g., 50%)
  * @param  initial_variance: How unsure we are about that guess (bigger = less sure)
  * @param  process_noise: How wobbly our battery’s behavior might be (Q)
  * @param  measurement_noise: How shaky our sensor readings are (R)
  * @retval None (just sets things up)
  */
void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise)
{
    kf->state = initial_state;              // Our first guess (e.g., 50% charge)
    kf->variance = initial_variance;        // How much we trust that guess (small = more trust)
    kf->process_noise = process_noise;      // How much the battery might surprise us
    kf->measurement_noise = measurement_noise; // How much our sensors wiggle
}

/**
  * @brief  Makes a new, better guess using a fresh measurement
  * @param  kf: The filter’s toolbox with our current guess
  * @param  measurement: A new, wobbly reading from the battery sensor
  * @retval The updated, smoother guess (e.g., new SOC %)
  */
float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    // Step 1: Guess Ahead (Prediction)
    // Add some wiggle room because the battery might change a bit on its own
    kf->variance += kf->process_noise;

    // Step 2: Fix the Guess (Update)
    // Figure out how much to trust the new measurement vs. our old guess
    float kalman_gain = kf->variance / (kf->variance + kf->measurement_noise);
    // Tweak our guess using the new reading, but only a little if it’s shaky
    kf->state += kalman_gain * (measurement - kf->state);
    // We’re more sure now, so shrink the wiggle room
    kf->variance *= (1.0f - kalman_gain);

    // Give back the new, smoother guess
    return kf->state;
}
