#include "kalman_filter.h"

void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise)
{
    kf->state = initial_state;
    kf->variance = initial_variance;
    kf->process_noise = process_noise;
    kf->measurement_noise = measurement_noise;
}

float KalmanFilter_Update(KalmanFilter *kf, float measurement)
{
    // Predict
    kf->variance += kf->process_noise;

    // Update
    float kalman_gain = kf->variance / (kf->variance + kf->measurement_noise);
    kf->state += kalman_gain * (measurement - kf->state);
    kf->variance *= (1.0f - kalman_gain);
    return kf->state;
}
