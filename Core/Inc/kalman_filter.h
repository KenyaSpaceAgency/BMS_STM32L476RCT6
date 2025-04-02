#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float state;      // Estimated state (e.g., SOC or SOH)
    float variance;   // Estimated variance
    float process_noise; // Process noise (Q)
    float measurement_noise; // Measurement noise (R)
} KalmanFilter;

/* Exported functions prototypes ---------------------------------------------*/
void KalmanFilter_Init(KalmanFilter *kf, float initial_state, float initial_variance, float process_noise, float measurement_noise);
float KalmanFilter_Update(KalmanFilter *kf, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_FILTER_H */
