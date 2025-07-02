// Header guard to prevent the file from being included multiple times
#ifndef INC_KALMANFILTER_H_
#define INC_KALMANFILTER_H_

// Define a data structure to hold all the variables needed for the Kalman filter
typedef struct {
    float x;  // Estimated state (e.g., battery voltage, temperature, etc.)
    float P;  // Estimate error covariance (uncertainty in the estimate)
    float Q;  // Process noise covariance (uncertainty in how the state changes)
    float R;  // Measurement noise covariance (uncertainty in the measurements)
} KalmanFilter;

// Function to initialize the Kalman filter
// kf: Pointer to the KalmanFilter structure
// initial_state_estimate: Your best guess of the initial value
// initial_estimate_error_cov: How uncertain you are about that initial guess
// measurement_noise_cov: How noisy your sensor is (larger = noisier)
void kalman_filter_init(KalmanFilter *kf, float initial_state_estimate, float initial_estimate_error_cov, float measurement_noise_cov);

// Function to update the Kalman filter with a new measurement
// kf: Pointer to the KalmanFilter structure
// z: New sensor measurement
// measurement_noise_cov: Optional—if sensor noise changes over time, you can update it here
float kalman_filter_update(KalmanFilter *kf, float z, float measurement_noise_cov);

// End of the header guard
#endif /* INC_KALMANFILTER_H_ */
