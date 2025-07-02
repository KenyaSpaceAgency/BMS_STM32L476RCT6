#include "kalman_filter.h"  // Include the Kalman filter header file

//---------------------------------------------------------------------------------------
// Function: kalman_filter_init
// Purpose : Initialize the Kalman filter state and parameters
// Inputs  :
//   - KalmanFilter *kf: Pointer to the KalmanFilter structure (holds filter state)
//   - float initial_state_estimate: The first estimate of the state (e.g., initial SOC)
//   - float initial_estimate_error_cov: Initial guess of the error in the estimate
//   - float measurement_noise_cov: Estimated noise in the measurements (sensor uncertainty)
// Output  : None (modifies the KalmanFilter struct directly)
// Significance:
//   This function prepares the Kalman filter to begin fusing new measurements with predictions.
//   Used in this project to smooth SOC and SOH estimations from noisy sensor data.
//---------------------------------------------------------------------------------------
void kalman_filter_init(KalmanFilter *kf, float initial_state_estimate, float initial_estimate_error_cov, float measurement_noise_cov) {
    kf->x = initial_state_estimate;            // Set the initial estimated value (e.g., 50% SOC)
    kf->P = initial_estimate_error_cov;        // Set the initial uncertainty in estimate (how confident we are in x)
    kf->Q = 0.05f;                              // Process noise: how much we expect the system to vary between updates
    kf->R = measurement_noise_cov;             // Measurement noise: how noisy the sensor data is
}

//---------------------------------------------------------------------------------------
// Function: kalman_filter_update
// Purpose : Perform one step of the Kalman filter update (estimation refinement)
// Inputs  :
//   - KalmanFilter *kf: Pointer to the KalmanFilter structure (contains filter state)
//   - float z: The new measurement (e.g., sensor SOC reading)
//   - float measurement_noise_cov: Updated estimate of measurement noise
// Output  :
//   - float: The updated, fused estimate (filtered value of SOC or SOH)
// Significance:
//   This function uses the new measurement to update the estimate and reduce error.
//   It blends the previous estimate and the new data using their uncertainties.
//   In this project, it improves reliability of battery state reporting.
//---------------------------------------------------------------------------------------
float kalman_filter_update(KalmanFilter *kf, float z, float measurement_noise_cov) {
    kf->R = measurement_noise_cov;            // Update measurement noise in case it has changed
    float K = kf->P / (kf->P + kf->R);        // Calculate Kalman Gain (how much to trust measurement vs estimate)
    kf->x = kf->x + K * (z - kf->x);          // Update state estimate using weighted average
    kf->P = (1 - K) * kf->P;                  // Update estimate uncertainty (P shrinks as estimate improves)
    return kf->x;                             // Return the refined (filtered) estimate
}
