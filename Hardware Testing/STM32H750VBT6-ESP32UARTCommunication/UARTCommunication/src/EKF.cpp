#include "EKF.h"
#include "attitude_EKF.h" // The MATLAB generated header (Now expecting floats)
#include <Arduino.h>      // Needed for Serial.println

// ==========================================================
// 1. THE EKF MEMORY & TUNING ZONE (Hidden from Main loop)
// ==========================================================
static float current_x[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float current_P[49]; 

static float tuning_Q[7]  = {0.002f, 0.002f, 0.002f, 0.002f, 0.003f, 0.003f, 0.003f};
static float tuning_R[6]  = {0.05f, 0.05f, 0.05f, 2.0f, 2.0f, 2.0f};

// Adaptive Mag Tuning Parameters
const float BASE_MAG_R = 20.0f;     
const float MAX_MAG_R  = 10000.0f;  

// --- YOUR NEWLY CALIBRATED MAGNETIC MATRICES ---
static float b_mag[3] = {70.3812f, -38.6215f, -0.2548f};
static float A_mag[9] = {0.9767f, 0.0186f, -0.1272f, 
                         0.0186f, 1.0381f, 0.0393f, 
                        -0.1272f, 0.0393f, 1.0049f}; 

// --- TARE (ZEROING) VARIABLES ---
static bool is_tared = false;
static int tare_samples = 0;
static float offset_roll = 0.0f, offset_pitch = 0.0f, offset_yaw = 0.0f;

// ==========================================================
// 2. INITIALIZATION
// ==========================================================
void init_biped_EKF() {
  for(int i=0; i<49; i++) {
    if(i % 8 == 0) current_P[i] = 1.0f; 
    else current_P[i] = 0.0f;          
  }
  is_tared = false;
  tare_samples = 0;
  offset_roll = 0.0f; offset_pitch = 0.0f; offset_yaw = 0.0f;
}

// ==========================================================
// 3. THE MAIN WRAPPER FUNCTION
// ==========================================================
void rpy_EKF(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt, float motor_penalty,
             float *roll, float *pitch, float *yaw) {
  
  float next_x[7], next_P[49];
  float raw_roll, raw_pitch, raw_yaw;

  // --- ADAPTIVE COVARIANCE INJECTION ---
  if(motor_penalty > 1.0f) motor_penalty = 1.0f;
  if(motor_penalty < 0.0f) motor_penalty = 0.0f;

  float dynamic_mag_variance = BASE_MAG_R + (MAX_MAG_R * motor_penalty);
  tuning_R[3] = dynamic_mag_variance;
  tuning_R[4] = dynamic_mag_variance;
  tuning_R[5] = dynamic_mag_variance;

  // --- CALL THE MATLAB BRAIN ---
  // The variables are now natively floats, which perfectly matches the updated MATLAB Coder output
  attitude_EKF(ax, ay, az, gx, gy, gz, mx, my, mz, dt, 
                 current_x, current_P, tuning_Q, tuning_R, b_mag, A_mag, 
                 &raw_roll, &raw_pitch, &raw_yaw, next_x, next_P);

  // Overwrite memory for the next loop
  for(int i=0; i<7; i++) current_x[i] = next_x[i];
  for(int i=0; i<49; i++) current_P[i] = next_P[i];

  // ==========================================================
  // 4. THE TARE SEQUENCE (Zeroing on Startup)
  // ==========================================================
  if (!is_tared) {
    offset_roll += raw_roll;
    offset_pitch += raw_pitch;
    offset_yaw += raw_yaw;
    tare_samples++;
    
    // Wait for 100 samples (~1 second at 100Hz) to let the EKF settle
    if (tare_samples >= 100) {
      offset_roll /= 100.0f;
      offset_pitch /= 100.0f;
      offset_yaw /= 100.0f;
      is_tared = true;
      Serial.println("EKF Settled and Tared! Angles are now ZERO.");
    }
    
    // While taring, output strict zeros so the motors don't jerk
    *roll = 0.0f; *pitch = 0.0f; *yaw = 0.0f; 
    return;
  }

  // ==========================================================
  // 5. APPLY TARE TO FINAL OUTPUTS
  // ==========================================================
  *roll  = raw_roll - offset_roll;
  *pitch = raw_pitch - offset_pitch;
  *yaw   = raw_yaw - offset_yaw;
  
  // Keep Yaw wrapped nicely between -180 and +180
  if (*yaw > 180.0f)  *yaw -= 360.0f;
  if (*yaw < -180.0f) *yaw += 360.0f;
}