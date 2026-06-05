#include "EKF.h"
#include "attitude_EKF.h" // The MATLAB generated header
#include <Arduino.h>      // Needed for Serial.println

// ==========================================================
// 1. THE EKF MEMORY & TUNING ZONE (Hidden from Main loop)
// ==========================================================
static double current_x[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double current_P[49]; 

static double tuning_Q[7]  = {0.0008, 0.0008, 0.0008, 0.0008, 0.003, 0.003, 0.003};
static double tuning_R[6]  = {0.05, 0.05, 0.05, 2.0, 2.0, 2.0};

// Adaptive Mag Tuning Parameters
const double BASE_MAG_R = 20.0;     
const double MAX_MAG_R  = 10000.0;  

// --- YOUR NEWLY CALIBRATED MAGNETIC MATRICES ---
static double b_mag[3] = {70.3812, -38.6215, -0.2548};
static double A_mag[9] = {0.9767, 0.0186, -0.1272, 
                          0.0186, 1.0381, 0.0393, 
                         -0.1272, 0.0393, 1.0049}; 

// --- TARE (ZEROING) VARIABLES ---
static bool is_tared = false;
static int tare_samples = 0;
static double offset_roll = 0.0, offset_pitch = 0.0, offset_yaw = 0.0;

// ==========================================================
// 2. INITIALIZATION
// ==========================================================
void init_biped_EKF() {
  for(int i=0; i<49; i++) {
    if(i % 8 == 0) current_P[i] = 1.0; 
    else current_P[i] = 0.0;           
  }
  is_tared = false;
  tare_samples = 0;
  offset_roll = 0.0; offset_pitch = 0.0; offset_yaw = 0.0;
}

// ==========================================================
// 3. THE MAIN WRAPPER FUNCTION
// ==========================================================
void rpy_EKF(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz, double dt, double motor_penalty,
             double *roll, double *pitch, double *yaw) {
  
  double next_x[7], next_P[49];
  double raw_roll, raw_pitch, raw_yaw;

  // --- ADAPTIVE COVARIANCE INJECTION ---
  if(motor_penalty > 1.0) motor_penalty = 1.0;
  if(motor_penalty < 0.0) motor_penalty = 0.0;

  double dynamic_mag_variance = BASE_MAG_R + (MAX_MAG_R * motor_penalty);
  tuning_R[3] = dynamic_mag_variance;
  tuning_R[4] = dynamic_mag_variance;
  tuning_R[5] = dynamic_mag_variance;

  // --- CALL THE MATLAB BRAIN ---
  // Note: Depending on your exact MATLAB Coder version, the argument order might slightly differ.
  // Double-check 'attitude_EKF.h' if the compiler complains about argument types.
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
      offset_roll /= 100.0;
      offset_pitch /= 100.0;
      offset_yaw /= 100.0;
      is_tared = true;
      Serial.println("EKF Settled and Tared! Angles are now ZERO.");
    }
    
    // While taring, output strict zeros so the motors don't jerk
    *roll = 0.0; *pitch = 0.0; *yaw = 0.0; 
    return;
  }

  // ==========================================================
  // 5. APPLY TARE TO FINAL OUTPUTS
  // ==========================================================
  *roll  = raw_roll - offset_roll;
  *pitch = raw_pitch - offset_pitch;
  *yaw   = raw_yaw - offset_yaw;
  
  // Keep Yaw wrapped nicely between -180 and +180
  if (*yaw > 180.0)  *yaw -= 360.0;
  if (*yaw < -180.0) *yaw += 360.0;
}