#include "EKF.h"
#include "attitude_EKF.h" // The MATLAB generated header (Now expecting floats)
#include <Arduino.h>      // Needed for Serial.println

// ==========================================================
// 1. THE EKF MEMORY & TUNING ZONE (Hidden from Main loop)
// ==========================================================
static float current_x[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
static float current_P[49]; 

static float tuning_Q[7]  = {0.0005f, 0.0005f, 0.0005f, 0.0005f, 0.0025f, 0.0025f, 0.0025f};
static float tuning_R[6]  = {0.25f, 0.25f, 0.25f, 2.0f, 2.0f, 2.0f};

// Adaptive Mag Tuning Parameters
const float BASE_MAG_R = 20.0f;     
const float MAX_MAG_R  = 10000.0f;  

// --- YOUR NEWLY CALIBRATED MAGNETIC MATRICES ---
static float b_mag[3] = {87.3945f, 88.8523f, 50.9644f};
static float A_mag[9] = {1.4002f, 0.0000f, 0.0000f, 0.0000f, 0.6374f, 0.0000f, 0.0000f, 0.0000f, 1.1205f}; 

// --- TARE (ZEROING) VARIABLES ---
static bool is_tared = false;
static int tare_samples = 0;
static float offset_roll = 0.0f, offset_pitch = 0.0f, offset_yaw = 0.0f;

// ==========================================================
// 2. INITIALIZATION
// ==========================================================
void init_biped_EKF() {
  for(int i=0; i<49; i++) {
    if(i % 8 == 0) current_P[i] = 70.0f; 
    else current_P[i] = 0.0f;          
  }
  is_tared = false;
  tare_samples = 0;
  offset_roll = 0.0f; offset_pitch = 0.0f; offset_yaw = 0.0f;
}

// ==========================================================
// 3. THE MAIN WRAPPER FUNCTION
// ==========================================================
void rpy_EKF(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt, float motor_penalty, bool is_startup,
             float *roll, float *pitch, float *yaw, float *bx, float *by, float *bz) { // <-- Updated signature
  
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
  attitude_EKF(ax, ay, az, gx, gy, gz, mx, my, mz, dt, 
                 current_x, current_P, tuning_Q, tuning_R, b_mag, A_mag, 
                 &raw_roll, &raw_pitch, &raw_yaw, next_x, next_P);

  // Overwrite memory for the next loop
  for(int i=0; i<7; i++) current_x[i] = next_x[i];
  for(int i=0; i<49; i++) current_P[i] = next_P[i];

  // --- NEW: EXPORT DYNAMIC BIASES ---
  // The state vector x is [q0, q1, q2, q3, bx, by, bz]
  *bx = next_x[4];
  *by = next_x[5];
  *bz = next_x[6];

  *roll  = raw_roll; 
  *pitch = raw_pitch;
  *yaw   = raw_yaw;
}