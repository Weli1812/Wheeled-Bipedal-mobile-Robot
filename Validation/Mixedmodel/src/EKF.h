#ifndef EKF_H
#define EKF_H

// Call this ONCE in setup()
void init_biped_EKF();

// Call this in the loop() to get the perfect angles
// Added 'motor_penalty' (0.0 to 1.0) to dynamically scale magnetometer trust
// Change your function declaration in EKF.h to include "bool is_startup"
void rpy_EKF(float ax, float ay, float az, 
             float gx, float gy, float gz, 
             float mx, float my, float mz, 
             float dt, float motor_penalty, bool is_startup,
             float *roll, float *pitch, float *yaw,
             float *bx, float *by, float *bz);

#endif