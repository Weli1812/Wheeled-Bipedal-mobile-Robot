//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// attitude_EKF.h
//
// Code generation for function 'attitude_EKF'
//

#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void attitude_EKF(float ax, float ay, float az, float gx, float gy,
                         float gz, float mx, float my, float mz, float dt,
                         const float x_in[7], const float P_in[49],
                         const float Q_diag[7], const float R_diag[6],
                         const float b_mag[3], const float A_mag[9],
                         float *ekf_roll, float *ekf_pitch, float *ekf_yaw,
                         float x_out[7], float P_out[49]);

#endif
// End of code generation (attitude_EKF.h)
