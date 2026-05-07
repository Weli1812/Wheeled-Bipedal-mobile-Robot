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
extern void attitude_EKF(double ax, double ay, double az, double gx, double gy,
                         double gz, double mx, double my, double mz, double dt,
                         const double x_in[7], const double P_in[49],
                         const double Q_diag[7], const double R_diag[6],
                         const double b_mag[3], const double A_mag[9],
                         double *ekf_roll, double *ekf_pitch, double *ekf_yaw,
                         double x_out[7], double P_out[49]);

#endif
// End of code generation (attitude_EKF.h)
