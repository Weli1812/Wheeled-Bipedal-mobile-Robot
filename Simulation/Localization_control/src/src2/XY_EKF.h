//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// XY_EKF.h
//
// Code generation for function 'XY_EKF'
//

#ifndef XY_EKF_H
#define XY_EKF_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void XY_EKF(const double X[3], const double P[9], double v_R, double v_L,
                   double Z, double dt, double L, double X_new[3],
                   double P_new[9]);

#endif
// End of code generation (XY_EKF.h)
