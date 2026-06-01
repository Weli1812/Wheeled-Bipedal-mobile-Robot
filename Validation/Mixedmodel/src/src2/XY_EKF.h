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
extern void XY_EKF(const float X[3], const float P[9], float v_R, float v_L,
                   float Z, float dt, float L, float X_new[3], float P_new[9]);

#endif
// End of code generation (XY_EKF.h)
