//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// XY_EKF_initialize.cpp
//
// Code generation for function 'XY_EKF_initialize'
//

// Include files
#include "XY_EKF_initialize.h"
#include "XY_EKF_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void XY_EKF_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_XY_EKF = true;
}

// End of code generation (XY_EKF_initialize.cpp)
