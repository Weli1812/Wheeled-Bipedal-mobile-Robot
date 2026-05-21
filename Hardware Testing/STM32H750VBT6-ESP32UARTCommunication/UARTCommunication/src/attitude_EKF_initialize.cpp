//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// attitude_EKF_initialize.cpp
//
// Code generation for function 'attitude_EKF_initialize'
//

// Include files
#include "attitude_EKF_initialize.h"
#include "attitude_EKF_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void attitude_EKF_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_attitude_EKF = true;
}

// End of code generation (attitude_EKF_initialize.cpp)
