//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// LQRwheels_initialize.cpp
//
// Code generation for function 'LQRwheels_initialize'
//

// Include files
#include "LQRwheels_initialize.h"
#include "LQRwheels.h"
#include "LQRwheels_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void LQRwheels_initialize()
{
  rt_InitInfAndNaN();
  LQRwheels_init();
  isInitialized_LQRwheels = true;
}

// End of code generation (LQRwheels_initialize.cpp)
