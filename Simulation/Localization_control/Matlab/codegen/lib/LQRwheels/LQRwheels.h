//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// LQRwheels.h
//
// Code generation for function 'LQRwheels'
//

#ifndef LQRWHEELS_H
#define LQRWHEELS_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void LQRwheels(double x_curr, double y_curr, double theta_curr,
                      double dt, double xd_k, double yd_k, double thetad_k,
                      double vd_k, double wd_k, double vr_actual,
                      double vl_actual, double *pwmr, double *pwml,
                      boolean_T *dirr, boolean_T *dirl);

void LQRwheels_init();

#endif
// End of code generation (LQRwheels.h)
