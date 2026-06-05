//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_LQRwheels_api.h
//
// Code generation for function 'LQRwheels'
//

#ifndef _CODER_LQRWHEELS_API_H
#define _CODER_LQRWHEELS_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void LQRwheels(real_T x_curr, real_T y_curr, real_T theta_curr, real_T dt,
               real_T xd_k, real_T yd_k, real_T thetad_k, real_T vd_k,
               real_T wd_k, real_T vr_actual, real_T vl_actual, real_T *pwmr,
               real_T *pwml, boolean_T *dirr, boolean_T *dirl);

void LQRwheels_api(const mxArray *const prhs[11], int32_T nlhs,
                   const mxArray *plhs[4]);

void LQRwheels_atexit();

void LQRwheels_initialize();

void LQRwheels_terminate();

void LQRwheels_xil_shutdown();

void LQRwheels_xil_terminate();

#endif
// End of code generation (_coder_LQRwheels_api.h)
