//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_compute_and_control_api.h
//
// Code generation for function 'compute_and_control'
//

#ifndef _CODER_COMPUTE_AND_CONTROL_API_H
#define _CODER_COMPUTE_AND_CONTROL_API_H

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
void compute_and_control(real_T state[6], real_T L, real_T Iy, real_T Iz,
                         real_T Mw, real_T r, real_T d, real_T Iw, real_T Mb,
                         real_T Q_diag[6], real_T R_diag[2], real_T Kt,
                         real_T V_batt, real_T *pwm_R, real_T *pwm_L,
                         real_T K[12]);

void compute_and_control_api(const mxArray *const prhs[13], int32_T nlhs,
                             const mxArray *plhs[3]);

void compute_and_control_atexit();

void compute_and_control_initialize();

void compute_and_control_terminate();

void compute_and_control_xil_shutdown();

void compute_and_control_xil_terminate();

#endif
// End of code generation (_coder_compute_and_control_api.h)
