/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_compute_robot_lqr_api.h
 *
 * Code generation for function 'compute_robot_lqr'
 *
 */

#ifndef _CODER_COMPUTE_ROBOT_LQR_API_H
#define _CODER_COMPUTE_ROBOT_LQR_API_H

/* Include files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void compute_robot_lqr(real_T L, real_T Iy, real_T Iz, real_T Mw, real_T r,
                       real_T d, real_T Iw, real_T Mb, real_T Q_diag[6],
                       real_T R_diag[2], real_T K[12], creal_T cl_poles[6]);

void compute_robot_lqr_api(const mxArray *const prhs[10], int32_T nlhs,
                           const mxArray *plhs[2]);

void compute_robot_lqr_atexit(void);

void compute_robot_lqr_initialize(void);

void compute_robot_lqr_terminate(void);

void compute_robot_lqr_xil_shutdown(void);

void compute_robot_lqr_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (_coder_compute_robot_lqr_api.h) */
