/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_lqr_tracker_step_codegen_mex.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 19-Apr-2026 13:12:28
 */

#ifndef _CODER_LQR_TRACKER_STEP_CODEGEN_MEX_H
#define _CODER_LQR_TRACKER_STEP_CODEGEN_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_lqr_tracker_step_codegen_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                                 int32_T nrhs,
                                                 const mxArray *prhs[11]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_lqr_tracker_step_codegen_mex.h
 *
 * [EOF]
 */
