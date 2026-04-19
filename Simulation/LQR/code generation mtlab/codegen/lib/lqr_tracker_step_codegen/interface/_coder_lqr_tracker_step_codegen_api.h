/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_lqr_tracker_step_codegen_api.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 19-Apr-2026 13:12:28
 */

#ifndef _CODER_LQR_TRACKER_STEP_CODEGEN_API_H
#define _CODER_LQR_TRACKER_STEP_CODEGEN_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T
struct emxArray_real_T {
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};
#endif /* struct_emxArray_real_T */
#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T
typedef struct emxArray_real_T emxArray_real_T;
#endif /* typedef_emxArray_real_T */

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void lqr_tracker_step_codegen(real_T x, real_T y, real_T theta,
                              emxArray_real_T *P, real_T theta_ref_data[],
                              int32_T theta_ref_size[1], real_T vd_ref_data[],
                              int32_T vd_ref_size[1], real_T wd_ref_data[],
                              int32_T wd_ref_size[1], real_T K[6],
                              real_T goalPose[3], real_T goalPosTol,
                              real_T goalThetaTol, real_T *v_cmd, real_T *w_cmd,
                              boolean_T *done, int32_T *idx);

void lqr_tracker_step_codegen_api(const mxArray *const prhs[11], int32_T nlhs,
                                  const mxArray *plhs[4]);

void lqr_tracker_step_codegen_atexit(void);

void lqr_tracker_step_codegen_initialize(void);

void lqr_tracker_step_codegen_terminate(void);

void lqr_tracker_step_codegen_xil_shutdown(void);

void lqr_tracker_step_codegen_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_lqr_tracker_step_codegen_api.h
 *
 * [EOF]
 */
