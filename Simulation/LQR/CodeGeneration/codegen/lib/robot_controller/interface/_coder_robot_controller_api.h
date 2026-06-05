/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_robot_controller_api.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 20-Apr-2026 23:37:54
 */

#ifndef _CODER_ROBOT_CONTROLLER_API_H
#define _CODER_ROBOT_CONTROLLER_API_H

/* Include Files */
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
void robot_controller(real_T currentPose[3], real_T refPath_data[],
                      int32_T refPath_size[2], real_T vd_ref_data[],
                      int32_T vd_ref_size[1], real_T wd_ref_data[],
                      int32_T wd_ref_size[1], real_T stepIndex, real_T *v_cmd,
                      real_T *w_cmd, boolean_T *reached);

void robot_controller_api(const mxArray *const prhs[5], int32_T nlhs,
                          const mxArray *plhs[3]);

void robot_controller_atexit(void);

void robot_controller_initialize(void);

void robot_controller_terminate(void);

void robot_controller_xil_shutdown(void);

void robot_controller_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_robot_controller_api.h
 *
 * [EOF]
 */
