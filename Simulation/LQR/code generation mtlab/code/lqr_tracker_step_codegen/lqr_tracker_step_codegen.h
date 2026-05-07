/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lqr_tracker_step_codegen.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 19-Apr-2026 13:12:28
 */

#ifndef LQR_TRACKER_STEP_CODEGEN_H
#define LQR_TRACKER_STEP_CODEGEN_H

/* Include Files */
#include "lqr_tracker_step_codegen_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void lqr_tracker_step_codegen(
    double x, double y, double theta, const emxArray_real_T *P,
    const double theta_ref_data[], const int theta_ref_size[1],
    const double vd_ref_data[], const int vd_ref_size[1],
    const double wd_ref_data[], const int wd_ref_size[1], const double K[6],
    const double goalPose[3], double goalPosTol, double goalThetaTol,
    double *v_cmd, double *w_cmd, boolean_T *done, int *idx);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for lqr_tracker_step_codegen.h
 *
 * [EOF]
 */
