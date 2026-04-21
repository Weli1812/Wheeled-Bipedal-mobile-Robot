/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: robot_controller.h
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 20-Apr-2026 23:37:54
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void
robot_controller(const double currentPose[3], const double refPath_data[],
                 const int refPath_size[2], const double vd_ref_data[],
                 const int vd_ref_size[1], const double wd_ref_data[],
                 const int wd_ref_size[1], double stepIndex, double *v_cmd,
                 double *w_cmd, boolean_T *reached);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for robot_controller.h
 *
 * [EOF]
 */
