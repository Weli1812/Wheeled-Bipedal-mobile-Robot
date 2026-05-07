/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: robot_controller.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 20-Apr-2026 23:37:54
 */

/* Include Files */
#include "robot_controller.h"
#include <math.h>

/* Function Declarations */
static double rt_roundd_snf(double u);

/* Function Definitions */
/*
 * Arguments    : double u
 * Return Type  : double
 */
static double rt_roundd_snf(double u)
{
  double y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }
  return y;
}

/*
 * Inputs:
 *    currentPose  - [1x3] double: [x, y, theta]
 *    refPath      - [Nx3] double: path from run_planner
 *    vd_ref       - [Nx1] double: reference linear velocities
 *    wd_ref       - [Nx1] double: reference angular velocities
 *    stepIndex    - scalar double: current step (1-based)
 *
 *  Outputs:
 *    v_cmd   - linear  velocity command
 *    w_cmd   - angular velocity command
 *    reached - true when goal is reached
 *
 * Arguments    : const double currentPose[3]
 *                const double refPath_data[]
 *                const int refPath_size[2]
 *                const double vd_ref_data[]
 *                const int vd_ref_size[1]
 *                const double wd_ref_data[]
 *                const int wd_ref_size[1]
 *                double stepIndex
 *                double *v_cmd
 *                double *w_cmd
 *                boolean_T *reached
 * Return Type  : void
 */
void robot_controller(const double currentPose[3], const double refPath_data[],
                      const int refPath_size[2], const double vd_ref_data[],
                      const int vd_ref_size[1], const double wd_ref_data[],
                      const int wd_ref_size[1], double stepIndex, double *v_cmd,
                      double *w_cmd, boolean_T *reached)
{
  double c;
  double d;
  double e_local_x;
  double e_local_y;
  double et;
  double ex;
  double ey;
  double rt;
  double rw;
  double rv;
  double rx;
  double ry;
  double s;
  int idx;
  int numPoints;
  int row;

  *reached = false;
  *v_cmd = 0.0;
  *w_cmd = 0.0;

  numPoints = refPath_size[0];
  if ((numPoints <= 0) || (refPath_size[1] != 3) || (vd_ref_size[0] < numPoints) ||
      (wd_ref_size[0] < numPoints)) {
    *reached = true;
    return;
  }

  d = rt_roundd_snf(stepIndex);
  if (d < 2.147483648E+9) {
    if (d >= -2.147483648E+9) {
      idx = (int)d;
    } else {
      idx = MIN_int32_T;
    }
  } else if (d >= 2.147483648E+9) {
    idx = MAX_int32_T;
  } else {
    idx = 0;
  }

  if ((idx >= numPoints) || (idx < 1)) {
    *reached = true;
    return;
  }

  row = idx - 1;
  rx = refPath_data[row];
  ry = refPath_data[row + numPoints];
  rt = refPath_data[row + (numPoints << 1)];
  rv = vd_ref_data[row];
  rw = wd_ref_data[row];

  ex = rx - currentPose[0];
  ey = ry - currentPose[1];
  et = atan2(sin(rt - currentPose[2]), cos(rt - currentPose[2]));

  c = cos(currentPose[2]);
  s = sin(currentPose[2]);
  e_local_x = c * ex + s * ey;
  e_local_y = -s * ex + c * ey;

  *v_cmd = rv + 2.2361 * e_local_x;
  *w_cmd = rw + (2.8868 * e_local_y + 2.5456 * et);

  if (*v_cmd < 0.0) {
    *v_cmd = 0.0;
  } else if (*v_cmd > 0.3) {
    *v_cmd = 0.3;
  }

  if (*w_cmd < -0.25) {
    *w_cmd = -0.25;
  } else if (*w_cmd > 0.25) {
    *w_cmd = 0.25;
  }
}

/*
 * File trailer for robot_controller.c
 *
 * [EOF]
 */
