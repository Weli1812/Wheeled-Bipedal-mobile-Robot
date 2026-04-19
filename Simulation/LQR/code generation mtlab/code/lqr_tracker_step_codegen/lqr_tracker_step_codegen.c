/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: lqr_tracker_step_codegen.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 19-Apr-2026 13:12:28
 */

/* Include Files */
#include "lqr_tracker_step_codegen.h"
#include "atan2.h"
#include "lqr_tracker_step_codegen_types.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * P            Nx2
 *  theta_ref    Nx1
 *  vd_ref       Nx1
 *  wd_ref       Nx1
 *  K            2x3
 *  goalPose     1x3   [xg yg thetag]
 *
 * Arguments    : double x
 *                double y
 *                double theta
 *                const emxArray_real_T *P
 *                const double theta_ref_data[]
 *                const int theta_ref_size[1]
 *                const double vd_ref_data[]
 *                const int vd_ref_size[1]
 *                const double wd_ref_data[]
 *                const int wd_ref_size[1]
 *                const double K[6]
 *                const double goalPose[3]
 *                double goalPosTol
 *                double goalThetaTol
 *                double *v_cmd
 *                double *w_cmd
 *                boolean_T *done
 *                int *idx
 * Return Type  : void
 */
void lqr_tracker_step_codegen(
    double x, double y, double theta, const emxArray_real_T *P,
    const double theta_ref_data[], const int theta_ref_size[1],
    const double vd_ref_data[], const int vd_ref_size[1],
    const double wd_ref_data[], const int wd_ref_size[1], const double K[6],
    const double goalPose[3], double goalPosTol, double goalThetaTol,
    double *v_cmd, double *w_cmd, boolean_T *done, int *idx)
{
  double b_K[6];
  double u[2];
  const double *P_data;
  double bestD2;
  double dxi;
  double dyi;
  int b_i;
  int i;
  boolean_T b_done;
  (void)theta_ref_size;
  (void)vd_ref_size;
  (void)wd_ref_size;
  P_data = P->data;
  /*  ------------------------------------------------- */
  /*  1) Find nearest reference point */
  /*  ------------------------------------------------- */
  bestD2 = rtInf;
  *idx = 1;
  i = P->size[0];
  for (b_i = 0; b_i < i; b_i++) {
    dxi = P_data[b_i] - x;
    dyi = P_data[b_i + P->size[0]] - y;
    dxi = dxi * dxi + dyi * dyi;
    if (dxi < bestD2) {
      bestD2 = dxi;
      *idx = b_i + 1;
    }
  }
  __m128d r;
  __m128d r1;
  double c;
  double s;
  /*  ------------------------------------------------- */
  /*  2) Get reference at nearest point */
  /*  ------------------------------------------------- */
  /*  3) Tracking error in reference frame */
  /*  ------------------------------------------------- */
  dyi = P_data[*idx - 1] - x;
  bestD2 = P_data[(*idx + P->size[0]) - 1] - y;
  dxi = theta_ref_data[*idx - 1];
  c = cos(dxi);
  s = sin(dxi);
  /*  ------------------------------------------------- */
  /*  4) LQR correction */
  /*  ------------------------------------------------- */
  r = _mm_set1_pd(-1.0);
  _mm_storeu_pd(&b_K[0], _mm_mul_pd(_mm_loadu_pd(&K[0]), r));
  _mm_storeu_pd(&b_K[2], _mm_mul_pd(_mm_loadu_pd(&K[2]), r));
  _mm_storeu_pd(&b_K[4], _mm_mul_pd(_mm_loadu_pd(&K[4]), r));
  dxi -= theta;
  memset(&u[0], 0, sizeof(double) << 1);
  r = _mm_loadu_pd(&b_K[0]);
  r1 = _mm_loadu_pd(&u[0]);
  _mm_storeu_pd(
      &u[0], _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(c * dyi + s * bestD2))));
  r = _mm_loadu_pd(&b_K[2]);
  r1 = _mm_loadu_pd(&u[0]);
  _mm_storeu_pd(
      &u[0], _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(-s * dyi + c * bestD2))));
  r = _mm_loadu_pd(&b_K[4]);
  r1 = _mm_loadu_pd(&u[0]);
  _mm_storeu_pd(
      &u[0],
      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(b_atan2(sin(dxi), cos(dxi))))));
  *v_cmd = vd_ref_data[*idx - 1] + u[0];
  *w_cmd = wd_ref_data[*idx - 1] + u[1];
  /*  ------------------------------------------------- */
  /*  5) Saturation */
  /*  ------------------------------------------------- */
  if (*v_cmd < 0.0) {
    *v_cmd = 0.0;
  } else if (*v_cmd > 0.3) {
    *v_cmd = 0.3;
  }
  if (*w_cmd > 0.25) {
    *w_cmd = 0.25;
  } else if (*w_cmd < -0.25) {
    *w_cmd = -0.25;
  }
  /*  ------------------------------------------------- */
  /*  6) Goal stop */
  /*  ------------------------------------------------- */
  dxi = goalPose[0] - x;
  dyi = goalPose[1] - y;
  if (sqrt(dxi * dxi + dyi * dyi) <= goalPosTol) {
    dxi = goalPose[2] - theta;
    if (fabs(b_atan2(sin(dxi), cos(dxi))) <= goalThetaTol) {
      b_done = true;
    } else {
      b_done = false;
    }
  } else {
    b_done = false;
  }
  *done = b_done;
  if (b_done) {
    *v_cmd = 0.0;
    *w_cmd = 0.0;
  }
}

/*
 * File trailer for lqr_tracker_step_codegen.c
 *
 * [EOF]
 */
