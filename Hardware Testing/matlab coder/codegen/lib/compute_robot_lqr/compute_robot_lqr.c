/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_robot_lqr.c
 *
 * Code generation for function 'compute_robot_lqr'
 *
 */

/* Include files */
#include "compute_robot_lqr.h"
#include "diag.h"
#include "eig.h"
#include "mldivide.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void compute_robot_lqr(double L, double Iy, double Iz, double Mw, double r,
                       double d, double Iw, double Mb, const double Q_diag[6],
                       const double R_diag[2], double K[12],
                       creal_T cl_poles[6])
{
  static const signed char iv[6] = {0, 0, 0, 1, 0, 0};
  static const signed char iv1[6] = {0, 0, 0, 0, 1, 0};
  static const signed char iv2[6] = {0, 0, 0, 0, 0, 1};
  creal_T D[144];
  creal_T V[144];
  creal_T V_stable[72];
  creal_T b_V_stable[36];
  creal_T dcv[36];
  creal_T d_eig[12];
  double c_A[144];
  double A[36];
  double b_A[36];
  double b_B[36];
  double B[12];
  double b_b_tmp[12];
  double b_tmp[12];
  double R[4];
  double b41;
  double det_sag;
  double m11;
  double m12;
  double m22;
  double m22_tmp;
  int A_tmp;
  int count;
  int i;
  int k;
  /*     %% 1. Mass Matrix & State-Space */
  m11 = Mb * (L * L) + Iy;
  m12 = Mb * L;
  m22_tmp = r * r;
  m22 = (Mb + 2.0 * Mw) + 2.0 * (Iw / m22_tmp);
  det_sag = m11 * m22 - m12 * m12;
  A[3] = Mb * 9.81 * L * m22 / det_sag;
  A[9] = 0.0;
  A[15] = 0.0;
  A[21] = 0.0;
  A[27] = 0.0;
  A[33] = 0.0;
  A[4] = -Mb * 9.81 * L * m12 / det_sag;
  A[10] = 0.0;
  A[16] = 0.0;
  A[22] = 0.0;
  A[28] = 0.0;
  A[34] = 0.0;
  m22 = det_sag * r;
  b41 = -m12 / m22;
  m12 = m11 / m22;
  det_sag = d * d;
  m22 = d / (2.0 * r *
             ((det_sag / 2.0 * Mw + det_sag / (2.0 * m22_tmp) * Iw) + Iz));
  B[0] = 0.0;
  B[1] = 0.0;
  B[2] = 0.0;
  B[6] = 0.0;
  B[7] = 0.0;
  B[8] = 0.0;
  B[3] = b41;
  B[9] = b41;
  B[4] = m12;
  B[10] = m12;
  B[5] = m22;
  B[11] = -m22;
  b_diag(R_diag, R);
  /*     %% 2. Manual LQR Solver (Hamiltonian Method) */
  for (k = 0; k < 6; k++) {
    A[6 * k] = iv[k];
    A[6 * k + 1] = iv1[k];
    A[6 * k + 2] = iv2[k];
    A[6 * k + 5] = 0.0;
    count = k << 1;
    b_tmp[count] = B[k];
    b_tmp[count + 1] = B[k + 6];
  }
  /*  Solve Eigenvalues */
  mldivide(R, b_tmp, b_b_tmp);
  diag(Q_diag, b_A);
  for (k = 0; k < 6; k++) {
    m22 = B[k];
    det_sag = B[k + 6];
    for (i = 0; i < 6; i++) {
      count = i << 1;
      b_B[k + 6 * i] = -(m22 * b_b_tmp[count] + det_sag * b_b_tmp[count + 1]);
      c_A[i + 12 * k] = A[i + 6 * k];
    }
  }
  for (k = 0; k < 6; k++) {
    for (i = 0; i < 6; i++) {
      count = i + 6 * k;
      A_tmp = i + 12 * (k + 6);
      c_A[A_tmp] = b_B[count];
      c_A[(i + 12 * k) + 6] = -b_A[count];
      c_A[A_tmp + 6] = -A[k + 6 * i];
    }
  }
  eig(c_A, V, D);
  for (k = 0; k < 12; k++) {
    d_eig[k] = D[k + 12 * k];
  }
  /*  --- FIX: Explicitly initialize V_stable as complex --- */
  memset(&V_stable[0], 0, 72U * sizeof(creal_T));
  count = 1;
  for (k = 0; k < 12; k++) {
    /*  We look for eigenvalues with negative real parts (stable manifold) */
    if ((d_eig[k].re < 0.0) && (count <= 6)) {
      memcpy(&V_stable[count * 12 + -12], &V[k * 12], 12U * sizeof(creal_T));
      count++;
    }
  }
  /*  Partition the complex stable eigenvectors */
  /*  Solve for Riccati solution P = V2 / V1 */
  /*  P should be theoretically real, but we force it for Coder */
  /*  --- FIX: Final Gain K must be real --- */
  for (k = 0; k < 6; k++) {
    memcpy(&dcv[k * 6], &V_stable[k * 12 + 6], 6U * sizeof(creal_T));
    memcpy(&b_V_stable[k * 6], &V_stable[k * 12], 6U * sizeof(creal_T));
  }
  mrdiv(dcv, b_V_stable);
  memset(&b_b_tmp[0], 0, 12U * sizeof(double));
  for (k = 0; k < 6; k++) {
    count = k << 1;
    for (i = 0; i < 6; i++) {
      __m128d b_r;
      __m128d r1;
      b_r = _mm_loadu_pd(&b_tmp[i << 1]);
      r1 = _mm_loadu_pd(&b_b_tmp[count]);
      _mm_storeu_pd(
          &b_b_tmp[count],
          _mm_add_pd(r1, _mm_mul_pd(b_r, _mm_set1_pd(dcv[i + 6 * k].re))));
    }
  }
  mldivide(R, b_b_tmp, K);
  /*  Closed-loop poles can stay complex (this is normal) */
  for (k = 0; k < 6; k++) {
    m22 = B[k];
    det_sag = B[k + 6];
    for (i = 0; i < 6; i++) {
      count = i << 1;
      A_tmp = k + 6 * i;
      b_A[A_tmp] = A[A_tmp] - (m22 * K[count] + det_sag * K[count + 1]);
    }
  }
  b_eig(b_A, cl_poles);
}

/* End of code generation (compute_robot_lqr.c) */
