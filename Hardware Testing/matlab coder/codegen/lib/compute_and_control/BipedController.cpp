//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// BipedController.cpp
//
// Code generation for function 'BipedController'
//

// Include files
#include "BipedController.h"
#include "diag.h"
#include "eig.h"
#include "mldivide.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
BipedController::BipedController() = default;

BipedController::~BipedController() = default;

void BipedController::compute_and_control(const double state[6], double L,
                                          double Iy, double Iz, double Mw,
                                          double r, double d, double Iw,
                                          double Mb, const double Q_diag[6],
                                          const double R_diag[2], double Kt,
                                          double V_batt, double *pwm_R,
                                          double *pwm_L, double K[12])
{
  __m128d b_r;
  __m128d r1;
  creal_T D[144];
  creal_T V[144];
  creal_T V_stable[72];
  creal_T b_V_stable[36];
  creal_T dcv[36];
  creal_T d_eig[12];
  double b_A[144];
  double A[36];
  double b_B[36];
  double dv[36];
  double B[12];
  double b_b_tmp[12];
  double b_tmp[12];
  double H_tmp[4];
  double torque[2];
  double det_sag;
  double m11;
  double m12;
  double m22;
  double m22_tmp;
  int count;
  //     %% --- 1. Physics & State-Space ---
  m11 = Mb * (L * L) + Iy;
  m12 = Mb * L;
  m22_tmp = r * r;
  m22 = (Mb + 2.0 * Mw) + 2.0 * (Iw / m22_tmp);
  det_sag = m11 * m22 - m12 * m12;
  std::memset(&A[0], 0, 36U * sizeof(double));
  A[18] = 1.0;
  A[25] = 1.0;
  A[32] = 1.0;
  A[3] = Mb * 9.81 * L * m22 / det_sag;
  A[4] = -Mb * 9.81 * L * m12 / det_sag;
  std::memset(&B[0], 0, 12U * sizeof(double));
  det_sag *= r;
  B[3] = -m12 / det_sag;
  B[9] = B[3];
  B[4] = m11 / det_sag;
  B[10] = B[4];
  det_sag = d * d;
  B[5] = d / (2.0 * r *
              ((det_sag / 2.0 * Mw + det_sag / (2.0 * m22_tmp) * Iw) + Iz));
  B[11] = -B[5];
  //     %% --- 2. Solve LQR ---
  coder::diag(R_diag, H_tmp);
  for (int k{0}; k < 6; k++) {
    count = k << 1;
    b_tmp[count] = B[k];
    b_tmp[count + 1] = B[k + 6];
  }
  coder::mldivide(H_tmp, b_tmp, b_b_tmp);
  coder::b_diag(Q_diag, dv);
  for (int k{0}; k < 6; k++) {
    det_sag = B[k];
    m22 = B[k + 6];
    for (int i{0}; i < 6; i++) {
      count = i << 1;
      b_B[k + 6 * i] = -(det_sag * b_b_tmp[count] + m22 * b_b_tmp[count + 1]);
      b_A[i + 12 * k] = A[i + 6 * k];
    }
  }
  for (int k{0}; k < 6; k++) {
    for (int i{0}; i < 6; i++) {
      int A_tmp;
      count = i + 6 * k;
      A_tmp = i + 12 * (k + 6);
      b_A[A_tmp] = b_B[count];
      b_A[(i + 12 * k) + 6] = -dv[count];
      b_A[A_tmp + 6] = -A[k + 6 * i];
    }
  }
  coder::eig(b_A, V, D);
  for (int k{0}; k < 12; k++) {
    d_eig[k] = D[k + 12 * k];
  }
  std::memset(&V_stable[0], 0, 72U * sizeof(creal_T));
  count = 1;
  for (int k{0}; k < 12; k++) {
    if ((d_eig[k].re < 0.0) && (count <= 6)) {
      std::copy(&V[k * 12],
                &V[static_cast<int>(static_cast<unsigned int>(k * 12) + 12U)],
                &V_stable[count * 12 + -12]);
      count++;
    }
  }
  //  K = inv(R) * B' * P
  for (int k{0}; k < 6; k++) {
    std::copy(&V_stable[k * 12 + 6], &V_stable[k * 12 + 12], &dcv[k * 6]);
    std::copy(
        &V_stable[k * 12],
        &V_stable[static_cast<int>(static_cast<unsigned int>(k * 12) + 6U)],
        &b_V_stable[k * 6]);
  }
  coder::internal::mrdiv(dcv, b_V_stable);
  std::memset(&b_b_tmp[0], 0, 12U * sizeof(double));
  for (int k{0}; k < 6; k++) {
    count = k << 1;
    for (int i{0}; i < 6; i++) {
      b_r = _mm_loadu_pd(&b_tmp[i << 1]);
      r1 = _mm_loadu_pd(&b_b_tmp[count]);
      _mm_storeu_pd(
          &b_b_tmp[count],
          _mm_add_pd(r1, _mm_mul_pd(b_r, _mm_set1_pd(dcv[i + 6 * k].re))));
    }
  }
  coder::mldivide(H_tmp, b_b_tmp, K);
  //     %% --- 3. Compute PWM ---
  //  Control Law
  for (int k{0}; k <= 10; k += 2) {
    b_r = _mm_loadu_pd(&K[k]);
    _mm_storeu_pd(&b_b_tmp[k], _mm_mul_pd(b_r, _mm_set1_pd(-1.0)));
  }
  std::memset(&torque[0], 0, sizeof(double) << 1);
  for (int k{0}; k < 6; k++) {
    b_r = _mm_loadu_pd(&b_b_tmp[k << 1]);
    r1 = _mm_loadu_pd(&torque[0]);
    _mm_storeu_pd(&torque[0],
                  _mm_add_pd(r1, _mm_mul_pd(b_r, _mm_set1_pd(state[k]))));
  }
  //  Torque to PWM (0.0 to 1.0 scale)
  //  PWM = (Torque / Kt) / V_batt
  //  We multiply by a small safety factor if your driver has dead-zones
  //  Saturation for 24V safety
  det_sag = Kt * V_batt;
  *pwm_R = std::fmax(-1.0, std::fmin(1.0, torque[0] / det_sag));
  *pwm_L = std::fmax(-1.0, std::fmin(1.0, torque[1] / det_sag));
}

// End of code generation (BipedController.cpp)
