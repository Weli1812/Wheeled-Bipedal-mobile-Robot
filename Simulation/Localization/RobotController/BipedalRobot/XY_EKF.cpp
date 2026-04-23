//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// XY_EKF.cpp
//
// Code generation for function 'XY_EKF'
//

// Include files
#include "XY_EKF.h"
#include "XY_EKF_data.h"
#include "XY_EKF_initialize.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include <math.h>

// Function Definitions
void XY_EKF(const double X[3], const double P[9], double v_R, double v_L,
            double Z, double dt, double L, double X_new[3], double P_new[9])
{
  static const double Q[9] = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
  static const signed char b_I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char a[3] = {0, 0, 1};
  static const signed char iv[3] = {0, 0, 1};
  double F[9];
  double P_pred[9];
  double b_F[9];
  double K[3];
  double V;
  double X_new_tmp;
  double avg_yaw;
  double avg_yaw_tmp;
  double d;
  int F_tmp;
  int P_pred_tmp;
  int i1;
  if (!isInitialized_XY_EKF) {
    XY_EKF_initialize();
  }
  //  --- Tuning Matrices (Explicitly defined for C++ safety) ---
  //  --- Differential Kinematics ---
  V = (v_R + v_L) / 2.0;
  //  --- STEP 1: PREDICTION ---
  avg_yaw_tmp = (v_R - v_L) / L * dt;
  avg_yaw = X[2] + avg_yaw_tmp / 2.0;
  X_new_tmp = V * cos(avg_yaw) * dt;
  X_new[0] = X[0] + X_new_tmp;
  d = sin(avg_yaw);
  X_new[1] = X[1] + V * d * dt;
  X_new[2] = X[2] + avg_yaw_tmp;
  F[0] = 1.0;
  F[3] = 0.0;
  F[6] = -V * d * dt;
  F[1] = 0.0;
  F[4] = 1.0;
  F[7] = X_new_tmp;
  F[2] = 0.0;
  F[5] = 0.0;
  F[8] = 1.0;
  //  --- STEP 2: UPDATE ---
  avg_yaw_tmp = 0.0;
  for (int i = 0; i < 3; i++) {
    i1 = static_cast<int>(F[i]);
    P_pred_tmp = static_cast<int>(F[i + 3]);
    d = F[i + 6];
    for (F_tmp = 0; F_tmp < 3; F_tmp++) {
      b_F[i + 3 * F_tmp] =
          (static_cast<double>(i1) * P[3 * F_tmp] +
           static_cast<double>(P_pred_tmp) * P[3 * F_tmp + 1]) +
          d * P[3 * F_tmp + 2];
    }
    d = b_F[i];
    avg_yaw = b_F[i + 3];
    V = b_F[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      P_pred_tmp = i + 3 * i1;
      P_pred[P_pred_tmp] =
          ((d * F[i1] + avg_yaw * F[i1 + 3]) + V * F[i1 + 6]) + Q[P_pred_tmp];
    }
    avg_yaw_tmp += static_cast<double>(a[i]) * X_new[i];
  }
  avg_yaw = Z - avg_yaw_tmp;
  avg_yaw = coder::b_atan2(sin(avg_yaw), cos(avg_yaw));
  //  Safe angle wrap
  avg_yaw_tmp = 0.0;
  for (int i = 0; i < 3; i++) {
    avg_yaw_tmp +=
        ((0.0 * P_pred[3 * i] + 0.0 * P_pred[3 * i + 1]) + P_pred[3 * i + 2]) *
        static_cast<double>(iv[i]);
  }
  for (int i = 0; i < 3; i++) {
    d = ((P_pred[i] * 0.0 + P_pred[i + 3] * 0.0) + P_pred[i + 6]) /
        (avg_yaw_tmp + 0.05);
    K[i] = d;
    X_new[i] += d * avg_yaw;
  }
  d = K[0];
  avg_yaw = K[1];
  V = K[2];
  for (int i = 0; i < 3; i++) {
    P_pred_tmp = a[i];
    F[3 * i] =
        static_cast<double>(b_I[3 * i]) - d * static_cast<double>(P_pred_tmp);
    F_tmp = 3 * i + 1;
    F[F_tmp] = static_cast<double>(b_I[F_tmp]) -
               avg_yaw * static_cast<double>(P_pred_tmp);
    F_tmp = 3 * i + 2;
    F[F_tmp] =
        static_cast<double>(b_I[F_tmp]) - V * static_cast<double>(P_pred_tmp);
  }
  for (int i = 0; i < 3; i++) {
    d = F[i];
    avg_yaw = F[i + 3];
    V = F[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      P_new[i + 3 * i1] = (d * P_pred[3 * i1] + avg_yaw * P_pred[3 * i1 + 1]) +
                          V * P_pred[3 * i1 + 2];
    }
  }
}

// End of code generation (XY_EKF.cpp)
