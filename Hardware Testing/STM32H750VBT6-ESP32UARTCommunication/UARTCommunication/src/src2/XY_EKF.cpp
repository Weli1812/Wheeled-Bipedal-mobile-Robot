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
void XY_EKF(const float X[3], const float P[9], float v_R, float v_L, float Z,
            float dt, float L, float X_new[3], float P_new[9])
{
  static const float fv[9] = {0.01F, 0.0F, 0.0F, 0.0F, 0.01F,
                              0.0F,  0.0F, 0.0F, 0.01F};
  static const signed char iv2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char b_a[3] = {0, 0, 1};
  static const signed char iv[3] = {0, 0, 1};
  static const signed char iv1[3] = {0, 0, 1};
  double X_pred[3];
  double a;
  float F[9];
  float P_pred[9];
  float b_F[9];
  float K[3];
  float V;
  float X_pred_tmp;
  float avg_yaw;
  float avg_yaw_tmp;
  float f;
  int F_tmp;
  int P_pred_tmp;
  int i1;
  if (!isInitialized_XY_EKF) {
    XY_EKF_initialize();
  }
  //  --- Tuning Matrices (Explicitly defined for C++ safety) ---
  //  --- Differential Kinematics ---
  V = (v_R + v_L) / 2.0F;
  //  --- STEP 1: PREDICTION ---
  avg_yaw_tmp = (v_R - v_L) / L * dt;
  avg_yaw = X[2] + avg_yaw_tmp / 2.0F;
  X_pred_tmp = V * static_cast<float>(cos(static_cast<double>(avg_yaw))) * dt;
  X_pred[0] = X[0] + X_pred_tmp;
  f = static_cast<float>(sin(static_cast<double>(avg_yaw)));
  X_pred[1] = X[1] + V * f * dt;
  X_pred[2] = X[2] + avg_yaw_tmp;
  F[0] = 1.0F;
  F[3] = 0.0F;
  F[6] = -V * f * dt;
  F[1] = 0.0F;
  F[4] = 1.0F;
  F[7] = X_pred_tmp;
  F[2] = 0.0F;
  F[5] = 0.0F;
  F[8] = 1.0F;
  //  --- STEP 2: UPDATE ---
  a = 0.0;
  for (int i = 0; i < 3; i++) {
    i1 = static_cast<int>(F[i]);
    P_pred_tmp = static_cast<int>(F[i + 3]);
    f = F[i + 6];
    for (F_tmp = 0; F_tmp < 3; F_tmp++) {
      b_F[i + 3 * F_tmp] = (static_cast<float>(i1) * P[3 * F_tmp] +
                            static_cast<float>(P_pred_tmp) * P[3 * F_tmp + 1]) +
                           f * P[3 * F_tmp + 2];
    }
    f = b_F[i];
    avg_yaw = b_F[i + 3];
    V = b_F[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      P_pred_tmp = i + 3 * i1;
      P_pred[P_pred_tmp] =
          ((f * F[i1] + avg_yaw * F[i1 + 3]) + V * F[i1 + 6]) + fv[P_pred_tmp];
    }
    a += static_cast<double>(b_a[i]) * X_pred[i];
  }
  avg_yaw = Z - static_cast<float>(a);
  avg_yaw =
      coder::b_atan2(static_cast<float>(sin(static_cast<double>(avg_yaw))),
                     static_cast<float>(cos(static_cast<double>(avg_yaw))));
  //  Safe angle wrap
  V = 0.0F;
  for (int i = 0; i < 3; i++) {
    V += ((0.0F * P_pred[3 * i] + 0.0F * P_pred[3 * i + 1]) +
          P_pred[3 * i + 2]) *
         static_cast<float>(iv[i]);
  }
  for (P_pred_tmp = 0; P_pred_tmp < 3; P_pred_tmp++) {
    f = ((P_pred[P_pred_tmp] * 0.0F + P_pred[P_pred_tmp + 3] * 0.0F) +
         P_pred[P_pred_tmp + 6]) /
        (V + 0.05F);
    K[P_pred_tmp] = f;
    X_new[P_pred_tmp] = static_cast<float>(X_pred[P_pred_tmp]) + f * avg_yaw;
  }
  f = K[0];
  avg_yaw = K[1];
  V = K[2];
  for (int i = 0; i < 3; i++) {
    P_pred_tmp = iv1[i];
    F[3 * i] =
        static_cast<float>(iv2[3 * i]) - f * static_cast<float>(P_pred_tmp);
    F_tmp = 3 * i + 1;
    F[F_tmp] = static_cast<float>(iv2[F_tmp]) -
               avg_yaw * static_cast<float>(P_pred_tmp);
    F_tmp = 3 * i + 2;
    F[F_tmp] =
        static_cast<float>(iv2[F_tmp]) - V * static_cast<float>(P_pred_tmp);
  }
  for (int i = 0; i < 3; i++) {
    f = F[i];
    avg_yaw = F[i + 3];
    V = F[i + 6];
    for (i1 = 0; i1 < 3; i1++) {
      P_new[i + 3 * i1] = (f * P_pred[3 * i1] + avg_yaw * P_pred[3 * i1 + 1]) +
                          V * P_pred[3 * i1 + 2];
    }
  }
}

// End of code generation (XY_EKF.cpp)
