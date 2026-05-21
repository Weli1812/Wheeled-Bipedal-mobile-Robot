//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// attitude_EKF.cpp
//
// Code generation for function 'attitude_EKF'
//

// Include files
#include "attitude_EKF.h"
#include "atan2.h"
#include "attitude_EKF_data.h"
#include "attitude_EKF_initialize.h"
#include "diag.h"
#include "mrdivide_helper.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

// Function Definitions
void attitude_EKF(float ax, float ay, float az, float gx, float gy, float gz,
                  float mx, float my, float mz, float dt, const float x_in[7],
                  const float P_in[49], const float Q_diag[7],
                  const float R_diag[6], const float b_mag[3],
                  const float A_mag[9], float *ekf_roll, float *ekf_pitch,
                  float *ekf_yaw, float x_out[7], float P_out[49])
{
  static const signed char b_I[49] = {
      1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[49] = {
      1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1};
  double F_mat[49];
  double H_mat[42];
  double F_mat_tmp;
  double b_F_mat_tmp;
  double c_F_mat_tmp;
  float P_pred[49];
  float b_F_mat[49];
  float fv[49];
  float K[42];
  float b_H_mat[42];
  float y_tmp[42];
  float c_H_mat[36];
  float fv1[36];
  float b_x_pred_tmp[16];
  float x_pred[7];
  float c_ax[6];
  float b_ax[3];
  float cal_mag[3];
  float b_x_pred_tmp_tmp;
  float c_x_pred_tmp_tmp;
  float n_a;
  float n_m;
  float wx;
  float wy;
  float wz;
  float x_pred_tmp;
  float x_pred_tmp_tmp;
  int b_i;
  if (!isInitialized_attitude_EKF) {
    attitude_EKF_initialize();
  }
  //  This directive forces MATLAB to restrict this function to C++ compatible
  //  operations.
  //  =====================================================
  //  1. BUILD MATRICES FROM LIVE INPUTS
  //  =====================================================
  //  2. SENSOR CALIBRATION & NORMALIZATION
  //  =====================================================
  wx = mx - b_mag[0];
  n_m = my - b_mag[1];
  n_a = mz - b_mag[2];
  for (int i = 0; i < 3; i++) {
    cal_mag[i] = (A_mag[i] * wx + A_mag[i + 3] * n_m) + A_mag[i + 6] * n_a;
  }
  mx = cal_mag[0];
  my = cal_mag[1];
  mz = cal_mag[2];
  //  Normalize Accelerometer and Magnetometer vectors
  b_ax[0] = ax;
  b_ax[1] = ay;
  b_ax[2] = az;
  n_a = coder::b_norm(b_ax);
  if (n_a > 0.0F) {
    ax /= n_a;
    ay /= n_a;
    az /= n_a;
  }
  b_ax[0] = cal_mag[0];
  b_ax[1] = cal_mag[1];
  b_ax[2] = cal_mag[2];
  n_m = coder::b_norm(b_ax);
  if (n_m > 0.0F) {
    mx = cal_mag[0] / n_m;
    my = cal_mag[1] / n_m;
    mz = cal_mag[2] / n_m;
  }
  //  =====================================================
  //  3. EKF PREDICTION STEP
  //  =====================================================
  //  Extract bias to clean the gyro
  wx = gx - x_in[4];
  wy = gy - x_in[5];
  wz = gz - x_in[6];
  for (b_i = 0; b_i < 7; b_i++) {
    x_pred[b_i] = x_in[b_i];
  }
  x_pred_tmp = 0.5F * dt;
  b_x_pred_tmp[0] = x_pred_tmp * 0.0F;
  n_a = x_pred_tmp * -wx;
  b_x_pred_tmp[4] = n_a;
  x_pred_tmp_tmp = x_pred_tmp * -wy;
  b_x_pred_tmp[8] = x_pred_tmp_tmp;
  n_m = x_pred_tmp * -wz;
  b_x_pred_tmp[12] = n_m;
  b_x_pred_tmp_tmp = x_pred_tmp * wx;
  b_x_pred_tmp[1] = b_x_pred_tmp_tmp;
  b_x_pred_tmp[5] = x_pred_tmp * 0.0F;
  c_x_pred_tmp_tmp = x_pred_tmp * wz;
  b_x_pred_tmp[9] = c_x_pred_tmp_tmp;
  b_x_pred_tmp[13] = x_pred_tmp_tmp;
  x_pred_tmp_tmp = x_pred_tmp * wy;
  b_x_pred_tmp[2] = x_pred_tmp_tmp;
  b_x_pred_tmp[6] = n_m;
  b_x_pred_tmp[10] = x_pred_tmp * 0.0F;
  b_x_pred_tmp[14] = b_x_pred_tmp_tmp;
  b_x_pred_tmp[3] = c_x_pred_tmp_tmp;
  b_x_pred_tmp[7] = x_pred_tmp_tmp;
  b_x_pred_tmp[11] = n_a;
  b_x_pred_tmp[15] = x_pred_tmp * 0.0F;
  for (int i = 0; i < 4; i++) {
    x_pred[i] = x_in[i] +
                (((b_x_pred_tmp[i] * x_in[0] + b_x_pred_tmp[i + 4] * x_in[1]) +
                  b_x_pred_tmp[i + 8] * x_in[2]) +
                 b_x_pred_tmp[i + 12] * x_in[3]);
  }
  n_a = coder::c_norm(&x_pred[0]);
  x_pred[0] /= n_a;
  x_pred[1] /= n_a;
  x_pred[2] /= n_a;
  x_pred[3] /= n_a;
  //  Extract newly predicted quaternions to build F and H matrices
  for (int i = 0; i < 49; i++) {
    F_mat[i] = b_I[i];
  }
  double d_F_mat_tmp;
  double e_F_mat_tmp;
  double f_F_mat_tmp;
  F_mat[0] = 1.0;
  n_a = -0.5F * dt;
  F_mat_tmp = n_a * wx;
  F_mat[7] = F_mat_tmp;
  b_F_mat_tmp = n_a * wy;
  F_mat[14] = b_F_mat_tmp;
  c_F_mat_tmp = n_a * wz;
  F_mat[21] = c_F_mat_tmp;
  d_F_mat_tmp = x_pred_tmp * x_pred[1];
  F_mat[28] = d_F_mat_tmp;
  e_F_mat_tmp = x_pred_tmp * x_pred[2];
  F_mat[35] = e_F_mat_tmp;
  f_F_mat_tmp = x_pred_tmp * x_pred[3];
  F_mat[42] = f_F_mat_tmp;
  F_mat[1] = b_x_pred_tmp_tmp;
  F_mat[8] = 1.0;
  F_mat[15] = c_x_pred_tmp_tmp;
  F_mat[22] = b_F_mat_tmp;
  b_F_mat_tmp = n_a * x_pred[0];
  F_mat[29] = b_F_mat_tmp;
  F_mat[36] = f_F_mat_tmp;
  F_mat[43] = n_a * x_pred[2];
  F_mat[2] = x_pred_tmp_tmp;
  F_mat[9] = c_F_mat_tmp;
  F_mat[16] = 1.0;
  F_mat[23] = b_x_pred_tmp_tmp;
  F_mat[30] = n_a * x_pred[3];
  F_mat[37] = b_F_mat_tmp;
  F_mat[44] = d_F_mat_tmp;
  F_mat[3] = c_x_pred_tmp_tmp;
  F_mat[10] = x_pred_tmp_tmp;
  F_mat[17] = F_mat_tmp;
  F_mat[24] = 1.0;
  F_mat[31] = e_F_mat_tmp;
  F_mat[38] = n_a * x_pred[1];
  F_mat[45] = b_F_mat_tmp;
  coder::diag(Q_diag, fv);
  for (int i = 0; i < 7; i++) {
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += static_cast<float>(F_mat[i + 7 * b_i]) * P_in[b_i + 7 * i1];
      }
      b_F_mat[i + 7 * i1] = n_a;
    }
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += b_F_mat[i + 7 * b_i] * static_cast<float>(F_mat[i1 + 7 * b_i]);
      }
      b_i = i + 7 * i1;
      P_pred[b_i] = n_a + fv[b_i];
    }
  }
  //  =====================================================
  //  4. EKF UPDATE STEP
  //  =====================================================
  x_pred_tmp = x_pred[1] * x_pred[3];
  x_pred_tmp_tmp = x_pred[0] * x_pred[2];
  b_x_pred_tmp_tmp = 2.0F * (x_pred_tmp - x_pred_tmp_tmp);
  c_x_pred_tmp_tmp = 2.0F * (x_pred[0] * x_pred[1] + x_pred[2] * x_pred[3]);
  memset(&H_mat[0], 0, 42U * sizeof(double));
  n_a = -2.0F * x_pred[2];
  H_mat[0] = n_a;
  n_m = 2.0F * x_pred[3];
  H_mat[6] = n_m;
  H_mat[12] = -2.0F * x_pred[0];
  wx = 2.0F * x_pred[1];
  H_mat[18] = wx;
  H_mat[1] = wx;
  wx = 2.0F * x_pred[0];
  H_mat[7] = wx;
  H_mat[13] = n_m;
  H_mat[19] = 2.0F * x_pred[2];
  H_mat[2] = wx;
  H_mat[8] = -2.0F * x_pred[1];
  H_mat[14] = n_a;
  H_mat[20] = n_m;
  n_a = 1.38F * x_pred[2];
  F_mat_tmp = 1.448F * x_pred[0] - n_a;
  H_mat[3] = F_mat_tmp;
  b_F_mat_tmp = 1.448F * x_pred[1] + 1.38F * x_pred[3];
  H_mat[9] = b_F_mat_tmp;
  n_m = 1.38F * x_pred[0];
  H_mat[15] = -1.448F * x_pred[2] - n_m;
  wx = 1.38F * x_pred[1];
  c_F_mat_tmp = -1.448F * x_pred[3] + wx;
  H_mat[21] = c_F_mat_tmp;
  H_mat[4] = c_F_mat_tmp;
  c_F_mat_tmp = 1.448F * x_pred[2] + n_m;
  H_mat[10] = c_F_mat_tmp;
  H_mat[16] = b_F_mat_tmp;
  H_mat[22] = -1.448F * x_pred[0] + n_a;
  H_mat[5] = c_F_mat_tmp;
  H_mat[11] = 1.448F * x_pred[3] - wx;
  H_mat[17] = F_mat_tmp;
  H_mat[23] = b_F_mat_tmp;
  for (int i = 0; i < 6; i++) {
    for (int i1 = 0; i1 < 7; i1++) {
      y_tmp[i1 + 7 * i] = static_cast<float>(H_mat[i + 6 * i1]);
    }
  }
  coder::b_diag(R_diag, fv1);
  for (int i = 0; i < 7; i++) {
    for (int i1 = 0; i1 < 6; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += P_pred[i + 7 * b_i] * y_tmp[b_i + 7 * i1];
      }
      K[i + 7 * i1] = n_a;
    }
  }
  for (int i = 0; i < 6; i++) {
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += static_cast<float>(H_mat[i + 6 * b_i]) * P_pred[b_i + 7 * i1];
      }
      b_H_mat[i + 6 * i1] = n_a;
    }
    for (int i1 = 0; i1 < 6; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += b_H_mat[i + 6 * b_i] * y_tmp[b_i + 7 * i1];
      }
      b_i = i + 6 * i1;
      c_H_mat[b_i] = n_a + fv1[b_i];
    }
  }
  coder::internal::mrdiv(K, c_H_mat);
  c_ax[0] = ax - b_x_pred_tmp_tmp;
  c_ax[1] = ay - c_x_pred_tmp_tmp;
  n_a = x_pred[0] * x_pred[0];
  n_m = x_pred[1] * x_pred[1];
  wx = x_pred[2] * x_pred[2];
  wy = x_pred[3] * x_pred[3];
  wz = ((n_a - n_m) - wx) + wy;
  c_ax[2] = az - wz;
  c_ax[3] =
      mx - (0.724F * (((n_a + n_m) - wx) - wy) + 0.69F * b_x_pred_tmp_tmp);
  c_ax[4] =
      my - (0.724F * (2.0F * (x_pred[1] * x_pred[2] - x_pred[0] * x_pred[3])) +
            0.69F * c_x_pred_tmp_tmp);
  c_ax[5] = mz - (0.724F * (2.0F * (x_pred_tmp + x_pred_tmp_tmp)) + 0.69F * wz);
  for (int i = 0; i < 7; i++) {
    n_a = 0.0F;
    for (int i1 = 0; i1 < 6; i1++) {
      n_a += K[i + 7 * i1] * c_ax[i1];
    }
    x_out[i] = x_pred[i] + n_a;
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 6; b_i++) {
        n_a += K[i + 7 * b_i] * static_cast<float>(H_mat[b_i + 6 * i1]);
      }
      b_i = i + 7 * i1;
      fv[b_i] = static_cast<float>(iv[b_i]) - n_a;
    }
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0F;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += fv[i + 7 * b_i] * P_pred[b_i + 7 * i1];
      }
      P_out[i + 7 * i1] = n_a;
    }
  }
  n_a = coder::c_norm(&x_out[0]);
  x_out[0] /= n_a;
  x_out[1] /= n_a;
  x_out[2] /= n_a;
  x_out[3] /= n_a;
  //  =====================================================
  //  5. EULER ANGLE EXTRACTION
  //  =====================================================
  n_m = x_out[2] * x_out[2];
  *ekf_roll = coder::b_atan2(2.0F * (x_out[0] * x_out[1] + x_out[2] * x_out[3]),
                             1.0F - 2.0F * (x_out[1] * x_out[1] + n_m)) *
              180.0F / 3.14159274F;
  n_a = 2.0F * (x_out[0] * x_out[2] - x_out[1] * x_out[3]);
  if (!(n_a <= 1.0F)) {
    n_a = 1.0F;
  }
  if (!(n_a >= -1.0F)) {
    n_a = -1.0F;
  }
  *ekf_pitch =
      static_cast<float>(asin(static_cast<double>(n_a))) * 180.0F / 3.14159274F;
  *ekf_yaw = coder::b_atan2(2.0F * (x_out[0] * x_out[3] + x_out[1] * x_out[2]),
                            1.0F - 2.0F * (n_m + x_out[3] * x_out[3])) *
             180.0F / 3.14159274F;
}

// End of code generation (attitude_EKF.cpp)
