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
void attitude_EKF(double ax, double ay, double az, double gx, double gy,
                  double gz, double mx, double my, double mz, double dt,
                  const double x_in[7], const double P_in[49],
                  const double Q_diag[7], const double R_diag[6],
                  const double b_mag[3], const double A_mag[9],
                  double *ekf_roll, double *ekf_pitch, double *ekf_yaw,
                  double x_out[7], double P_out[49])
{
  static const signed char b_I[49] = {
      1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1};
  double F_mat[49];
  double P_pred[49];
  double b_F_mat[49];
  double dv[49];
  double H_mat[42];
  double K[42];
  double b_H_mat[42];
  double y_tmp[42];
  double c_H_mat[36];
  double dv1[36];
  double a[16];
  double x_pred[7];
  double c_ax[6];
  double b_ax[3];
  double cal_mag[3];
  double F_mat_tmp;
  double a_tmp;
  double b_F_mat_tmp;
  double b_a_tmp;
  double c_a_tmp;
  double mx_idx_0;
  double n_a;
  double n_m;
  double wx;
  double wy;
  double wz;
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
  mx_idx_0 = mx - b_mag[0];
  n_m = my - b_mag[1];
  n_a = mz - b_mag[2];
  for (int i = 0; i < 3; i++) {
    cal_mag[i] =
        (A_mag[i] * mx_idx_0 + A_mag[i + 3] * n_m) + A_mag[i + 6] * n_a;
  }
  mx = cal_mag[0];
  my = cal_mag[1];
  mz = cal_mag[2];
  //  Normalize Accelerometer and Magnetometer vectors
  b_ax[0] = ax;
  b_ax[1] = ay;
  b_ax[2] = az;
  n_a = coder::b_norm(b_ax);
  if (n_a > 0.0) {
    ax /= n_a;
    ay /= n_a;
    az /= n_a;
  }
  b_ax[0] = cal_mag[0];
  b_ax[1] = cal_mag[1];
  b_ax[2] = cal_mag[2];
  n_m = coder::b_norm(b_ax);
  if (n_m > 0.0) {
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
  n_a = 0.5 * dt;
  a[0] = n_a * 0.0;
  n_m = n_a * -wx;
  a[4] = n_m;
  a_tmp = n_a * -wy;
  a[8] = a_tmp;
  mx_idx_0 = n_a * -wz;
  a[12] = mx_idx_0;
  b_a_tmp = n_a * wx;
  a[1] = b_a_tmp;
  a[5] = n_a * 0.0;
  c_a_tmp = n_a * wz;
  a[9] = c_a_tmp;
  a[13] = a_tmp;
  a_tmp = n_a * wy;
  a[2] = a_tmp;
  a[6] = mx_idx_0;
  a[10] = n_a * 0.0;
  a[14] = b_a_tmp;
  a[3] = c_a_tmp;
  a[7] = a_tmp;
  a[11] = n_m;
  a[15] = n_a * 0.0;
  for (int i = 0; i < 4; i++) {
    x_pred[i] = x_in[i] +
                (((a[i] * x_in[0] + a[i + 4] * x_in[1]) + a[i + 8] * x_in[2]) +
                 a[i + 12] * x_in[3]);
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
  F_mat[0] = 1.0;
  F_mat_tmp = -0.5 * dt * wx;
  F_mat[7] = F_mat_tmp;
  b_F_mat_tmp = -0.5 * dt * wy;
  F_mat[14] = b_F_mat_tmp;
  n_a = -0.5 * dt * wz;
  F_mat[21] = n_a;
  n_m = 0.5 * dt * x_pred[1];
  F_mat[28] = n_m;
  mx_idx_0 = 0.5 * dt * x_pred[2];
  F_mat[35] = mx_idx_0;
  wx = 0.5 * dt * x_pred[3];
  F_mat[42] = wx;
  F_mat[1] = b_a_tmp;
  F_mat[8] = 1.0;
  F_mat[15] = c_a_tmp;
  F_mat[22] = b_F_mat_tmp;
  b_F_mat_tmp = -0.5 * dt * x_pred[0];
  F_mat[29] = b_F_mat_tmp;
  F_mat[36] = wx;
  F_mat[43] = -0.5 * dt * x_pred[2];
  F_mat[2] = a_tmp;
  F_mat[9] = n_a;
  F_mat[16] = 1.0;
  F_mat[23] = b_a_tmp;
  F_mat[30] = -0.5 * dt * x_pred[3];
  F_mat[37] = b_F_mat_tmp;
  F_mat[44] = n_m;
  F_mat[3] = c_a_tmp;
  F_mat[10] = a_tmp;
  F_mat[17] = F_mat_tmp;
  F_mat[24] = 1.0;
  F_mat[31] = mx_idx_0;
  F_mat[38] = -0.5 * dt * x_pred[1];
  F_mat[45] = b_F_mat_tmp;
  coder::diag(Q_diag, dv);
  for (int i = 0; i < 7; i++) {
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += F_mat[i + 7 * b_i] * P_in[b_i + 7 * i1];
      }
      b_F_mat[i + 7 * i1] = n_a;
    }
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += b_F_mat[i + 7 * b_i] * F_mat[i1 + 7 * b_i];
      }
      b_i = i + 7 * i1;
      P_pred[b_i] = n_a + dv[b_i];
    }
  }
  //  =====================================================
  //  4. EKF UPDATE STEP
  //  =====================================================
  memset(&H_mat[0], 0, 42U * sizeof(double));
  H_mat[0] = -2.0 * x_pred[2];
  H_mat[6] = 2.0 * x_pred[3];
  H_mat[12] = -2.0 * x_pred[0];
  H_mat[18] = 2.0 * x_pred[1];
  H_mat[1] = 2.0 * x_pred[1];
  H_mat[7] = 2.0 * x_pred[0];
  H_mat[13] = 2.0 * x_pred[3];
  H_mat[19] = 2.0 * x_pred[2];
  H_mat[2] = 2.0 * x_pred[0];
  H_mat[8] = -2.0 * x_pred[1];
  H_mat[14] = -2.0 * x_pred[2];
  H_mat[20] = 2.0 * x_pred[3];
  n_a = 1.448 * x_pred[0] - 1.38 * x_pred[2];
  H_mat[3] = n_a;
  n_m = 1.448 * x_pred[1] + 1.38 * x_pred[3];
  H_mat[9] = n_m;
  H_mat[15] = -1.448 * x_pred[2] - 1.38 * x_pred[0];
  mx_idx_0 = -1.448 * x_pred[3] + 1.38 * x_pred[1];
  H_mat[21] = mx_idx_0;
  H_mat[4] = mx_idx_0;
  mx_idx_0 = 1.448 * x_pred[2] + 1.38 * x_pred[0];
  H_mat[10] = mx_idx_0;
  H_mat[16] = n_m;
  H_mat[22] = -1.448 * x_pred[0] + 1.38 * x_pred[2];
  H_mat[5] = mx_idx_0;
  H_mat[11] = 1.448 * x_pred[3] - 1.38 * x_pred[1];
  H_mat[17] = n_a;
  H_mat[23] = n_m;
  for (int i = 0; i < 6; i++) {
    for (int i1 = 0; i1 < 7; i1++) {
      y_tmp[i1 + 7 * i] = H_mat[i + 6 * i1];
    }
  }
  coder::b_diag(R_diag, dv1);
  for (int i = 0; i < 7; i++) {
    for (int i1 = 0; i1 < 6; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += P_pred[i + 7 * b_i] * y_tmp[b_i + 7 * i1];
      }
      K[i + 7 * i1] = n_a;
    }
  }
  for (int i = 0; i < 6; i++) {
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += H_mat[i + 6 * b_i] * P_pred[b_i + 7 * i1];
      }
      b_H_mat[i + 6 * i1] = n_a;
    }
    for (int i1 = 0; i1 < 6; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += b_H_mat[i + 6 * b_i] * y_tmp[b_i + 7 * i1];
      }
      b_i = i + 6 * i1;
      c_H_mat[b_i] = n_a + dv1[b_i];
    }
  }
  coder::internal::mrdiv(K, c_H_mat);
  n_a = x_pred[1] * x_pred[3];
  n_m = x_pred[0] * x_pred[2];
  mx_idx_0 = 2.0 * (n_a - n_m);
  c_ax[0] = ax - mx_idx_0;
  wx = 2.0 * (x_pred[0] * x_pred[1] + x_pred[2] * x_pred[3]);
  c_ax[1] = ay - wx;
  b_F_mat_tmp = x_pred[0] * x_pred[0];
  wy = x_pred[1] * x_pred[1];
  F_mat_tmp = x_pred[2] * x_pred[2];
  wz = x_pred[3] * x_pred[3];
  a_tmp = ((b_F_mat_tmp - wy) - F_mat_tmp) + wz;
  c_ax[2] = az - a_tmp;
  c_ax[3] =
      mx - (0.724 * (((b_F_mat_tmp + wy) - F_mat_tmp) - wz) + 0.69 * mx_idx_0);
  c_ax[4] =
      my - (0.724 * (2.0 * (x_pred[1] * x_pred[2] - x_pred[0] * x_pred[3])) +
            0.69 * wx);
  c_ax[5] = mz - (0.724 * (2.0 * (n_a + n_m)) + 0.69 * a_tmp);
  for (int i = 0; i < 7; i++) {
    n_a = 0.0;
    for (int i1 = 0; i1 < 6; i1++) {
      n_a += K[i + 7 * i1] * c_ax[i1];
    }
    x_out[i] = x_pred[i] + n_a;
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 6; b_i++) {
        n_a += K[i + 7 * b_i] * H_mat[b_i + 6 * i1];
      }
      b_i = i + 7 * i1;
      F_mat[b_i] = static_cast<double>(b_I[b_i]) - n_a;
    }
    for (int i1 = 0; i1 < 7; i1++) {
      n_a = 0.0;
      for (b_i = 0; b_i < 7; b_i++) {
        n_a += F_mat[i + 7 * b_i] * P_pred[b_i + 7 * i1];
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
  *ekf_roll = coder::b_atan2(2.0 * (x_out[0] * x_out[1] + x_out[2] * x_out[3]),
                             1.0 - 2.0 * (x_out[1] * x_out[1] + n_m)) *
              180.0 / 3.1415926535897931;
  n_a = 2.0 * (x_out[0] * x_out[2] - x_out[1] * x_out[3]);
  if (!(n_a <= 1.0)) {
    n_a = 1.0;
  }
  if (!(n_a >= -1.0)) {
    n_a = -1.0;
  }
  *ekf_pitch = asin(n_a) * 180.0 / 3.1415926535897931;
  *ekf_yaw = coder::b_atan2(2.0 * (x_out[0] * x_out[3] + x_out[1] * x_out[2]),
                            1.0 - 2.0 * (n_m + x_out[3] * x_out[3])) *
             180.0 / 3.1415926535897931;
}

// End of code generation (attitude_EKF.cpp)
