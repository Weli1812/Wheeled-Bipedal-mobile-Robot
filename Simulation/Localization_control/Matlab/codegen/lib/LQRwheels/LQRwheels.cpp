//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// LQRwheels.cpp
//
// Code generation for function 'LQRwheels'
//

// Include files
#include "LQRwheels.h"
#include "LQRwheels_data.h"
#include "LQRwheels_initialize.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include <math.h>

// Variable Definitions
static double int_R;

static double int_L;

// Function Definitions
void LQRwheels(double x_curr, double y_curr, double theta_curr, double dt,
               double xd_k, double yd_k, double thetad_k, double vd_k,
               double wd_k, double vr_actual, double vl_actual, double *pwmr,
               double *pwml, boolean_T *dirr, boolean_T *dirl)
{
  static const double a[6] = {-2.2361, -0.0, -0.0, -4.0825, -0.0, -2.6535};
  double u[2];
  double dx;
  double dy;
  double err_R;
  double topwml_pre;
  double topwmr;
  double topwmr_pre;
  if (!isInitialized_LQRwheels) {
    LQRwheels_initialize();
  }
  //  lqr_pid_controller Calculates motor PWM commands for a differential drive
  //  robot Inputs:
  //    x_curr, y_curr, theta_curr : Current estimated pose of the robot
  //    xd_k, yd_k, thetad_k       : Desired reference pose
  //    vd_k, wd_k                 : Desired reference velocities
  //    vr_actual, vl_actual       : Actual wheel speeds from encoders
  //  Outputs:
  //    pwmr, pwml                 : PWM duty cycles (0.0 to 1.0)
  //    dirr, dirl                 : Direction booleans (0 = forward, 1 =
  //    backward)
  //     %% 1. Parameters
  //  wheelbase [m]
  //  Sample time [s]
  //  Saturation limit
  //  LQR Gain Matrix
  //  PID Parameters
  //     %% 2. State Memory (Integrators)
  //  Persistent variables keep their values between function calls
  //  Initialize integrators on the first run
  //     %% 3. Error Calculation (Kinematics)
  dx = x_curr - xd_k;
  dy = y_curr - yd_k;
  topwmr_pre = sin(thetad_k);
  topwml_pre = cos(thetad_k);
  //     %% 4. LQR Control Law
  err_R = topwml_pre * dx + topwmr_pre * dy;
  dy = -topwmr_pre * dx + topwml_pre * dy;
  dx = theta_curr - thetad_k;
  dx = coder::b_atan2(sin(dx), cos(dx));
  for (int i = 0; i < 2; i++) {
    u[i] = (a[i] * err_R + a[i + 2] * dy) + a[i + 4] * dx;
  }
  //     %% 5. Velocity Superposition & Inverse Kinematics
  dx = vd_k + u[0];
  dy = wd_k + u[1];
  //     %% 6. Motor PID Controllers with Clamping Anti-Windup
  err_R = (dx + 0.075 * dy) - vr_actual;
  dy = (dx - 0.075 * dy) - vl_actual;
  //  Proportional + Integral Calculation
  topwmr_pre = 5.0 * err_R + 0.07 * int_R;
  topwml_pre = 5.0 * dy + 0.07 * int_L;
  //  Saturation Clamping
  if (topwmr_pre <= 5.0) {
    dx = topwmr_pre;
  } else {
    dx = 5.0;
  }
  if (dx >= -5.0) {
    topwmr = dx;
  } else {
    topwmr = -5.0;
  }
  if (topwml_pre <= 5.0) {
    dx = topwml_pre;
  } else {
    dx = 5.0;
  }
  if (!(dx >= -5.0)) {
    dx = -5.0;
  }
  //  Anti-Windup Logic: Only integrate if not pushing further into saturation
  if (((!(topwmr_pre > 5.0)) || (!(err_R > 0.0))) &&
      ((!(topwmr_pre < -5.0)) || (!(err_R < 0.0)))) {
    int_R += err_R * dt;
  }
  if (((!(topwml_pre > 5.0)) || (!(dy > 0.0))) &&
      ((!(topwml_pre < -5.0)) || (!(dy < 0.0)))) {
    int_L += dy * dt;
  }
  //     %% 7. Outputs Mapping
  //  Map absolute PWM to [0, 1] scale
  *pwmr = fabs(topwmr) / 5.0;
  *pwml = fabs(dx) / 5.0;
  //  Direction: 1 if commanded voltage is negative, 0 otherwise
  *dirr = (topwmr < 0.0);
  *dirl = (dx < 0.0);
}

void LQRwheels_init()
{
  int_R = 0.0;
  int_L = 0.0;
}

// End of code generation (LQRwheels.cpp)
