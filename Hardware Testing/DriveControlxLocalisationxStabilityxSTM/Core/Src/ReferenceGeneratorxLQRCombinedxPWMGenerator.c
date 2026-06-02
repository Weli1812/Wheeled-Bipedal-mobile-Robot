/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ReferenceGeneratorxLQRCombinedxPWMGenerator.c
 *
 * Code generated for Simulink model 'ReferenceGeneratorxLQRCombinedxPWMGenerator'.
 *
 * Model version                  : 1.30
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sun May 31 04:40:19 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 * 1. Execution efficiency
 * 2. RAM efficiency
 * Validation result: Not run
 */

#include "ReferenceGeneratorxLQRCombinedxPWMGenerator.h"
#include <math.h>
#include "rtwtypes.h"
#include "math.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_atan2d_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 * Used to specify that a function parameter (argument) is required but not
 * accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Return rtInf needed by the generated code. */
static real_T rtGetInf(void)
{
  return rtInf;
}

/* Get rtInfF needed by the generated code. */
static real32_T rtGetInfF(void)
{
  return rtInfF;
}

/* Return rtMinusInf needed by the generated code. */
static real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

/* Return rtMinusInfF needed by the generated code. */
static real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
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

/* Model step function */
void ReferenceGeneratorxLQRCombinedxPWMGenerator_step(void)
{
  real_T b[6];
  real_T d_a;
  real_T dy;
  real_T u_des_idx_0;
  real_T u_des_idx_1;
  int32_T k_min;
  int32_T k_next;
  int32_T k_prev;
  int32_T y;
  int32_T y_0;

  // FIX: Padded the end of the X array with 45.0 instead of 0.0
  // Path arrays updated to match CSV (100 points, 0.0 to 4.95m)
    static const real_T b_0[200] = {
      // X Coordinates (0.00m to 4.95m)
      0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
      0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95,
      1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45,
      1.50, 1.55, 1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95,
      2.00, 2.05, 2.10, 2.15, 2.20, 2.25, 2.30, 2.35, 2.40, 2.45,
      2.50, 2.55, 2.60, 2.65, 2.70, 2.75, 2.80, 2.85, 2.90, 2.95,
      3.00, 3.05, 3.10, 3.15, 3.20, 3.25, 3.30, 3.35, 3.40, 3.45,
      3.50, 3.55, 3.60, 3.65, 3.70, 3.75, 3.80, 3.85, 3.90, 3.95,
      4.00, 4.05, 4.10, 4.15, 4.20, 4.25, 4.30, 4.35, 4.40, 4.45,
      4.50, 4.55, 4.60, 4.65, 4.70, 4.75, 4.80, 4.85, 4.90, 4.95,

      // Y Coordinates (100 points, all 0.00m)
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
      0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00
    };

    static const real_T c[100] = {
      // S_Path (matches X distance)
      0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45,
      0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95,
      1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45,
      1.50, 1.55, 1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95,
      2.00, 2.05, 2.10, 2.15, 2.20, 2.25, 2.30, 2.35, 2.40, 2.45,
      2.50, 2.55, 2.60, 2.65, 2.70, 2.75, 2.80, 2.85, 2.90, 2.95,
      3.00, 3.05, 3.10, 3.15, 3.20, 3.25, 3.30, 3.35, 3.40, 3.45,
      3.50, 3.55, 3.60, 3.65, 3.70, 3.75, 3.80, 3.85, 3.90, 3.95,
      4.00, 4.05, 4.10, 4.15, 4.20, 4.25, 4.30, 4.35, 4.40, 4.45,
      4.50, 4.55, 4.60, 4.65, 4.70, 4.75, 4.80, 4.85, 4.90, 4.95
    };

    static const real_T a[12] = { // K matrix (LQR Gains) - REVERSED POLARITY
          // Format: { Right_Wheel_Gain, Left_Wheel_Gain }

         -5.616882069954595, -5.6168820699546,    // State 1: Pitch Angle Error (e_phi)
         -0.6180339887498989,-0.6180339887499,    // State 2: Path Displacement Error (e_s)
          2.762277660168384, -2.762277660168376,  // State 3: Heading Error (e_theta)
         -1.238330914554923, -1.2383309145549221, // State 4: Pitch Angular Rate (phi_dot)
         -2.0613551855486412,-2.0613551855486438, // State 5: Forward Velocity Error (e_v)
          1.622468529505869, -1.6224685295058661  // State 6: Yaw Rate (omega)
        };

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   * Inport: '<Root>/x_robot'
   * Inport: '<Root>/y_robot'
   */
  u_des_idx_1 = (rtInf);
  k_min = 0;
  for (k_next = 0; k_next < 100; k_next++) {
    u_des_idx_0 = b_0[k_next] - rtU.x_robot;
    dy = b_0[k_next + 100] - rtU.y_robot;
    u_des_idx_0 = u_des_idx_0 * u_des_idx_0 + dy * dy;
    if (u_des_idx_0 < u_des_idx_1) {
      u_des_idx_1 = u_des_idx_0;
      k_min = k_next;
    }
  }

  if (k_min + 2 <= 100) {
    y = k_min + 2;
    k_prev = k_min + 2;
  } else {
    y = 100;
    k_prev = 100;
  }

  k_next = k_prev - 1;
  if (k_min >= 1) {
    y_0 = k_min;
    k_prev = k_min;
  } else {
    y_0 = 1;
    k_prev = 1;
  }

  k_prev--;
  u_des_idx_1 = b_0[y - 1] - rtU.x_robot;
  u_des_idx_0 = b_0[y + 99] - rtU.y_robot;
  dy = b_0[y_0 - 1] - rtU.x_robot;
  d_a = b_0[y_0 + 99] - rtU.y_robot;
  if (u_des_idx_1 * u_des_idx_1 + u_des_idx_0 * u_des_idx_0 < dy * dy + d_a *
      d_a) {
    k_prev = k_min;
  } else {
    k_next = k_min;
  }

  if (k_prev == k_next) {
    u_des_idx_1 = 0.0;
  } else {
    u_des_idx_1 = b_0[k_next] - b_0[k_prev];
    d_a = b_0[k_prev + 100];
    u_des_idx_0 = b_0[k_next + 100] - d_a;
    dy = u_des_idx_1 * u_des_idx_1 + u_des_idx_0 * u_des_idx_0;
    if (dy > 1.0E-6) {
      u_des_idx_1 = fmax(0.0, fmin(1.0, ((rtU.y_robot - d_a) * u_des_idx_0 +
        (rtU.x_robot - b_0[k_prev]) * u_des_idx_1) / dy));
    } else {
      u_des_idx_1 = 0.0;
    }
  }

  /* MATLAB Function: '<Root>/CombinedLQR' incorporates:
   * Inport: '<Root>/omega'
   * Inport: '<Root>/phi'
   * Inport: '<Root>/phi_dot'
   * Inport: '<Root>/s'
   * Inport: '<Root>/theta'
   * Inport: '<Root>/v'
   * Inport: '<Root>/x_robot'
   * Inport: '<Root>/y_robot'
   * MATLAB Function: '<Root>/MATLAB Function'
   */
  b[3] = rtU.phi_dot;
  u_des_idx_0 = ((((1.0 - u_des_idx_1) * b_0[k_prev] + u_des_idx_1 * b_0[k_next])
                  - rtU.x_robot) * -sin(rtU.theta) + (((1.0 - u_des_idx_1) *
    b_0[k_prev + 100] + b_0[k_next + 100] * u_des_idx_1) - rtU.y_robot) * cos
                 (rtU.theta)) * 0.1;
  if (u_des_idx_0 > 0.1396263) {
    u_des_idx_0 = 0.1396263;
  } else if (u_des_idx_0 < -0.1396263) {
    u_des_idx_0 = -0.1396263;
  }

  /* --- MANUAL DEADBAND INSERTION FOR PHI --- */
  {
    real_T raw_phi_error = rtU.phi - 0.0032767583132328297;
    real_T phi_threshold = 0.05; /* Adjust this value as needed */

    if (raw_phi_error > phi_threshold) {
      b[0] = raw_phi_error - phi_threshold;
    } else if (raw_phi_error < -phi_threshold) {
       b[0] = raw_phi_error + phi_threshold;
    } else {
      b[0] = 0.0;
    }
  }

  /* --- CUSTOM VELOCITY PROFILE (Spatial Ramp) --- */
  /* Determine where the robot is along the path (S_target) */
    real_T s_target = ((1.0 - u_des_idx_1) * c[k_prev] + u_des_idx_1 * c[k_next]);

    // 1. Calculate the raw spatial error (this provides your required 5cm pull)
    real_T raw_e_s = rtU.s - s_target;

    // 2. Smooth the introduction of the spatial error to prevent startup jerk
    static real_T smoothed_e_s = 0.0;

    // Filter coefficient. Lower = gentler startup tip. Higher = sharper response.
    real_T alpha_s = 0.02;

    smoothed_e_s = ((1.0 - alpha_s) * smoothed_e_s) + (alpha_s * raw_e_s);

    b[1] = smoothed_e_s; // Feed the smoothed spatial error to the LQR

    // 3. Keep velocity error standard, letting the spatial pull do the work
    b[4] = rtU.v - 0.15;
    /* ---------------------------------------------- */

  u_des_idx_0 = rtU.theta - u_des_idx_0;
  b[2] = rt_atan2d_snf(sin(u_des_idx_0), cos(u_des_idx_0));
  b[5] = rtU.omega;
  u_des_idx_0 = 0.0;
  u_des_idx_1 = 0.0;
  for (k_prev = 0; k_prev < 6; k_prev++) {
    dy = b[k_prev];
    k_min = k_prev << 1;
    u_des_idx_0 += a[k_min] * dy;
    u_des_idx_1 += a[k_min + 1] * dy;
  }

  if (u_des_idx_0 > 3.0) {
    u_des_idx_0 = 3.0;
  } else if (u_des_idx_0 < -3.0) {
    u_des_idx_0 = -3.0;
  }

  if (u_des_idx_1 > 3.0) {
    u_des_idx_1 = 3.0;
  } else if (u_des_idx_1 < -3.0) {
    u_des_idx_1 = -3.0;
  }

  /* MATLAB Function: '<Root>/MATLAB Function1' incorporates:
   * MATLAB Function: '<Root>/CombinedLQR'
   */
  u_des_idx_0 = rt_roundd_snf(u_des_idx_0 / 3 * 4000.0);
  u_des_idx_1 = rt_roundd_snf(u_des_idx_1 / 3 * 4000.0);
  if (u_des_idx_0 != 0.0) {
    if (rtIsNaN(u_des_idx_0)) {
      dy = (rtNaN);
    } else if (u_des_idx_0 < 0.0) {
      dy = -1.0;
    } else {
      dy = (u_des_idx_0 > 0.0);
    }

    u_des_idx_0 += dy * 130.0;
  }

  if (u_des_idx_1 != 0.0) {
    if (rtIsNaN(u_des_idx_1)) {
      dy = (rtNaN);
    } else if (u_des_idx_1 < 0.0) {
      dy = -1.0;
    } else {
      dy = (u_des_idx_1 > 0.0);
    }

    u_des_idx_1 += dy * 130.0;
  }

  u_des_idx_0 = fmax(fmin(u_des_idx_0, 4000.0), -4000.0);
  u_des_idx_1 = fmax(fmin(u_des_idx_1, 4000.0), -4000.0);

  /* Outport: '<Root>/RPWM_R' incorporates:
   * MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtY.RPWM_R = fmax(0.0, u_des_idx_0);

  /* Outport: '<Root>/LPWM_R' incorporates:
   * MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtY.LPWM_R = fmax(0.0, -u_des_idx_0);

  /* Outport: '<Root>/RPWM_L' incorporates:
   * MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtY.RPWM_L = fmax(0.0, u_des_idx_1);

  /* Outport: '<Root>/LPWM_L' incorporates:
   * MATLAB Function: '<Root>/MATLAB Function1'
   */
  rtY.LPWM_L = fmax(0.0, -u_des_idx_1);
}

/* Model initialize function */
void ReferenceGeneratorxLQRCombinedxPWMGenerator_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
