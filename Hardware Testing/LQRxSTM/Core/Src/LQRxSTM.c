/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LQRxSTM.c
 *
 * Code generated for Simulink model 'LQRxSTM'.
 *
 * Model version                  : 1.27
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu May  7 18:37:17 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "LQRxSTM.h"
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
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
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
void LQRxSTM_step(void)
{
  real_T tmp[12];
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

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/P'
   *  Inport: '<Root>/x_robot'
   *  Inport: '<Root>/y_robot'
   */
  u_des_idx_0 = (rtInf);
  k_min = 0;
  for (k_next = 0; k_next < 100; k_next++) {
    u_des_idx_1 = rtU.P_b[k_next] - rtU.x_robot;
    dy = rtU.P_b[k_next + 100] - rtU.y_robot;
    u_des_idx_1 = u_des_idx_1 * u_des_idx_1 + dy * dy;
    if (u_des_idx_1 < u_des_idx_0) {
      u_des_idx_0 = u_des_idx_1;
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
  u_des_idx_0 = rtU.P_b[y - 1] - rtU.x_robot;
  u_des_idx_1 = rtU.P_b[y + 99] - rtU.y_robot;
  dy = rtU.P_b[y_0 - 1] - rtU.x_robot;
  d_a = rtU.P_b[y_0 + 99] - rtU.y_robot;
  if (u_des_idx_0 * u_des_idx_0 + u_des_idx_1 * u_des_idx_1 < dy * dy + d_a *
      d_a) {
    k_prev = k_min;
  } else {
    k_next = k_min;
  }

  if (k_prev == k_next) {
    u_des_idx_0 = 0.0;
  } else {
    u_des_idx_0 = rtU.P_b[k_next] - rtU.P_b[k_prev];
    d_a = rtU.P_b[k_prev + 100];
    u_des_idx_1 = rtU.P_b[k_next + 100] - d_a;
    dy = u_des_idx_0 * u_des_idx_0 + u_des_idx_1 * u_des_idx_1;
    if (dy > 1.0E-6) {
      u_des_idx_0 = fmax(0.0, fmin(1.0, ((rtU.y_robot - d_a) * u_des_idx_1 +
        (rtU.x_robot - rtU.P_b[k_prev]) * u_des_idx_0) / dy));
    } else {
      u_des_idx_0 = 0.0;
    }
  }

  /* MATLAB Function: '<Root>/CombinedLQR' incorporates:
   *  Constant: '<Root>/K_lqr'
   *  Inport: '<Root>/P'
   *  Inport: '<Root>/S_path'
   *  Inport: '<Root>/omega'
   *  Inport: '<Root>/phi'
   *  Inport: '<Root>/phi_dot'
   *  Inport: '<Root>/s'
   *  Inport: '<Root>/theta'
   *  Inport: '<Root>/theta_ref_unwrapped'
   *  Inport: '<Root>/v'
   *  Inport: '<Root>/vd_path'
   *  Inport: '<Root>/wd_path'
   *  Inport: '<Root>/x_robot'
   *  Inport: '<Root>/y_robot'
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  b[0] = rtU.phi;
  b[3] = rtU.phi_dot;
  u_des_idx_1 = ((((1.0 - u_des_idx_0) * rtU.P_b[k_prev] + u_des_idx_0 *
                   rtU.P_b[k_next]) - rtU.x_robot) * -sin(rtU.theta) + (((1.0 -
    u_des_idx_0) * rtU.P_b[k_prev + 100] + rtU.P_b[k_next + 100] * u_des_idx_0)
    - rtU.y_robot) * cos(rtU.theta)) * 0.1;
  if (u_des_idx_1 > 0.3490658503988659) {
    u_des_idx_1 = 0.3490658503988659;
  } else if (u_des_idx_1 < -0.3490658503988659) {
    u_des_idx_1 = -0.3490658503988659;
  }

  b[1] = rtU.s - ((1.0 - u_des_idx_0) * rtU.S_path[k_prev] + u_des_idx_0 *
                  rtU.S_path[k_next]);
  u_des_idx_1 = rtU.theta - (((1.0 - u_des_idx_0) *
    rtU.theta_ref_unwrapped[k_prev] + u_des_idx_0 *
    rtU.theta_ref_unwrapped[k_next]) + u_des_idx_1);
  b[2] = rt_atan2d_snf(sin(u_des_idx_1), cos(u_des_idx_1));
  b[4] = rtU.v - fmax(0.0, fmin((1.0 - u_des_idx_0) * rtU.vd_path[k_prev] +
    u_des_idx_0 * rtU.vd_path[k_next], 0.3));
  b[5] = rtU.omega - fmax(fmin((1.0 - u_des_idx_0) * rtU.wd_path[k_prev] +
    u_des_idx_0 * rtU.wd_path[k_next], 0.334), -0.334);
  for (k_prev = 0; k_prev < 12; k_prev++) {
    tmp[k_prev] = -rtConstP.K_lqr_Value[k_prev];
  }

  u_des_idx_0 = 0.0;
  u_des_idx_1 = 0.0;
  for (k_prev = 0; k_prev < 6; k_prev++) {
    dy = b[k_prev];
    k_min = k_prev << 1;
    u_des_idx_0 += tmp[k_min] * dy;
    u_des_idx_1 += tmp[k_min + 1] * dy;
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
   *  Inport: '<Root>/omega'
   *  Inport: '<Root>/v'
   *  MATLAB Function: '<Root>/CombinedLQR'
   */
  dy = rtU.omega * 0.45 / 2.0;
  k_min = (int32_T)fmax(fmin(rt_roundd_snf(((dy + rtU.v) / 0.06 * 0.6746 +
    u_des_idx_0 / 0.6746 * 0.9796) / 24.0 * 1000.0), 1000.0), -1000.0);
  k_next = (int32_T)fmax(fmin(rt_roundd_snf(((rtU.v - dy) / 0.06 * 0.6746 +
    u_des_idx_1 / 0.6746 * 0.9796) / 24.0 * 1000.0), 1000.0), -1000.0);
  if (k_min >= 0) {
    /* Outport: '<Root>/RPWM_R' */
    rtY.RPWM_R = k_min;

    /* Outport: '<Root>/LPWM_R' */
    rtY.LPWM_R = 0.0;
  } else {
    /* Outport: '<Root>/RPWM_R' */
    rtY.RPWM_R = 0.0;

    /* Outport: '<Root>/LPWM_R' */
    rtY.LPWM_R = fabs((real_T)k_min);
  }

  if (k_next >= 0) {
    /* Outport: '<Root>/RPWM_L' */
    rtY.RPWM_L = k_next;

    /* Outport: '<Root>/LPWM_L' */
    rtY.LPWM_L = 0.0;
  } else {
    /* Outport: '<Root>/RPWM_L' */
    rtY.RPWM_L = 0.0;

    /* Outport: '<Root>/LPWM_L' */
    rtY.LPWM_L = fabs((real_T)k_next);
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function1' */
}

/* Model initialize function */
void LQRxSTM_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
