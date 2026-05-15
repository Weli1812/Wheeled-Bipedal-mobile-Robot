/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Simulink.c
 *
 * Code generated for Simulink model 'Simulink'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu May 14 02:49:46 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Simulink.h"
#include <math.h>
#include "rtwtypes.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_roundd_snf(real_T u);
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
void Simulink_step(void)
{
  real_T tmp;
  real_T u_idx_0;
  real_T u_idx_1;
  int32_T pwm_l;
  int32_T pwm_r;
  static const real_T a[12] = { 12.4395, 12.4395, 0.7071, 0.7071, -0.7071,
    0.7071, 2.9489, 2.9489, 1.3601, 1.3601, -0.7289, 0.7289 };

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/state_x'
   */
  u_idx_0 = 0.0;
  u_idx_1 = 0.0;
  for (pwm_r = 0; pwm_r < 6; pwm_r++) {
    tmp = rtU.state_x[pwm_r];
    pwm_l = pwm_r << 1;
    u_idx_0 += a[pwm_l] * tmp;
    u_idx_1 += a[pwm_l + 1] * tmp;
  }

  pwm_r = (int32_T)fmax(fmin(rt_roundd_snf(u_idx_0 / 24.0 * 1000.0), 1000.0),
                        -1000.0);
  pwm_l = (int32_T)fmax(fmin(rt_roundd_snf(u_idx_1 / 24.0 * 1000.0), 1000.0),
                        -1000.0);
  if (pwm_r >= 0) {
    /* Outport: '<Root>/RPWM_R' */
    rtY.RPWM_R = pwm_r;

    /* Outport: '<Root>/RPWM_R1' */
    rtY.RPWM_R1 = 0.0;
  } else {
    /* Outport: '<Root>/RPWM_R' */
    rtY.RPWM_R = 0.0;

    /* Outport: '<Root>/RPWM_R1' */
    rtY.RPWM_R1 = fabs((real_T)pwm_r);
  }

  if (pwm_l >= 0) {
    /* Outport: '<Root>/RPWM_R2' */
    rtY.RPWM_R2 = pwm_l;

    /* Outport: '<Root>/RPWM_R3' */
    rtY.RPWM_R3 = 0.0;
  } else {
    /* Outport: '<Root>/RPWM_R2' */
    rtY.RPWM_R2 = 0.0;

    /* Outport: '<Root>/RPWM_R3' */
    rtY.RPWM_R3 = fabs((real_T)pwm_l);
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function' */
}

/* Model initialize function */
void Simulink_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
