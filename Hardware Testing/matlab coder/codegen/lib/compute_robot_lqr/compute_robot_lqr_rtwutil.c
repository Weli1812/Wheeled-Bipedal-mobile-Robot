/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_robot_lqr_rtwutil.c
 *
 * Code generation for function 'compute_robot_lqr_rtwutil'
 *
 */

/* Include files */
#include "compute_robot_lqr_rtwutil.h"
#include "rt_nonfinite.h"

/* Function Definitions */
int div_nde_s32_floor(int numerator, int denominator)
{
  int quotient;
  if (((numerator < 0) != (denominator < 0)) &&
      (numerator % denominator != 0)) {
    quotient = -1;
  } else {
    quotient = 0;
  }
  quotient += numerator / denominator;
  return quotient;
}

/* End of code generation (compute_robot_lqr_rtwutil.c) */
