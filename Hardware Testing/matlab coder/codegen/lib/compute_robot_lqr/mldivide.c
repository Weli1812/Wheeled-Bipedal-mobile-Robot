/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mldivide.c
 *
 * Code generation for function 'mldivide'
 *
 */

/* Include files */
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
void mldivide(const double A[4], const double B[12], double Y[12])
{
  double a21;
  double a22;
  double a22_tmp;
  int k;
  int r1;
  int r2;
  if (fabs(A[1]) > fabs(A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }
  a21 = A[r2] / A[r1];
  a22_tmp = A[r1 + 2];
  a22 = A[r2 + 2] - a21 * a22_tmp;
  for (k = 0; k < 6; k++) {
    double d;
    double d1;
    int i;
    i = k << 1;
    d = B[r1 + i];
    d1 = (B[r2 + i] - d * a21) / a22;
    Y[i + 1] = d1;
    Y[i] = (d - d1 * a22_tmp) / A[r1];
  }
}

/* End of code generation (mldivide.c) */
