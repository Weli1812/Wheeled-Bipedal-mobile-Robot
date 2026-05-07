/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * insertionsort.c
 *
 * Code generation for function 'insertionsort'
 *
 */

/* Include files */
#include "insertionsort.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void insertionsort(double x[6])
{
  int k;
  for (k = 2; k < 7; k++) {
    double xc;
    int idx;
    boolean_T exitg1;
    xc = x[k - 1];
    idx = k - 1;
    exitg1 = false;
    while ((!exitg1) && (idx >= 1)) {
      double d;
      d = x[idx - 1];
      if (xc < d) {
        x[idx] = d;
        idx--;
      } else {
        exitg1 = true;
      }
    }
    x[idx] = xc;
  }
}

/* End of code generation (insertionsort.c) */
