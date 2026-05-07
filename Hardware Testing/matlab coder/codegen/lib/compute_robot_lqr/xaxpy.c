/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xaxpy.c
 *
 * Code generation for function 'xaxpy'
 *
 */

/* Include files */
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>

/* Function Definitions */
void xaxpy(int n, double a, const double x[144], int ix0, double y[36])
{
  int k;
  if ((n >= 1) && (!(a == 0.0))) {
    int scalarLB;
    int vectorUB;
    scalarLB = n / 2 * 2;
    vectorUB = scalarLB - 2;
    for (k = 0; k <= vectorUB; k += 2) {
      __m128d r;
      __m128d r1;
      r = _mm_loadu_pd(&x[(ix0 + k) - 1]);
      r = _mm_mul_pd(_mm_set1_pd(a), r);
      r1 = _mm_loadu_pd(&y[k + 24]);
      r = _mm_add_pd(r1, r);
      _mm_storeu_pd(&y[k + 24], r);
    }
    for (k = scalarLB; k < n; k++) {
      y[k + 24] += a * x[(ix0 + k) - 1];
    }
  }
}

/* End of code generation (xaxpy.c) */
