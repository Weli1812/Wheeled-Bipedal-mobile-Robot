/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzunghr.c
 *
 * Code generation for function 'xzunghr'
 *
 */

/* Include files */
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void xzunghr(int ilo, int ihi, double A[144], const double tau[11])
{
  double work[12];
  int a;
  int b_i;
  int b_ia;
  int c_i;
  int i;
  int ia;
  int ia0;
  int j;
  int nh;
  nh = ihi - ilo;
  a = ilo + 1;
  for (j = ihi; j >= a; j--) {
    b_ia = (j - 1) * 12;
    ia = (unsigned char)(j - 1);
    memset(&A[b_ia], 0, (unsigned int)ia * sizeof(double));
    ia = j + 1;
    for (i = ia; i <= ihi; i++) {
      b_i = b_ia + i;
      A[b_i - 1] = A[b_i - 13];
    }
    ia = ihi + 1;
    if (ia <= 12) {
      memset(&A[(ia + b_ia) + -1], 0,
             (unsigned int)(((b_ia - ia) - b_ia) + 13) * sizeof(double));
    }
  }
  ia = (unsigned char)ilo;
  for (i = 0; i < ia; i++) {
    b_ia = i * 12;
    memset(&A[b_ia], 0, 12U * sizeof(double));
    A[b_ia + i] = 1.0;
  }
  ia = ihi + 1;
  for (i = ia; i < 13; i++) {
    b_ia = (i - 1) * 12;
    memset(&A[b_ia], 0, 12U * sizeof(double));
    A[(b_ia + i) - 1] = 1.0;
  }
  ia0 = ilo + ilo * 12;
  if (nh >= 1) {
    int itau;
    for (i = nh; i < nh; i++) {
      ia = ia0 + i * 12;
      memset(&A[ia], 0, (unsigned int)nh * sizeof(double));
      A[ia + i] = 1.0;
    }
    itau = (ilo + nh) - 2;
    memset(&work[0], 0, 12U * sizeof(double));
    for (c_i = nh; c_i >= 1; c_i--) {
      int iaii;
      iaii = (ia0 + c_i) + (c_i - 1) * 12;
      if (c_i < nh) {
        A[iaii - 1] = 1.0;
        ia = nh - c_i;
        xzlarf(ia + 1, ia, iaii, tau[itau], A, iaii + 12, work);
        ia = iaii + 1;
        b_i = (iaii + nh) - c_i;
        a = ((b_i - ia) + 1) / 2 * 2 + ia;
        b_ia = a - 2;
        for (i = ia; i <= b_ia; i += 2) {
          __m128d r;
          r = _mm_loadu_pd(&A[i - 1]);
          r = _mm_mul_pd(_mm_set1_pd(-tau[itau]), r);
          _mm_storeu_pd(&A[i - 1], r);
        }
        for (i = a; i <= b_i; i++) {
          A[i - 1] *= -tau[itau];
        }
      }
      A[iaii - 1] = 1.0 - tau[itau];
      ia = (unsigned char)(c_i - 1);
      for (i = 0; i < ia; i++) {
        A[(iaii - i) - 2] = 0.0;
      }
      itau--;
    }
  }
}

/* End of code generation (xzunghr.c) */
