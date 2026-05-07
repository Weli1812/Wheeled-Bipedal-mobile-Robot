/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzgebal.c
 *
 * Code generation for function 'xzgebal'
 *
 */

/* Include files */
#include "xzgebal.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>

/* Function Definitions */
int xzgebal(double A[144], int *ihi, double scale[12])
{
  double temp;
  int b_ix;
  int c_ix;
  int exitg5;
  int i;
  int ilo;
  int ix;
  int iy;
  int j;
  int kend;
  int l;
  boolean_T notdone;
  boolean_T skipThisRow;
  for (i = 0; i < 12; i++) {
    scale[i] = 1.0;
  }
  ilo = 1;
  l = 12;
  notdone = true;
  do {
    exitg5 = 0;
    if (notdone) {
      int exitg4;
      notdone = false;
      j = l;
      do {
        exitg4 = 0;
        if (j > 0) {
          boolean_T exitg6;
          skipThisRow = false;
          b_ix = 0;
          exitg6 = false;
          while ((!exitg6) && (b_ix <= (unsigned char)l - 1)) {
            if ((b_ix + 1 == j) || (!(A[(j + 12 * b_ix) - 1] != 0.0))) {
              b_ix++;
            } else {
              skipThisRow = true;
              exitg6 = true;
            }
          }
          if (skipThisRow) {
            j--;
          } else {
            scale[l - 1] = j;
            if (j != l) {
              b_ix = (j - 1) * 12;
              iy = (l - 1) * 12;
              c_ix = (unsigned char)l;
              for (i = 0; i < c_ix; i++) {
                kend = b_ix + i;
                temp = A[kend];
                ix = iy + i;
                A[kend] = A[ix];
                A[ix] = temp;
              }
              for (i = 0; i < 12; i++) {
                b_ix = (j + i * 12) - 1;
                temp = A[b_ix];
                iy = (l + i * 12) - 1;
                A[b_ix] = A[iy];
                A[iy] = temp;
              }
            }
            exitg4 = 1;
          }
        } else {
          exitg4 = 2;
        }
      } while (exitg4 == 0);
      if (exitg4 == 1) {
        if (l == 1) {
          ilo = 1;
          *ihi = 1;
          exitg5 = 1;
        } else {
          l--;
          notdone = true;
        }
      }
    } else {
      notdone = true;
      while (notdone) {
        boolean_T exitg6;
        notdone = false;
        ix = ilo;
        exitg6 = false;
        while ((!exitg6) && (ix <= l)) {
          boolean_T exitg7;
          skipThisRow = false;
          b_ix = ilo;
          exitg7 = false;
          while ((!exitg7) && (b_ix <= l)) {
            if ((b_ix == ix) || (!(A[(b_ix + 12 * (ix - 1)) - 1] != 0.0))) {
              b_ix++;
            } else {
              skipThisRow = true;
              exitg7 = true;
            }
          }
          if (skipThisRow) {
            ix++;
          } else {
            scale[ilo - 1] = ix;
            if (ix != ilo) {
              c_ix = (ix - 1) * 12;
              j = (ilo - 1) * 12;
              kend = (unsigned char)l;
              for (i = 0; i < kend; i++) {
                b_ix = c_ix + i;
                temp = A[b_ix];
                iy = j + i;
                A[b_ix] = A[iy];
                A[iy] = temp;
              }
              ix = (j + ix) - 1;
              b_ix = (j + ilo) - 1;
              iy = (unsigned char)(13 - ilo);
              for (i = 0; i < iy; i++) {
                c_ix = ix + i * 12;
                temp = A[c_ix];
                kend = b_ix + i * 12;
                A[c_ix] = A[kend];
                A[kend] = temp;
              }
            }
            ilo++;
            notdone = true;
            exitg6 = true;
          }
        }
      }
      *ihi = l;
      skipThisRow = false;
      exitg5 = 2;
    }
  } while (exitg5 == 0);
  if (exitg5 != 1) {
    boolean_T exitg3;
    exitg3 = false;
    while ((!exitg3) && (!skipThisRow)) {
      int b_i;
      int exitg2;
      skipThisRow = true;
      b_i = ilo - 1;
      do {
        exitg2 = 0;
        if (b_i + 1 <= l) {
          double b_s;
          double c;
          double ca;
          double r;
          double s;
          b_ix = (l - ilo) + 1;
          c = xnrm2(b_ix, A, b_i * 12 + ilo);
          ix = (ilo - 1) * 12 + b_i;
          j = ix + 1;
          r = 0.0;
          if (b_ix >= 1) {
            if (b_ix == 1) {
              r = fabs(A[ix]);
            } else {
              temp = 3.3121686421112381E-170;
              kend = (ix + (b_ix - 1) * 12) + 1;
              for (i = j; i <= kend; i += 12) {
                s = fabs(A[i - 1]);
                if (s > temp) {
                  b_s = temp / s;
                  r = r * b_s * b_s + 1.0;
                  temp = s;
                } else {
                  b_s = s / temp;
                  r += b_s * b_s;
                }
              }
              r = temp * sqrt(r);
              if (rtIsNaN(r)) {
                b_ix = ix + 1;
                int exitg8;
                do {
                  exitg8 = 0;
                  if (b_ix <= kend) {
                    if (rtIsNaN(A[b_ix - 1])) {
                      exitg8 = 1;
                    } else {
                      b_ix += 12;
                    }
                  } else {
                    r = rtInf;
                    exitg8 = 1;
                  }
                } while (exitg8 == 0);
              }
            }
          }
          iy = b_i * 12;
          b_ix = 1;
          if (l > 1) {
            temp = fabs(A[iy]);
            for (i = 2; i <= l; i++) {
              s = fabs(A[(iy + i) - 1]);
              if (s > temp) {
                b_ix = i;
                temp = s;
              }
            }
          }
          ca = fabs(A[(b_ix + 12 * b_i) - 1]);
          b_ix = 13 - ilo;
          if (13 - ilo < 1) {
            c_ix = 0;
          } else {
            c_ix = 1;
            if (13 - ilo > 1) {
              temp = fabs(A[ix]);
              for (i = 2; i <= b_ix; i++) {
                s = fabs(A[ix + (i - 1) * 12]);
                if (s > temp) {
                  c_ix = i;
                  temp = s;
                }
              }
            }
          }
          temp = fabs(A[b_i + 12 * ((c_ix + ilo) - 2)]);
          if ((c == 0.0) || (r == 0.0)) {
            b_i++;
          } else {
            double f;
            int exitg1;
            s = r / 2.0;
            f = 1.0;
            b_s = c + r;
            do {
              exitg1 = 0;
              if ((c < s) && (fmax(f, fmax(c, ca)) < 4.9896007738368E+291) &&
                  (fmin(r, fmin(s, temp)) > 2.0041683600089728E-292)) {
                if (rtIsNaN(((((c + f) + ca) + r) + s) + temp)) {
                  exitg1 = 1;
                } else {
                  f *= 2.0;
                  c *= 2.0;
                  ca *= 2.0;
                  r /= 2.0;
                  s /= 2.0;
                  temp /= 2.0;
                }
              } else {
                s = c / 2.0;
                while (
                    (s >= r) && (fmax(r, temp) < 4.9896007738368E+291) &&
                    (fmin(fmin(f, c), fmin(s, ca)) > 2.0041683600089728E-292)) {
                  f /= 2.0;
                  c /= 2.0;
                  s /= 2.0;
                  ca /= 2.0;
                  r *= 2.0;
                  temp *= 2.0;
                }
                if ((!(c + r >= 0.95 * b_s)) &&
                    ((!(f < 1.0)) || (!(scale[b_i] < 1.0)) ||
                     (!(f * scale[b_i] <= 1.0020841800044864E-292))) &&
                    ((!(f > 1.0)) || (!(scale[b_i] > 1.0)) ||
                     (!(scale[b_i] >= 9.9792015476736E+291 / f)))) {
                  temp = 1.0 / f;
                  scale[b_i] *= f;
                  b_ix = (ix + 12 * (12 - ilo)) + 1;
                  for (i = j; i <= b_ix; i += 12) {
                    A[i - 1] *= temp;
                  }
                  b_ix = b_i * 12;
                  kend = b_ix + l;
                  iy = ((kend - b_ix) / 2 * 2 + b_ix) + 1;
                  c_ix = iy - 2;
                  for (i = b_ix + 1; i <= c_ix; i += 2) {
                    __m128d b_r;
                    b_r = _mm_loadu_pd(&A[i - 1]);
                    b_r = _mm_mul_pd(_mm_set1_pd(f), b_r);
                    _mm_storeu_pd(&A[i - 1], b_r);
                  }
                  for (i = iy; i <= kend; i++) {
                    A[i - 1] *= f;
                  }
                  skipThisRow = false;
                }
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = 2;
            } else {
              b_i++;
            }
          }
        } else {
          exitg2 = 1;
        }
      } while (exitg2 == 0);
      if (exitg2 != 1) {
        exitg3 = true;
      }
    }
  }
  return ilo;
}

/* End of code generation (xzgebal.c) */
