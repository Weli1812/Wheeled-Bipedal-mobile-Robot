/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzgehrd.c
 *
 * Code generation for function 'xzgehrd'
 *
 */

/* Include files */
#include "xzgehrd.h"
#include "compute_robot_lqr_rtwutil.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <emmintrin.h>
#include <string.h>

/* Function Definitions */
void b_xzgehrd(double a[36], int ilo, int ihi)
{
  double work[6];
  double tau[5];
  double alpha1;
  int b_i;
  int c_i;
  int ia;
  if ((ihi - ilo) + 1 > 1) {
    int i;
    i = (unsigned char)(ilo - 1);
    if (i - 1 >= 0) {
      memset(&tau[0], 0, (unsigned int)i * sizeof(double));
    }
    for (b_i = ihi; b_i < 6; b_i++) {
      tau[b_i - 1] = 0.0;
    }
    for (b_i = 0; b_i < 6; b_i++) {
      work[b_i] = 0.0;
    }
    for (c_i = ilo; c_i < ihi; c_i++) {
      double d;
      double temp;
      int alpha1_tmp;
      int b_lastc;
      int b_lastv;
      int coltop;
      int exitg1;
      int ic0;
      int in;
      int jA;
      int lastc;
      int lastv;
      boolean_T exitg2;
      i = (c_i - 1) * 6;
      in = c_i * 6;
      lastv = ihi - c_i;
      alpha1_tmp = c_i + i;
      alpha1 = a[alpha1_tmp];
      if (c_i + 2 <= 6) {
        coltop = c_i + 1;
      } else {
        coltop = 5;
      }
      temp = c_xzlarfg(lastv, &alpha1, a, (coltop + i) + 1);
      tau[c_i - 1] = temp;
      a[alpha1_tmp] = 1.0;
      ic0 = in + 1;
      if (temp != 0.0) {
        b_lastv = lastv;
        i = alpha1_tmp + lastv;
        while ((b_lastv > 0) && (a[i - 1] == 0.0)) {
          b_lastv--;
          i--;
        }
        lastc = ihi;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          i = in + lastc;
          coltop = i;
          do {
            exitg1 = 0;
            if (coltop <= i + (b_lastv - 1) * 6) {
              if (a[coltop - 1] != 0.0) {
                exitg1 = 1;
              } else {
                coltop += 6;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        lastc = 0;
      }
      if (b_lastv > 0) {
        if (lastc != 0) {
          i = (unsigned char)lastc;
          if (i - 1 >= 0) {
            memset(&work[0], 0, (unsigned int)i * sizeof(double));
          }
          i = alpha1_tmp;
          coltop = (in + 6 * (b_lastv - 1)) + 1;
          for (b_i = ic0; b_i <= coltop; b_i += 6) {
            b_lastc = b_i + lastc;
            for (ia = b_i; ia < b_lastc; ia++) {
              jA = ia - b_i;
              work[jA] += a[ia - 1] * a[i];
            }
            i++;
          }
        }
        d = -tau[c_i - 1];
        if (!(d == 0.0)) {
          int b_jA;
          b_jA = in;
          i = (unsigned char)b_lastv;
          for (b_i = 0; b_i < i; b_i++) {
            temp = a[alpha1_tmp + b_i];
            if (temp != 0.0) {
              temp *= d;
              coltop = b_jA + 1;
              b_lastc = lastc + b_jA;
              jA = ((b_lastc - coltop) + 1) / 2 * 2 + coltop;
              ic0 = jA - 2;
              for (ia = coltop; ia <= ic0; ia += 2) {
                __m128d r;
                __m128d r1;
                r = _mm_loadu_pd(&work[(ia - b_jA) - 1]);
                r = _mm_mul_pd(r, _mm_set1_pd(temp));
                r1 = _mm_loadu_pd(&a[ia - 1]);
                r = _mm_add_pd(r1, r);
                _mm_storeu_pd(&a[ia - 1], r);
              }
              for (ia = jA; ia <= b_lastc; ia++) {
                a[ia - 1] += work[(ia - b_jA) - 1] * temp;
              }
            }
            b_jA += 6;
          }
        }
      }
      jA = (c_i + in) + 1;
      d = tau[c_i - 1];
      if (d != 0.0) {
        i = alpha1_tmp + lastv;
        while ((lastv > 0) && (a[i - 1] == 0.0)) {
          lastv--;
          i--;
        }
        b_lastc = 6 - c_i;
        exitg2 = false;
        while ((!exitg2) && (b_lastc > 0)) {
          coltop = jA + (b_lastc - 1) * 6;
          i = coltop;
          do {
            exitg1 = 0;
            if (i <= (coltop + lastv) - 1) {
              if (a[i - 1] != 0.0) {
                exitg1 = 1;
              } else {
                i++;
              }
            } else {
              b_lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        b_lastc = 0;
      }
      if (lastv > 0) {
        if (b_lastc != 0) {
          i = (unsigned char)b_lastc;
          if (i - 1 >= 0) {
            memset(&work[0], 0, (unsigned int)i * sizeof(double));
          }
          coltop = jA + 6 * (b_lastc - 1);
          for (ia = jA; ia <= coltop; ia += 6) {
            temp = 0.0;
            i = ia + lastv;
            for (b_i = ia; b_i < i; b_i++) {
              temp += a[b_i - 1] * a[(alpha1_tmp + b_i) - ia];
            }
            i = div_nde_s32_floor(ia - jA, 6);
            work[i] += temp;
          }
        }
        if (!(-d == 0.0)) {
          i = (unsigned char)b_lastc;
          for (b_i = 0; b_i < i; b_i++) {
            temp = work[b_i];
            if (temp != 0.0) {
              temp *= -d;
              coltop = lastv + jA;
              for (ia = jA; ia < coltop; ia++) {
                a[ia - 1] += a[(alpha1_tmp + ia) - jA] * temp;
              }
            }
            jA += 6;
          }
        }
      }
      a[alpha1_tmp] = alpha1;
    }
  }
}

void xzgehrd(double a[144], int ilo, int ihi, double tau[11])
{
  double work[12];
  double alpha1;
  int b_i;
  int c_i;
  int ia;
  if ((ihi - ilo) + 1 > 1) {
    int i;
    i = (unsigned char)(ilo - 1);
    if (i - 1 >= 0) {
      memset(&tau[0], 0, (unsigned int)i * sizeof(double));
    }
    for (b_i = ihi; b_i < 12; b_i++) {
      tau[b_i - 1] = 0.0;
    }
    memset(&work[0], 0, 12U * sizeof(double));
    for (c_i = ilo; c_i < ihi; c_i++) {
      double temp;
      int alpha1_tmp;
      int ic0;
      int in;
      int lastc;
      int lastv;
      int n;
      int rowleft;
      i = (c_i - 1) * 12;
      in = c_i * 12;
      n = ihi - c_i;
      alpha1_tmp = c_i + i;
      alpha1 = a[alpha1_tmp];
      if (c_i + 2 <= 12) {
        rowleft = c_i + 1;
      } else {
        rowleft = 11;
      }
      temp = xzlarfg(n, &alpha1, a, (rowleft + i) + 1);
      tau[c_i - 1] = temp;
      a[alpha1_tmp] = 1.0;
      ic0 = in + 1;
      if (temp != 0.0) {
        boolean_T exitg2;
        lastv = n;
        i = alpha1_tmp + n;
        while ((lastv > 0) && (a[i - 1] == 0.0)) {
          lastv--;
          i--;
        }
        lastc = ihi;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int exitg1;
          rowleft = in + lastc;
          i = rowleft;
          do {
            exitg1 = 0;
            if (i <= rowleft + (lastv - 1) * 12) {
              if (a[i - 1] != 0.0) {
                exitg1 = 1;
              } else {
                i += 12;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        double d;
        int d_i;
        int work_tmp;
        if (lastc != 0) {
          rowleft = (unsigned char)lastc;
          memset(&work[0], 0, (unsigned int)rowleft * sizeof(double));
          i = alpha1_tmp;
          rowleft = (in + 12 * (lastv - 1)) + 1;
          for (b_i = ic0; b_i <= rowleft; b_i += 12) {
            d_i = b_i + lastc;
            for (ia = b_i; ia < d_i; ia++) {
              work_tmp = ia - b_i;
              work[work_tmp] += a[ia - 1] * a[i];
            }
            i++;
          }
        }
        d = -tau[c_i - 1];
        if (!(d == 0.0)) {
          int jA;
          jA = in;
          i = (unsigned char)lastv;
          for (b_i = 0; b_i < i; b_i++) {
            temp = a[alpha1_tmp + b_i];
            if (temp != 0.0) {
              temp *= d;
              rowleft = jA + 1;
              d_i = lastc + jA;
              work_tmp = ((d_i - rowleft) + 1) / 2 * 2 + rowleft;
              ic0 = work_tmp - 2;
              for (ia = rowleft; ia <= ic0; ia += 2) {
                __m128d r;
                __m128d r1;
                r = _mm_loadu_pd(&work[(ia - jA) - 1]);
                r = _mm_mul_pd(r, _mm_set1_pd(temp));
                r1 = _mm_loadu_pd(&a[ia - 1]);
                r = _mm_add_pd(r1, r);
                _mm_storeu_pd(&a[ia - 1], r);
              }
              for (ia = work_tmp; ia <= d_i; ia++) {
                a[ia - 1] += work[(ia - jA) - 1] * temp;
              }
            }
            jA += 12;
          }
        }
      }
      xzlarf(n, 12 - c_i, alpha1_tmp + 1, tau[c_i - 1], a, (c_i + in) + 1,
             work);
      a[alpha1_tmp] = alpha1;
    }
  }
}

/* End of code generation (xzgehrd.c) */
