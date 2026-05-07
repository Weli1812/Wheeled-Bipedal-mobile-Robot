/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xdsterf.c
 *
 * Code generation for function 'xdsterf'
 *
 */

/* Include files */
#include "xdsterf.h"
#include "insertionsort.h"
#include "rt_nonfinite.h"
#include "xdlaev2.h"
#include "xzlascl.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>

/* Function Definitions */
int xdsterf(double d[6], double e[5])
{
  double p;
  int i;
  int info;
  int jtot;
  int l1;
  info = 0;
  jtot = 0;
  l1 = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (l1 > 6) {
      insertionsort(d);
      exitg1 = 1;
    } else {
      int l;
      int lend;
      int lendsv;
      int lsv;
      int m;
      boolean_T exitg2;
      if (l1 > 1) {
        e[l1 - 2] = 0.0;
      }
      m = l1;
      exitg2 = false;
      while ((!exitg2) && (m < 6)) {
        if (fabs(e[m - 1]) <=
            sqrt(fabs(d[m - 1])) * sqrt(fabs(d[m])) * 2.2204460492503131E-16) {
          e[m - 1] = 0.0;
          exitg2 = true;
        } else {
          m++;
        }
      }
      l = l1;
      lsv = l1;
      lend = m;
      lendsv = m + 1;
      l1 = m + 1;
      if (m != l) {
        double anorm;
        int n_tmp;
        int scalarLB;
        int vectorUB;
        n_tmp = m - l;
        if (n_tmp + 1 <= 0) {
          anorm = 0.0;
        } else {
          anorm = fabs(d[(l + n_tmp) - 1]);
          scalarLB = -1;
          exitg2 = false;
          while ((!exitg2) && (scalarLB + 1 <= n_tmp - 1)) {
            vectorUB = l + scalarLB;
            p = fabs(d[vectorUB]);
            if (rtIsNaN(p)) {
              anorm = rtNaN;
              exitg2 = true;
            } else {
              if (p > anorm) {
                anorm = p;
              }
              p = fabs(e[vectorUB]);
              if (rtIsNaN(p)) {
                anorm = rtNaN;
                exitg2 = true;
              } else {
                if (p > anorm) {
                  anorm = p;
                }
                scalarLB++;
              }
            }
          }
        }
        if (!(anorm == 0.0)) {
          int iscale;
          iscale = 0;
          if (anorm > 2.2346346549904327E+153) {
            iscale = 1;
            e_xzlascl(anorm, 2.2346346549904327E+153, n_tmp + 1, d, l);
            f_xzlascl(anorm, 2.2346346549904327E+153, n_tmp, e, l);
          } else if (anorm < 3.02546243347603E-123) {
            iscale = 2;
            e_xzlascl(anorm, 3.02546243347603E-123, n_tmp + 1, d, l);
            f_xzlascl(anorm, 3.02546243347603E-123, n_tmp, e, l);
          }
          scalarLB = ((n_tmp / 2) << 1) + l;
          vectorUB = scalarLB - 2;
          for (i = l; i <= vectorUB; i += 2) {
            __m128d r;
            r = _mm_loadu_pd(&e[i - 1]);
            _mm_storeu_pd(&e[i - 1], _mm_mul_pd(r, r));
          }
          for (i = scalarLB; i < m; i++) {
            p = e[i - 1];
            e[i - 1] = p * p;
          }
          if (fabs(d[m - 1]) < fabs(d[l - 1])) {
            lend = lsv;
            l = m;
          }
          if (lend >= l) {
            int exitg4;
            do {
              exitg4 = 0;
              if (l != lend) {
                m = l;
                while ((m < lend) &&
                       (!(fabs(e[m - 1]) <= 4.9303806576313238E-32 *
                                                fabs(d[m - 1]) * fabs(d[m])))) {
                  m++;
                }
              } else {
                m = lend;
              }
              if (m < lend) {
                e[m - 1] = 0.0;
              }
              if (m == l) {
                l++;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (m == l + 1) {
                d[l - 1] = b_xdlaev2(d[l - 1], sqrt(e[l - 1]), d[l], &p);
                d[l] = p;
                e[l - 1] = 0.0;
                l += 2;
                if (l > lend) {
                  exitg4 = 1;
                }
              } else if (jtot == 180) {
                exitg4 = 1;
              } else {
                double b_gamma;
                double b_r;
                double c;
                double oldgam;
                double rte;
                double s;
                double sigma;
                jtot++;
                rte = sqrt(e[l - 1]);
                oldgam = d[l - 1];
                sigma = (d[l] - oldgam) / (2.0 * rte);
                p = fabs(sigma);
                if (p < 1.0) {
                  p = sqrt(p * p + 1.0);
                } else if (p > 1.0) {
                  b_r = 1.0 / p;
                  p *= sqrt(b_r * b_r + 1.0);
                } else {
                  p *= 1.4142135623730951;
                }
                if (!(sigma >= 0.0)) {
                  p = -p;
                }
                sigma = oldgam - rte / (sigma + p);
                c = 1.0;
                s = 0.0;
                b_gamma = d[m - 1] - sigma;
                p = b_gamma * b_gamma;
                scalarLB = m - 1;
                for (i = scalarLB; i >= l; i--) {
                  double oldc;
                  rte = e[i - 1];
                  b_r = p + rte;
                  if (i != m - 1) {
                    e[i] = s * b_r;
                  }
                  oldc = c;
                  c = p / b_r;
                  s = rte / b_r;
                  oldgam = b_gamma;
                  p = d[i - 1];
                  b_gamma = c * (p - sigma) - s * b_gamma;
                  d[i] = oldgam + (p - b_gamma);
                  if (c != 0.0) {
                    p = b_gamma * b_gamma / c;
                  } else {
                    p = oldc * rte;
                  }
                }
                e[l - 1] = s * p;
                d[l - 1] = sigma + b_gamma;
              }
            } while (exitg4 == 0);
          } else {
            int exitg3;
            do {
              exitg3 = 0;
              m = l;
              while ((m > lend) && (!(fabs(e[m - 2]) <= 4.9303806576313238E-32 *
                                                            fabs(d[m - 1]) *
                                                            fabs(d[m - 2])))) {
                m--;
              }
              if (m > lend) {
                e[m - 2] = 0.0;
              }
              if (m == l) {
                l--;
                if (l < lend) {
                  exitg3 = 1;
                }
              } else if (m == l - 1) {
                d[l - 1] = b_xdlaev2(d[l - 1], sqrt(e[l - 2]), d[l - 2], &p);
                d[l - 2] = p;
                e[l - 2] = 0.0;
                l -= 2;
                if (l < lend) {
                  exitg3 = 1;
                }
              } else if (jtot == 180) {
                exitg3 = 1;
              } else {
                double b_gamma;
                double b_r;
                double c;
                double oldgam;
                double rte;
                double s;
                double sigma;
                jtot++;
                rte = sqrt(e[l - 2]);
                oldgam = d[l - 1];
                sigma = (d[l - 2] - oldgam) / (2.0 * rte);
                p = fabs(sigma);
                if (p < 1.0) {
                  p = sqrt(p * p + 1.0);
                } else if (p > 1.0) {
                  b_r = 1.0 / p;
                  p *= sqrt(b_r * b_r + 1.0);
                } else {
                  p *= 1.4142135623730951;
                }
                if (!(sigma >= 0.0)) {
                  p = -p;
                }
                sigma = oldgam - rte / (sigma + p);
                c = 1.0;
                s = 0.0;
                b_gamma = d[m - 1] - sigma;
                p = b_gamma * b_gamma;
                for (i = m; i < l; i++) {
                  double oldc;
                  rte = e[i - 1];
                  b_r = p + rte;
                  if (i != m) {
                    e[i - 2] = s * b_r;
                  }
                  oldc = c;
                  c = p / b_r;
                  s = rte / b_r;
                  oldgam = b_gamma;
                  b_gamma = c * (d[i] - sigma) - s * b_gamma;
                  d[i - 1] = oldgam + (d[i] - b_gamma);
                  if (c != 0.0) {
                    p = b_gamma * b_gamma / c;
                  } else {
                    p = oldc * rte;
                  }
                }
                e[l - 2] = s * p;
                d[l - 1] = sigma + b_gamma;
              }
            } while (exitg3 == 0);
          }
          if (iscale == 1) {
            e_xzlascl(2.2346346549904327E+153, anorm, lendsv - lsv, d, lsv);
          } else if (iscale == 2) {
            e_xzlascl(3.02546243347603E-123, anorm, lendsv - lsv, d, lsv);
          }
          if (jtot >= 180) {
            for (i = 0; i < 5; i++) {
              if (e[i] != 0.0) {
                info++;
              }
            }
            exitg1 = 1;
          }
        }
      }
    }
  } while (exitg1 == 0);
  return info;
}

/* End of code generation (xdsterf.c) */
