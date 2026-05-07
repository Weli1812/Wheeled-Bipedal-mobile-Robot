/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eigStandard.c
 *
 * Code generation for function 'eigStandard'
 *
 */

/* Include files */
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdtrevc3.h"
#include "xnrm2.h"
#include "xzgebal.h"
#include "xzgehrd.h"
#include "xzlascl.h"
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Definitions */
void b_eigStandard(const double A[36], creal_T V[6])
{
  double b_A[36];
  double absxk;
  double anrm;
  int i;
  int ix;
  boolean_T exitg1;
  boolean_T skipThisRow;
  memcpy(&b_A[0], &A[0], 36U * sizeof(double));
  anrm = 0.0;
  ix = 0;
  exitg1 = false;
  while ((!exitg1) && (ix < 36)) {
    absxk = fabs(A[ix]);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      ix++;
    }
  }
  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    for (i = 0; i < 6; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
  } else {
    double wi[6];
    double wr[6];
    double cscale;
    int b_ix;
    int c_ix;
    int exitg5;
    int iy;
    int k;
    int kend;
    int l;
    boolean_T notdone;
    boolean_T scalea;
    cscale = anrm;
    scalea = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      scalea = true;
      cscale = 6.7178761075670888E-139;
      d_xzlascl(anrm, cscale, b_A);
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      cscale = 1.4885657073574029E+138;
      d_xzlascl(anrm, cscale, b_A);
    }
    for (i = 0; i < 6; i++) {
      wr[i] = 1.0;
    }
    k = 1;
    l = 6;
    notdone = true;
    do {
      exitg5 = 0;
      if (notdone) {
        int exitg4;
        int j;
        notdone = false;
        j = l;
        do {
          exitg4 = 0;
          if (j > 0) {
            boolean_T exitg6;
            skipThisRow = false;
            ix = 0;
            exitg6 = false;
            while ((!exitg6) && (ix <= (unsigned char)l - 1)) {
              if ((ix + 1 == j) || (!(b_A[(j + 6 * ix) - 1] != 0.0))) {
                ix++;
              } else {
                skipThisRow = true;
                exitg6 = true;
              }
            }
            if (skipThisRow) {
              j--;
            } else {
              wr[l - 1] = j;
              if (j != l) {
                ix = (j - 1) * 6;
                iy = (l - 1) * 6;
                c_ix = (unsigned char)l;
                for (i = 0; i < c_ix; i++) {
                  kend = ix + i;
                  absxk = b_A[kend];
                  b_ix = iy + i;
                  b_A[kend] = b_A[b_ix];
                  b_A[b_ix] = absxk;
                }
                for (i = 0; i < 6; i++) {
                  ix = (j + i * 6) - 1;
                  absxk = b_A[ix];
                  kend = (l + i * 6) - 1;
                  b_A[ix] = b_A[kend];
                  b_A[kend] = absxk;
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
            k = 1;
            l = 1;
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
          b_ix = k;
          exitg6 = false;
          while ((!exitg6) && (b_ix <= l)) {
            boolean_T exitg7;
            skipThisRow = false;
            ix = k;
            exitg7 = false;
            while ((!exitg7) && (ix <= l)) {
              if ((ix == b_ix) || (!(b_A[(ix + 6 * (b_ix - 1)) - 1] != 0.0))) {
                ix++;
              } else {
                skipThisRow = true;
                exitg7 = true;
              }
            }
            if (skipThisRow) {
              b_ix++;
            } else {
              wr[k - 1] = b_ix;
              if (b_ix != k) {
                int j;
                c_ix = (b_ix - 1) * 6;
                j = (k - 1) * 6;
                kend = (unsigned char)l;
                for (i = 0; i < kend; i++) {
                  ix = c_ix + i;
                  absxk = b_A[ix];
                  iy = j + i;
                  b_A[ix] = b_A[iy];
                  b_A[iy] = absxk;
                }
                b_ix = (j + b_ix) - 1;
                ix = (j + k) - 1;
                kend = (unsigned char)(7 - k);
                for (i = 0; i < kend; i++) {
                  iy = b_ix + i * 6;
                  absxk = b_A[iy];
                  c_ix = ix + i * 6;
                  b_A[iy] = b_A[c_ix];
                  b_A[c_ix] = absxk;
                }
              }
              k++;
              notdone = true;
              exitg6 = true;
            }
          }
        }
        skipThisRow = false;
        exitg5 = 2;
      }
    } while (exitg5 == 0);
    if (exitg5 != 1) {
      exitg1 = false;
      while ((!exitg1) && (!skipThisRow)) {
        int exitg3;
        skipThisRow = true;
        b_ix = k - 1;
        do {
          exitg3 = 0;
          if (b_ix + 1 <= l) {
            double b_s;
            double c;
            double ca;
            double r;
            double s;
            ix = (l - k) + 1;
            c = c_xnrm2(ix, b_A, b_ix * 6 + k);
            iy = (k - 1) * 6 + b_ix;
            c_ix = iy + 1;
            r = 0.0;
            if (ix >= 1) {
              if (ix == 1) {
                r = fabs(b_A[iy]);
              } else {
                absxk = 3.3121686421112381E-170;
                kend = (iy + (ix - 1) * 6) + 1;
                for (i = c_ix; i <= kend; i += 6) {
                  s = fabs(b_A[i - 1]);
                  if (s > absxk) {
                    b_s = absxk / s;
                    r = r * b_s * b_s + 1.0;
                    absxk = s;
                  } else {
                    b_s = s / absxk;
                    r += b_s * b_s;
                  }
                }
                r = absxk * sqrt(r);
                if (rtIsNaN(r)) {
                  ix = iy + 1;
                  int exitg8;
                  do {
                    exitg8 = 0;
                    if (ix <= kend) {
                      if (rtIsNaN(b_A[ix - 1])) {
                        exitg8 = 1;
                      } else {
                        ix += 6;
                      }
                    } else {
                      r = rtInf;
                      exitg8 = 1;
                    }
                  } while (exitg8 == 0);
                }
              }
            }
            kend = b_ix * 6;
            ix = 1;
            if (l > 1) {
              absxk = fabs(b_A[kend]);
              for (i = 2; i <= l; i++) {
                s = fabs(b_A[(kend + i) - 1]);
                if (s > absxk) {
                  ix = i;
                  absxk = s;
                }
              }
            }
            ca = fabs(b_A[(ix + 6 * b_ix) - 1]);
            kend = 7 - k;
            if (7 - k < 1) {
              ix = 0;
            } else {
              ix = 1;
              if (7 - k > 1) {
                absxk = fabs(b_A[iy]);
                for (i = 2; i <= kend; i++) {
                  s = fabs(b_A[iy + (i - 1) * 6]);
                  if (s > absxk) {
                    ix = i;
                    absxk = s;
                  }
                }
              }
            }
            absxk = fabs(b_A[b_ix + 6 * ((ix + k) - 2)]);
            if ((c == 0.0) || (r == 0.0)) {
              b_ix++;
            } else {
              double f;
              int exitg2;
              s = r / 2.0;
              f = 1.0;
              b_s = c + r;
              do {
                exitg2 = 0;
                if ((c < s) && (fmax(f, fmax(c, ca)) < 4.9896007738368E+291) &&
                    (fmin(r, fmin(s, absxk)) > 2.0041683600089728E-292)) {
                  if (rtIsNaN(((((c + f) + ca) + r) + s) + absxk)) {
                    exitg2 = 1;
                  } else {
                    f *= 2.0;
                    c *= 2.0;
                    ca *= 2.0;
                    r /= 2.0;
                    s /= 2.0;
                    absxk /= 2.0;
                  }
                } else {
                  s = c / 2.0;
                  while ((s >= r) && (fmax(r, absxk) < 4.9896007738368E+291) &&
                         (fmin(fmin(f, c), fmin(s, ca)) >
                          2.0041683600089728E-292)) {
                    f /= 2.0;
                    c /= 2.0;
                    s /= 2.0;
                    ca /= 2.0;
                    r *= 2.0;
                    absxk *= 2.0;
                  }
                  if ((!(c + r >= 0.95 * b_s)) &&
                      ((!(f < 1.0)) || (!(wr[b_ix] < 1.0)) ||
                       (!(f * wr[b_ix] <= 1.0020841800044864E-292))) &&
                      ((!(f > 1.0)) || (!(wr[b_ix] > 1.0)) ||
                       (!(wr[b_ix] >= 9.9792015476736E+291 / f)))) {
                    absxk = 1.0 / f;
                    wr[b_ix] *= f;
                    ix = (iy + 6 * (6 - k)) + 1;
                    for (i = c_ix; i <= ix; i += 6) {
                      b_A[i - 1] *= absxk;
                    }
                    kend = b_ix * 6;
                    ix = kend + l;
                    iy = ((((ix - kend) / 2) << 1) + kend) + 1;
                    c_ix = iy - 2;
                    for (i = kend + 1; i <= c_ix; i += 2) {
                      __m128d b_r;
                      b_r = _mm_loadu_pd(&b_A[i - 1]);
                      _mm_storeu_pd(&b_A[i - 1],
                                    _mm_mul_pd(_mm_set1_pd(f), b_r));
                    }
                    for (i = iy; i <= ix; i++) {
                      b_A[i - 1] *= f;
                    }
                    skipThisRow = false;
                  }
                  exitg2 = 2;
                }
              } while (exitg2 == 0);
              if (exitg2 == 1) {
                exitg3 = 2;
              } else {
                b_ix++;
              }
            }
          } else {
            exitg3 = 1;
          }
        } while (exitg3 == 0);
        if (exitg3 != 1) {
          exitg1 = true;
        }
      }
    }
    b_xzgehrd(b_A, k, l);
    ix = b_xdlahqr(k, l, b_A, &absxk, wr, wi);
    if (scalea) {
      e_xzlascl(cscale, anrm, 6 - ix, wr, ix + 1);
      e_xzlascl(cscale, anrm, 6 - ix, wi, ix + 1);
      if (ix != 0) {
        e_xzlascl(cscale, anrm, k - 1, wr, 1);
        e_xzlascl(cscale, anrm, k - 1, wi, 1);
      }
    }
    if (ix != 0) {
      for (i = k; i <= ix; i++) {
        wr[i - 1] = rtNaN;
        wi[i - 1] = 0.0;
      }
    }
    for (i = 0; i < 6; i++) {
      V[i].re = wr[i];
      V[i].im = wi[i];
    }
  }
}

void eigStandard(const double A[144], creal_T V[144], creal_T D[144])
{
  creal_T W[12];
  double b_A[144];
  double vr[144];
  double scale[12];
  double absxk;
  double anrm;
  int b_k;
  int i;
  int ihi;
  int k;
  boolean_T exitg1;
  memcpy(&b_A[0], &A[0], 144U * sizeof(double));
  anrm = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 144)) {
    absxk = fabs(A[k]);
    if (rtIsNaN(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      k++;
    }
  }
  if (rtIsInf(anrm) || rtIsNaN(anrm)) {
    for (i = 0; i < 12; i++) {
      W[i].re = rtNaN;
      W[i].im = 0.0;
    }
    for (i = 0; i < 144; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
  } else {
    double wi[12];
    double wr[12];
    double tau[11];
    double cscale;
    int ilo;
    int info;
    boolean_T scalea;
    cscale = anrm;
    scalea = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      scalea = true;
      cscale = 6.7178761075670888E-139;
      xzlascl(anrm, cscale, b_A);
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      cscale = 1.4885657073574029E+138;
      xzlascl(anrm, cscale, b_A);
    }
    ilo = xzgebal(b_A, &ihi, scale);
    xzgehrd(b_A, ilo, ihi, tau);
    memcpy(&vr[0], &b_A[0], 144U * sizeof(double));
    xzunghr(ilo, ihi, vr, tau);
    info = xdlahqr(ilo, ihi, b_A, ilo, ihi, vr, wr, wi);
    if (info == 0) {
      double temp;
      int ix;
      int vr_tmp;
      xdtrevc3(b_A, vr);
      if (ilo != ihi) {
        for (i = ilo; i <= ihi; i++) {
          k = i + 132;
          for (b_k = i; b_k <= k; b_k += 12) {
            vr[b_k - 1] *= scale[i - 1];
          }
        }
      }
      k = ilo - 1;
      for (i = k; i >= 1; i--) {
        absxk = scale[i - 1];
        if ((int)absxk != i) {
          for (b_k = 0; b_k < 12; b_k++) {
            ix = (i + b_k * 12) - 1;
            temp = vr[ix];
            vr_tmp = ((int)absxk + b_k * 12) - 1;
            vr[ix] = vr[vr_tmp];
            vr[vr_tmp] = temp;
          }
        }
      }
      k = ihi + 1;
      for (i = k; i < 13; i++) {
        absxk = scale[i - 1];
        if ((int)absxk != i) {
          for (b_k = 0; b_k < 12; b_k++) {
            ix = (i + b_k * 12) - 1;
            temp = vr[ix];
            vr_tmp = ((int)absxk + b_k * 12) - 1;
            vr[ix] = vr[vr_tmp];
            vr[vr_tmp] = temp;
          }
        }
      }
      for (b_k = 0; b_k < 12; b_k++) {
        absxk = wi[b_k];
        if (!(absxk < 0.0)) {
          if ((b_k + 1 != 12) && (absxk > 0.0)) {
            __m128d r;
            double c;
            double d;
            double f1_tmp;
            double g1_tmp;
            int b_tmp;
            absxk = fabs(xnrm2(12, vr, b_k * 12 + 1));
            b_tmp = (b_k + 1) * 12;
            temp = fabs(xnrm2(12, vr, b_tmp + 1));
            if (absxk < temp) {
              absxk /= temp;
              absxk = temp * sqrt(absxk * absxk + 1.0);
            } else if (absxk > temp) {
              temp /= absxk;
              absxk *= sqrt(temp * temp + 1.0);
            } else if (rtIsNaN(temp)) {
              absxk = rtNaN;
            } else {
              absxk *= 1.4142135623730951;
            }
            absxk = 1.0 / absxk;
            k = b_k * 12;
            ix = k + 12;
            vr_tmp = k + 13;
            ihi = k + 11;
            for (i = k + 1; i <= ihi; i += 2) {
              r = _mm_loadu_pd(&vr[i - 1]);
              _mm_storeu_pd(&vr[i - 1], _mm_mul_pd(_mm_set1_pd(absxk), r));
            }
            for (i = vr_tmp; i <= ix; i++) {
              vr[i - 1] *= absxk;
            }
            k = b_tmp + 12;
            ix = b_tmp + 13;
            vr_tmp = b_tmp + 11;
            for (i = b_tmp + 1; i <= vr_tmp; i += 2) {
              r = _mm_loadu_pd(&vr[i - 1]);
              _mm_storeu_pd(&vr[i - 1], _mm_mul_pd(_mm_set1_pd(absxk), r));
            }
            for (i = ix; i <= k; i++) {
              vr[i - 1] *= absxk;
            }
            for (i = 0; i <= 10; i += 2) {
              __m128d r1;
              r = _mm_loadu_pd(&vr[i + 12 * b_k]);
              r1 = _mm_loadu_pd(&vr[i + b_tmp]);
              _mm_storeu_pd(&scale[i],
                            _mm_add_pd(_mm_mul_pd(r, r), _mm_mul_pd(r1, r1)));
            }
            k = 0;
            absxk = scale[0];
            for (i = 0; i < 11; i++) {
              temp = scale[i + 1];
              if (temp > absxk) {
                k = i + 1;
                absxk = temp;
              }
            }
            f1_tmp = vr[k + 12 * b_k];
            temp = fabs(f1_tmp);
            k += b_tmp;
            g1_tmp = vr[k];
            absxk = fabs(g1_tmp);
            if (g1_tmp == 0.0) {
              c = 1.0;
              absxk = 0.0;
            } else if (f1_tmp == 0.0) {
              c = 0.0;
              if (g1_tmp >= 0.0) {
                absxk = 1.0;
              } else {
                absxk = -1.0;
              }
            } else if ((temp > 1.4916681462400413E-154) &&
                       (temp < 4.7403759540545887E+153) &&
                       (absxk > 1.4916681462400413E-154) &&
                       (absxk < 4.7403759540545887E+153)) {
              d = sqrt(f1_tmp * f1_tmp + g1_tmp * g1_tmp);
              c = temp / d;
              if (!(f1_tmp >= 0.0)) {
                d = -d;
              }
              absxk = g1_tmp / d;
            } else {
              absxk = fmin(4.49423283715579E+307,
                           fmax(2.2250738585072014E-308, fmax(temp, absxk)));
              temp = f1_tmp / absxk;
              absxk = g1_tmp / absxk;
              d = sqrt(temp * temp + absxk * absxk);
              c = fabs(temp) / d;
              if (!(f1_tmp >= 0.0)) {
                d = -d;
              }
              absxk /= d;
            }
            ix = b_k * 12;
            for (i = 0; i < 12; i++) {
              vr_tmp = b_tmp + i;
              temp = vr[vr_tmp];
              ihi = ix + i;
              d = vr[ihi];
              vr[vr_tmp] = c * temp - absxk * d;
              vr[ihi] = c * d + absxk * temp;
            }
            vr[k] = 0.0;
          } else {
            absxk = 1.0 / xnrm2(12, vr, b_k * 12 + 1);
            k = b_k * 12;
            ix = k + 12;
            vr_tmp = k + 13;
            ihi = k + 11;
            for (i = k + 1; i <= ihi; i += 2) {
              __m128d r;
              r = _mm_loadu_pd(&vr[i - 1]);
              _mm_storeu_pd(&vr[i - 1], _mm_mul_pd(_mm_set1_pd(absxk), r));
            }
            for (i = vr_tmp; i <= ix; i++) {
              vr[i - 1] *= absxk;
            }
          }
        }
      }
      for (i = 0; i < 144; i++) {
        V[i].re = vr[i];
        V[i].im = 0.0;
      }
      for (i = 0; i < 11; i++) {
        if ((wi[i] > 0.0) && (wi[i + 1] < 0.0)) {
          for (b_k = 0; b_k < 12; b_k++) {
            k = b_k + 12 * i;
            ix = b_k + 12 * (i + 1);
            absxk = V[ix].re;
            V[k].im = absxk;
            V[ix].re = V[k].re;
            V[ix].im = -absxk;
          }
        }
      }
    } else {
      for (i = 0; i < 144; i++) {
        V[i].re = rtNaN;
        V[i].im = 0.0;
      }
    }
    if (scalea) {
      b_xzlascl(cscale, anrm, 12 - info, wr, info + 1);
      b_xzlascl(cscale, anrm, 12 - info, wi, info + 1);
      if (info != 0) {
        b_xzlascl(cscale, anrm, ilo - 1, wr, 1);
        b_xzlascl(cscale, anrm, ilo - 1, wi, 1);
      }
    }
    if (info != 0) {
      for (i = ilo; i <= info; i++) {
        wr[i - 1] = rtNaN;
        wi[i - 1] = 0.0;
      }
    }
    for (i = 0; i < 12; i++) {
      W[i].re = wr[i];
      W[i].im = wi[i];
    }
  }
  memset(&D[0], 0, 144U * sizeof(creal_T));
  for (i = 0; i < 12; i++) {
    D[i + 12 * i] = W[i];
  }
}

/* End of code generation (eigStandard.c) */
