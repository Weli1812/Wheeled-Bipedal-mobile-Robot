/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eig.c
 *
 * Code generation for function 'eig'
 *
 */

/* Include files */
#include "eig.h"
#include "compute_robot_lqr_data.h"
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdsterf.h"
#include "xzgehrd.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include "xzsteqr.h"
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include <emmintrin.h>
#include <math.h>
#include <string.h>

/* Function Definitions */
void b_eig(const double A[36], creal_T V[6])
{
  double b_A[36];
  double a__3[6];
  double e[5];
  double tau[5];
  double absx;
  int c_i;
  int i;
  int ii;
  boolean_T iscale;
  iscale = true;
  for (i = 0; i < 36; i++) {
    if (iscale) {
      absx = A[i];
      if (rtIsInf(absx) || rtIsNaN(absx)) {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }
  if (!iscale) {
    for (i = 0; i < 6; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
  } else {
    int b_i;
    int exitg1;
    int j;
    boolean_T exitg2;
    iscale = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 6)) {
      b_i = 0;
      do {
        exitg1 = 0;
        if (b_i <= j) {
          if (!(A[b_i + 6 * j] == A[j + 6 * b_i])) {
            iscale = false;
            exitg1 = 1;
          } else {
            b_i++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (iscale) {
      double anrm;
      memcpy(&b_A[0], &A[0], 36U * sizeof(double));
      anrm = 0.0;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 6)) {
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= j) {
            absx = fabs(A[b_i + 6 * j]);
            if (rtIsNaN(absx)) {
              anrm = rtNaN;
              exitg1 = 1;
            } else {
              if (absx > anrm) {
                anrm = absx;
              }
              b_i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (rtIsInf(anrm) || rtIsNaN(anrm)) {
        for (i = 0; i < 6; i++) {
          a__3[i] = rtNaN;
        }
      } else {
        __m128d r;
        __m128d r1;
        iscale = false;
        if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
          iscale = true;
          anrm = 1.0010415475915505E-146 / anrm;
          d_xzlascl(1.0, anrm, b_A);
        } else if (anrm > 9.9895953610111751E+145) {
          iscale = true;
          anrm = 9.9895953610111751E+145 / anrm;
          d_xzlascl(1.0, anrm, b_A);
        }
        for (c_i = 0; c_i < 5; c_i++) {
          double taui;
          int e_tmp_tmp;
          e_tmp_tmp = c_i + 6 * c_i;
          e[c_i] = b_A[e_tmp_tmp + 1];
          j = c_i + 3;
          if (j > 6) {
            j = 6;
          }
          taui = c_xzlarfg(5 - c_i, &e[c_i], b_A, c_i * 6 + j);
          if (taui != 0.0) {
            double temp1;
            double temp2;
            int b_tau_tmp;
            int d_i;
            int scalarLB;
            int tau_tmp;
            int temp1_tmp;
            b_A[e_tmp_tmp + 1] = 1.0;
            for (i = c_i + 1; i < 6; i++) {
              tau[i - 1] = 0.0;
            }
            j = 4 - c_i;
            b_i = 6 - c_i;
            for (i = 0; i <= j; i++) {
              temp1_tmp = c_i + i;
              temp1 = taui * b_A[(temp1_tmp + 6 * c_i) + 1];
              temp2 = 0.0;
              tau_tmp = 6 * (temp1_tmp + 1);
              tau[temp1_tmp] += temp1 * b_A[(temp1_tmp + tau_tmp) + 1];
              scalarLB = i + 2;
              for (ii = scalarLB; ii < b_i; ii++) {
                b_tau_tmp = c_i + ii;
                absx = b_A[b_tau_tmp + tau_tmp];
                tau[b_tau_tmp - 1] += temp1 * absx;
                temp2 += absx * b_A[b_tau_tmp + 6 * c_i];
              }
              tau[temp1_tmp] += taui * temp2;
            }
            j = 4 - c_i;
            absx = 0.0;
            for (i = 0; i <= j; i++) {
              absx += tau[c_i + i] * b_A[(e_tmp_tmp + i) + 1];
            }
            absx *= -0.5 * taui;
            if (!(absx == 0.0)) {
              temp1_tmp = 5 - c_i;
              tau_tmp = ((5 - c_i) / 2) << 1;
              j = tau_tmp - 2;
              for (i = 0; i <= j; i += 2) {
                r = _mm_loadu_pd(&b_A[(e_tmp_tmp + i) + 1]);
                b_i = c_i + i;
                r1 = _mm_loadu_pd(&tau[b_i]);
                _mm_storeu_pd(&tau[b_i],
                              _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(absx), r)));
              }
              for (i = tau_tmp; i < temp1_tmp; i++) {
                j = c_i + i;
                tau[j] += absx * b_A[(e_tmp_tmp + i) + 1];
              }
            }
            b_tau_tmp = 4 - c_i;
            d_i = 6 - c_i;
            for (ii = 0; ii <= b_tau_tmp; ii++) {
              int A_tmp_tmp;
              b_i = c_i + ii;
              temp1 = b_A[(b_i + 6 * c_i) + 1];
              temp2 = tau[b_i];
              absx = temp2 * temp1;
              A_tmp_tmp = 6 * (b_i + 1);
              j = (b_i + A_tmp_tmp) + 1;
              b_A[j] = (b_A[j] - absx) - absx;
              tau_tmp = ii + 2;
              scalarLB = ((((4 - b_i) / 2) << 1) + ii) + 2;
              j = scalarLB - 2;
              for (i = tau_tmp; i <= j; i += 2) {
                __m128d r2;
                b_i = c_i + i;
                r = _mm_loadu_pd(&tau[b_i - 1]);
                temp1_tmp = b_i + A_tmp_tmp;
                r1 = _mm_loadu_pd(&b_A[temp1_tmp]);
                r2 = _mm_loadu_pd(&b_A[b_i + 6 * c_i]);
                _mm_storeu_pd(
                    &b_A[temp1_tmp],
                    _mm_sub_pd(
                        _mm_sub_pd(r1, _mm_mul_pd(r, _mm_set1_pd(temp1))),
                        _mm_mul_pd(r2, _mm_set1_pd(temp2))));
              }
              for (i = scalarLB; i < d_i; i++) {
                j = c_i + i;
                b_i = j + A_tmp_tmp;
                b_A[b_i] =
                    (b_A[b_i] - tau[j - 1] * temp1) - b_A[j + 6 * c_i] * temp2;
              }
            }
          }
          b_A[e_tmp_tmp + 1] = e[c_i];
          a__3[c_i] = b_A[e_tmp_tmp];
          tau[c_i] = taui;
        }
        a__3[5] = b_A[35];
        j = xdsterf(a__3, e);
        if (j != 0) {
          for (i = 0; i < 6; i++) {
            a__3[i] = rtNaN;
          }
        } else if (iscale) {
          r = _mm_loadu_pd(&a__3[0]);
          r1 = _mm_set1_pd(1.0 / anrm);
          _mm_storeu_pd(&a__3[0], _mm_mul_pd(r1, r));
          r = _mm_loadu_pd(&a__3[2]);
          _mm_storeu_pd(&a__3[2], _mm_mul_pd(r1, r));
          r = _mm_loadu_pd(&a__3[4]);
          _mm_storeu_pd(&a__3[4], _mm_mul_pd(r1, r));
        }
      }
      for (i = 0; i < 6; i++) {
        V[i].re = a__3[i];
        V[i].im = 0.0;
      }
    } else {
      iscale = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 6)) {
        b_i = 0;
        do {
          exitg1 = 0;
          if (b_i <= j) {
            if (!(A[b_i + 6 * j] == -A[j + 6 * b_i])) {
              iscale = false;
              exitg1 = 1;
            } else {
              b_i++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (iscale) {
        double wi[6];
        memcpy(&b_A[0], &A[0], 36U * sizeof(double));
        b_xzgehrd(b_A, 1, 6);
        j = b_xdlahqr(1, 6, b_A, &absx, a__3, wi);
        b_i = (unsigned char)j;
        for (i = 0; i < b_i; i++) {
          V[i].re = rtNaN;
          V[i].im = 0.0;
        }
        j++;
        for (i = j; i < 7; i++) {
          V[i - 1].re = 0.0;
          V[i - 1].im = wi[i - 1];
        }
      } else {
        b_eigStandard(A, V);
      }
    }
  }
}

void eig(const double A[144], creal_T V[144], creal_T D[144])
{
  double Q[144];
  double b_A[144];
  double a__4[12];
  double work[12];
  double e[11];
  double tau[11];
  double absx;
  int b_i;
  int c_i;
  int i;
  boolean_T iscale;
  iscale = true;
  for (i = 0; i < 144; i++) {
    if (iscale) {
      absx = A[i];
      if (rtIsInf(absx) || rtIsNaN(absx)) {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }
  if (!iscale) {
    for (i = 0; i < 144; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
      D[i].re = 0.0;
      D[i].im = 0.0;
    }
    for (i = 0; i < 12; i++) {
      int j;
      j = i + 12 * i;
      D[j].re = rtNaN;
      D[j].im = 0.0;
    }
  } else {
    int exitg1;
    int itau;
    int j;
    boolean_T exitg2;
    iscale = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 12)) {
      itau = 0;
      do {
        exitg1 = 0;
        if (itau <= j) {
          if (!(A[itau + 12 * j] == A[j + 12 * itau])) {
            iscale = false;
            exitg1 = 1;
          } else {
            itau++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (iscale) {
      double anrm;
      memcpy(&b_A[0], &A[0], 144U * sizeof(double));
      anrm = 0.0;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 12)) {
        itau = 0;
        do {
          exitg1 = 0;
          if (itau <= j) {
            absx = fabs(A[itau + 12 * j]);
            if (rtIsNaN(absx)) {
              anrm = rtNaN;
              exitg1 = 1;
            } else {
              if (absx > anrm) {
                anrm = absx;
              }
              itau++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (rtIsInf(anrm) || rtIsNaN(anrm)) {
        for (i = 0; i < 12; i++) {
          a__4[i] = rtNaN;
        }
        for (i = 0; i < 144; i++) {
          b_A[i] = rtNaN;
        }
      } else {
        __m128d r;
        int iaii;
        int scalarLB;
        int sgn;
        int temp1_tmp;
        iscale = false;
        if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
          iscale = true;
          anrm = 1.0010415475915505E-146 / anrm;
          xzlascl(1.0, anrm, b_A);
        } else if (anrm > 9.9895953610111751E+145) {
          iscale = true;
          anrm = 9.9895953610111751E+145 / anrm;
          xzlascl(1.0, anrm, b_A);
        }
        for (b_i = 0; b_i < 11; b_i++) {
          double taui;
          int e_tmp_tmp;
          e_tmp_tmp = b_i + 12 * b_i;
          e[b_i] = b_A[e_tmp_tmp + 1];
          j = b_i + 3;
          if (j > 12) {
            j = 12;
          }
          taui = xzlarfg(11 - b_i, &e[b_i], b_A, b_i * 12 + j);
          if (taui != 0.0) {
            __m128d r1;
            double temp1;
            double temp2;
            int d_i;
            b_A[e_tmp_tmp + 1] = 1.0;
            for (i = b_i + 1; i < 12; i++) {
              tau[i - 1] = 0.0;
            }
            j = 10 - b_i;
            sgn = 12 - b_i;
            for (i = 0; i <= j; i++) {
              temp1_tmp = b_i + i;
              temp1 = taui * b_A[(temp1_tmp + 12 * b_i) + 1];
              temp2 = 0.0;
              itau = 12 * (temp1_tmp + 1);
              tau[temp1_tmp] += temp1 * b_A[(temp1_tmp + itau) + 1];
              scalarLB = i + 2;
              for (c_i = scalarLB; c_i < sgn; c_i++) {
                iaii = b_i + c_i;
                absx = b_A[iaii + itau];
                tau[iaii - 1] += temp1 * absx;
                temp2 += absx * b_A[iaii + 12 * b_i];
              }
              tau[temp1_tmp] += taui * temp2;
            }
            j = 10 - b_i;
            absx = 0.0;
            for (i = 0; i <= j; i++) {
              absx += tau[b_i + i] * b_A[(e_tmp_tmp + i) + 1];
            }
            absx *= -0.5 * taui;
            if (!(absx == 0.0)) {
              sgn = 11 - b_i;
              temp1_tmp = ((11 - b_i) / 2) << 1;
              itau = temp1_tmp - 2;
              for (i = 0; i <= itau; i += 2) {
                r = _mm_loadu_pd(&b_A[(e_tmp_tmp + i) + 1]);
                j = b_i + i;
                r1 = _mm_loadu_pd(&tau[j]);
                _mm_storeu_pd(&tau[j],
                              _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(absx), r)));
              }
              for (i = temp1_tmp; i < sgn; i++) {
                j = b_i + i;
                tau[j] += absx * b_A[(e_tmp_tmp + i) + 1];
              }
            }
            temp1_tmp = 10 - b_i;
            d_i = 12 - b_i;
            for (c_i = 0; c_i <= temp1_tmp; c_i++) {
              int A_tmp_tmp;
              j = b_i + c_i;
              temp1 = b_A[(j + 12 * b_i) + 1];
              temp2 = tau[j];
              absx = temp2 * temp1;
              A_tmp_tmp = 12 * (j + 1);
              itau = (j + A_tmp_tmp) + 1;
              b_A[itau] = (b_A[itau] - absx) - absx;
              scalarLB = c_i + 2;
              iaii = ((((10 - j) / 2) << 1) + c_i) + 2;
              j = iaii - 2;
              for (i = scalarLB; i <= j; i += 2) {
                __m128d r2;
                sgn = b_i + i;
                r = _mm_loadu_pd(&tau[sgn - 1]);
                itau = sgn + A_tmp_tmp;
                r1 = _mm_loadu_pd(&b_A[itau]);
                r2 = _mm_loadu_pd(&b_A[sgn + 12 * b_i]);
                _mm_storeu_pd(
                    &b_A[itau],
                    _mm_sub_pd(
                        _mm_sub_pd(r1, _mm_mul_pd(r, _mm_set1_pd(temp1))),
                        _mm_mul_pd(r2, _mm_set1_pd(temp2))));
              }
              for (i = iaii; i < d_i; i++) {
                j = b_i + i;
                sgn = j + A_tmp_tmp;
                b_A[sgn] =
                    (b_A[sgn] - tau[j - 1] * temp1) - b_A[j + 12 * b_i] * temp2;
              }
            }
          }
          b_A[e_tmp_tmp + 1] = e[b_i];
          a__4[b_i] = b_A[e_tmp_tmp];
          tau[b_i] = taui;
        }
        a__4[11] = b_A[143];
        for (i = 10; i >= 0; i--) {
          j = 12 * (i + 1);
          b_A[j] = 0.0;
          itau = i + 3;
          for (c_i = itau; c_i < 13; c_i++) {
            b_A[(c_i + j) - 1] = b_A[(c_i + 12 * i) - 1];
          }
        }
        b_A[0] = 1.0;
        memset(&b_A[1], 0, 11U * sizeof(double));
        itau = 10;
        memset(&work[0], 0, 12U * sizeof(double));
        for (c_i = 10; c_i >= 0; c_i--) {
          iaii = (c_i + c_i * 12) + 13;
          if (c_i + 1 < 11) {
            b_A[iaii] = 1.0;
            xzlarf(11 - c_i, 10 - c_i, iaii + 1, tau[itau], b_A, iaii + 13,
                   work);
            j = iaii + 2;
            sgn = (iaii - c_i) + 11;
            scalarLB = (((((sgn - iaii) - 1) / 2) << 1) + iaii) + 2;
            temp1_tmp = scalarLB - 2;
            for (i = j; i <= temp1_tmp; i += 2) {
              r = _mm_loadu_pd(&b_A[i - 1]);
              _mm_storeu_pd(&b_A[i - 1],
                            _mm_mul_pd(_mm_set1_pd(-tau[itau]), r));
            }
            for (i = scalarLB; i <= sgn; i++) {
              b_A[i - 1] *= -tau[itau];
            }
          }
          b_A[iaii] = 1.0 - tau[itau];
          for (i = 0; i < c_i; i++) {
            b_A[(iaii - i) - 1] = 0.0;
          }
          itau = c_i - 1;
        }
        j = xzsteqr(a__4, e, b_A);
        if (j != 0) {
          for (i = 0; i < 12; i++) {
            a__4[i] = rtNaN;
          }
          for (i = 0; i < 144; i++) {
            b_A[i] = rtNaN;
          }
        } else if (iscale) {
          absx = 1.0 / anrm;
          for (i = 0; i <= 10; i += 2) {
            r = _mm_loadu_pd(&a__4[i]);
            _mm_storeu_pd(&a__4[i], _mm_mul_pd(_mm_set1_pd(absx), r));
          }
        }
      }
      memset(&D[0], 0, 144U * sizeof(creal_T));
      for (i = 0; i < 12; i++) {
        j = i + 12 * i;
        D[j].re = a__4[i];
        D[j].im = 0.0;
      }
      for (i = 0; i < 144; i++) {
        V[i].re = b_A[i];
        V[i].im = 0.0;
      }
    } else {
      iscale = true;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 12)) {
        itau = 0;
        do {
          exitg1 = 0;
          if (itau <= j) {
            if (!(A[itau + 12 * j] == -A[j + 12 * itau])) {
              iscale = false;
              exitg1 = 1;
            } else {
              itau++;
            }
          } else {
            j++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (iscale) {
        int sgn;
        memcpy(&b_A[0], &A[0], 144U * sizeof(double));
        xzgehrd(b_A, 1, 12, tau);
        memcpy(&Q[0], &b_A[0], 144U * sizeof(double));
        xzunghr(1, 12, Q, tau);
        sgn = xdlahqr(1, 12, b_A, 1, 12, Q, a__4, work);
        memset(&D[0], 0, 144U * sizeof(creal_T));
        j = (unsigned char)sgn;
        for (i = 0; i < j; i++) {
          itau = i + 12 * i;
          D[itau].re = rtNaN;
          D[itau].im = 0.0;
        }
        j = sgn + 1;
        for (i = j; i < 13; i++) {
          itau = (i + 12 * (i - 1)) - 1;
          D[itau].re = 0.0;
          D[itau].im = work[i - 1];
        }
        if (sgn == 0) {
          for (i = 0; i < 144; i++) {
            V[i].re = Q[i];
            V[i].im = 0.0;
          }
          j = 1;
          do {
            exitg1 = 0;
            if (j <= 12) {
              if (j != 12) {
                itau = 12 * (j - 1);
                absx = b_A[j + itau];
                if (absx != 0.0) {
                  if (absx < 0.0) {
                    sgn = 1;
                  } else {
                    sgn = -1;
                  }
                  for (i = 0; i < 12; i++) {
                    double temp1;
                    int scalarLB;
                    int temp1_tmp;
                    temp1_tmp = i + itau;
                    absx = V[temp1_tmp].re;
                    scalarLB = i + 12 * j;
                    temp1 = (double)sgn * V[scalarLB].re;
                    if (temp1 == 0.0) {
                      V[temp1_tmp].re = absx / 1.4142135623730951;
                      V[temp1_tmp].im = 0.0;
                    } else if (absx == 0.0) {
                      V[temp1_tmp].re = 0.0;
                      V[temp1_tmp].im = temp1 / 1.4142135623730951;
                    } else {
                      V[temp1_tmp].re = absx / 1.4142135623730951;
                      V[temp1_tmp].im = temp1 / 1.4142135623730951;
                    }
                    V[scalarLB].re = V[temp1_tmp].re;
                    V[scalarLB].im = -V[temp1_tmp].im;
                  }
                  j += 2;
                } else {
                  j++;
                }
              } else {
                j++;
              }
            } else {
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        } else {
          for (i = 0; i < 144; i++) {
            V[i].re = rtNaN;
            V[i].im = 0.0;
          }
        }
      } else {
        eigStandard(A, V, D);
      }
    }
  }
}

/* End of code generation (eig.c) */
