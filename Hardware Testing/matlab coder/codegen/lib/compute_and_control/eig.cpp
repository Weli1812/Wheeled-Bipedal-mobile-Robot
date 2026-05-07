//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eig.cpp
//
// Code generation for function 'eig'
//

// Include files
#include "eig.h"
#include "compute_and_control_data.h"
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xzgehrd.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include "xzsteqr.h"
#include "xzunghr.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
void eig(const double A[144], creal_T V[144], creal_T D[144])
{
  double Q[144];
  double b_A[144];
  double a__4[12];
  double work[12];
  double e[11];
  double tau[11];
  double absx;
  boolean_T iscale;
  iscale = true;
  for (int i{0}; i < 144; i++) {
    if (iscale) {
      absx = A[i];
      if (std::isinf(absx) || std::isnan(absx)) {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }
  if (!iscale) {
    for (int i{0}; i < 144; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
      D[i].re = 0.0;
      D[i].im = 0.0;
    }
    for (int i{0}; i < 12; i++) {
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
      std::copy(&A[0], &A[144], &b_A[0]);
      anrm = 0.0;
      j = 0;
      exitg2 = false;
      while ((!exitg2) && (j < 12)) {
        itau = 0;
        do {
          exitg1 = 0;
          if (itau <= j) {
            absx = std::abs(A[itau + 12 * j]);
            if (std::isnan(absx)) {
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
      if (std::isinf(anrm) || std::isnan(anrm)) {
        for (int i{0}; i < 12; i++) {
          a__4[i] = rtNaN;
        }
        for (int i{0}; i < 144; i++) {
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
          internal::reflapack::xzlascl(1.0, anrm, b_A);
        } else if (anrm > 9.9895953610111751E+145) {
          iscale = true;
          anrm = 9.9895953610111751E+145 / anrm;
          internal::reflapack::xzlascl(1.0, anrm, b_A);
        }
        for (int b_i{0}; b_i < 11; b_i++) {
          double taui;
          int e_tmp_tmp;
          e_tmp_tmp = b_i + 12 * b_i;
          e[b_i] = b_A[e_tmp_tmp + 1];
          j = b_i + 3;
          if (j > 12) {
            j = 12;
          }
          taui =
              internal::reflapack::xzlarfg(11 - b_i, e[b_i], b_A, b_i * 12 + j);
          if (taui != 0.0) {
            __m128d r1;
            double temp1;
            double temp2;
            int d_i;
            b_A[e_tmp_tmp + 1] = 1.0;
            for (int i{b_i + 1}; i < 12; i++) {
              tau[i - 1] = 0.0;
            }
            j = 10 - b_i;
            sgn = 12 - b_i;
            for (int i{0}; i <= j; i++) {
              temp1_tmp = b_i + i;
              temp1 = taui * b_A[(temp1_tmp + 12 * b_i) + 1];
              temp2 = 0.0;
              itau = 12 * (temp1_tmp + 1);
              tau[temp1_tmp] += temp1 * b_A[(temp1_tmp + itau) + 1];
              scalarLB = i + 2;
              for (int c_i{scalarLB}; c_i < sgn; c_i++) {
                iaii = b_i + c_i;
                absx = b_A[iaii + itau];
                tau[iaii - 1] += temp1 * absx;
                temp2 += absx * b_A[iaii + 12 * b_i];
              }
              tau[temp1_tmp] += taui * temp2;
            }
            j = 10 - b_i;
            absx = 0.0;
            for (int i{0}; i <= j; i++) {
              absx += tau[b_i + i] * b_A[(e_tmp_tmp + i) + 1];
            }
            absx *= -0.5 * taui;
            if (!(absx == 0.0)) {
              sgn = 11 - b_i;
              temp1_tmp = ((11 - b_i) / 2) << 1;
              itau = temp1_tmp - 2;
              for (int i{0}; i <= itau; i += 2) {
                r = _mm_loadu_pd(&b_A[(e_tmp_tmp + i) + 1]);
                j = b_i + i;
                r1 = _mm_loadu_pd(&tau[j]);
                _mm_storeu_pd(&tau[j],
                              _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(absx), r)));
              }
              for (int i{temp1_tmp}; i < sgn; i++) {
                j = b_i + i;
                tau[j] += absx * b_A[(e_tmp_tmp + i) + 1];
              }
            }
            temp1_tmp = 10 - b_i;
            d_i = 12 - b_i;
            for (int c_i{0}; c_i <= temp1_tmp; c_i++) {
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
              for (int i{scalarLB}; i <= j; i += 2) {
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
              for (int i{iaii}; i < d_i; i++) {
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
        for (int i{10}; i >= 0; i--) {
          j = 12 * (i + 1);
          b_A[j] = 0.0;
          itau = i + 3;
          for (int c_i{itau}; c_i < 13; c_i++) {
            b_A[(c_i + j) - 1] = b_A[(c_i + 12 * i) - 1];
          }
        }
        b_A[0] = 1.0;
        std::memset(&b_A[1], 0, 11U * sizeof(double));
        itau = 10;
        std::memset(&work[0], 0, 12U * sizeof(double));
        for (int c_i{10}; c_i >= 0; c_i--) {
          iaii = (c_i + c_i * 12) + 13;
          if (c_i + 1 < 11) {
            b_A[iaii] = 1.0;
            internal::reflapack::xzlarf(11 - c_i, 10 - c_i, iaii + 1, tau[itau],
                                        b_A, iaii + 13, work);
            j = iaii + 2;
            sgn = (iaii - c_i) + 11;
            scalarLB = (((((sgn - iaii) - 1) / 2) << 1) + iaii) + 2;
            temp1_tmp = scalarLB - 2;
            for (int i{j}; i <= temp1_tmp; i += 2) {
              r = _mm_loadu_pd(&b_A[i - 1]);
              _mm_storeu_pd(&b_A[i - 1],
                            _mm_mul_pd(_mm_set1_pd(-tau[itau]), r));
            }
            for (int i{scalarLB}; i <= sgn; i++) {
              b_A[i - 1] *= -tau[itau];
            }
          }
          b_A[iaii] = 1.0 - tau[itau];
          for (int i{0}; i < c_i; i++) {
            b_A[(iaii - i) - 1] = 0.0;
          }
          itau = c_i - 1;
        }
        j = internal::reflapack::xzsteqr(a__4, e, b_A);
        if (j != 0) {
          for (int i{0}; i < 12; i++) {
            a__4[i] = rtNaN;
          }
          for (int i{0}; i < 144; i++) {
            b_A[i] = rtNaN;
          }
        } else if (iscale) {
          absx = 1.0 / anrm;
          for (int i{0}; i <= 10; i += 2) {
            r = _mm_loadu_pd(&a__4[i]);
            _mm_storeu_pd(&a__4[i], _mm_mul_pd(_mm_set1_pd(absx), r));
          }
        }
      }
      std::memset(&D[0], 0, 144U * sizeof(creal_T));
      for (int i{0}; i < 12; i++) {
        j = i + 12 * i;
        D[j].re = a__4[i];
        D[j].im = 0.0;
      }
      for (int i{0}; i < 144; i++) {
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
        std::copy(&A[0], &A[144], &b_A[0]);
        internal::reflapack::xzgehrd(b_A, 1, 12, tau);
        std::copy(&b_A[0], &b_A[144], &Q[0]);
        internal::reflapack::xzunghr(1, 12, Q, tau);
        sgn = internal::reflapack::xdlahqr(1, 12, b_A, 1, 12, Q, a__4, work);
        std::memset(&D[0], 0, 144U * sizeof(creal_T));
        j = static_cast<unsigned char>(sgn);
        for (int i{0}; i < j; i++) {
          itau = i + 12 * i;
          D[itau].re = rtNaN;
          D[itau].im = 0.0;
        }
        j = sgn + 1;
        for (int i{j}; i < 13; i++) {
          itau = (i + 12 * (i - 1)) - 1;
          D[itau].re = 0.0;
          D[itau].im = work[i - 1];
        }
        if (sgn == 0) {
          for (int i{0}; i < 144; i++) {
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
                  for (int i{0}; i < 12; i++) {
                    double temp1;
                    int scalarLB;
                    int temp1_tmp;
                    temp1_tmp = i + itau;
                    absx = V[temp1_tmp].re;
                    scalarLB = i + 12 * j;
                    temp1 = static_cast<double>(sgn) * V[scalarLB].re;
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
          for (int i{0}; i < 144; i++) {
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

} // namespace coder

// End of code generation (eig.cpp)
