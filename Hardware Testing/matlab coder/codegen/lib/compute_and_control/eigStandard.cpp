//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// eigStandard.cpp
//
// Code generation for function 'eigStandard'
//

// Include files
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdtrevc3.h"
#include "xnrm2.h"
#include "xzgebal.h"
#include "xzgehrd.h"
#include "xzlascl.h"
#include "xzunghr.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
void eigStandard(const double A[144], creal_T V[144], creal_T D[144])
{
  creal_T W[12];
  double b_A[144];
  double vr[144];
  double scale[12];
  double absxk;
  double anrm;
  int ihi;
  int k;
  boolean_T exitg1;
  std::copy(&A[0], &A[144], &b_A[0]);
  anrm = 0.0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 144)) {
    absxk = std::abs(A[k]);
    if (std::isnan(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      k++;
    }
  }
  if (std::isinf(anrm) || std::isnan(anrm)) {
    for (int i{0}; i < 12; i++) {
      W[i].re = rtNaN;
      W[i].im = 0.0;
    }
    for (int i{0}; i < 144; i++) {
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
      internal::reflapack::xzlascl(anrm, cscale, b_A);
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      cscale = 1.4885657073574029E+138;
      internal::reflapack::xzlascl(anrm, cscale, b_A);
    }
    ilo = internal::reflapack::xzgebal(b_A, ihi, scale);
    internal::reflapack::xzgehrd(b_A, ilo, ihi, tau);
    std::copy(&b_A[0], &b_A[144], &vr[0]);
    internal::reflapack::xzunghr(ilo, ihi, vr, tau);
    info = internal::reflapack::xdlahqr(ilo, ihi, b_A, ilo, ihi, vr, wr, wi);
    if (info == 0) {
      double temp;
      int ix;
      int vr_tmp;
      internal::reflapack::xdtrevc3(b_A, vr);
      if (ilo != ihi) {
        for (int i{ilo}; i <= ihi; i++) {
          k = i + 132;
          for (int b_k{i}; b_k <= k; b_k += 12) {
            vr[b_k - 1] *= scale[i - 1];
          }
        }
      }
      k = ilo - 1;
      for (int i{k}; i >= 1; i--) {
        absxk = scale[i - 1];
        if (static_cast<int>(absxk) != i) {
          for (int b_k{0}; b_k < 12; b_k++) {
            ix = (i + b_k * 12) - 1;
            temp = vr[ix];
            vr_tmp = (static_cast<int>(absxk) + b_k * 12) - 1;
            vr[ix] = vr[vr_tmp];
            vr[vr_tmp] = temp;
          }
        }
      }
      k = ihi + 1;
      for (int i{k}; i < 13; i++) {
        absxk = scale[i - 1];
        if (static_cast<int>(absxk) != i) {
          for (int b_k{0}; b_k < 12; b_k++) {
            ix = (i + b_k * 12) - 1;
            temp = vr[ix];
            vr_tmp = (static_cast<int>(absxk) + b_k * 12) - 1;
            vr[ix] = vr[vr_tmp];
            vr[vr_tmp] = temp;
          }
        }
      }
      for (int b_k{0}; b_k < 12; b_k++) {
        absxk = wi[b_k];
        if (!(absxk < 0.0)) {
          if ((b_k + 1 != 12) && (absxk > 0.0)) {
            __m128d r;
            double c;
            double d;
            double f1_tmp;
            double g1_tmp;
            int b_tmp;
            absxk = std::abs(internal::blas::xnrm2(12, vr, b_k * 12 + 1));
            b_tmp = (b_k + 1) * 12;
            temp = std::abs(internal::blas::xnrm2(12, vr, b_tmp + 1));
            if (absxk < temp) {
              absxk /= temp;
              absxk = temp * std::sqrt(absxk * absxk + 1.0);
            } else if (absxk > temp) {
              temp /= absxk;
              absxk *= std::sqrt(temp * temp + 1.0);
            } else if (std::isnan(temp)) {
              absxk = rtNaN;
            } else {
              absxk *= 1.4142135623730951;
            }
            absxk = 1.0 / absxk;
            k = b_k * 12;
            ix = k + 12;
            vr_tmp = k + 13;
            ihi = k + 11;
            for (int i{k + 1}; i <= ihi; i += 2) {
              r = _mm_loadu_pd(&vr[i - 1]);
              _mm_storeu_pd(&vr[i - 1], _mm_mul_pd(_mm_set1_pd(absxk), r));
            }
            for (int i{vr_tmp}; i <= ix; i++) {
              vr[i - 1] *= absxk;
            }
            k = b_tmp + 12;
            ix = b_tmp + 13;
            vr_tmp = b_tmp + 11;
            for (int i{b_tmp + 1}; i <= vr_tmp; i += 2) {
              r = _mm_loadu_pd(&vr[i - 1]);
              _mm_storeu_pd(&vr[i - 1], _mm_mul_pd(_mm_set1_pd(absxk), r));
            }
            for (int i{ix}; i <= k; i++) {
              vr[i - 1] *= absxk;
            }
            for (int i{0}; i <= 10; i += 2) {
              __m128d r1;
              r = _mm_loadu_pd(&vr[i + 12 * b_k]);
              r1 = _mm_loadu_pd(&vr[i + b_tmp]);
              _mm_storeu_pd(&scale[i],
                            _mm_add_pd(_mm_mul_pd(r, r), _mm_mul_pd(r1, r1)));
            }
            k = 0;
            absxk = scale[0];
            for (int i{0}; i < 11; i++) {
              temp = scale[i + 1];
              if (temp > absxk) {
                k = i + 1;
                absxk = temp;
              }
            }
            f1_tmp = vr[k + 12 * b_k];
            temp = std::abs(f1_tmp);
            k += b_tmp;
            g1_tmp = vr[k];
            absxk = std::abs(g1_tmp);
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
              d = std::sqrt(f1_tmp * f1_tmp + g1_tmp * g1_tmp);
              c = temp / d;
              if (!(f1_tmp >= 0.0)) {
                d = -d;
              }
              absxk = g1_tmp / d;
            } else {
              absxk = std::fmin(
                  4.49423283715579E+307,
                  std::fmax(2.2250738585072014E-308, std::fmax(temp, absxk)));
              temp = f1_tmp / absxk;
              absxk = g1_tmp / absxk;
              d = std::sqrt(temp * temp + absxk * absxk);
              c = std::abs(temp) / d;
              if (!(f1_tmp >= 0.0)) {
                d = -d;
              }
              absxk /= d;
            }
            ix = b_k * 12;
            for (int i{0}; i < 12; i++) {
              vr_tmp = b_tmp + i;
              temp = vr[vr_tmp];
              ihi = ix + i;
              d = vr[ihi];
              vr[vr_tmp] = c * temp - absxk * d;
              vr[ihi] = c * d + absxk * temp;
            }
            vr[k] = 0.0;
          } else {
            absxk = 1.0 / internal::blas::xnrm2(12, vr, b_k * 12 + 1);
            k = b_k * 12;
            ix = k + 12;
            vr_tmp = k + 13;
            ihi = k + 11;
            for (int i{k + 1}; i <= ihi; i += 2) {
              __m128d r;
              r = _mm_loadu_pd(&vr[i - 1]);
              _mm_storeu_pd(&vr[i - 1], _mm_mul_pd(_mm_set1_pd(absxk), r));
            }
            for (int i{vr_tmp}; i <= ix; i++) {
              vr[i - 1] *= absxk;
            }
          }
        }
      }
      for (int i{0}; i < 144; i++) {
        V[i].re = vr[i];
        V[i].im = 0.0;
      }
      for (int i{0}; i < 11; i++) {
        if ((wi[i] > 0.0) && (wi[i + 1] < 0.0)) {
          for (int b_k{0}; b_k < 12; b_k++) {
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
      for (int i{0}; i < 144; i++) {
        V[i].re = rtNaN;
        V[i].im = 0.0;
      }
    }
    if (scalea) {
      internal::reflapack::xzlascl(cscale, anrm, 12 - info, wr, info + 1);
      internal::reflapack::xzlascl(cscale, anrm, 12 - info, wi, info + 1);
      if (info != 0) {
        internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wr, 1);
        internal::reflapack::xzlascl(cscale, anrm, ilo - 1, wi, 1);
      }
    }
    if (info != 0) {
      for (int i{ilo}; i <= info; i++) {
        wr[i - 1] = rtNaN;
        wi[i - 1] = 0.0;
      }
    }
    for (int i{0}; i < 12; i++) {
      W[i].re = wr[i];
      W[i].im = wi[i];
    }
  }
  std::memset(&D[0], 0, 144U * sizeof(creal_T));
  for (int i{0}; i < 12; i++) {
    D[i + 12 * i] = W[i];
  }
}

} // namespace coder

// End of code generation (eigStandard.cpp)
