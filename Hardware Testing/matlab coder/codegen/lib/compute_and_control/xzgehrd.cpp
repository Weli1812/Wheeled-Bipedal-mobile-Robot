//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzgehrd.cpp
//
// Code generation for function 'xzgehrd'
//

// Include files
#include "xzgehrd.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzgehrd(double a[144], int ilo, int ihi, double tau[11])
{
  double work[12];
  if ((ihi - ilo) + 1 > 1) {
    int i;
    i = static_cast<unsigned char>(ilo - 1);
    if (i - 1 >= 0) {
      std::memset(&tau[0], 0, static_cast<unsigned int>(i) * sizeof(double));
    }
    for (int b_i{ihi}; b_i < 12; b_i++) {
      tau[b_i - 1] = 0.0;
    }
    std::memset(&work[0], 0, 12U * sizeof(double));
    for (int c_i{ilo}; c_i < ihi; c_i++) {
      double alpha1;
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
      temp = xzlarfg(n, alpha1, a, (rowleft + i) + 1);
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
          rowleft = static_cast<unsigned char>(lastc);
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(rowleft) * sizeof(double));
          i = alpha1_tmp;
          rowleft = (in + 12 * (lastv - 1)) + 1;
          for (int b_i{ic0}; b_i <= rowleft; b_i += 12) {
            d_i = b_i + lastc;
            for (int ia{b_i}; ia < d_i; ia++) {
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
          i = static_cast<unsigned char>(lastv);
          for (int b_i{0}; b_i < i; b_i++) {
            temp = a[alpha1_tmp + b_i];
            if (temp != 0.0) {
              temp *= d;
              rowleft = jA + 1;
              d_i = lastc + jA;
              work_tmp = ((d_i - rowleft) + 1) / 2 * 2 + rowleft;
              ic0 = work_tmp - 2;
              for (int ia{rowleft}; ia <= ic0; ia += 2) {
                __m128d r;
                __m128d r1;
                r = _mm_loadu_pd(&work[(ia - jA) - 1]);
                r = _mm_mul_pd(r, _mm_set1_pd(temp));
                r1 = _mm_loadu_pd(&a[ia - 1]);
                r = _mm_add_pd(r1, r);
                _mm_storeu_pd(&a[ia - 1], r);
              }
              for (int ia{work_tmp}; ia <= d_i; ia++) {
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

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzgehrd.cpp)
