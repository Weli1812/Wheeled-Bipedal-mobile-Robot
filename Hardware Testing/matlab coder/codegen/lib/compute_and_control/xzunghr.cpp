//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzunghr.cpp
//
// Code generation for function 'xzunghr'
//

// Include files
#include "xzunghr.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzunghr(int ilo, int ihi, double A[144], const double tau[11])
{
  double work[12];
  int a;
  int b_i;
  int b_ia;
  int ia;
  int ia0;
  int nh;
  nh = ihi - ilo;
  a = ilo + 1;
  for (int j{ihi}; j >= a; j--) {
    b_ia = (j - 1) * 12;
    ia = static_cast<unsigned char>(j - 1);
    std::memset(&A[b_ia], 0, static_cast<unsigned int>(ia) * sizeof(double));
    ia = j + 1;
    for (int i{ia}; i <= ihi; i++) {
      b_i = b_ia + i;
      A[b_i - 1] = A[b_i - 13];
    }
    ia = ihi + 1;
    if (ia <= 12) {
      std::memset(&A[(ia + b_ia) + -1], 0,
                  static_cast<unsigned int>(((b_ia - ia) - b_ia) + 13) *
                      sizeof(double));
    }
  }
  ia = static_cast<unsigned char>(ilo);
  for (int i{0}; i < ia; i++) {
    b_ia = i * 12;
    std::memset(&A[b_ia], 0, 12U * sizeof(double));
    A[b_ia + i] = 1.0;
  }
  ia = ihi + 1;
  for (int i{ia}; i < 13; i++) {
    b_ia = (i - 1) * 12;
    std::memset(&A[b_ia], 0, 12U * sizeof(double));
    A[(b_ia + i) - 1] = 1.0;
  }
  ia0 = ilo + ilo * 12;
  if (nh >= 1) {
    int itau;
    for (int i{nh}; i < nh; i++) {
      ia = ia0 + i * 12;
      std::memset(&A[ia], 0, static_cast<unsigned int>(nh) * sizeof(double));
      A[ia + i] = 1.0;
    }
    itau = (ilo + nh) - 2;
    std::memset(&work[0], 0, 12U * sizeof(double));
    for (int c_i{nh}; c_i >= 1; c_i--) {
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
        for (int i{ia}; i <= b_ia; i += 2) {
          __m128d r;
          r = _mm_loadu_pd(&A[i - 1]);
          r = _mm_mul_pd(_mm_set1_pd(-tau[itau]), r);
          _mm_storeu_pd(&A[i - 1], r);
        }
        for (int i{a}; i <= b_i; i++) {
          A[i - 1] *= -tau[itau];
        }
      }
      A[iaii - 1] = 1.0 - tau[itau];
      ia = static_cast<unsigned char>(c_i - 1);
      for (int i{0}; i < ia; i++) {
        A[(iaii - i) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzunghr.cpp)
