//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemv.cpp
//
// Code generation for function 'xgemv'
//

// Include files
#include "xgemv.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgemv(int n, const double x[36], double beta1, double y[144], int iy0)
{
  int iyend;
  int scalarLB;
  int vectorUB;
  iyend = iy0 + 11;
  if (beta1 != 1.0) {
    if (beta1 == 0.0) {
      if (iy0 <= iyend) {
        std::memset(&y[iy0 + -1], 0,
                    static_cast<unsigned int>((iyend - iy0) + 1) *
                        sizeof(double));
      }
    } else {
      scalarLB = ((iyend - iy0) + 1) / 2 * 2 + iy0;
      vectorUB = scalarLB - 2;
      for (int iac{iy0}; iac <= vectorUB; iac += 2) {
        __m128d r;
        r = _mm_loadu_pd(&y[iac - 1]);
        r = _mm_mul_pd(_mm_set1_pd(beta1), r);
        _mm_storeu_pd(&y[iac - 1], r);
      }
      for (int iac{scalarLB}; iac <= iyend; iac++) {
        y[iac - 1] *= beta1;
      }
    }
  }
  iyend = 24;
  scalarLB = 12 * (n - 1) + 1;
  for (int iac{1}; iac <= scalarLB; iac += 12) {
    vectorUB = iac + 11;
    for (int ia{iac}; ia <= vectorUB; ia++) {
      int i;
      i = ((iy0 + ia) - iac) - 1;
      y[i] += y[ia - 1] * x[iyend];
    }
    iyend++;
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgemv.cpp)
