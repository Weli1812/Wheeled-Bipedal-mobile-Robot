//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlarf.cpp
//
// Code generation for function 'xzlarf'
//

// Include files
#include "xzlarf.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Declarations
static int div_nde_s32_floor(int numerator);

// Function Definitions
static int div_nde_s32_floor(int numerator)
{
  int quotient;
  if ((numerator < 0) && (numerator % 12 != 0)) {
    quotient = -1;
  } else {
    quotient = 0;
  }
  quotient += numerator / 12;
  return quotient;
}

namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int m, int n, int iv0, double tau, double C[144], int ic0,
            double work[12])
{
  int i;
  int ia;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int exitg1;
      i = ic0 + lastc * 12;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
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
    lastc = -1;
  }
  if (lastv > 0) {
    double c;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        std::memset(&work[0], 0,
                    static_cast<unsigned int>(lastc + 1) * sizeof(double));
      }
      ia = ic0 + 12 * lastc;
      for (int iac{ic0}; iac <= ia; iac += 12) {
        c = 0.0;
        i = iac + lastv;
        for (int b_ia{iac}; b_ia < i; b_ia++) {
          c += C[b_ia - 1] * C[((iv0 + b_ia) - iac) - 1];
        }
        i = div_nde_s32_floor(iac - ic0);
        work[i] += c;
      }
    }
    if (!(-tau == 0.0)) {
      i = ic0;
      for (int b_ia{0}; b_ia <= lastc; b_ia++) {
        c = work[b_ia];
        if (c != 0.0) {
          c *= -tau;
          ia = lastv + i;
          for (int iac{i}; iac < ia; iac++) {
            C[iac - 1] += C[((iv0 + iac) - i) - 1] * c;
          }
        }
        i += 12;
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlarf.cpp)
