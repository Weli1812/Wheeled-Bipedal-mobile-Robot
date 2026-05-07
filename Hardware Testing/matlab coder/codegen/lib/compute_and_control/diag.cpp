//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// diag.cpp
//
// Code generation for function 'diag'
//

// Include files
#include "diag.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
void b_diag(const double v[6], double d[36])
{
  std::memset(&d[0], 0, 36U * sizeof(double));
  for (int j{0}; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

void diag(const double v[2], double d[4])
{
  d[1] = 0.0;
  d[2] = 0.0;
  d[0] = v[0];
  d[3] = v[1];
}

} // namespace coder

// End of code generation (diag.cpp)
