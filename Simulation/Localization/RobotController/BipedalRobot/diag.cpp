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
#include <string.h>

// Function Definitions
namespace coder {
void b_diag(const double v[6], double d[36])
{
  memset(&d[0], 0, 36U * sizeof(double));
  for (int j = 0; j < 6; j++) {
    d[j + 6 * j] = v[j];
  }
}

void diag(const double v[7], double d[49])
{
  memset(&d[0], 0, 49U * sizeof(double));
  for (int j = 0; j < 7; j++) {
    d[j + 7 * j] = v[j];
  }
}

} // namespace coder

// End of code generation (diag.cpp)
