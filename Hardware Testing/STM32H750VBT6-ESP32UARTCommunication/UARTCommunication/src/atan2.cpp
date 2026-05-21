//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// atan2.cpp
//
// Code generation for function 'atan2'
//

// Include files
#include "atan2.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

// Function Definitions
namespace coder {
float b_atan2(float y, float x)
{
  float r;
  if (rtIsNaNF(y) || rtIsNaNF(x)) {
    r = rtNaNF;
  } else if (rtIsInfF(y) && rtIsInfF(x)) {
    int i;
    int i1;
    if (y > 0.0F) {
      i = 1;
    } else {
      i = -1;
    }
    if (x > 0.0F) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    r = static_cast<float>(atan2(static_cast<double>(static_cast<float>(i)),
                                 static_cast<double>(static_cast<float>(i1))));
  } else if (x == 0.0F) {
    if (y > 0.0F) {
      r = RT_PIF / 2.0F;
    } else if (y < 0.0F) {
      r = -(RT_PIF / 2.0F);
    } else {
      r = 0.0F;
    }
  } else {
    r = static_cast<float>(
        atan2(static_cast<double>(y), static_cast<double>(x)));
  }
  return r;
}

} // namespace coder

// End of code generation (atan2.cpp)
