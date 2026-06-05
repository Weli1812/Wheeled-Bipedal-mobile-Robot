//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// norm.cpp
//
// Code generation for function 'norm'
//

// Include files
#include "norm.h"
#include "rt_nonfinite.h"
#include <math.h>

// Function Definitions
namespace coder {
float b_norm(const float x[3])
{
  float absxk;
  float scale;
  float t;
  float y;
  scale = 1.29246971E-26F;
  absxk = static_cast<float>(fabs(static_cast<double>(x[0])));
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }
  absxk = static_cast<float>(fabs(static_cast<double>(x[1])));
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = static_cast<float>(fabs(static_cast<double>(x[2])));
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  return scale * static_cast<float>(sqrt(static_cast<double>(y)));
}

float c_norm(const float x[4])
{
  float absxk;
  float scale;
  float t;
  float y;
  scale = 1.29246971E-26F;
  absxk = static_cast<float>(fabs(static_cast<double>(x[0])));
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }
  absxk = static_cast<float>(fabs(static_cast<double>(x[1])));
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = static_cast<float>(fabs(static_cast<double>(x[2])));
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = static_cast<float>(fabs(static_cast<double>(x[3])));
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  return scale * static_cast<float>(sqrt(static_cast<double>(y)));
}

} // namespace coder

// End of code generation (norm.cpp)
