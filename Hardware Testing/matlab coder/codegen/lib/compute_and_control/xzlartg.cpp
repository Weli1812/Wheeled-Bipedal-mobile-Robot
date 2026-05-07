//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlartg.cpp
//
// Code generation for function 'xzlartg'
//

// Include files
#include "xzlartg.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xzlartg(double f, double g, double &sn, double &r)
{
  double cs;
  double g1;
  cs = std::abs(f);
  g1 = std::abs(g);
  if (g == 0.0) {
    cs = 1.0;
    sn = 0.0;
    r = f;
  } else if (f == 0.0) {
    cs = 0.0;
    if (g >= 0.0) {
      sn = 1.0;
    } else {
      sn = -1.0;
    }
    r = g1;
  } else if ((cs > 1.4916681462400413E-154) && (cs < 4.7403759540545887E+153) &&
             (g1 > 1.4916681462400413E-154) && (g1 < 4.7403759540545887E+153)) {
    double d;
    d = std::sqrt(f * f + g * g);
    cs /= d;
    r = d;
    if (!(f >= 0.0)) {
      r = -d;
    }
    sn = g / r;
  } else {
    double d;
    double gs;
    g1 = std::fmin(4.49423283715579E+307,
                   std::fmax(2.2250738585072014E-308, std::fmax(cs, g1)));
    cs = f / g1;
    gs = g / g1;
    d = std::sqrt(cs * cs + gs * gs);
    cs = std::abs(cs) / d;
    r = d;
    if (!(f >= 0.0)) {
      r = -d;
    }
    sn = gs / r;
    r *= g1;
  }
  return cs;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlartg.cpp)
