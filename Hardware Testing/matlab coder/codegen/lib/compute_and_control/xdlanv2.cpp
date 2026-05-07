//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlanv2.cpp
//
// Code generation for function 'xdlanv2'
//

// Include files
#include "xdlanv2.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xdlanv2(double &a, double &b, double &c, double &d, double &rt1i,
               double &rt2r, double &rt2i, double &cs, double &sn)
{
  double rt1r;
  if (c == 0.0) {
    cs = 1.0;
    sn = 0.0;
  } else if (b == 0.0) {
    double temp;
    cs = 0.0;
    sn = 1.0;
    temp = d;
    d = a;
    a = temp;
    b = -c;
    c = 0.0;
  } else {
    double temp;
    temp = a - d;
    if ((temp == 0.0) && ((b < 0.0) != (c < 0.0))) {
      cs = 1.0;
      sn = 0.0;
    } else {
      double bcmax;
      double bcmis;
      double p;
      double tau;
      double z;
      int count;
      int i;
      p = 0.5 * temp;
      rt1r = std::abs(b);
      tau = std::abs(c);
      bcmax = std::fmax(rt1r, tau);
      if (!(b < 0.0)) {
        count = 1;
      } else {
        count = -1;
      }
      if (!(c < 0.0)) {
        i = 1;
      } else {
        i = -1;
      }
      bcmis = std::fmin(rt1r, tau) * static_cast<double>(count) *
              static_cast<double>(i);
      rt1r = std::fmax(std::abs(p), bcmax);
      z = p / rt1r * p + bcmax / rt1r * bcmis;
      if (z >= 8.8817841970012523E-16) {
        rt1r = std::sqrt(rt1r) * std::sqrt(z);
        if (p < 0.0) {
          rt1r = -rt1r;
        }
        z = p + rt1r;
        a = d + z;
        d -= bcmax / z * bcmis;
        bcmis = std::abs(z);
        if (tau < bcmis) {
          rt1r = tau / bcmis;
          tau = bcmis * std::sqrt(rt1r * rt1r + 1.0);
        } else if (tau > bcmis) {
          bcmis /= tau;
          tau *= std::sqrt(bcmis * bcmis + 1.0);
        } else if (std::isnan(bcmis)) {
          tau = rtNaN;
        } else {
          tau *= 1.4142135623730951;
        }
        cs = z / tau;
        sn = c / tau;
        b -= c;
        c = 0.0;
      } else {
        z = b + c;
        rt1r = std::fmax(std::abs(temp), std::abs(z));
        count = 0;
        while ((rt1r >= 7.4428285367870146E+137) && (count <= 20)) {
          z *= 1.3435752215134178E-138;
          temp *= 1.3435752215134178E-138;
          rt1r = std::fmax(std::abs(temp), std::abs(z));
          count++;
        }
        while ((rt1r <= 1.3435752215134178E-138) && (count <= 20)) {
          z *= 7.4428285367870146E+137;
          temp *= 7.4428285367870146E+137;
          rt1r = std::fmax(std::abs(temp), std::abs(z));
          count++;
        }
        bcmax = std::abs(z);
        rt1r = std::abs(temp);
        if (bcmax < rt1r) {
          bcmis = bcmax / rt1r;
          tau = rt1r * std::sqrt(bcmis * bcmis + 1.0);
        } else if (bcmax > rt1r) {
          rt1r /= bcmax;
          tau = bcmax * std::sqrt(rt1r * rt1r + 1.0);
        } else if (std::isnan(rt1r)) {
          tau = rtNaN;
        } else {
          tau = bcmax * 1.4142135623730951;
        }
        cs = std::sqrt(0.5 * (bcmax / tau + 1.0));
        if (!(z < 0.0)) {
          count = 1;
        } else {
          count = -1;
        }
        sn = -(0.5 * temp / (tau * cs)) * static_cast<double>(count);
        rt1r = a * cs + b * sn;
        bcmis = -a * sn + b * cs;
        bcmax = c * cs + d * sn;
        z = -c * sn + d * cs;
        b = bcmis * cs + z * sn;
        c = -rt1r * sn + bcmax * cs;
        temp = 0.5 * ((rt1r * cs + bcmax * sn) + (-bcmis * sn + z * cs));
        a = temp;
        d = temp;
        if (c != 0.0) {
          if (b != 0.0) {
            if ((b < 0.0) == (c < 0.0)) {
              rt1r = std::sqrt(std::abs(b));
              bcmax = std::sqrt(std::abs(c));
              p = rt1r * bcmax;
              if (c < 0.0) {
                p = -p;
              }
              tau = 1.0 / std::sqrt(std::abs(b + c));
              a = temp + p;
              d = temp - p;
              b -= c;
              c = 0.0;
              bcmis = rt1r * tau;
              rt1r = bcmax * tau;
              temp = cs * bcmis - sn * rt1r;
              sn = cs * rt1r + sn * bcmis;
              cs = temp;
            }
          } else {
            b = -c;
            c = 0.0;
            temp = cs;
            cs = -sn;
            sn = temp;
          }
        }
      }
    }
  }
  rt1r = a;
  rt2r = d;
  if (c == 0.0) {
    rt1i = 0.0;
    rt2i = 0.0;
  } else {
    rt1i = std::sqrt(std::abs(b)) * std::sqrt(std::abs(c));
    rt2i = -rt1i;
  }
  return rt1r;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdlanv2.cpp)
