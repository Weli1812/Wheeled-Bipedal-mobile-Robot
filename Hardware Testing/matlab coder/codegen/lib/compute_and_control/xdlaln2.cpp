//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlaln2.cpp
//
// Code generation for function 'xdlaln2'
//

// Include files
#include "xdlaln2.h"
#include "rt_nonfinite.h"
#include "xdladiv.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xdlaln2(int na, int nw, double smin, const double A[144], int ia0,
               const double B[36], int ib0, double wr, double wi, double X[4],
               double &xnorm)
{
  static const signed char ipivot[16]{1, 2, 3, 4, 2, 1, 4, 3,
                                      3, 4, 1, 2, 4, 3, 2, 1};
  double scale;
  scale = 1.0;
  if (na == 1) {
    if (nw == 1) {
      double bbnd;
      double bnorm;
      double cnorm;
      double csr;
      csr = A[ia0 - 1] - wr;
      cnorm = std::abs(csr);
      if (cnorm < smin) {
        csr = smin;
        cnorm = smin;
      }
      bbnd = B[ib0 - 1];
      bnorm = std::abs(bbnd);
      if ((cnorm < 1.0) && (bnorm > 1.0) &&
          (bnorm > 2.2471164185778949E+307 * cnorm)) {
        scale = 1.0 / bnorm;
      }
      X[0] = bbnd * scale / csr;
      xnorm = std::abs(X[0]);
    } else {
      double bbnd;
      double bnorm;
      double cnorm;
      double csi;
      double csr;
      double xr1;
      csr = A[ia0 - 1] - wr;
      csi = -wi;
      cnorm = std::abs(csr) + std::abs(-wi);
      if (cnorm < smin) {
        csr = smin;
        csi = 0.0;
        cnorm = smin;
      }
      xr1 = B[ib0 - 1];
      bbnd = B[ib0 + 11];
      bnorm = std::abs(xr1) + std::abs(bbnd);
      if ((cnorm < 1.0) && (bnorm > 1.0) &&
          (bnorm > 2.2471164185778949E+307 * cnorm)) {
        scale = 1.0 / bnorm;
      }
      X[0] = xdladiv(scale * xr1, scale * bbnd, csr, csi, X[2]);
      xnorm = std::abs(X[0]) + std::abs(X[2]);
    }
  } else {
    double cr[4];
    cr[0] = A[ia0 - 1] - wr;
    cr[3] = A[ia0 + 12] - wr;
    cr[1] = A[ia0];
    cr[2] = A[ia0 + 11];
    if (nw == 1) {
      double bbnd;
      double cmax;
      int icmax;
      cmax = 0.0;
      icmax = -1;
      bbnd = std::abs(cr[0]);
      if (bbnd > 0.0) {
        cmax = bbnd;
        icmax = 0;
      }
      bbnd = std::abs(cr[1]);
      if (bbnd > cmax) {
        cmax = bbnd;
        icmax = 1;
      }
      bbnd = std::abs(cr[2]);
      if (bbnd > cmax) {
        cmax = bbnd;
        icmax = 2;
      }
      bbnd = std::abs(cr[3]);
      if (bbnd > cmax) {
        cmax = bbnd;
        icmax = 3;
      }
      if (cmax < smin) {
        double bnorm;
        double cnorm;
        bbnd = B[ib0 - 1];
        bnorm = std::fmax(std::abs(bbnd), std::abs(B[ib0]));
        if ((smin < 1.0) && (bnorm > 1.0) &&
            (bnorm > 2.2471164185778949E+307 * smin)) {
          scale = 1.0 / bnorm;
        }
        cnorm = scale / smin;
        X[0] = cnorm * bbnd;
        X[1] = cnorm * B[ib0];
        xnorm = cnorm * bnorm;
      } else {
        double bnorm;
        double br1;
        double br2;
        double lr21;
        double ur11r;
        double ur22;
        double xr1;
        int ur12_tmp;
        ur12_tmp = icmax << 2;
        bnorm = cr[ipivot[ur12_tmp + 2] - 1];
        ur11r = 1.0 / cr[icmax];
        lr21 = ur11r * cr[ipivot[ur12_tmp + 1] - 1];
        ur22 = cr[ipivot[ur12_tmp + 3] - 1] - bnorm * lr21;
        if (std::abs(ur22) < smin) {
          ur22 = smin;
        }
        if ((icmax + 1 == 2) || (icmax + 1 == 4)) {
          br1 = B[ib0];
          br2 = B[ib0 - 1];
        } else {
          br1 = B[ib0 - 1];
          br2 = B[ib0];
        }
        br2 -= lr21 * br1;
        bbnd = std::fmax(std::abs(br1 * (ur22 * ur11r)), std::abs(br2));
        if (bbnd > 1.0) {
          double csi;
          csi = std::abs(ur22);
          if ((csi < 1.0) && (bbnd >= 2.2471164185778949E+307 * csi)) {
            scale = 1.0 / bbnd;
          }
        }
        bbnd = br2 * scale / ur22;
        xr1 = scale * br1 * ur11r - bbnd * (ur11r * bnorm);
        if ((icmax + 1 == 3) || (icmax + 1 == 4)) {
          X[0] = bbnd;
          X[1] = xr1;
        } else {
          X[0] = xr1;
          X[1] = bbnd;
        }
        xnorm = std::fmax(std::abs(xr1), std::abs(bbnd));
        if ((xnorm > 1.0) && (cmax > 1.0) &&
            (xnorm > 2.2471164185778949E+307 / cmax)) {
          double cnorm;
          cnorm = cmax / 2.2471164185778949E+307;
          X[0] *= cnorm;
          X[1] *= cnorm;
          xnorm *= cnorm;
          scale *= cnorm;
        }
      }
    } else {
      double ci[4];
      double bbnd;
      double cmax;
      double cnorm;
      int icmax;
      ci[0] = -wi;
      ci[1] = 0.0;
      ci[2] = 0.0;
      ci[3] = -wi;
      cmax = 0.0;
      icmax = -1;
      bbnd = std::abs(-wi);
      cnorm = std::abs(cr[0]) + bbnd;
      if (cnorm > 0.0) {
        cmax = cnorm;
        icmax = 0;
      }
      cnorm = std::abs(cr[1]);
      if (cnorm > cmax) {
        cmax = cnorm;
        icmax = 1;
      }
      cnorm = std::abs(cr[2]);
      if (cnorm > cmax) {
        cmax = cnorm;
        icmax = 2;
      }
      cnorm = std::abs(cr[3]) + bbnd;
      if (cnorm > cmax) {
        cmax = cnorm;
        icmax = 3;
      }
      if (cmax < smin) {
        double bnorm;
        double csi;
        double xr1;
        bbnd = B[ib0 - 1];
        csi = B[ib0 + 11];
        xr1 = B[ib0 + 12];
        bnorm = std::fmax(std::abs(bbnd) + std::abs(csi),
                          std::abs(B[ib0]) + std::abs(xr1));
        if ((smin < 1.0) && (bnorm > 1.0) &&
            (bnorm > 2.2471164185778949E+307 * smin)) {
          scale = 1.0 / bnorm;
        }
        cnorm = scale / smin;
        X[0] = cnorm * bbnd;
        X[1] = cnorm * B[ib0];
        X[2] = cnorm * csi;
        X[3] = cnorm * xr1;
        xnorm = cnorm * bnorm;
      } else {
        double bi1;
        double bnorm;
        double br1;
        double br2;
        double csi;
        double csr;
        double lr21;
        double ui11r;
        double ui12s;
        double ur11r;
        double ur12s;
        double ur22;
        double xr1;
        int b_ur12_tmp;
        int cr21_tmp;
        int ur12_tmp;
        ur12_tmp = icmax << 2;
        cr21_tmp = ipivot[ur12_tmp + 1] - 1;
        csi = cr[cr21_tmp];
        b_ur12_tmp = ipivot[ur12_tmp + 2] - 1;
        bnorm = cr[b_ur12_tmp];
        bbnd = ci[b_ur12_tmp];
        ur12_tmp = ipivot[ur12_tmp + 3] - 1;
        csr = cr[ur12_tmp];
        if ((icmax == 0) || (icmax + 1 == 4)) {
          if (std::abs(cr[icmax]) > std::abs(ci[icmax])) {
            cnorm = ci[icmax] / cr[icmax];
            ur11r = 1.0 / (cr[icmax] * (cnorm * cnorm + 1.0));
            ui11r = -cnorm * ur11r;
          } else {
            cnorm = cr[icmax] / ci[icmax];
            ui11r = -1.0 / (ci[icmax] * (cnorm * cnorm + 1.0));
            ur11r = -cnorm * ui11r;
          }
          lr21 = csi * ur11r;
          xr1 = csi * ui11r;
          ur12s = bnorm * ur11r;
          ui12s = bnorm * ui11r;
          ur22 = csr - bnorm * lr21;
          cnorm = ci[ur12_tmp] - bnorm * xr1;
        } else {
          ur11r = 1.0 / cr[icmax];
          ui11r = 0.0;
          lr21 = csi * ur11r;
          xr1 = ci[cr21_tmp] * ur11r;
          ur12s = bnorm * ur11r;
          ui12s = bbnd * ur11r;
          ur22 = (csr - bnorm * lr21) + bbnd * xr1;
          cnorm = -bnorm * xr1 - bbnd * lr21;
        }
        csi = std::abs(ur22) + std::abs(cnorm);
        if (csi < smin) {
          ur22 = smin;
          cnorm = 0.0;
        }
        if ((icmax + 1 == 2) || (icmax + 1 == 4)) {
          br2 = B[ib0 - 1];
          br1 = B[ib0];
          bnorm = B[ib0 + 11];
          bi1 = B[ib0 + 12];
        } else {
          br1 = B[ib0 - 1];
          br2 = B[ib0];
          bi1 = B[ib0 + 11];
          bnorm = B[ib0 + 12];
        }
        br2 = (br2 - lr21 * br1) + xr1 * bi1;
        bnorm = (bnorm - xr1 * br1) - lr21 * bi1;
        bbnd = std::fmax((std::abs(br1) + std::abs(bi1)) *
                             (csi * (std::abs(ur11r) + std::abs(ui11r))),
                         std::abs(br2) + std::abs(bnorm));
        if ((bbnd > 1.0) && (csi < 1.0) &&
            (bbnd >= 2.2471164185778949E+307 * csi)) {
          scale = 1.0 / bbnd;
          br1 *= scale;
          bi1 *= scale;
          br2 *= scale;
          bnorm *= scale;
        }
        csi = xdladiv(br2, bnorm, ur22, cnorm, csr);
        xr1 = ((ur11r * br1 - ui11r * bi1) - ur12s * csi) + ui12s * csr;
        bbnd = ((ui11r * br1 + ur11r * bi1) - ui12s * csi) - ur12s * csr;
        if ((icmax + 1 == 3) || (icmax + 1 == 4)) {
          X[0] = csi;
          X[1] = xr1;
          X[2] = csr;
          X[3] = bbnd;
        } else {
          X[0] = xr1;
          X[1] = csi;
          X[2] = bbnd;
          X[3] = csr;
        }
        xnorm = std::fmax(std::abs(xr1) + std::abs(bbnd),
                          std::abs(csi) + std::abs(csr));
        if ((xnorm > 1.0) && (cmax > 1.0) &&
            (xnorm > 2.2471164185778949E+307 / cmax)) {
          cnorm = cmax / 2.2471164185778949E+307;
          X[0] *= cnorm;
          X[1] *= cnorm;
          X[2] *= cnorm;
          X[3] *= cnorm;
          xnorm *= cnorm;
          scale *= cnorm;
        }
      }
    }
  }
  return scale;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdlaln2.cpp)
