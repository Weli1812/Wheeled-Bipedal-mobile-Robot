//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mrdivide_helper.cpp
//
// Code generation for function 'mrdivide_helper'
//

// Include files
#include "mrdivide_helper.h"
#include "compute_and_control_data.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
void mrdiv(creal_T A[36], const creal_T B[36])
{
  creal_T b_A[36];
  double brm;
  double s;
  double smax;
  double temp_im;
  double temp_re;
  int jAcol;
  int jp1j;
  int kBcol;
  int mmj;
  int n;
  signed char ipiv[6];
  std::copy(&B[0], &B[36], &b_A[0]);
  for (int k{0}; k < 6; k++) {
    ipiv[k] = static_cast<signed char>(k + 1);
  }
  for (int j{0}; j < 5; j++) {
    int b;
    int jj;
    mmj = 4 - j;
    b = j * 7;
    jj = j * 7;
    jp1j = b + 2;
    n = 7 - j;
    kBcol = 0;
    smax = std::abs(b_A[jj].re) + std::abs(b_A[jj].im);
    for (int k{2}; k < n; k++) {
      jAcol = (b + k) - 1;
      s = std::abs(b_A[jAcol].re) + std::abs(b_A[jAcol].im);
      if (s > smax) {
        kBcol = k - 1;
        smax = s;
      }
    }
    n = jj + kBcol;
    if ((b_A[n].re != 0.0) || (b_A[n].im != 0.0)) {
      if (kBcol != 0) {
        n = j + kBcol;
        ipiv[j] = static_cast<signed char>(n + 1);
        for (int b_k{0}; b_k < 6; b_k++) {
          jAcol = j + b_k * 6;
          temp_re = b_A[jAcol].re;
          temp_im = b_A[jAcol].im;
          kBcol = n + b_k * 6;
          b_A[jAcol] = b_A[kBcol];
          b_A[kBcol].re = temp_re;
          b_A[kBcol].im = temp_im;
        }
      }
      jAcol = (jj - j) + 6;
      for (int k{jp1j}; k <= jAcol; k++) {
        double ai;
        double ar;
        double re;
        ar = b_A[k - 1].re;
        ai = b_A[k - 1].im;
        temp_im = b_A[jj].re;
        brm = b_A[jj].im;
        if (brm == 0.0) {
          if (ai == 0.0) {
            re = ar / temp_im;
            smax = 0.0;
          } else if (ar == 0.0) {
            re = 0.0;
            smax = ai / temp_im;
          } else {
            re = ar / temp_im;
            smax = ai / temp_im;
          }
        } else if (temp_im == 0.0) {
          if (ar == 0.0) {
            re = ai / brm;
            smax = 0.0;
          } else if (ai == 0.0) {
            re = 0.0;
            smax = -(ar / brm);
          } else {
            re = ai / brm;
            smax = -(ar / brm);
          }
        } else {
          temp_re = std::abs(temp_im);
          smax = std::abs(brm);
          if (temp_re > smax) {
            s = brm / temp_im;
            smax = temp_im + s * brm;
            re = (ar + s * ai) / smax;
            smax = (ai - s * ar) / smax;
          } else if (smax == temp_re) {
            if (temp_im > 0.0) {
              s = 0.5;
            } else {
              s = -0.5;
            }
            if (brm > 0.0) {
              smax = 0.5;
            } else {
              smax = -0.5;
            }
            re = (ar * s + ai * smax) / temp_re;
            smax = (ai * s - ar * smax) / temp_re;
          } else {
            s = temp_im / brm;
            smax = brm + s * temp_im;
            re = (s * ar + ai) / smax;
            smax = (s * ai - ar) / smax;
          }
        }
        b_A[k - 1].re = re;
        b_A[k - 1].im = smax;
      }
    }
    jp1j = jj;
    for (int b_k{0}; b_k <= mmj; b_k++) {
      jAcol = (b + b_k * 6) + 6;
      smax = b_A[jAcol].re;
      s = b_A[jAcol].im;
      if ((smax != 0.0) || (s != 0.0)) {
        temp_re = -smax - s * 0.0;
        temp_im = smax * 0.0 - s;
        n = jp1j + 8;
        jAcol = (jp1j - j) + 12;
        for (int k{n}; k <= jAcol; k++) {
          kBcol = ((jj + k) - jp1j) - 7;
          smax = b_A[kBcol].re;
          s = b_A[kBcol].im;
          b_A[k - 1].re += smax * temp_re - s * temp_im;
          b_A[k - 1].im += smax * temp_im + s * temp_re;
        }
      }
      jp1j += 6;
    }
  }
  for (int j{0}; j < 6; j++) {
    mmj = 6 * j - 1;
    jAcol = 6 * j;
    for (int k{0}; k < j; k++) {
      kBcol = 6 * k;
      n = k + jAcol;
      smax = b_A[n].re;
      s = b_A[n].im;
      if ((smax != 0.0) || (s != 0.0)) {
        for (int b_k{0}; b_k < 6; b_k++) {
          n = b_k + kBcol;
          brm = A[n].im;
          temp_im = A[n].re;
          n = (b_k + mmj) + 1;
          A[n].re -= smax * temp_im - s * brm;
          A[n].im -= smax * brm + s * temp_im;
        }
      }
    }
    n = j + jAcol;
    temp_re = b_A[n].re;
    temp_im = b_A[n].im;
    brm = std::abs(temp_re);
    smax = std::abs(temp_im);
    if (temp_im == 0.0) {
      temp_re = 1.0 / temp_re;
      temp_im = 0.0;
    } else if (temp_re == 0.0) {
      temp_re = 0.0;
      temp_im = -1.0 / temp_im;
    } else if (brm > smax) {
      s = temp_im / temp_re;
      smax = temp_re + s * temp_im;
      temp_re = 1.0 / smax;
      temp_im = -s / smax;
    } else if (brm == smax) {
      smax = 0.5;
      if (temp_re < 0.0) {
        smax = -0.5;
      }
      s = 0.5;
      if (temp_im < 0.0) {
        s = -0.5;
      }
      temp_re = smax / brm;
      temp_im = -s / brm;
    } else {
      s = temp_re / temp_im;
      smax = temp_im + s * temp_re;
      temp_re = s / smax;
      temp_im = -1.0 / smax;
    }
    for (int k{0}; k < 6; k++) {
      n = (k + mmj) + 1;
      smax = A[n].im;
      s = A[n].re;
      A[n].re = temp_re * s - temp_im * smax;
      A[n].im = temp_re * smax + temp_im * s;
    }
  }
  for (int b_k{5}; b_k >= 0; b_k--) {
    kBcol = 6 * b_k - 1;
    jp1j = b_k + 2;
    for (int j{jp1j}; j < 7; j++) {
      mmj = 6 * (j - 1);
      jAcol = j + kBcol;
      smax = b_A[jAcol].re;
      s = b_A[jAcol].im;
      if ((smax != 0.0) || (s != 0.0)) {
        for (int k{0}; k < 6; k++) {
          n = k + mmj;
          brm = A[n].im;
          temp_im = A[n].re;
          n = (k + kBcol) + 1;
          A[n].re -= smax * temp_im - s * brm;
          A[n].im -= smax * brm + s * temp_im;
        }
      }
    }
  }
  for (int k{4}; k >= 0; k--) {
    signed char i;
    i = ipiv[k];
    if (i != k + 1) {
      for (int b_k{0}; b_k < 6; b_k++) {
        n = b_k + 6 * k;
        temp_re = A[n].re;
        temp_im = A[n].im;
        jAcol = b_k + 6 * (i - 1);
        A[n] = A[jAcol];
        A[jAcol].re = temp_re;
        A[jAcol].im = temp_im;
      }
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (mrdivide_helper.cpp)
