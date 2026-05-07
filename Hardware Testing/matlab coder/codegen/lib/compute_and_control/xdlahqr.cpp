//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlahqr.cpp
//
// Code generation for function 'xdlahqr'
//

// Include files
#include "xdlahqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include <cmath>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
int xdlahqr(int ilo, int ihi, double h[144], int iloz, int ihiz, double z[144],
            double wr[12], double wi[12])
{
  double v[3];
  double h11;
  double h12;
  double h21;
  double h22;
  double rt1r;
  double rt2r;
  double tr;
  int info;
  int nh;
  info = 0;
  nh = static_cast<unsigned char>(ilo - 1);
  for (int i{0}; i < nh; i++) {
    wr[i] = h[i + 12 * i];
    wi[i] = 0.0;
  }
  nh = ihi + 1;
  for (int i{nh}; i < 13; i++) {
    wr[i - 1] = h[(i + 12 * (i - 1)) - 1];
    wi[i - 1] = 0.0;
  }
  if (ilo == ihi) {
    wr[ilo - 1] = h[(ilo + 12 * (ilo - 1)) - 1];
    wi[ilo - 1] = 0.0;
  } else {
    double smlnum;
    int b_i;
    int itmax;
    int kdefl;
    int nz;
    int temp_tmp_tmp;
    boolean_T exitg1;
    nh = ihi - 3;
    for (int i{ilo}; i <= nh; i++) {
      temp_tmp_tmp = i + 12 * (i - 1);
      h[temp_tmp_tmp + 1] = 0.0;
      h[temp_tmp_tmp + 2] = 0.0;
    }
    if (ilo <= ihi - 2) {
      h[(ihi + 12 * (ihi - 3)) - 1] = 0.0;
    }
    nh = (ihi - ilo) + 1;
    nz = (ihiz - iloz) + 1;
    smlnum = 2.2250738585072014E-308 *
             (static_cast<double>(nh) / 2.2204460492503131E-16);
    if (nh < 10) {
      nh = 10;
    }
    itmax = 30 * nh;
    kdefl = 0;
    b_i = ihi - 1;
    exitg1 = false;
    while ((!exitg1) && (b_i + 1 >= ilo)) {
      int b_temp_tmp_tmp;
      int its;
      int ix;
      int l;
      int nr;
      boolean_T converged;
      boolean_T exitg2;
      l = ilo;
      converged = false;
      its = 0;
      exitg2 = false;
      while ((!exitg2) && (its <= itmax)) {
        double s;
        int k;
        boolean_T exitg3;
        k = b_i;
        exitg3 = false;
        while ((!exitg3) && (k + 1 > l)) {
          nh = k + 12 * (k - 1);
          h22 = std::abs(h[nh]);
          if (h22 <= smlnum) {
            exitg3 = true;
          } else {
            temp_tmp_tmp = k + 12 * k;
            h21 = std::abs(h[temp_tmp_tmp]);
            h11 = std::abs(h[nh - 1]) + h21;
            if (h11 == 0.0) {
              if (k - 1 >= ilo) {
                h11 = std::abs(h[(k + 12 * (k - 2)) - 1]);
              }
              if (k + 2 <= ihi) {
                h11 += std::abs(h[temp_tmp_tmp + 1]);
              }
            }
            if (h22 <= 2.2204460492503131E-16 * h11) {
              h11 = std::abs(h[temp_tmp_tmp - 1]);
              h12 = std::abs(h[nh - 1] - h[temp_tmp_tmp]);
              tr = std::fmax(h21, h12);
              h12 = std::fmin(h21, h12);
              s = tr + h12;
              if (std::fmin(h22, h11) * (std::fmax(h22, h11) / s) <=
                  std::fmax(smlnum,
                            2.2204460492503131E-16 * (h12 * (tr / s)))) {
                exitg3 = true;
              } else {
                k--;
              }
            } else {
              k--;
            }
          }
        }
        l = k + 1;
        if (k + 1 > ilo) {
          h[k + 12 * (k - 1)] = 0.0;
        }
        if (k + 1 >= b_i) {
          converged = true;
          exitg2 = true;
        } else {
          __m128d r;
          int m;
          kdefl++;
          if (kdefl - kdefl / 20 * 20 == 0) {
            s = std::abs(h[b_i + 12 * (b_i - 1)]) +
                std::abs(h[(b_i + 12 * (b_i - 2)) - 1]);
            h11 = 0.75 * s + h[b_i + 12 * b_i];
            h12 = -0.4375 * s;
            h21 = s;
            h22 = h11;
          } else if (kdefl - kdefl / 10 * 10 == 0) {
            nh = k + 12 * k;
            s = std::abs(h[nh + 1]) + std::abs(h[(k + 12 * (k + 1)) + 2]);
            h11 = 0.75 * s + h[nh];
            h12 = -0.4375 * s;
            h21 = s;
            h22 = h11;
          } else {
            nh = b_i + 12 * (b_i - 1);
            h11 = h[nh - 1];
            h21 = h[nh];
            nh = b_i + 12 * b_i;
            h12 = h[nh - 1];
            h22 = h[nh];
          }
          s = ((std::abs(h11) + std::abs(h12)) + std::abs(h21)) + std::abs(h22);
          if (s == 0.0) {
            rt1r = 0.0;
            h11 = 0.0;
            rt2r = 0.0;
            h12 = 0.0;
          } else {
            h11 /= s;
            h21 /= s;
            h12 /= s;
            h22 /= s;
            tr = (h11 + h22) / 2.0;
            h11 = (h11 - tr) * (h22 - tr) - h12 * h21;
            h12 = std::sqrt(std::abs(h11));
            if (h11 >= 0.0) {
              rt1r = tr * s;
              rt2r = rt1r;
              h11 = h12 * s;
              h12 = -h11;
            } else {
              rt1r = tr + h12;
              rt2r = tr - h12;
              if (std::abs(rt1r - h22) <= std::abs(rt2r - h22)) {
                rt1r *= s;
                rt2r = rt1r;
              } else {
                rt2r *= s;
                rt1r = rt2r;
              }
              h11 = 0.0;
              h12 = 0.0;
            }
          }
          m = b_i - 1;
          exitg3 = false;
          while ((!exitg3) && (m >= k + 1)) {
            nh = m + 12 * (m - 1);
            h21 = h[nh - 1];
            tr = h21 - rt2r;
            s = (std::abs(tr) + std::abs(h12)) + std::abs(h[nh]);
            h22 = h[nh] / s;
            nr = m + 12 * m;
            v[0] = (h22 * h[nr - 1] + tr * (tr / s)) - h11 * (h12 / s);
            v[1] = h22 * (((h21 + h[nr]) - rt1r) - rt2r);
            v[2] = h22 * h[nr + 1];
            s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
            r = _mm_loadu_pd(&v[0]);
            _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
            v[2] /= s;
            if (m == k + 1) {
              exitg3 = true;
            } else {
              b_temp_tmp_tmp = m + 12 * (m - 2);
              if (std::abs(h[b_temp_tmp_tmp - 1]) *
                      (std::abs(v[1]) + std::abs(v[2])) <=
                  2.2204460492503131E-16 * std::abs(v[0]) *
                      ((std::abs(h[b_temp_tmp_tmp - 2]) + std::abs(h[nh - 1])) +
                       std::abs(h[nr]))) {
                exitg3 = true;
              } else {
                m--;
              }
            }
          }
          for (int c_i{m}; c_i <= b_i; c_i++) {
            double t1;
            nr = (b_i - c_i) + 2;
            if (nr >= 3) {
              nr = 3;
            }
            if (c_i > m) {
              nh = ((c_i - 2) * 12 + c_i) - 1;
              for (int i{0}; i < nr; i++) {
                v[i] = h[nh + i];
              }
            }
            h11 = v[0];
            t1 = xzlarfg(nr, h11, v);
            if (c_i > m) {
              nh = c_i + 12 * (c_i - 2);
              h[nh - 1] = h11;
              h[nh] = 0.0;
              if (c_i < b_i) {
                h[nh + 1] = 0.0;
              }
            } else if (m > k + 1) {
              nh = (c_i + 12 * (c_i - 2)) - 1;
              h[nh] *= 1.0 - t1;
            }
            rt2r = v[1];
            rt1r = t1 * v[1];
            if (nr == 3) {
              __m128d r1;
              __m128d r2;
              __m128d r3;
              int scalarLB;
              tr = v[2];
              h22 = t1 * v[2];
              for (int i{c_i}; i < 13; i++) {
                nh = c_i + 12 * (i - 1);
                h11 = h[nh - 1];
                h12 = h[nh];
                h21 = h[nh + 1];
                s = (h11 + rt2r * h12) + tr * h21;
                h11 -= s * t1;
                h[nh - 1] = h11;
                h12 -= s * rt1r;
                h[nh] = h12;
                h21 -= s * h22;
                h[nh + 1] = h21;
              }
              if (c_i + 3 <= b_i + 1) {
                nh = c_i;
              } else {
                nh = b_i - 2;
              }
              nr = static_cast<unsigned char>(nh + 3);
              scalarLB = (nr >> 1) << 1;
              nh = scalarLB - 2;
              for (int i{0}; i <= nh; i += 2) {
                b_temp_tmp_tmp = i + 12 * c_i;
                r = _mm_loadu_pd(&h[b_temp_tmp_tmp]);
                temp_tmp_tmp = i + 12 * (c_i + 1);
                r1 = _mm_loadu_pd(&h[temp_tmp_tmp]);
                ix = i + 12 * (c_i - 1);
                r2 = _mm_loadu_pd(&h[ix]);
                r3 =
                    _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(rt2r), r)),
                               _mm_mul_pd(_mm_set1_pd(tr), r1));
                _mm_storeu_pd(&h[ix],
                              _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(t1))));
                _mm_storeu_pd(&h[b_temp_tmp_tmp],
                              _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt1r))));
                _mm_storeu_pd(&h[temp_tmp_tmp],
                              _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(h22))));
              }
              for (int i{scalarLB}; i < nr; i++) {
                nh = i + 12 * (c_i - 1);
                h11 = h[nh];
                b_temp_tmp_tmp = i + 12 * c_i;
                h12 = h[b_temp_tmp_tmp];
                temp_tmp_tmp = i + 12 * (c_i + 1);
                h21 = h[temp_tmp_tmp];
                s = (h11 + rt2r * h12) + tr * h21;
                h11 -= s * t1;
                h[nh] = h11;
                h12 -= s * rt1r;
                h[b_temp_tmp_tmp] = h12;
                h21 -= s * h22;
                h[temp_tmp_tmp] = h21;
              }
              scalarLB = ((nz / 2) << 1) + iloz;
              nh = scalarLB - 2;
              for (int i{iloz}; i <= nh; i += 2) {
                b_temp_tmp_tmp = (i + 12 * c_i) - 1;
                r = _mm_loadu_pd(&z[b_temp_tmp_tmp]);
                temp_tmp_tmp = (i + 12 * (c_i + 1)) - 1;
                r1 = _mm_loadu_pd(&z[temp_tmp_tmp]);
                ix = (i + 12 * (c_i - 1)) - 1;
                r2 = _mm_loadu_pd(&z[ix]);
                r3 =
                    _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(rt2r), r)),
                               _mm_mul_pd(_mm_set1_pd(tr), r1));
                _mm_storeu_pd(&z[ix],
                              _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(t1))));
                _mm_storeu_pd(&z[b_temp_tmp_tmp],
                              _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(rt1r))));
                _mm_storeu_pd(&z[temp_tmp_tmp],
                              _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(h22))));
              }
              for (int i{scalarLB}; i <= ihiz; i++) {
                nh = (i + 12 * (c_i - 1)) - 1;
                h11 = z[nh];
                nr = (i + 12 * c_i) - 1;
                h12 = z[nr];
                b_temp_tmp_tmp = (i + 12 * (c_i + 1)) - 1;
                h21 = z[b_temp_tmp_tmp];
                s = (h11 + rt2r * h12) + tr * h21;
                h11 -= s * t1;
                z[nh] = h11;
                h12 -= s * rt1r;
                z[nr] = h12;
                h21 -= s * h22;
                z[b_temp_tmp_tmp] = h21;
              }
            } else if (nr == 2) {
              __m128d r1;
              __m128d r2;
              int scalarLB;
              for (int i{c_i}; i < 13; i++) {
                nh = c_i + 12 * (i - 1);
                h11 = h[nh - 1];
                h12 = h[nh];
                s = h11 + rt2r * h12;
                h11 -= s * t1;
                h[nh - 1] = h11;
                h12 -= s * rt1r;
                h[nh] = h12;
              }
              ix = static_cast<unsigned char>(b_i + 1);
              scalarLB = (static_cast<unsigned char>(b_i + 1) >> 1) << 1;
              nh = scalarLB - 2;
              for (int i{0}; i <= nh; i += 2) {
                b_temp_tmp_tmp = i + 12 * c_i;
                r = _mm_loadu_pd(&h[b_temp_tmp_tmp]);
                temp_tmp_tmp = i + 12 * (c_i - 1);
                r1 = _mm_loadu_pd(&h[temp_tmp_tmp]);
                r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(rt2r), r));
                _mm_storeu_pd(&h[temp_tmp_tmp],
                              _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(t1))));
                _mm_storeu_pd(&h[b_temp_tmp_tmp],
                              _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt1r))));
              }
              for (int i{scalarLB}; i < ix; i++) {
                nh = i + 12 * (c_i - 1);
                h11 = h[nh];
                nr = i + 12 * c_i;
                h12 = h[nr];
                s = h11 + rt2r * h12;
                h11 -= s * t1;
                h[nh] = h11;
                h12 -= s * rt1r;
                h[nr] = h12;
              }
              temp_tmp_tmp = ((nz / 2) << 1) + iloz;
              nh = temp_tmp_tmp - 2;
              for (int i{iloz}; i <= nh; i += 2) {
                nr = (i + 12 * c_i) - 1;
                r = _mm_loadu_pd(&z[nr]);
                b_temp_tmp_tmp = (i + 12 * (c_i - 1)) - 1;
                r1 = _mm_loadu_pd(&z[b_temp_tmp_tmp]);
                r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(rt2r), r));
                _mm_storeu_pd(&z[b_temp_tmp_tmp],
                              _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(t1))));
                _mm_storeu_pd(&z[nr],
                              _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(rt1r))));
              }
              for (int i{temp_tmp_tmp}; i <= ihiz; i++) {
                nh = (i + 12 * (c_i - 1)) - 1;
                h11 = z[nh];
                nr = (i + 12 * c_i) - 1;
                h12 = z[nr];
                s = h11 + rt2r * h12;
                h11 -= s * t1;
                z[nh] = h11;
                h12 -= s * rt1r;
                z[nr] = h12;
              }
            }
          }
          its++;
        }
      }
      if (!converged) {
        info = b_i + 1;
        exitg1 = true;
      } else {
        if (l == b_i + 1) {
          wr[b_i] = h[b_i + 12 * b_i];
          wi[b_i] = 0.0;
        } else if (l == b_i) {
          nh = b_i + 12 * b_i;
          h11 = h[nh - 1];
          ix = 12 * (b_i - 1);
          temp_tmp_tmp = b_i + ix;
          h12 = h[temp_tmp_tmp];
          h21 = h[nh];
          wr[b_i - 1] = xdlanv2(h[temp_tmp_tmp - 1], h11, h12, h21, wi[b_i - 1],
                                tr, h22, rt2r, rt1r);
          wr[b_i] = tr;
          wi[b_i] = h22;
          h[nh - 1] = h11;
          h[temp_tmp_tmp] = h12;
          h[nh] = h21;
          if (b_i + 1 < 12) {
            nh = (b_i + 1) * 12 + b_i;
            temp_tmp_tmp = static_cast<unsigned char>(11 - b_i);
            for (int i{0}; i < temp_tmp_tmp; i++) {
              nr = nh + i * 12;
              h11 = h[nr];
              h12 = h[nr - 1];
              h[nr] = rt2r * h11 - rt1r * h12;
              h[nr - 1] = rt2r * h12 + rt1r * h11;
            }
          }
          if (b_i - 1 >= 1) {
            nh = b_i * 12;
            nr = static_cast<unsigned char>(b_i - 1);
            for (int i{0}; i < nr; i++) {
              b_temp_tmp_tmp = nh + i;
              h11 = h[b_temp_tmp_tmp];
              temp_tmp_tmp = ix + i;
              h12 = h[temp_tmp_tmp];
              h[b_temp_tmp_tmp] = rt2r * h11 - rt1r * h12;
              h[temp_tmp_tmp] = rt2r * h12 + rt1r * h11;
            }
          }
          if (nz >= 1) {
            ix = (ix + iloz) - 1;
            nh = (b_i * 12 + iloz) - 1;
            temp_tmp_tmp = static_cast<unsigned char>(nz);
            for (int i{0}; i < temp_tmp_tmp; i++) {
              nr = nh + i;
              h11 = z[nr];
              b_temp_tmp_tmp = ix + i;
              h12 = z[b_temp_tmp_tmp];
              z[nr] = rt2r * h11 - rt1r * h12;
              z[b_temp_tmp_tmp] = rt2r * h12 + rt1r * h11;
            }
          }
        }
        kdefl = 0;
        b_i = l - 2;
      }
    }
    for (int i{0}; i < 10; i++) {
      for (int c_i{i + 3}; c_i < 13; c_i++) {
        h[(c_i + 12 * i) - 1] = 0.0;
      }
    }
  }
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdlahqr.cpp)
