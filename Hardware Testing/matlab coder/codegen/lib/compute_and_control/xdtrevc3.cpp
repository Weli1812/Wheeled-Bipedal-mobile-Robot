//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdtrevc3.cpp
//
// Code generation for function 'xdtrevc3'
//

// Include files
#include "xdtrevc3.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdlaln2.h"
#include "xgemv.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xdtrevc3(const double T[144], double vr[144])
{
  double work[36];
  double x[4];
  double smax;
  int ip;
  std::memset(&work[0], 0, 36U * sizeof(double));
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  work[0] = 0.0;
  for (int j{0}; j < 11; j++) {
    work[j + 1] = 0.0;
    for (int i{0}; i <= j; i++) {
      work[j + 1] += std::abs(T[i + 12 * (j + 1)]);
    }
  }
  ip = 0;
  for (int ki{11}; ki >= 0; ki--) {
    if (ip == -1) {
      ip = 1;
    } else {
      double smin;
      double wi;
      double wr;
      int iyend;
      if ((ki == 0) || (T[ki + 12 * (ki - 1)] == 0.0)) {
        ip = 0;
      } else {
        ip = -1;
      }
      iyend = ki + 12 * ki;
      wr = T[iyend];
      wi = 0.0;
      if (ip != 0) {
        wi = std::sqrt(std::abs(T[ki + 12 * (ki - 1)])) *
             std::sqrt(std::abs(T[iyend - 1]));
      }
      smin = std::fmax(2.2204460492503131E-16 * (std::abs(wr) + wi),
                       1.2025010160053837E-291);
      if (ip == 0) {
        __m128d r;
        double scale;
        int b_j;
        int b_vectorUB;
        int ii;
        int vectorUB;
        work[ki + 24] = 1.0;
        iyend = (ki / 2) << 1;
        ii = iyend - 2;
        for (int j{0}; j <= ii; j += 2) {
          _mm_storeu_pd(&work[j + 24], _mm_mul_pd(_mm_loadu_pd(&T[j + 12 * ki]),
                                                  _mm_set1_pd(-1.0)));
        }
        for (int j{iyend}; j < ki; j++) {
          work[j + 24] = -T[j + 12 * ki];
        }
        b_j = ki - 1;
        int exitg1;
        do {
          exitg1 = 0;
          if (b_j + 1 >= 1) {
            boolean_T guard1;
            guard1 = false;
            if (b_j + 1 == 1) {
              guard1 = true;
            } else {
              int b_i;
              b_i = 12 * (b_j - 1);
              iyend = b_j + b_i;
              if (T[iyend] == 0.0) {
                guard1 = true;
              } else {
                scale = xdlaln2(2, 1, smin, T, iyend, work, b_j + 24, wr, 0.0,
                                x, smax);
                if ((smax > 1.0) && (std::fmax(work[b_j - 1], work[b_j]) >
                                     8.3160012897279974E+290 / smax)) {
                  x[0] /= smax;
                  x[1] /= smax;
                  scale /= smax;
                }
                if (scale != 1.0) {
                  ii = ki + 25;
                  vectorUB = (((ki + 1) / 2) << 1) + 25;
                  b_vectorUB = vectorUB - 2;
                  for (int j{25}; j <= b_vectorUB; j += 2) {
                    r = _mm_loadu_pd(&work[j - 1]);
                    _mm_storeu_pd(&work[j - 1],
                                  _mm_mul_pd(_mm_set1_pd(scale), r));
                  }
                  for (int j{vectorUB}; j <= ii; j++) {
                    work[j - 1] *= scale;
                  }
                }
                work[b_j + 23] = x[0];
                work[b_j + 24] = x[1];
                blas::xaxpy(b_j - 1, -x[0], T, b_i + 1, work);
                blas::xaxpy(b_j - 1, -x[1], T, b_j * 12 + 1, work);
                b_j -= 2;
              }
            }
            if (guard1) {
              scale = xdlaln2(1, 1, smin, T, (b_j * 12 + b_j) + 1, work,
                              b_j + 25, wr, 0.0, x, smax);
              if ((smax > 1.0) &&
                  (work[b_j] > 8.3160012897279974E+290 / smax)) {
                x[0] /= smax;
                scale /= smax;
              }
              if (scale != 1.0) {
                iyend = ki + 25;
                ii = (((ki + 1) / 2) << 1) + 25;
                vectorUB = ii - 2;
                for (int j{25}; j <= vectorUB; j += 2) {
                  r = _mm_loadu_pd(&work[j - 1]);
                  _mm_storeu_pd(&work[j - 1],
                                _mm_mul_pd(_mm_set1_pd(scale), r));
                }
                for (int j{ii}; j <= iyend; j++) {
                  work[j - 1] *= scale;
                }
              }
              work[b_j + 24] = x[0];
              blas::xaxpy(b_j, -x[0], T, b_j * 12 + 1, work);
              b_j--;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        if (ki > 0) {
          blas::xgemv(ki, work, work[ki + 24], vr, ki * 12 + 1);
        }
        iyend = ki * 12;
        ii = 0;
        smax = std::abs(vr[iyend]);
        for (int j{0}; j < 11; j++) {
          scale = std::abs(vr[(iyend + j) + 1]);
          if (scale > smax) {
            ii = j + 1;
            smax = scale;
          }
        }
        smax = 1.0 / std::abs(vr[ii + 12 * ki]);
        iyend = ki * 12;
        ii = iyend + 12;
        vectorUB = ((ii - iyend) / 2 * 2 + iyend) + 1;
        b_vectorUB = vectorUB - 2;
        for (int j{iyend + 1}; j <= b_vectorUB; j += 2) {
          r = _mm_loadu_pd(&vr[j - 1]);
          r = _mm_mul_pd(_mm_set1_pd(smax), r);
          _mm_storeu_pd(&vr[j - 1], r);
        }
        for (int j{vectorUB}; j <= ii; j++) {
          vr[j - 1] *= smax;
        }
      } else {
        __m128d r;
        double scale;
        int b_j;
        int b_vectorUB;
        int ii;
        int ix0;
        int vectorUB;
        ix0 = 12 * (ki - 1);
        smax = T[ki + ix0];
        scale = T[iyend - 1];
        if (std::abs(scale) >= std::abs(smax)) {
          work[ki + 11] = 1.0;
          work[ki + 24] = wi / scale;
        } else {
          work[ki + 11] = -wi / smax;
          work[ki + 24] = 1.0;
        }
        work[ki + 12] = 0.0;
        work[ki + 23] = 0.0;
        for (int j{0}; j <= ki - 2; j++) {
          work[j + 12] = -work[ki + 11] * T[j + ix0];
          work[j + 24] = -work[ki + 24] * T[j + 12 * ki];
        }
        b_j = ki - 2;
        int exitg1;
        do {
          exitg1 = 0;
          if (b_j + 1 >= 1) {
            boolean_T guard1;
            guard1 = false;
            if (b_j + 1 == 1) {
              guard1 = true;
            } else {
              int b_i;
              b_i = 12 * (b_j - 1);
              iyend = b_j + b_i;
              if (T[iyend] == 0.0) {
                guard1 = true;
              } else {
                scale = xdlaln2(2, 2, smin, T, iyend, work, b_j + 12, wr, wi, x,
                                smax);
                if ((smax > 1.0) && (std::fmax(work[b_j - 1], work[b_j]) >
                                     8.3160012897279974E+290 / smax)) {
                  smax = 1.0 / smax;
                  x[0] *= smax;
                  x[2] *= smax;
                  x[1] *= smax;
                  x[3] *= smax;
                  scale *= smax;
                }
                if (scale != 1.0) {
                  ii = ki + 13;
                  b_vectorUB = ((ki + 1) / 2) << 1;
                  vectorUB = b_vectorUB + 13;
                  iyend = b_vectorUB + 11;
                  for (int j{13}; j <= iyend; j += 2) {
                    r = _mm_loadu_pd(&work[j - 1]);
                    _mm_storeu_pd(&work[j - 1],
                                  _mm_mul_pd(_mm_set1_pd(scale), r));
                  }
                  for (int j{vectorUB}; j <= ii; j++) {
                    work[j - 1] *= scale;
                  }
                  vectorUB = ki + 25;
                  iyend = b_vectorUB + 25;
                  ii = b_vectorUB + 23;
                  for (int j{25}; j <= ii; j += 2) {
                    r = _mm_loadu_pd(&work[j - 1]);
                    _mm_storeu_pd(&work[j - 1],
                                  _mm_mul_pd(_mm_set1_pd(scale), r));
                  }
                  for (int j{iyend}; j <= vectorUB; j++) {
                    work[j - 1] *= scale;
                  }
                }
                work[b_j + 11] = x[0];
                work[b_j + 12] = x[1];
                work[b_j + 23] = x[2];
                work[b_j + 24] = x[3];
                if ((b_j - 1 >= 1) && (!(-x[0] == 0.0))) {
                  ii = b_j - 2;
                  vectorUB = ((b_j - 1) / 2) << 1;
                  iyend = vectorUB - 2;
                  for (int j{0}; j <= iyend; j += 2) {
                    r = _mm_loadu_pd(&work[j + 12]);
                    _mm_storeu_pd(
                        &work[j + 12],
                        _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(-x[0]),
                                                 _mm_loadu_pd(&T[b_i + j]))));
                  }
                  for (int j{vectorUB}; j <= ii; j++) {
                    work[j + 12] += -x[0] * T[b_i + j];
                  }
                }
                if ((b_j - 1 >= 1) && (!(-x[1] == 0.0))) {
                  ii = b_j * 12;
                  vectorUB = b_j - 2;
                  iyend = ((b_j - 1) / 2) << 1;
                  b_vectorUB = iyend - 2;
                  for (int j{0}; j <= b_vectorUB; j += 2) {
                    r = _mm_loadu_pd(&work[j + 12]);
                    _mm_storeu_pd(
                        &work[j + 12],
                        _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(-x[1]),
                                                 _mm_loadu_pd(&T[ii + j]))));
                  }
                  for (int j{iyend}; j <= vectorUB; j++) {
                    work[j + 12] += -x[1] * T[ii + j];
                  }
                }
                blas::xaxpy(b_j - 1, -x[2], T, b_i + 1, work);
                blas::xaxpy(b_j - 1, -x[3], T, b_j * 12 + 1, work);
                b_j -= 2;
              }
            }
            if (guard1) {
              scale = xdlaln2(1, 2, smin, T, (b_j * 12 + b_j) + 1, work,
                              b_j + 13, wr, wi, x, smax);
              if ((smax > 1.0) &&
                  (work[b_j] > 8.3160012897279974E+290 / smax)) {
                x[0] /= smax;
                x[2] /= smax;
                scale /= smax;
              }
              if (scale != 1.0) {
                ii = ki + 13;
                b_vectorUB = ((ki + 1) / 2) << 1;
                vectorUB = b_vectorUB + 13;
                iyend = b_vectorUB + 11;
                for (int j{13}; j <= iyend; j += 2) {
                  r = _mm_loadu_pd(&work[j - 1]);
                  _mm_storeu_pd(&work[j - 1],
                                _mm_mul_pd(_mm_set1_pd(scale), r));
                }
                for (int j{vectorUB}; j <= ii; j++) {
                  work[j - 1] *= scale;
                }
                vectorUB = ki + 25;
                iyend = b_vectorUB + 25;
                ii = b_vectorUB + 23;
                for (int j{25}; j <= ii; j += 2) {
                  r = _mm_loadu_pd(&work[j - 1]);
                  _mm_storeu_pd(&work[j - 1],
                                _mm_mul_pd(_mm_set1_pd(scale), r));
                }
                for (int j{iyend}; j <= vectorUB; j++) {
                  work[j - 1] *= scale;
                }
              }
              work[b_j + 12] = x[0];
              work[b_j + 24] = x[2];
              if ((b_j >= 1) && (!(-x[0] == 0.0))) {
                ii = b_j * 12;
                vectorUB = (b_j / 2) << 1;
                iyend = vectorUB - 2;
                for (int j{0}; j <= iyend; j += 2) {
                  r = _mm_loadu_pd(&work[j + 12]);
                  _mm_storeu_pd(
                      &work[j + 12],
                      _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(-x[0]),
                                               _mm_loadu_pd(&T[ii + j]))));
                }
                for (int j{vectorUB}; j < b_j; j++) {
                  work[j + 12] += -x[0] * T[ii + j];
                }
              }
              blas::xaxpy(b_j, -x[2], T, b_j * 12 + 1, work);
              b_j--;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        if (ki + 1 > 2) {
          iyend = ix0 + 12;
          smax = work[ki + 11];
          if (smax != 1.0) {
            if (smax == 0.0) {
              if (ix0 + 1 <= iyend) {
                std::memset(&vr[ix0], 0,
                            static_cast<unsigned int>(iyend - ix0) *
                                sizeof(double));
              }
            } else {
              ii = ((iyend - ix0) / 2 * 2 + ix0) + 1;
              vectorUB = ii - 2;
              for (int j{ix0 + 1}; j <= vectorUB; j += 2) {
                r = _mm_loadu_pd(&vr[j - 1]);
                r = _mm_mul_pd(_mm_set1_pd(smax), r);
                _mm_storeu_pd(&vr[j - 1], r);
              }
              for (int j{ii}; j <= iyend; j++) {
                vr[j - 1] *= smax;
              }
            }
          }
          ii = 12;
          vectorUB = 12 * (ki - 2) + 1;
          for (int j{1}; j <= vectorUB; j += 12) {
            iyend = j + 11;
            for (int i{j}; i <= iyend; i++) {
              b_vectorUB = (ix0 + i) - j;
              vr[b_vectorUB] += vr[i - 1] * work[ii];
            }
            ii++;
          }
          blas::xgemv(ki - 1, work, work[ki + 24], vr, ki * 12 + 1);
        } else {
          ii = ix0 + 12;
          vectorUB = ((ii - ix0) / 2 * 2 + ix0) + 1;
          b_vectorUB = vectorUB - 2;
          for (int j{ix0 + 1}; j <= b_vectorUB; j += 2) {
            r = _mm_loadu_pd(&vr[j - 1]);
            r = _mm_mul_pd(_mm_set1_pd(work[12]), r);
            _mm_storeu_pd(&vr[j - 1], r);
          }
          for (int j{vectorUB}; j <= ii; j++) {
            vr[j - 1] *= work[12];
          }
          ii = ki * 12;
          vectorUB = ii + 12;
          iyend = ((vectorUB - ii) / 2 * 2 + ii) + 1;
          b_vectorUB = iyend - 2;
          for (int j{ii + 1}; j <= b_vectorUB; j += 2) {
            r = _mm_loadu_pd(&vr[j - 1]);
            r = _mm_mul_pd(_mm_set1_pd(work[ki + 24]), r);
            _mm_storeu_pd(&vr[j - 1], r);
          }
          for (int j{iyend}; j <= vectorUB; j++) {
            vr[j - 1] *= work[ki + 24];
          }
        }
        smax = 0.0;
        for (int j{0}; j < 12; j++) {
          smax = std::fmax(smax,
                           std::abs(vr[j + ix0]) + std::abs(vr[j + 12 * ki]));
        }
        smax = 1.0 / smax;
        ii = ix0 + 12;
        vectorUB = ((ii - ix0) / 2 * 2 + ix0) + 1;
        b_vectorUB = vectorUB - 2;
        for (int j{ix0 + 1}; j <= b_vectorUB; j += 2) {
          r = _mm_loadu_pd(&vr[j - 1]);
          r = _mm_mul_pd(_mm_set1_pd(smax), r);
          _mm_storeu_pd(&vr[j - 1], r);
        }
        for (int j{vectorUB}; j <= ii; j++) {
          vr[j - 1] *= smax;
        }
        ii = ki * 12;
        vectorUB = ii + 12;
        b_vectorUB = ((vectorUB - ii) / 2 * 2 + ii) + 1;
        iyend = b_vectorUB - 2;
        for (int j{ii + 1}; j <= iyend; j += 2) {
          r = _mm_loadu_pd(&vr[j - 1]);
          r = _mm_mul_pd(_mm_set1_pd(smax), r);
          _mm_storeu_pd(&vr[j - 1], r);
        }
        for (int j{b_vectorUB}; j <= vectorUB; j++) {
          vr[j - 1] *= smax;
        }
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdtrevc3.cpp)
