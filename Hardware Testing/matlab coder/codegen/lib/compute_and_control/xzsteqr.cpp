//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzsteqr.cpp
//
// Code generation for function 'xzsteqr'
//

// Include files
#include "xzsteqr.h"
#include "rt_nonfinite.h"
#include "xdlaev2.h"
#include "xzlartg.h"
#include "xzlascl.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
static void b_rotateRight(int n, double z[144], int iz0, const double cs[22],
                          int ic0, int is0);

static void rotateRight(int n, double z[144], int iz0, const double cs[22],
                        int ic0, int is0);

} // namespace reflapack
} // namespace internal
} // namespace coder

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
static void b_rotateRight(int n, double z[144], int iz0, const double cs[22],
                          int ic0, int is0)
{
  for (int j{0}; j <= n - 2; j++) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 1];
    stemp = cs[(is0 + j) - 1];
    offsetj = (j * 12 + iz0) - 2;
    offsetjp1 = ((j + 1) * 12 + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      for (int i{0}; i < 12; i++) {
        double temp;
        int b_i;
        int temp_tmp;
        temp_tmp = (offsetjp1 + i) + 1;
        temp = z[temp_tmp];
        b_i = (offsetj + i) + 1;
        z[temp_tmp] = ctemp * temp - stemp * z[b_i];
        z[b_i] = stemp * temp + ctemp * z[b_i];
      }
    }
  }
}

static void rotateRight(int n, double z[144], int iz0, const double cs[22],
                        int ic0, int is0)
{
  int i;
  i = n - 1;
  for (int j{i}; j >= 1; j--) {
    double ctemp;
    double stemp;
    int offsetj;
    int offsetjp1;
    ctemp = cs[(ic0 + j) - 2];
    stemp = cs[(is0 + j) - 2];
    offsetj = ((j - 1) * 12 + iz0) - 2;
    offsetjp1 = (j * 12 + iz0) - 2;
    if ((ctemp != 1.0) || (stemp != 0.0)) {
      for (int b_i{0}; b_i < 12; b_i++) {
        double temp;
        int i1;
        int temp_tmp;
        temp_tmp = (offsetjp1 + b_i) + 1;
        temp = z[temp_tmp];
        i1 = (offsetj + b_i) + 1;
        z[temp_tmp] = ctemp * temp - stemp * z[i1];
        z[i1] = stemp * temp + ctemp * z[i1];
      }
    }
  }
}

int xzsteqr(double d[12], double e[11], double z[144])
{
  double work[22];
  int info;
  int jtot;
  int l1;
  info = 0;
  std::memset(&work[0], 0, 22U * sizeof(double));
  jtot = 0;
  l1 = 1;
  int exitg1;
  do {
    exitg1 = 0;
    if (l1 > 12) {
      for (int ii{0}; ii < 11; ii++) {
        double p;
        double temp;
        int k;
        k = ii;
        p = d[ii];
        for (int i{ii + 2}; i < 13; i++) {
          temp = d[i - 1];
          if (temp < p) {
            k = i - 1;
            p = temp;
          }
        }
        if (k != ii) {
          int ix;
          d[k] = d[ii];
          d[ii] = p;
          ix = ii * 12;
          k *= 12;
          for (int i{0}; i < 12; i++) {
            int iscale;
            int n_tmp;
            iscale = ix + i;
            temp = z[iscale];
            n_tmp = k + i;
            z[iscale] = z[n_tmp];
            z[n_tmp] = temp;
          }
        }
      }
      exitg1 = 1;
    } else {
      double temp;
      int ix;
      int l;
      int lend;
      int lendsv;
      int lsv;
      boolean_T exitg2;
      if (l1 > 1) {
        e[l1 - 2] = 0.0;
      }
      ix = l1;
      exitg2 = false;
      while ((!exitg2) && (ix < 12)) {
        temp = std::abs(e[ix - 1]);
        if (temp == 0.0) {
          exitg2 = true;
        } else if (temp <= std::sqrt(std::abs(d[ix - 1])) *
                               std::sqrt(std::abs(d[ix])) *
                               2.2204460492503131E-16) {
          e[ix - 1] = 0.0;
          exitg2 = true;
        } else {
          ix++;
        }
      }
      l = l1 - 1;
      lsv = l1;
      lend = ix;
      lendsv = ix;
      l1 = ix + 1;
      if (ix != l + 1) {
        double anorm;
        int iscale;
        int k;
        int n_tmp;
        n_tmp = ix - l;
        if (n_tmp <= 0) {
          anorm = 0.0;
        } else {
          anorm = std::abs(d[(l + n_tmp) - 1]);
          k = 0;
          exitg2 = false;
          while ((!exitg2) && (k <= n_tmp - 2)) {
            iscale = l + k;
            temp = std::abs(d[iscale]);
            if (std::isnan(temp)) {
              anorm = rtNaN;
              exitg2 = true;
            } else {
              if (temp > anorm) {
                anorm = temp;
              }
              temp = std::abs(e[iscale]);
              if (std::isnan(temp)) {
                anorm = rtNaN;
                exitg2 = true;
              } else {
                if (temp > anorm) {
                  anorm = temp;
                }
                k++;
              }
            }
          }
        }
        iscale = 0;
        if (!(anorm == 0.0)) {
          if (std::isinf(anorm) || std::isnan(anorm)) {
            for (int i{0}; i < 12; i++) {
              d[i] = rtNaN;
            }
            for (int i{0}; i < 144; i++) {
              z[i] = rtNaN;
            }
            exitg1 = 1;
          } else {
            if (anorm > 2.2346346549904327E+153) {
              iscale = 1;
              xzlascl(anorm, 2.2346346549904327E+153, n_tmp, d, l + 1);
              b_xzlascl(anorm, 2.2346346549904327E+153, n_tmp - 1, e, l + 1);
            } else if (anorm < 3.02546243347603E-123) {
              iscale = 2;
              xzlascl(anorm, 3.02546243347603E-123, n_tmp, d, l + 1);
              b_xzlascl(anorm, 3.02546243347603E-123, n_tmp - 1, e, l + 1);
            }
            if (std::abs(d[ix - 1]) < std::abs(d[l])) {
              lend = lsv;
              l = ix - 1;
            }
            if (lend > l + 1) {
              int exitg4;
              do {
                exitg4 = 0;
                if (l + 1 != lend) {
                  ix = l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (ix < lend)) {
                    temp = std::abs(e[ix - 1]);
                    if (temp * temp <= 4.9303806576313238E-32 *
                                               std::abs(d[ix - 1]) *
                                               std::abs(d[ix]) +
                                           2.2250738585072014E-308) {
                      exitg2 = true;
                    } else {
                      ix++;
                    }
                  }
                } else {
                  ix = lend;
                }
                if (ix < lend) {
                  e[ix - 1] = 0.0;
                }
                if (ix == l + 1) {
                  l++;
                  if (l + 1 > lend) {
                    exitg4 = 1;
                  }
                } else if (ix == l + 2) {
                  double r;
                  d[l] = xdlaev2(d[l], e[l], d[l + 1], temp, work[l], r);
                  d[l + 1] = temp;
                  work[l + 11] = r;
                  rotateRight(2, z, l * 12 + 1, work, l + 1, l + 12);
                  e[l] = 0.0;
                  l += 2;
                  if (l + 1 > lend) {
                    exitg4 = 1;
                  }
                } else if (jtot == 360) {
                  exitg4 = 1;
                } else {
                  double c;
                  double g;
                  double p;
                  double r;
                  double s;
                  jtot++;
                  g = (d[l + 1] - d[l]) / (2.0 * e[l]);
                  temp = std::abs(g);
                  if (temp < 1.0) {
                    temp = std::sqrt(temp * temp + 1.0);
                  } else if (temp > 1.0) {
                    r = 1.0 / temp;
                    temp *= std::sqrt(r * r + 1.0);
                  } else {
                    temp *= 1.4142135623730951;
                  }
                  if (!(g >= 0.0)) {
                    temp = -temp;
                  }
                  g = (d[ix - 1] - d[l]) + e[l] / (g + temp);
                  s = 1.0;
                  c = 1.0;
                  p = 0.0;
                  k = ix - 1;
                  for (int i{k}; i >= l + 1; i--) {
                    double b;
                    temp = e[i - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, s, r);
                    if (i != ix - 1) {
                      e[i] = r;
                    }
                    g = d[i] - p;
                    temp = (d[i - 1] - g) * s + 2.0 * c * b;
                    p = s * temp;
                    d[i] = g + p;
                    g = c * temp - b;
                    work[i - 1] = c;
                    work[i + 10] = -s;
                  }
                  rotateRight(ix - l, z, l * 12 + 1, work, l + 1, l + 12);
                  d[l] -= p;
                  e[l] = g;
                }
              } while (exitg4 == 0);
            } else {
              int exitg3;
              do {
                exitg3 = 0;
                if (l + 1 != lend) {
                  ix = l + 1;
                  exitg2 = false;
                  while ((!exitg2) && (ix > lend)) {
                    temp = std::abs(e[ix - 2]);
                    if (temp * temp <= 4.9303806576313238E-32 *
                                               std::abs(d[ix - 1]) *
                                               std::abs(d[ix - 2]) +
                                           2.2250738585072014E-308) {
                      exitg2 = true;
                    } else {
                      ix--;
                    }
                  }
                } else {
                  ix = lend;
                }
                if (ix > lend) {
                  e[ix - 2] = 0.0;
                }
                if (ix == l + 1) {
                  l--;
                  if (l + 1 < lend) {
                    exitg3 = 1;
                  }
                } else if (ix == l) {
                  double r;
                  d[l - 1] =
                      xdlaev2(d[l - 1], e[l - 1], d[l], temp, work[ix - 1], r);
                  d[l] = temp;
                  work[ix + 10] = r;
                  b_rotateRight(2, z, (l - 1) * 12 + 1, work, ix, ix + 11);
                  e[l - 1] = 0.0;
                  l -= 2;
                  if (l + 1 < lend) {
                    exitg3 = 1;
                  }
                } else if (jtot == 360) {
                  exitg3 = 1;
                } else {
                  double c;
                  double g;
                  double g_tmp;
                  double p;
                  double s;
                  jtot++;
                  g_tmp = e[l - 1];
                  g = (d[l - 1] - d[l]) / (2.0 * g_tmp);
                  temp = std::abs(g);
                  if (temp < 1.0) {
                    temp = std::sqrt(temp * temp + 1.0);
                  } else if (temp > 1.0) {
                    double r;
                    r = 1.0 / temp;
                    temp *= std::sqrt(r * r + 1.0);
                  } else {
                    temp *= 1.4142135623730951;
                  }
                  if (!(g >= 0.0)) {
                    temp = -temp;
                  }
                  g = (d[ix - 1] - d[l]) + g_tmp / (g + temp);
                  s = 1.0;
                  c = 1.0;
                  p = 0.0;
                  for (int i{ix}; i <= l; i++) {
                    double b;
                    temp = e[i - 1];
                    b = c * temp;
                    c = xzlartg(g, s * temp, s, g_tmp);
                    if (i != ix) {
                      e[i - 2] = g_tmp;
                    }
                    g = d[i - 1] - p;
                    temp = (d[i] - g) * s + 2.0 * c * b;
                    p = s * temp;
                    d[i - 1] = g + p;
                    g = c * temp - b;
                    work[i - 1] = c;
                    work[i + 10] = s;
                  }
                  b_rotateRight((l - ix) + 2, z, (ix - 1) * 12 + 1, work, ix,
                                ix + 11);
                  d[l] -= p;
                  e[l - 1] = g;
                }
              } while (exitg3 == 0);
            }
            if (iscale == 1) {
              k = lendsv - lsv;
              xzlascl(2.2346346549904327E+153, anorm, k + 1, d, lsv);
              b_xzlascl(2.2346346549904327E+153, anorm, k, e, lsv);
            } else if (iscale == 2) {
              k = lendsv - lsv;
              xzlascl(3.02546243347603E-123, anorm, k + 1, d, lsv);
              b_xzlascl(3.02546243347603E-123, anorm, k, e, lsv);
            }
            if (jtot >= 360) {
              for (int i{0}; i < 11; i++) {
                if (e[i] != 0.0) {
                  info++;
                }
              }
              exitg1 = 1;
            }
          }
        }
      }
    }
  } while (exitg1 == 0);
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzsteqr.cpp)
