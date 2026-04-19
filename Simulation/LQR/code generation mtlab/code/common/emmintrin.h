#ifndef EMMINTRIN_FALLBACK_H
#define EMMINTRIN_FALLBACK_H

typedef struct {
  double v[2];
} __m128d;

static inline __m128d _mm_set1_pd(double a)
{
  __m128d r;
  r.v[0] = a;
  r.v[1] = a;
  return r;
}

static inline __m128d _mm_set_pd(double e1, double e0)
{
  __m128d r;
  r.v[0] = e0;
  r.v[1] = e1;
  return r;
}

static inline __m128d _mm_loadu_pd(const double *p)
{
  __m128d r;
  r.v[0] = p[0];
  r.v[1] = p[1];
  return r;
}

static inline void _mm_storeu_pd(double *p, __m128d a)
{
  p[0] = a.v[0];
  p[1] = a.v[1];
}

static inline __m128d _mm_add_pd(__m128d a, __m128d b)
{
  __m128d r;
  r.v[0] = a.v[0] + b.v[0];
  r.v[1] = a.v[1] + b.v[1];
  return r;
}

static inline __m128d _mm_mul_pd(__m128d a, __m128d b)
{
  __m128d r;
  r.v[0] = a.v[0] * b.v[0];
  r.v[1] = a.v[1] * b.v[1];
  return r;
}

#endif