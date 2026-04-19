#ifndef TMWTYPES_H
#define TMWTYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef double real_T;
typedef float real32_T;

typedef int8_t int8_T;
typedef uint8_t uint8_T;
typedef int16_t int16_T;
typedef uint16_t uint16_T;
typedef int32_t int32_T;
typedef uint32_t uint32_T;
typedef int64_t int64_T;
typedef uint64_t uint64_T;

typedef long long longlong_T;
typedef unsigned long long ulonglong_T;

typedef int int_T;
typedef unsigned int uint_T;

typedef char char_T;
typedef signed char schar_T;
typedef unsigned char uchar_T;
typedef unsigned char byte_T;
typedef uint8_T boolean_T;

typedef double time_T;

#ifndef true
#define true (1U)
#endif

#ifndef false
#define false (0U)
#endif

#ifdef __cplusplus
}
#endif

#endif