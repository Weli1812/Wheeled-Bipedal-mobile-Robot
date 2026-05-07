/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzgehrd.h
 *
 * Code generation for function 'xzgehrd'
 *
 */

#ifndef XZGEHRD_H
#define XZGEHRD_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xzgehrd(double a[36], int ilo, int ihi);

void xzgehrd(double a[144], int ilo, int ihi, double tau[11]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (xzgehrd.h) */
