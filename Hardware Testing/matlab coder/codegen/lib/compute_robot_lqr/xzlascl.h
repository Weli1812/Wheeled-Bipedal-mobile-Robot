/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xzlascl.h
 *
 * Code generation for function 'xzlascl'
 *
 */

#ifndef XZLASCL_H
#define XZLASCL_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_xzlascl(double cfrom, double cto, int m, double A[12], int iA0);

void c_xzlascl(double cfrom, double cto, int m, double A[11], int iA0);

void d_xzlascl(double cfrom, double cto, double A[36]);

void e_xzlascl(double cfrom, double cto, int m, double A[6], int iA0);

void f_xzlascl(double cfrom, double cto, int m, double A[5], int iA0);

void xzlascl(double cfrom, double cto, double A[144]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (xzlascl.h) */
