/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eigStandard.h
 *
 * Code generation for function 'eigStandard'
 *
 */

#ifndef EIGSTANDARD_H
#define EIGSTANDARD_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_eigStandard(const double A[36], creal_T V[6]);

void eigStandard(const double A[144], creal_T V[144], creal_T D[144]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (eigStandard.h) */
