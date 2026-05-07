/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * xdlahqr.h
 *
 * Code generation for function 'xdlahqr'
 *
 */

#ifndef XDLAHQR_H
#define XDLAHQR_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int b_xdlahqr(int ilo, int ihi, double h[36], double *z, double wr[6],
              double wi[6]);

int xdlahqr(int ilo, int ihi, double h[144], int iloz, int ihiz, double z[144],
            double wr[12], double wi[12]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (xdlahqr.h) */
