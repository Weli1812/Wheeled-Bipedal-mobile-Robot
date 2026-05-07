/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * compute_robot_lqr.h
 *
 * Code generation for function 'compute_robot_lqr'
 *
 */

#ifndef COMPUTE_ROBOT_LQR_H
#define COMPUTE_ROBOT_LQR_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void compute_robot_lqr(double L, double Iy, double Iz, double Mw,
                              double r, double d, double Iw, double Mb,
                              const double Q_diag[6], const double R_diag[2],
                              double K[12], creal_T cl_poles[6]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (compute_robot_lqr.h) */
