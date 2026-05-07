//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// BipedController.h
//
// Code generation for function 'BipedController'
//

#ifndef BIPEDCONTROLLER_H
#define BIPEDCONTROLLER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class BipedController {
public:
  BipedController();
  ~BipedController();
  void compute_and_control(const double state[6], double L, double Iy,
                           double Iz, double Mw, double r, double d, double Iw,
                           double Mb, const double Q_diag[6],
                           const double R_diag[2], double Kt, double V_batt,
                           double *pwm_R, double *pwm_L, double K[12]);
};

#endif
// End of code generation (BipedController.h)
