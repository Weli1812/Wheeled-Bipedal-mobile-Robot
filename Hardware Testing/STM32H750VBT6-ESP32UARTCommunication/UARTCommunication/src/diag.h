//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// diag.h
//
// Code generation for function 'diag'
//

#ifndef DIAG_H
#define DIAG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void b_diag(const float v[6], float d[36]);

void diag(const float v[7], float d[49]);

} // namespace coder

#endif
// End of code generation (diag.h)
