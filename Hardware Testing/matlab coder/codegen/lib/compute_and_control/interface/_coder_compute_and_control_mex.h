//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_compute_and_control_mex.h
//
// Code generation for function 'compute_and_control'
//

#ifndef _CODER_COMPUTE_AND_CONTROL_MEX_H
#define _CODER_COMPUTE_AND_CONTROL_MEX_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_compute_and_control_mexFunction(int32_T nlhs, mxArray *plhs[3],
                                            int32_T nrhs,
                                            const mxArray *prhs[13]);

#endif
// End of code generation (_coder_compute_and_control_mex.h)
