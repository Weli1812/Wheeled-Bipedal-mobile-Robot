//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_LQRwheels_mex.cpp
//
// Code generation for function 'LQRwheels'
//

// Include files
#include "_coder_LQRwheels_mex.h"
#include "_coder_LQRwheels_api.h"

// Function Definitions
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&LQRwheels_atexit);
  LQRwheels_initialize();
  unsafe_LQRwheels_mexFunction(nlhs, plhs, nrhs, prhs);
  LQRwheels_terminate();
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

void unsafe_LQRwheels_mexFunction(int32_T nlhs, mxArray *plhs[4], int32_T nrhs,
                                  const mxArray *prhs[11])
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  const mxArray *b_prhs[11];
  const mxArray *outputs[4];
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  // Check for proper number of arguments.
  if (nrhs != 11) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 11, 4,
                        9, "LQRwheels");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 9,
                        "LQRwheels");
  }
  // Call the function.
  for (int32_T i = 0; i < 11; i++) {
    b_prhs[i] = prhs[i];
  }
  LQRwheels_api(b_prhs, nlhs, outputs);
  // Copy over outputs to the caller.
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

// End of code generation (_coder_LQRwheels_mex.cpp)
