/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_lqr_tracker_step_codegen_mex.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 19-Apr-2026 13:12:28
 */

/* Include Files */
#include "_coder_lqr_tracker_step_codegen_mex.h"
#include "_coder_lqr_tracker_step_codegen_api.h"

/* Function Definitions */
/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&lqr_tracker_step_codegen_atexit);
  lqr_tracker_step_codegen_initialize();
  unsafe_lqr_tracker_step_codegen_mexFunction(nlhs, plhs, nrhs, prhs);
  lqr_tracker_step_codegen_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[4]
 *                int32_T nrhs
 *                const mxArray *prhs[11]
 * Return Type  : void
 */
void unsafe_lqr_tracker_step_codegen_mexFunction(int32_T nlhs, mxArray *plhs[4],
                                                 int32_T nrhs,
                                                 const mxArray *prhs[11])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *b_prhs[11];
  const mxArray *outputs[4];
  int32_T i;
  int32_T i1;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 11) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 11, 4,
                        24, "lqr_tracker_step_codegen");
  }
  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 24,
                        "lqr_tracker_step_codegen");
  }
  /* Call the function. */
  for (i = 0; i < 11; i++) {
    b_prhs[i] = prhs[i];
  }
  lqr_tracker_step_codegen_api(b_prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i1 = 1;
  } else {
    i1 = nlhs;
  }
  emlrtReturnArrays(i1, &plhs[0], &outputs[0]);
}

/*
 * File trailer for _coder_lqr_tracker_step_codegen_mex.c
 *
 * [EOF]
 */
