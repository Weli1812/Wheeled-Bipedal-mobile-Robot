//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_LQRwheels_api.cpp
//
// Code generation for function 'LQRwheels'
//

// Include files
#include "_coder_LQRwheels_api.h"
#include "_coder_LQRwheels_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131659U,                                              // fVersionInfo
    NULL,                                                 // fErrorFunction
    "LQRwheels",                                          // fFunctionName
    NULL,                                                 // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    NULL                                                  // fSigMem
};

// Function Declarations
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static const mxArray *b_emlrt_marshallOut(const boolean_T u);

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                               const char_T *identifier);

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId);

static const mxArray *emlrt_marshallOut(const real_T u);

// Function Definitions
static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static const mxArray *b_emlrt_marshallOut(const boolean_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateLogicalScalar(u);
  emlrtAssign(&y, m);
  return y;
}

static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                               const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = b_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m);
  return y;
}

void LQRwheels_api(const mxArray *const prhs[11], int32_T nlhs,
                   const mxArray *plhs[4])
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  real_T dt;
  real_T pwml;
  real_T pwmr;
  real_T theta_curr;
  real_T thetad_k;
  real_T vd_k;
  real_T vl_actual;
  real_T vr_actual;
  real_T wd_k;
  real_T x_curr;
  real_T xd_k;
  real_T y_curr;
  real_T yd_k;
  boolean_T dirl;
  boolean_T dirr;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  x_curr = emlrt_marshallIn(st, emlrtAliasP(prhs[0]), "x_curr");
  y_curr = emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "y_curr");
  theta_curr = emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "theta_curr");
  dt = emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "dt");
  xd_k = emlrt_marshallIn(st, emlrtAliasP(prhs[4]), "xd_k");
  yd_k = emlrt_marshallIn(st, emlrtAliasP(prhs[5]), "yd_k");
  thetad_k = emlrt_marshallIn(st, emlrtAliasP(prhs[6]), "thetad_k");
  vd_k = emlrt_marshallIn(st, emlrtAliasP(prhs[7]), "vd_k");
  wd_k = emlrt_marshallIn(st, emlrtAliasP(prhs[8]), "wd_k");
  vr_actual = emlrt_marshallIn(st, emlrtAliasP(prhs[9]), "vr_actual");
  vl_actual = emlrt_marshallIn(st, emlrtAliasP(prhs[10]), "vl_actual");
  // Invoke the target function
  LQRwheels(x_curr, y_curr, theta_curr, dt, xd_k, yd_k, thetad_k, vd_k, wd_k,
            vr_actual, vl_actual, &pwmr, &pwml, &dirr, &dirl);
  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(pwmr);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(pwml);
  }
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(dirr);
  }
  if (nlhs > 3) {
    plhs[3] = b_emlrt_marshallOut(dirl);
  }
}

void LQRwheels_atexit()
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(
      &st, false, NULL, (void *)&emlrtExitTimeCleanupDtorFcn, NULL, NULL, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  LQRwheels_xil_terminate();
  LQRwheels_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void LQRwheels_initialize()
{
  emlrtStack st = {
      NULL, // site
      NULL, // tls
      NULL  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void LQRwheels_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (_coder_LQRwheels_api.cpp)
