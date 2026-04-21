/*
 * Traj2.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Traj2".
 *
 * Model version              : 3.6
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C source code generated on : Mon Apr 20 22:37:27 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "Traj2.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "Traj2_private.h"
#include <string.h>
#include "rt_defines.h"

/* Block signals (default storage) */
B_Traj2_T Traj2_B;

/* Continuous states */
X_Traj2_T Traj2_X;

/* Disabled State Vector */
XDis_Traj2_T Traj2_XDis;

/* Block states (default storage) */
DW_Traj2_T Traj2_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Traj2_T Traj2_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Traj2_T Traj2_Y;

/* Real-time model */
static RT_MODEL_Traj2_T Traj2_M_;
RT_MODEL_Traj2_T *const Traj2_M = &Traj2_M_;

/* Forward declaration for local functions */
static real_T Traj2_maximum2(real_T x, real_T y);
static real_T Traj2_minimum2(real_T x, real_T y);
static void Traj2_flip(real_T x[1012]);
static int32_T Traj2_bsearch(const real_T x[1012], real_T xi);
static real_T Traj2_interp1(const real_T varargin_1[1012], const real_T
  varargin_2[1012], real_T varargin_3);
static void Traj2_sin(real_T *x);
static void Traj2_cos(real_T *x);
static real_T Traj2_atan2(real_T y, real_T x);
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (Traj2_M->Timing.TaskCounters.TID[2])++;
  if ((Traj2_M->Timing.TaskCounters.TID[2]) >
      (Traj2_M->Timing.CtrlRateNumTicksToNextHit[0]*1-1)) {/* Sample time: [0.1s, -20.0s] */
    Traj2_M->Timing.TaskCounters.TID[2] = 0;
  }

  (Traj2_M->Timing.TaskCounters.TID[3])++;
  if ((Traj2_M->Timing.TaskCounters.TID[3]) >
      (Traj2_M->Timing.CtrlRateNumTicksToNextHit[1]*1-1)) {/* Sample time: [0.1s, -21.0s] */
    Traj2_M->Timing.TaskCounters.TID[3] = 0;
  }
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 7;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Traj2_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  Traj2_step();
  Traj2_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  Traj2_step();
  Traj2_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T Traj2_maximum2(real_T x, real_T y)
{
  return fmax(x, y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T Traj2_minimum2(real_T x, real_T y)
{
  return fmin(x, y);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static void Traj2_flip(real_T x[1012])
{
  int32_T k;
  for (k = 0; k < 506; k++) {
    real_T tmp;
    tmp = x[k];
    x[k] = x[1011 - k];
    x[1011 - k] = tmp;
  }
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static int32_T Traj2_bsearch(const real_T x[1012], real_T xi)
{
  int32_T high_i;
  int32_T low_ip1;
  int32_T n;
  n = 1;
  low_ip1 = 2;
  high_i = 1012;
  while (high_i > low_ip1) {
    int32_T mid_i;
    mid_i = (n + high_i) >> 1;
    if (xi >= x[mid_i - 1]) {
      n = mid_i;
      low_ip1 = mid_i + 1;
    } else {
      high_i = mid_i;
    }
  }

  return n;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T Traj2_interp1(const real_T varargin_1[1012], const real_T
  varargin_2[1012], real_T varargin_3)
{
  real_T Vq;
  real_T xtmp;
  int32_T i;
  memcpy(&Traj2_B.y_m[0], &varargin_2[0], 1012U * sizeof(real_T));
  memcpy(&Traj2_B.x_c[0], &varargin_1[0], 1012U * sizeof(real_T));
  if (varargin_1[1] < varargin_1[0]) {
    for (i = 0; i < 506; i++) {
      xtmp = Traj2_B.x_c[i];
      Traj2_B.x_c[i] = Traj2_B.x_c[1011 - i];
      Traj2_B.x_c[1011 - i] = xtmp;
    }

    Traj2_flip(Traj2_B.y_m);
  }

  Vq = (rtNaN);
  if ((!rtIsNaN(varargin_3)) && (!(varargin_3 > Traj2_B.x_c[1011])) &&
      (!(varargin_3 < Traj2_B.x_c[0]))) {
    i = Traj2_bsearch(Traj2_B.x_c, varargin_3) - 1;
    xtmp = (varargin_3 - Traj2_B.x_c[i]) / (Traj2_B.x_c[i + 1] - Traj2_B.x_c[i]);
    if (xtmp == 0.0) {
      Vq = Traj2_B.y_m[i];
    } else if (xtmp == 1.0) {
      Vq = Traj2_B.y_m[i + 1];
    } else if (Traj2_B.y_m[i + 1] == Traj2_B.y_m[i]) {
      Vq = Traj2_B.y_m[i];
    } else {
      Vq = (1.0 - xtmp) * Traj2_B.y_m[i] + Traj2_B.y_m[i + 1] * xtmp;
    }
  }

  return Vq;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void Traj2_sin(real_T *x)
{
  *x = sin(*x);
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static void Traj2_cos(real_T *x)
{
  *x = cos(*x);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp_0, tmp);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function1' */
static real_T Traj2_atan2(real_T y, real_T x)
{
  real_T r;
  r = rt_atan2d_snf(y, x);
  return r;
}

/* Model step function */
void Traj2_step(void)
{
  if (rtmIsMajorTimeStep(Traj2_M)) {
    /* set solver stop time */
    if (!(Traj2_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&Traj2_M->solverInfo, ((Traj2_M->Timing.clockTickH0
        + 1) * Traj2_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&Traj2_M->solverInfo, ((Traj2_M->Timing.clockTick0 +
        1) * Traj2_M->Timing.stepSize0 + Traj2_M->Timing.clockTickH0 *
        Traj2_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(Traj2_M)) {
    Traj2_M->Timing.t[0] = rtsiGetT(&Traj2_M->solverInfo);
  }

  {
    real_T tmp[6];
    real_T b_ex_tmp;
    real_T dy;
    real_T ex_tmp;
    real_T t_clamped;
    int32_T i;
    uint32_T numTicksToNextSampleHit;
    boolean_T tmp_0;

    /* Clock: '<Root>/Clock' */
    Traj2_B.Clock = Traj2_M->Timing.t[0];

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Constant: '<Root>/Constant'
     *  Constant: '<Root>/Constant1'
     *  Constant: '<Root>/Constant2'
     *  Constant: '<Root>/Constant3'
     *  Constant: '<Root>/Constant4'
     */
    /* :  t_vec = (0:size(P,1)-1)' * Ts_ref; */
    for (i = 0; i <= 1010; i += 2) {
      Traj2_B.t_vec[i] = i * Traj2_P.Ts_ref;
      Traj2_B.t_vec[i + 1] = (i + 1) * Traj2_P.Ts_ref;
    }

    /* :  t_clamped = min(max(t, t_vec(1)), t_vec(end)); */
    t_clamped = Traj2_minimum2(Traj2_maximum2(Traj2_B.Clock, Traj2_B.t_vec[0]),
      Traj2_B.t_vec[1011]);

    /* :  xd     = interp1(t_vec, P(:,1), t_clamped, 'linear'); */
    Traj2_B.xd = Traj2_interp1(Traj2_B.t_vec, &Traj2_P.P[0], t_clamped);

    /* :  yd     = interp1(t_vec, P(:,2), t_clamped, 'linear'); */
    Traj2_B.yd = Traj2_interp1(Traj2_B.t_vec, &Traj2_P.P[1012], t_clamped);

    /* :  thetad = interp1(t_vec, theta_ref_unwrapped, t_clamped, 'linear'); */
    Traj2_B.thetad = Traj2_interp1(Traj2_B.t_vec, Traj2_P.theta_ref_unwrapped,
      t_clamped);

    /* :  vd     = interp1(t_vec, vd_ref, t_clamped, 'linear'); */
    /* :  wd     = interp1(t_vec, wd_ref, t_clamped, 'linear'); */
    Traj2_B.wd = Traj2_interp1(Traj2_B.t_vec, Traj2_P.wd_ref, t_clamped);

    /* :  vd = max(vd, 0); */
    Traj2_B.vd = Traj2_maximum2(Traj2_interp1(Traj2_B.t_vec, Traj2_P.vd_ref,
      t_clamped), 0.0);

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* Integrator: '<Root>/Integrator' */
    Traj2_B.x = Traj2_X.Integrator_CSTATE;

    /* Integrator: '<Root>/Integrator1' */
    Traj2_B.y = Traj2_X.Integrator1_CSTATE;

    /* Integrator: '<Root>/Integrator2' */
    Traj2_B.theta = Traj2_X.Integrator2_CSTATE;

    /* MATLAB Function: '<Root>/MATLAB Function1' */
    /* :  dx = x - xd; */
    t_clamped = Traj2_B.x - Traj2_B.xd;

    /* :  dy = y - yd; */
    dy = Traj2_B.y - Traj2_B.yd;

    /* :  ex =  cos(thetad)*dx + sin(thetad)*dy; */
    ex_tmp = Traj2_B.thetad;
    Traj2_sin(&ex_tmp);
    b_ex_tmp = Traj2_B.thetad;
    Traj2_cos(&b_ex_tmp);
    Traj2_B.ex = b_ex_tmp * t_clamped + ex_tmp * dy;

    /* :  ey = -sin(thetad)*dx + cos(thetad)*dy; */
    Traj2_B.ey = -ex_tmp * t_clamped + b_ex_tmp * dy;

    /* :  etheta = atan2(sin(theta - thetad), cos(theta - thetad)); */
    t_clamped = Traj2_B.theta - Traj2_B.thetad;
    Traj2_sin(&t_clamped);
    dy = Traj2_B.theta - Traj2_B.thetad;
    Traj2_cos(&dy);
    Traj2_B.etheta = Traj2_atan2(t_clamped, dy);

    /* SignalConversion generated from: '<Root>/Gain2' */
    Traj2_B.e[0] = Traj2_B.ex;
    Traj2_B.e[1] = Traj2_B.ey;
    Traj2_B.e[2] = Traj2_B.etheta;

    /* Gain: '<Root>/Gain2' */
    for (i = 0; i <= 4; i += 2) {
      tmp[i] = -Traj2_P.K[i];
      tmp[i + 1] = -Traj2_P.K[i + 1];
    }

    Traj2_B.Gain2[0] = 0.0;
    Traj2_B.Gain2[1] = 0.0;
    t_clamped = Traj2_B.Gain2[0];
    dy = Traj2_B.Gain2[1];
    for (i = 0; i < 3; i++) {
      t_clamped += tmp[i << 1] * Traj2_B.e[i];
      dy += tmp[(i << 1) + 1] * Traj2_B.e[i];
    }

    Traj2_B.Gain2[1] = dy;
    Traj2_B.Gain2[0] = t_clamped;

    /* End of Gain: '<Root>/Gain2' */

    /* Sum: '<Root>/Sum2' incorporates:
     *  Sum: '<Root>/Sum5'
     */
    /* Sum: '<Root>/Sum5' */
    Traj2_B.Sum5 = Traj2_B.vd + Traj2_B.Gain2[0];

    /* Sum: '<Root>/Sum2' */
    Traj2_B.Sum2 = Traj2_B.wd + Traj2_B.Gain2[1];

    /* Gain: '<Root>/Gain3' */
    t_clamped = Traj2_P.L / 2.0;

    /* Gain: '<Root>/Gain3' */
    Traj2_B.Gain3 = t_clamped * Traj2_B.Sum2;

    /* Sum: '<Root>/Sum6' */
    Traj2_B.vr_ref = Traj2_B.Sum5 + Traj2_B.Gain3;

    /* Sum: '<Root>/Sum8' incorporates:
     *  Inport: '<Root>/EncoderR'
     */
    Traj2_B.Sum8 = Traj2_B.vr_ref - Traj2_U.vr_actual;

    /* Gain: '<S50>/Proportional Gain' */
    Traj2_B.ProportionalGain = Traj2_P.PIDController_P * Traj2_B.Sum8;

    /* Integrator: '<S45>/Integrator' */
    Traj2_B.Integrator = Traj2_X.Integrator_CSTATE_i;

    /* Gain: '<S38>/Derivative Gain' */
    Traj2_B.DerivativeGain = Traj2_P.PIDController_D * Traj2_B.Sum8;

    /* Integrator: '<S40>/Filter' */
    Traj2_B.Filter = Traj2_X.Filter_CSTATE;

    /* Sum: '<S40>/SumD' */
    Traj2_B.SumD = Traj2_B.DerivativeGain - Traj2_B.Filter;

    /* Gain: '<S48>/Filter Coefficient' */
    Traj2_B.FilterCoefficient = Traj2_P.PIDController_N * Traj2_B.SumD;

    /* Sum: '<S54>/Sum' */
    Traj2_B.Sum = (Traj2_B.ProportionalGain + Traj2_B.Integrator) +
      Traj2_B.FilterCoefficient;

    /* Saturate: '<S52>/Saturation' */
    t_clamped = Traj2_B.Sum;
    dy = Traj2_P.PIDController_LowerSaturationLi;
    ex_tmp = Traj2_P.PIDController_UpperSaturationLi;
    if (t_clamped > ex_tmp) {
      /* Saturate: '<S52>/Saturation' */
      Traj2_B.Saturation = ex_tmp;
    } else if (t_clamped < dy) {
      /* Saturate: '<S52>/Saturation' */
      Traj2_B.Saturation = dy;
    } else {
      /* Saturate: '<S52>/Saturation' */
      Traj2_B.Saturation = t_clamped;
    }

    /* End of Saturate: '<S52>/Saturation' */

    /* Product: '<Root>/Divide' incorporates:
     *  Constant: '<Root>/Constant6'
     */
    Traj2_B.Divide = Traj2_B.Saturation / Traj2_P.Constant6_Value;

    /* RelationalOperator: '<S1>/Compare' incorporates:
     *  Constant: '<S1>/Constant'
     */
    Traj2_B.Compare = (Traj2_B.Divide < Traj2_P.Constant_Value);

    /* Outport: '<Root>/dirr' */
    Traj2_Y.dirr = Traj2_B.Compare;

    /* Gain: '<Root>/Gain4' */
    t_clamped = Traj2_P.L / 2.0;

    /* Gain: '<Root>/Gain4' */
    Traj2_B.Gain4 = t_clamped * Traj2_B.Sum2;

    /* Sum: '<Root>/Sum7' */
    Traj2_B.vl_ref = Traj2_B.Sum5 - Traj2_B.Gain4;

    /* Sum: '<Root>/Sum9' incorporates:
     *  Inport: '<Root>/EncoderL'
     */
    Traj2_B.Sum9 = Traj2_B.vl_ref - Traj2_U.vl_actual;

    /* Gain: '<S104>/Proportional Gain' */
    Traj2_B.ProportionalGain_p = Traj2_P.PIDController1_P * Traj2_B.Sum9;

    /* Integrator: '<S99>/Integrator' */
    Traj2_B.Integrator_k = Traj2_X.Integrator_CSTATE_m;

    /* Gain: '<S92>/Derivative Gain' */
    Traj2_B.DerivativeGain_d = Traj2_P.PIDController1_D * Traj2_B.Sum9;

    /* Integrator: '<S94>/Filter' */
    Traj2_B.Filter_e = Traj2_X.Filter_CSTATE_a;

    /* Sum: '<S94>/SumD' */
    Traj2_B.SumD_o = Traj2_B.DerivativeGain_d - Traj2_B.Filter_e;

    /* Gain: '<S102>/Filter Coefficient' */
    Traj2_B.FilterCoefficient_k = Traj2_P.PIDController1_N * Traj2_B.SumD_o;

    /* Sum: '<S108>/Sum' */
    Traj2_B.Sum_d = (Traj2_B.ProportionalGain_p + Traj2_B.Integrator_k) +
      Traj2_B.FilterCoefficient_k;

    /* Saturate: '<S106>/Saturation' */
    t_clamped = Traj2_B.Sum_d;
    dy = Traj2_P.PIDController1_LowerSaturationL;
    ex_tmp = Traj2_P.PIDController1_UpperSaturationL;
    if (t_clamped > ex_tmp) {
      /* Saturate: '<S106>/Saturation' */
      Traj2_B.Saturation_b = ex_tmp;
    } else if (t_clamped < dy) {
      /* Saturate: '<S106>/Saturation' */
      Traj2_B.Saturation_b = dy;
    } else {
      /* Saturate: '<S106>/Saturation' */
      Traj2_B.Saturation_b = t_clamped;
    }

    /* End of Saturate: '<S106>/Saturation' */

    /* Product: '<Root>/Divide1' incorporates:
     *  Constant: '<Root>/Constant5'
     */
    Traj2_B.Divide1 = Traj2_B.Saturation_b / Traj2_P.Constant5_Value;

    /* RelationalOperator: '<S2>/Compare' incorporates:
     *  Constant: '<S2>/Constant'
     */
    Traj2_B.Compare_a = (Traj2_B.Divide1 < Traj2_P.Constant_Value_o);

    /* Outport: '<Root>/dirl' */
    Traj2_Y.dirl = Traj2_B.Compare_a;

    /* Abs: '<Root>/Abs' */
    Traj2_B.Abs = fabs(Traj2_B.Divide);
    tmp_0 = (rtmIsMajorTimeStep(Traj2_M) &&
             Traj2_M->Timing.TaskCounters.TID[1] == 0);
    if (tmp_0) {
      /* Constant: '<S7>/Constant' */
      Traj2_B.Constant = Traj2_P.PWM_Period;
    }

    /* VariablePulseGenerator: '<S7>/Variable Pulse Generator' */
    if (rtmIsMajorTimeStep(Traj2_M) &&
        Traj2_M->Timing.TaskCounters.TID[2] == 0) {
      t_clamped = floor((2.8421709430404007E-14 * Traj2_B.Abs * Traj2_B.Constant
                         + Traj2_B.Abs * Traj2_B.Constant) / 0.1);
      if (rtIsNaN(t_clamped) || rtIsInf(t_clamped)) {
        t_clamped = 0.0;
      } else {
        t_clamped = fmod(t_clamped, 4.294967296E+9);
      }

      if (t_clamped < 0.0) {
        numTicksToNextSampleHit = (uint32_T)-(int32_T)(uint32_T)-t_clamped;
      } else {
        numTicksToNextSampleHit = (uint32_T)t_clamped;
      }

      if (((numTicksToNextSampleHit == 0U) || (Traj2_B.Abs <= 0.0)) &&
          Traj2_DW.isStartOfNextCycle) {
        /* Outport: '<Root>/pwmr' */
        Traj2_Y.pwmr = 0.0;
        Traj2_DW.nextOutput = false;
      } else {
        /* Outport: '<Root>/pwmr' */
        Traj2_Y.pwmr = Traj2_DW.nextOutput;
      }
    }

    /* End of VariablePulseGenerator: '<S7>/Variable Pulse Generator' */

    /* Abs: '<Root>/Abs1' */
    Traj2_B.Abs1 = fabs(Traj2_B.Divide1);
    if (tmp_0) {
      /* Constant: '<S8>/Constant' */
      Traj2_B.Constant_f = Traj2_P.PWM1_Period;
    }

    /* VariablePulseGenerator: '<S8>/Variable Pulse Generator' */
    if (rtmIsMajorTimeStep(Traj2_M) &&
        Traj2_M->Timing.TaskCounters.TID[3] == 0) {
      t_clamped = floor((2.8421709430404007E-14 * Traj2_B.Abs1 *
                         Traj2_B.Constant_f + Traj2_B.Abs1 * Traj2_B.Constant_f)
                        / 0.1);
      if (rtIsNaN(t_clamped) || rtIsInf(t_clamped)) {
        t_clamped = 0.0;
      } else {
        t_clamped = fmod(t_clamped, 4.294967296E+9);
      }

      if (t_clamped < 0.0) {
        numTicksToNextSampleHit = (uint32_T)-(int32_T)(uint32_T)-t_clamped;
      } else {
        numTicksToNextSampleHit = (uint32_T)t_clamped;
      }

      if (((numTicksToNextSampleHit == 0U) || (Traj2_B.Abs1 <= 0.0)) &&
          Traj2_DW.isStartOfNextCycle_d) {
        /* Outport: '<Root>/pwml' */
        Traj2_Y.pwml = 0.0;
        Traj2_DW.nextOutput_p = false;
      } else {
        /* Outport: '<Root>/pwml' */
        Traj2_Y.pwml = Traj2_DW.nextOutput_p;
      }
    }

    /* End of VariablePulseGenerator: '<S8>/Variable Pulse Generator' */
    if (tmp_0) {
      /* ToWorkspace: '<Root>/To Workspace' */
      if (rtmIsMajorTimeStep(Traj2_M)) {
        rt_UpdateLogVar((LogVar *)(LogVar*)
                        (Traj2_DW.ToWorkspace_PWORK.LoggedData), &Traj2_B.y, 0);
      }

      /* ToWorkspace: '<Root>/To Workspace1' */
      if (rtmIsMajorTimeStep(Traj2_M)) {
        rt_UpdateLogVar((LogVar *)(LogVar*)
                        (Traj2_DW.ToWorkspace1_PWORK.LoggedData), &Traj2_B.theta,
                        0);
      }

      /* ToWorkspace: '<Root>/To Workspace2' */
      if (rtmIsMajorTimeStep(Traj2_M)) {
        rt_UpdateLogVar((LogVar *)(LogVar*)
                        (Traj2_DW.ToWorkspace2_PWORK.LoggedData), &Traj2_B.x, 0);
      }
    }

    /* Trigonometry: '<Root>/ ' */
    Traj2_B.u = sin(Traj2_B.theta);

    /* Trigonometry: '<Root>/Cos' */
    Traj2_B.Cos = cos(Traj2_B.theta);

    /* Sum: '<Root>/Sum' incorporates:
     *  Inport: '<Root>/EncoderL'
     *  Inport: '<Root>/EncoderR'
     */
    Traj2_B.Sum_e = Traj2_U.vr_actual + Traj2_U.vl_actual;

    /* Gain: '<Root>/Gain' */
    Traj2_B.Gain = Traj2_P.Gain_Gain * Traj2_B.Sum_e;

    /* Sum: '<Root>/Sum1' incorporates:
     *  Inport: '<Root>/EncoderL'
     *  Inport: '<Root>/EncoderR'
     */
    Traj2_B.Sum1 = Traj2_U.vr_actual - Traj2_U.vl_actual;

    /* Gain: '<Root>/Gain1' */
    t_clamped = 1.0 / Traj2_P.L;

    /* Gain: '<Root>/Gain1' */
    Traj2_B.Gain1 = t_clamped * Traj2_B.Sum1;

    /* Gain: '<S35>/ZeroGain' */
    Traj2_B.ZeroGain = Traj2_P.ZeroGain_Gain * Traj2_B.Sum;

    /* DeadZone: '<S37>/DeadZone' */
    if (Traj2_B.Sum > Traj2_P.PIDController_UpperSaturationLi) {
      /* DeadZone: '<S37>/DeadZone' */
      Traj2_B.DeadZone = Traj2_B.Sum - Traj2_P.PIDController_UpperSaturationLi;
    } else if (Traj2_B.Sum >= Traj2_P.PIDController_LowerSaturationLi) {
      /* DeadZone: '<S37>/DeadZone' */
      Traj2_B.DeadZone = 0.0;
    } else {
      /* DeadZone: '<S37>/DeadZone' */
      Traj2_B.DeadZone = Traj2_B.Sum - Traj2_P.PIDController_LowerSaturationLi;
    }

    /* End of DeadZone: '<S37>/DeadZone' */

    /* RelationalOperator: '<S35>/NotEqual' */
    Traj2_B.NotEqual = (Traj2_B.ZeroGain != Traj2_B.DeadZone);

    /* Signum: '<S35>/SignPreSat' */
    t_clamped = Traj2_B.DeadZone;
    if (rtIsNaN(t_clamped)) {
      /* Signum: '<S35>/SignPreSat' */
      Traj2_B.SignPreSat = (rtNaN);
    } else if (t_clamped < 0.0) {
      /* Signum: '<S35>/SignPreSat' */
      Traj2_B.SignPreSat = -1.0;
    } else {
      /* Signum: '<S35>/SignPreSat' */
      Traj2_B.SignPreSat = (t_clamped > 0.0);
    }

    /* End of Signum: '<S35>/SignPreSat' */

    /* DataTypeConversion: '<S35>/DataTypeConv1' */
    t_clamped = floor(Traj2_B.SignPreSat);
    if (rtIsNaN(t_clamped) || rtIsInf(t_clamped)) {
      t_clamped = 0.0;
    } else {
      t_clamped = fmod(t_clamped, 256.0);
    }

    if (t_clamped < 0.0) {
      /* DataTypeConversion: '<S35>/DataTypeConv1' */
      Traj2_B.DataTypeConv1 = (int8_T)-(int8_T)(uint8_T)-t_clamped;
    } else {
      /* DataTypeConversion: '<S35>/DataTypeConv1' */
      Traj2_B.DataTypeConv1 = (int8_T)(uint8_T)t_clamped;
    }

    /* End of DataTypeConversion: '<S35>/DataTypeConv1' */

    /* Gain: '<S42>/Integral Gain' */
    Traj2_B.IntegralGain = Traj2_P.PIDController_I * Traj2_B.Sum8;

    /* Signum: '<S35>/SignPreIntegrator' */
    t_clamped = Traj2_B.IntegralGain;
    if (rtIsNaN(t_clamped)) {
      /* Signum: '<S35>/SignPreIntegrator' */
      Traj2_B.SignPreIntegrator = (rtNaN);
    } else if (t_clamped < 0.0) {
      /* Signum: '<S35>/SignPreIntegrator' */
      Traj2_B.SignPreIntegrator = -1.0;
    } else {
      /* Signum: '<S35>/SignPreIntegrator' */
      Traj2_B.SignPreIntegrator = (t_clamped > 0.0);
    }

    /* End of Signum: '<S35>/SignPreIntegrator' */

    /* DataTypeConversion: '<S35>/DataTypeConv2' */
    t_clamped = floor(Traj2_B.SignPreIntegrator);
    if (rtIsNaN(t_clamped) || rtIsInf(t_clamped)) {
      t_clamped = 0.0;
    } else {
      t_clamped = fmod(t_clamped, 256.0);
    }

    if (t_clamped < 0.0) {
      /* DataTypeConversion: '<S35>/DataTypeConv2' */
      Traj2_B.DataTypeConv2 = (int8_T)-(int8_T)(uint8_T)-t_clamped;
    } else {
      /* DataTypeConversion: '<S35>/DataTypeConv2' */
      Traj2_B.DataTypeConv2 = (int8_T)(uint8_T)t_clamped;
    }

    /* End of DataTypeConversion: '<S35>/DataTypeConv2' */

    /* RelationalOperator: '<S35>/Equal1' */
    Traj2_B.Equal1 = (Traj2_B.DataTypeConv1 == Traj2_B.DataTypeConv2);

    /* Logic: '<S35>/AND3' */
    Traj2_B.AND3 = (Traj2_B.NotEqual && Traj2_B.Equal1);
    if (tmp_0) {
      /* Memory: '<S35>/Memory' */
      Traj2_B.Memory = Traj2_DW.Memory_PreviousInput;
    }

    /* Switch: '<S35>/Switch' */
    if (Traj2_B.Memory) {
      /* Switch: '<S35>/Switch' incorporates:
       *  Constant: '<S35>/Constant1'
       */
      Traj2_B.Switch = Traj2_P.Constant1_Value;
    } else {
      /* Switch: '<S35>/Switch' */
      Traj2_B.Switch = Traj2_B.IntegralGain;
    }

    /* End of Switch: '<S35>/Switch' */

    /* Gain: '<S89>/ZeroGain' */
    Traj2_B.ZeroGain_g = Traj2_P.ZeroGain_Gain_n * Traj2_B.Sum_d;

    /* DeadZone: '<S91>/DeadZone' */
    if (Traj2_B.Sum_d > Traj2_P.PIDController1_UpperSaturationL) {
      /* DeadZone: '<S91>/DeadZone' */
      Traj2_B.DeadZone_n = Traj2_B.Sum_d -
        Traj2_P.PIDController1_UpperSaturationL;
    } else if (Traj2_B.Sum_d >= Traj2_P.PIDController1_LowerSaturationL) {
      /* DeadZone: '<S91>/DeadZone' */
      Traj2_B.DeadZone_n = 0.0;
    } else {
      /* DeadZone: '<S91>/DeadZone' */
      Traj2_B.DeadZone_n = Traj2_B.Sum_d -
        Traj2_P.PIDController1_LowerSaturationL;
    }

    /* End of DeadZone: '<S91>/DeadZone' */

    /* RelationalOperator: '<S89>/NotEqual' */
    Traj2_B.NotEqual_m = (Traj2_B.ZeroGain_g != Traj2_B.DeadZone_n);

    /* Signum: '<S89>/SignPreSat' */
    t_clamped = Traj2_B.DeadZone_n;
    if (rtIsNaN(t_clamped)) {
      /* Signum: '<S89>/SignPreSat' */
      Traj2_B.SignPreSat_b = (rtNaN);
    } else if (t_clamped < 0.0) {
      /* Signum: '<S89>/SignPreSat' */
      Traj2_B.SignPreSat_b = -1.0;
    } else {
      /* Signum: '<S89>/SignPreSat' */
      Traj2_B.SignPreSat_b = (t_clamped > 0.0);
    }

    /* End of Signum: '<S89>/SignPreSat' */

    /* DataTypeConversion: '<S89>/DataTypeConv1' */
    t_clamped = floor(Traj2_B.SignPreSat_b);
    if (rtIsNaN(t_clamped) || rtIsInf(t_clamped)) {
      t_clamped = 0.0;
    } else {
      t_clamped = fmod(t_clamped, 256.0);
    }

    if (t_clamped < 0.0) {
      /* DataTypeConversion: '<S89>/DataTypeConv1' */
      Traj2_B.DataTypeConv1_c = (int8_T)-(int8_T)(uint8_T)-t_clamped;
    } else {
      /* DataTypeConversion: '<S89>/DataTypeConv1' */
      Traj2_B.DataTypeConv1_c = (int8_T)(uint8_T)t_clamped;
    }

    /* End of DataTypeConversion: '<S89>/DataTypeConv1' */

    /* Gain: '<S96>/Integral Gain' */
    Traj2_B.IntegralGain_m = Traj2_P.PIDController1_I * Traj2_B.Sum9;

    /* Signum: '<S89>/SignPreIntegrator' */
    t_clamped = Traj2_B.IntegralGain_m;
    if (rtIsNaN(t_clamped)) {
      /* Signum: '<S89>/SignPreIntegrator' */
      Traj2_B.SignPreIntegrator_d = (rtNaN);
    } else if (t_clamped < 0.0) {
      /* Signum: '<S89>/SignPreIntegrator' */
      Traj2_B.SignPreIntegrator_d = -1.0;
    } else {
      /* Signum: '<S89>/SignPreIntegrator' */
      Traj2_B.SignPreIntegrator_d = (t_clamped > 0.0);
    }

    /* End of Signum: '<S89>/SignPreIntegrator' */

    /* DataTypeConversion: '<S89>/DataTypeConv2' */
    t_clamped = floor(Traj2_B.SignPreIntegrator_d);
    if (rtIsNaN(t_clamped) || rtIsInf(t_clamped)) {
      t_clamped = 0.0;
    } else {
      t_clamped = fmod(t_clamped, 256.0);
    }

    if (t_clamped < 0.0) {
      /* DataTypeConversion: '<S89>/DataTypeConv2' */
      Traj2_B.DataTypeConv2_h = (int8_T)-(int8_T)(uint8_T)-t_clamped;
    } else {
      /* DataTypeConversion: '<S89>/DataTypeConv2' */
      Traj2_B.DataTypeConv2_h = (int8_T)(uint8_T)t_clamped;
    }

    /* End of DataTypeConversion: '<S89>/DataTypeConv2' */

    /* RelationalOperator: '<S89>/Equal1' */
    Traj2_B.Equal1_k = (Traj2_B.DataTypeConv1_c == Traj2_B.DataTypeConv2_h);

    /* Logic: '<S89>/AND3' */
    Traj2_B.AND3_p = (Traj2_B.NotEqual_m && Traj2_B.Equal1_k);
    if (tmp_0) {
      /* Memory: '<S89>/Memory' */
      Traj2_B.Memory_i = Traj2_DW.Memory_PreviousInput_o;
    }

    /* Switch: '<S89>/Switch' */
    if (Traj2_B.Memory_i) {
      /* Switch: '<S89>/Switch' incorporates:
       *  Constant: '<S89>/Constant1'
       */
      Traj2_B.Switch_n = Traj2_P.Constant1_Value_g;
    } else {
      /* Switch: '<S89>/Switch' */
      Traj2_B.Switch_n = Traj2_B.IntegralGain_m;
    }

    /* End of Switch: '<S89>/Switch' */

    /* Product: '<Root>/Product' */
    Traj2_B.x_dot = Traj2_B.Cos * Traj2_B.Gain;

    /* Product: '<Root>/Product1' */
    Traj2_B.y_dot = Traj2_B.u * Traj2_B.Gain;
  }

  if (rtmIsMajorTimeStep(Traj2_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(Traj2_M->rtwLogInfo, (Traj2_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Traj2_M)) {
    real_T dc;
    uint32_T numTicksToNextSampleHit;
    uint32_T totalTicksInPeriod;

    /* Update for VariablePulseGenerator: '<S7>/Variable Pulse Generator' */
    if (rtmIsMajorTimeStep(Traj2_M) &&
        Traj2_M->Timing.TaskCounters.TID[2] == 0) {
      if (Traj2_DW.nextOutput) {
        dc = Traj2_B.Abs;
        if (dc > 1.0) {
          dc = 1.0;
        } else if (1.0 - Traj2_B.Abs < 2.8421709430404007E-14) {
          dc = 1.0;
        }

        dc = floor((2.2204460492503131E-16 * dc * Traj2_B.Constant + dc *
                    Traj2_B.Constant) / 0.1);
        if (rtIsNaN(dc) || rtIsInf(dc)) {
          dc = 0.0;
        } else {
          dc = fmod(dc, 4.294967296E+9);
        }

        if (dc < 0.0) {
          numTicksToNextSampleHit = (uint32_T)-(int32_T)(uint32_T)-dc;
        } else {
          numTicksToNextSampleHit = (uint32_T)dc;
        }

        dc = floor((2.2204460492503131E-16 * Traj2_B.Constant + Traj2_B.Constant)
                   / 0.1);
        if (rtIsNaN(dc) || rtIsInf(dc)) {
          dc = 0.0;
        } else {
          dc = fmod(dc, 4.294967296E+9);
        }

        if (dc < 0.0) {
          totalTicksInPeriod = (uint32_T)-(int32_T)(uint32_T)-dc;
        } else {
          totalTicksInPeriod = (uint32_T)dc;
        }

        if (numTicksToNextSampleHit < totalTicksInPeriod) {
          Traj2_M->Timing.CtrlRateNumTicksToNextHit[0] = ((real_T)
            numTicksToNextSampleHit);
          Traj2_DW.nextOutput = false;
          Traj2_DW.isStartOfNextCycle = false;
        } else {
          Traj2_M->Timing.CtrlRateNumTicksToNextHit[0] = ((real_T)
            totalTicksInPeriod);
          Traj2_DW.nextOutput = true;
          Traj2_DW.isStartOfNextCycle = true;
        }

        Traj2_DW.startTimeOfNextCycle = totalTicksInPeriod -
          numTicksToNextSampleHit;
      } else {
        if (Traj2_DW.isStartOfNextCycle) {
          dc = floor((2.2204460492503131E-16 * Traj2_B.Constant +
                      Traj2_B.Constant) / 0.1);
          if (rtIsNaN(dc) || rtIsInf(dc)) {
            dc = 0.0;
          } else {
            dc = fmod(dc, 4.294967296E+9);
          }

          if (dc < 0.0) {
            totalTicksInPeriod = (uint32_T)-(int32_T)(uint32_T)-dc;
          } else {
            totalTicksInPeriod = (uint32_T)dc;
          }

          Traj2_M->Timing.CtrlRateNumTicksToNextHit[0] = ((real_T)
            totalTicksInPeriod);
        } else if (Traj2_DW.startTimeOfNextCycle > 0.0) {
          Traj2_M->Timing.CtrlRateNumTicksToNextHit[0] =
            (Traj2_DW.startTimeOfNextCycle);
        }

        Traj2_DW.nextOutput = true;
        Traj2_DW.isStartOfNextCycle = true;
      }
    }

    /* End of Update for VariablePulseGenerator: '<S7>/Variable Pulse Generator' */

    /* Update for VariablePulseGenerator: '<S8>/Variable Pulse Generator' */
    if (rtmIsMajorTimeStep(Traj2_M) &&
        Traj2_M->Timing.TaskCounters.TID[3] == 0) {
      if (Traj2_DW.nextOutput_p) {
        dc = Traj2_B.Abs1;
        if (dc > 1.0) {
          dc = 1.0;
        } else if (1.0 - Traj2_B.Abs1 < 2.8421709430404007E-14) {
          dc = 1.0;
        }

        dc = floor((2.2204460492503131E-16 * dc * Traj2_B.Constant_f + dc *
                    Traj2_B.Constant_f) / 0.1);
        if (rtIsNaN(dc) || rtIsInf(dc)) {
          dc = 0.0;
        } else {
          dc = fmod(dc, 4.294967296E+9);
        }

        if (dc < 0.0) {
          numTicksToNextSampleHit = (uint32_T)-(int32_T)(uint32_T)-dc;
        } else {
          numTicksToNextSampleHit = (uint32_T)dc;
        }

        dc = floor((2.2204460492503131E-16 * Traj2_B.Constant_f +
                    Traj2_B.Constant_f) / 0.1);
        if (rtIsNaN(dc) || rtIsInf(dc)) {
          dc = 0.0;
        } else {
          dc = fmod(dc, 4.294967296E+9);
        }

        if (dc < 0.0) {
          totalTicksInPeriod = (uint32_T)-(int32_T)(uint32_T)-dc;
        } else {
          totalTicksInPeriod = (uint32_T)dc;
        }

        if (numTicksToNextSampleHit < totalTicksInPeriod) {
          Traj2_M->Timing.CtrlRateNumTicksToNextHit[1] = ((real_T)
            numTicksToNextSampleHit);
          Traj2_DW.nextOutput_p = false;
          Traj2_DW.isStartOfNextCycle_d = false;
        } else {
          Traj2_M->Timing.CtrlRateNumTicksToNextHit[1] = ((real_T)
            totalTicksInPeriod);
          Traj2_DW.nextOutput_p = true;
          Traj2_DW.isStartOfNextCycle_d = true;
        }

        Traj2_DW.startTimeOfNextCycle_p = totalTicksInPeriod -
          numTicksToNextSampleHit;
      } else {
        if (Traj2_DW.isStartOfNextCycle_d) {
          dc = floor((2.2204460492503131E-16 * Traj2_B.Constant_f +
                      Traj2_B.Constant_f) / 0.1);
          if (rtIsNaN(dc) || rtIsInf(dc)) {
            dc = 0.0;
          } else {
            dc = fmod(dc, 4.294967296E+9);
          }

          if (dc < 0.0) {
            totalTicksInPeriod = (uint32_T)-(int32_T)(uint32_T)-dc;
          } else {
            totalTicksInPeriod = (uint32_T)dc;
          }

          Traj2_M->Timing.CtrlRateNumTicksToNextHit[1] = ((real_T)
            totalTicksInPeriod);
        } else if (Traj2_DW.startTimeOfNextCycle_p > 0.0) {
          Traj2_M->Timing.CtrlRateNumTicksToNextHit[1] =
            (Traj2_DW.startTimeOfNextCycle_p);
        }

        Traj2_DW.nextOutput_p = true;
        Traj2_DW.isStartOfNextCycle_d = true;
      }
    }

    /* End of Update for VariablePulseGenerator: '<S8>/Variable Pulse Generator' */
    if (rtmIsMajorTimeStep(Traj2_M) &&
        Traj2_M->Timing.TaskCounters.TID[1] == 0) {
      /* Update for Memory: '<S35>/Memory' */
      Traj2_DW.Memory_PreviousInput = Traj2_B.AND3;

      /* Update for Memory: '<S89>/Memory' */
      Traj2_DW.Memory_PreviousInput_o = Traj2_B.AND3_p;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Traj2_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(Traj2_M)!=-1) &&
          !((rtmGetTFinal(Traj2_M)-(((Traj2_M->Timing.clockTick1+
               Traj2_M->Timing.clockTickH1* 4294967296.0)) * 0.1)) >
            (((Traj2_M->Timing.clockTick1+Traj2_M->Timing.clockTickH1*
               4294967296.0)) * 0.1) * (DBL_EPSILON))) {
        rtmSetErrorStatus(Traj2_M, "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&Traj2_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++Traj2_M->Timing.clockTick0)) {
      ++Traj2_M->Timing.clockTickH0;
    }

    Traj2_M->Timing.t[0] = rtsiGetSolverStopTime(&Traj2_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.1s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.1, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      Traj2_M->Timing.clockTick1++;
      if (!Traj2_M->Timing.clockTick1) {
        Traj2_M->Timing.clockTickH1++;
      }
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Traj2_derivatives(void)
{
  XDot_Traj2_T *_rtXdot;
  _rtXdot = ((XDot_Traj2_T *) Traj2_M->derivs);

  /* Derivatives for Integrator: '<Root>/Integrator' */
  _rtXdot->Integrator_CSTATE = Traj2_B.x_dot;

  /* Derivatives for Integrator: '<Root>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = Traj2_B.y_dot;

  /* Derivatives for Integrator: '<Root>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = Traj2_B.Gain1;

  /* Derivatives for Integrator: '<S45>/Integrator' */
  _rtXdot->Integrator_CSTATE_i = Traj2_B.Switch;

  /* Derivatives for Integrator: '<S40>/Filter' */
  _rtXdot->Filter_CSTATE = Traj2_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S99>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = Traj2_B.Switch_n;

  /* Derivatives for Integrator: '<S94>/Filter' */
  _rtXdot->Filter_CSTATE_a = Traj2_B.FilterCoefficient_k;
}

/* Model initialize function */
void Traj2_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Traj2_M, 0,
                sizeof(RT_MODEL_Traj2_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&Traj2_M->solverInfo, &Traj2_M->Timing.simTimeStep);
    rtsiSetTPtr(&Traj2_M->solverInfo, &rtmGetTPtr(Traj2_M));
    rtsiSetStepSizePtr(&Traj2_M->solverInfo, &Traj2_M->Timing.stepSize0);
    rtsiSetdXPtr(&Traj2_M->solverInfo, &Traj2_M->derivs);
    rtsiSetContStatesPtr(&Traj2_M->solverInfo, (real_T **) &Traj2_M->contStates);
    rtsiSetNumContStatesPtr(&Traj2_M->solverInfo, &Traj2_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&Traj2_M->solverInfo,
      &Traj2_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&Traj2_M->solverInfo,
      &Traj2_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&Traj2_M->solverInfo,
      &Traj2_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&Traj2_M->solverInfo, (boolean_T**)
      &Traj2_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&Traj2_M->solverInfo, (&rtmGetErrorStatus(Traj2_M)));
    rtsiSetRTModelPtr(&Traj2_M->solverInfo, Traj2_M);
  }

  rtsiSetSimTimeStep(&Traj2_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&Traj2_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&Traj2_M->solverInfo, false);
  Traj2_M->intgData.y = Traj2_M->odeY;
  Traj2_M->intgData.f[0] = Traj2_M->odeF[0];
  Traj2_M->intgData.f[1] = Traj2_M->odeF[1];
  Traj2_M->intgData.f[2] = Traj2_M->odeF[2];
  Traj2_M->contStates = ((X_Traj2_T *) &Traj2_X);
  Traj2_M->contStateDisabled = ((XDis_Traj2_T *) &Traj2_XDis);
  Traj2_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&Traj2_M->solverInfo, (void *)&Traj2_M->intgData);
  rtsiSetSolverName(&Traj2_M->solverInfo,"ode3");
  rtmSetTPtr(Traj2_M, &Traj2_M->Timing.tArray[0]);
  rtmSetTFinal(Traj2_M, -1);
  Traj2_M->Timing.stepSize0 = 0.1;
  Traj2_M->Timing.CtrlRateNumTicksToNextHit[0] = 1;
  Traj2_M->Timing.CtrlRateNumTicksToNextHit[1] = 1;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Traj2_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Traj2_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Traj2_M->rtwLogInfo, (NULL));
    rtliSetLogT(Traj2_M->rtwLogInfo, "tout");
    rtliSetLogX(Traj2_M->rtwLogInfo, "");
    rtliSetLogXFinal(Traj2_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Traj2_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Traj2_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Traj2_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Traj2_M->rtwLogInfo, 1);
    rtliSetLogY(Traj2_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Traj2_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Traj2_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &Traj2_B), 0,
                sizeof(B_Traj2_T));

  /* states (continuous) */
  {
    (void) memset((void *)&Traj2_X, 0,
                  sizeof(X_Traj2_T));
  }

  /* disabled states */
  {
    (void) memset((void *)&Traj2_XDis, 0,
                  sizeof(XDis_Traj2_T));
  }

  /* states (dwork) */
  (void) memset((void *)&Traj2_DW, 0,
                sizeof(DW_Traj2_T));

  /* external inputs */
  (void)memset(&Traj2_U, 0, sizeof(ExtU_Traj2_T));

  /* external outputs */
  (void)memset(&Traj2_Y, 0, sizeof(ExtY_Traj2_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Traj2_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Traj2_M), Traj2_M->Timing.stepSize0, (&rtmGetErrorStatus(Traj2_M)));

  /* SetupRuntimeResources for ToWorkspace: '<Root>/To Workspace' */
  {
    int_T dimensions[1] = { 1 };

    Traj2_DW.ToWorkspace_PWORK.LoggedData = rt_CreateLogVar(
      Traj2_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(Traj2_M),
      Traj2_M->Timing.stepSize0,
      (&rtmGetErrorStatus(Traj2_M)),
      "y_out",
      SS_DOUBLE,
      0,
      0,
      0,
      1,
      1,
      dimensions,
      NO_LOGVALDIMS,
      (NULL),
      (NULL),
      0,
      1,
      0.1,
      1);
    if (Traj2_DW.ToWorkspace_PWORK.LoggedData == (NULL))
      return;
  }

  /* SetupRuntimeResources for ToWorkspace: '<Root>/To Workspace1' */
  {
    int_T dimensions[1] = { 1 };

    Traj2_DW.ToWorkspace1_PWORK.LoggedData = rt_CreateLogVar(
      Traj2_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(Traj2_M),
      Traj2_M->Timing.stepSize0,
      (&rtmGetErrorStatus(Traj2_M)),
      "theta_out",
      SS_DOUBLE,
      0,
      0,
      0,
      1,
      1,
      dimensions,
      NO_LOGVALDIMS,
      (NULL),
      (NULL),
      0,
      1,
      0.1,
      1);
    if (Traj2_DW.ToWorkspace1_PWORK.LoggedData == (NULL))
      return;
  }

  /* SetupRuntimeResources for ToWorkspace: '<Root>/To Workspace2' */
  {
    int_T dimensions[1] = { 1 };

    Traj2_DW.ToWorkspace2_PWORK.LoggedData = rt_CreateLogVar(
      Traj2_M->rtwLogInfo,
      0.0,
      rtmGetTFinal(Traj2_M),
      Traj2_M->Timing.stepSize0,
      (&rtmGetErrorStatus(Traj2_M)),
      "x_out",
      SS_DOUBLE,
      0,
      0,
      0,
      1,
      1,
      dimensions,
      NO_LOGVALDIMS,
      (NULL),
      (NULL),
      0,
      1,
      0.1,
      1);
    if (Traj2_DW.ToWorkspace2_PWORK.LoggedData == (NULL))
      return;
  }

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  Traj2_X.Integrator_CSTATE = Traj2_P.x0;

  /* InitializeConditions for Integrator: '<Root>/Integrator1' */
  Traj2_X.Integrator1_CSTATE = Traj2_P.y0;

  /* InitializeConditions for Integrator: '<Root>/Integrator2' */
  Traj2_X.Integrator2_CSTATE = Traj2_P.theta0;

  /* InitializeConditions for Integrator: '<S45>/Integrator' */
  Traj2_X.Integrator_CSTATE_i = Traj2_P.PIDController_InitialConditio_g;

  /* InitializeConditions for Integrator: '<S40>/Filter' */
  Traj2_X.Filter_CSTATE = Traj2_P.PIDController_InitialConditionF;

  /* InitializeConditions for Integrator: '<S99>/Integrator' */
  Traj2_X.Integrator_CSTATE_m = Traj2_P.PIDController1_InitialConditi_c;

  /* InitializeConditions for Integrator: '<S94>/Filter' */
  Traj2_X.Filter_CSTATE_a = Traj2_P.PIDController1_InitialCondition;

  /* InitializeConditions for VariablePulseGenerator: '<S7>/Variable Pulse Generator' */
  Traj2_DW.startTimeOfNextCycle = 0.0;
  Traj2_DW.nextOutput = true;
  Traj2_DW.isStartOfNextCycle = true;
  Traj2_DW.isFirstWarningDCGreaterThanOne = true;
  Traj2_DW.isFirstWarningDCLessThanZero = true;

  /* InitializeConditions for VariablePulseGenerator: '<S8>/Variable Pulse Generator' */
  Traj2_DW.startTimeOfNextCycle_p = 0.0;
  Traj2_DW.nextOutput_p = true;
  Traj2_DW.isStartOfNextCycle_d = true;
  Traj2_DW.isFirstWarningDCGreaterThanOn_k = true;
  Traj2_DW.isFirstWarningDCLessThanZero_f = true;

  /* InitializeConditions for Memory: '<S35>/Memory' */
  Traj2_DW.Memory_PreviousInput = Traj2_P.Memory_InitialCondition;

  /* InitializeConditions for Memory: '<S89>/Memory' */
  Traj2_DW.Memory_PreviousInput_o = Traj2_P.Memory_InitialCondition_f;

  /* Enable for VariablePulseGenerator: '<S7>/Variable Pulse Generator' */
  Traj2_M->Timing.CtrlRateNumTicksToNextHit[0] = (1.0);

  /* Enable for VariablePulseGenerator: '<S8>/Variable Pulse Generator' */
  Traj2_M->Timing.CtrlRateNumTicksToNextHit[1] = (1.0);
}

/* Model terminate function */
void Traj2_terminate(void)
{
  /* (no terminate code required) */
}
