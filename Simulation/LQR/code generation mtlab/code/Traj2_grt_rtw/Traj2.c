/*
 * Traj2.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Traj2".
 *
 * Model version              : 3.3
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C source code generated on : Sun Apr 19 12:22:50 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "Traj2.h"
#include "rtwtypes.h"
#include <emmintrin.h>
#include <math.h>
#include "Traj2_private.h"
#include <string.h>
#include "rt_nonfinite.h"
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
static real_T Traj2_interp1(const real_T varargin_1[1276], const real_T
  varargin_2[1276], real_T varargin_3);

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
  int_T nXc = 9;
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

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/MATLAB Function' */
static real_T Traj2_interp1(const real_T varargin_1[1276], const real_T
  varargin_2[1276], real_T varargin_3)
{
  real_T Vq;
  real_T xtmp;
  int32_T i;
  memcpy(&Traj2_B.y_m[0], &varargin_2[0], 1276U * sizeof(real_T));
  memcpy(&Traj2_B.x_c[0], &varargin_1[0], 1276U * sizeof(real_T));
  if (varargin_1[1] < varargin_1[0]) {
    for (i = 0; i < 638; i++) {
      xtmp = Traj2_B.x_c[i];
      Traj2_B.x_c[i] = Traj2_B.x_c[1275 - i];
      Traj2_B.x_c[1275 - i] = xtmp;
      xtmp = Traj2_B.y_m[i];
      Traj2_B.y_m[i] = Traj2_B.y_m[1275 - i];
      Traj2_B.y_m[1275 - i] = xtmp;
    }
  }

  Vq = (rtNaN);
  if ((!rtIsNaN(varargin_3)) && (!(varargin_3 > Traj2_B.x_c[1275])) &&
      (!(varargin_3 < Traj2_B.x_c[0]))) {
    int32_T high_i;
    int32_T low_ip1;
    i = 1;
    low_ip1 = 2;
    high_i = 1276;
    while (high_i > low_ip1) {
      int32_T mid_i;
      mid_i = (i + high_i) >> 1;
      if (varargin_3 >= Traj2_B.x_c[mid_i - 1]) {
        i = mid_i;
        low_ip1 = mid_i + 1;
      } else {
        high_i = mid_i;
      }
    }

    xtmp = Traj2_B.x_c[i - 1];
    xtmp = (varargin_3 - xtmp) / (Traj2_B.x_c[i] - xtmp);
    if (xtmp == 0.0) {
      Vq = Traj2_B.y_m[i - 1];
    } else if (xtmp == 1.0) {
      Vq = Traj2_B.y_m[i];
    } else if (Traj2_B.y_m[i - 1] == Traj2_B.y_m[i]) {
      Vq = Traj2_B.y_m[i - 1];
    } else {
      Vq = (1.0 - xtmp) * Traj2_B.y_m[i - 1] + xtmp * Traj2_B.y_m[i];
    }
  }

  return Vq;
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

/* Model step function */
void Traj2_step(void)
{
  __m128d tmp_0;
  real_T tmp[2];
  real_T tmp_1[2];
  real_T dy;
  real_T riseValLimit;
  real_T tau;
  int32_T i;
  boolean_T limitedCache;
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
  for (i = 0; i <= 1274; i += 2) {
    tmp[0] = i;
    tmp[1] = i + 1;
    tmp_0 = _mm_loadu_pd(&tmp[0]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.05));
    _mm_storeu_pd(&Traj2_B.t_vec[i], tmp_0);
  }

  /* :  T_hold = 1; */
  /* :  T_soft = 3; */
  /* :  beta0  = 0.25; */
  /* :  if t <= T_hold */
  if (Traj2_B.Clock <= 1.0) {
    /* :  t_eff = 0; */
    tau = 0.0;
  } else if (Traj2_B.Clock <= 4.0) {
    /* :  elseif t <= (T_hold + T_soft) */
    /* :  dt  = t - T_hold; */
    /* :  tau = dt / T_soft; */
    tau = (Traj2_B.Clock - 1.0) / 3.0;

    /* :  t_eff = beta0*dt + (1 - beta0)*T_soft*(tau^3 - 0.5*tau^4); */
    tau = (rt_powd_snf(tau, 3.0) - 0.5 * rt_powd_snf(tau, 4.0)) * 2.25 +
      (Traj2_B.Clock - 1.0) * 0.25;
  } else {
    /* :  else */
    /* :  t_soft_end = T_soft * (0.5 + 0.5*beta0); */
    /* :  t_eff = t_soft_end + (t - T_hold - T_soft); */
    tau = ((Traj2_B.Clock - 1.0) - 3.0) + 1.875;
  }

  /* :  t_clamped = min(max(t_eff, 0), t_vec(end)); */
  tau = fmin(fmax(tau, 0.0), Traj2_B.t_vec[1275]);

  /* :  xd     = interp1(t_vec, P(:,1), t_clamped, 'linear'); */
  Traj2_B.xd = Traj2_interp1(Traj2_B.t_vec, &Traj2_ConstP.Constant_Value[0], tau);

  /* :  yd     = interp1(t_vec, P(:,2), t_clamped, 'linear'); */
  Traj2_B.yd = Traj2_interp1(Traj2_B.t_vec, &Traj2_ConstP.Constant_Value[1276],
    tau);

  /* :  thetad = interp1(t_vec, theta_ref_unwrapped, t_clamped, 'linear'); */
  Traj2_B.thetad = Traj2_interp1(Traj2_B.t_vec, Traj2_ConstP.Constant1_Value,
    tau);

  /* :  vd     = interp1(t_vec, vd_ref, t_clamped, 'linear'); */
  /* :  wd     = interp1(t_vec, wd_ref, t_clamped, 'linear'); */
  Traj2_B.wd = Traj2_interp1(Traj2_B.t_vec, Traj2_ConstP.Constant3_Value, tau);

  /* :  vd = max(vd, 0); */
  Traj2_B.vd = fmax(Traj2_interp1(Traj2_B.t_vec, Traj2_ConstP.Constant2_Value,
    tau), 0.0);

  /* End of MATLAB Function: '<Root>/MATLAB Function' */

  /* Integrator: '<Root>/Integrator' */
  Traj2_B.x = Traj2_X.Integrator_CSTATE;

  /* Integrator: '<Root>/Integrator1' */
  Traj2_B.y = Traj2_X.Integrator1_CSTATE;

  /* Integrator: '<Root>/Integrator2' */
  Traj2_B.theta = Traj2_X.Integrator2_CSTATE;

  /* MATLAB Function: '<Root>/MATLAB Function1' */
  /* :  dx = x - xd; */
  tau = Traj2_B.x - Traj2_B.xd;

  /* :  dy = y - yd; */
  dy = Traj2_B.y - Traj2_B.yd;

  /* :  ex =  cos(thetad)*dx + sin(thetad)*dy; */
  Traj2_B.ex = cos(Traj2_B.thetad) * tau + sin(Traj2_B.thetad) * dy;

  /* :  ey = -sin(thetad)*dx + cos(thetad)*dy; */
  Traj2_B.ey = -sin(Traj2_B.thetad) * tau + cos(Traj2_B.thetad) * dy;

  /* :  etheta = atan2(sin(theta - thetad), cos(theta - thetad)); */
  Traj2_B.etheta = rt_atan2d_snf(sin(Traj2_B.theta - Traj2_B.thetad), cos
    (Traj2_B.theta - Traj2_B.thetad));

  /* SignalConversion generated from: '<Root>/Gain2' */
  Traj2_B.e[0] = Traj2_B.ex;
  Traj2_B.e[1] = Traj2_B.ey;
  Traj2_B.e[2] = Traj2_B.etheta;

  /* Gain: '<Root>/Gain2' */
  Traj2_B.Gain2[0] = 0.0;
  Traj2_B.Gain2[1] = 0.0;
  tau = Traj2_B.Gain2[0];
  dy = Traj2_B.Gain2[1];
  for (i = 0; i < 3; i++) {
    tmp_0 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&Traj2_ConstP.Gain2_Gain[i << 1]),
      _mm_set1_pd(Traj2_B.e[i])), _mm_set_pd(dy, tau));
    _mm_storeu_pd(&tmp_1[0], tmp_0);
    tau = tmp_1[0];
    dy = tmp_1[1];
  }

  Traj2_B.Gain2[1] = dy;
  Traj2_B.Gain2[0] = tau;

  /* End of Gain: '<Root>/Gain2' */

  /* Sum: '<Root>/Sum5' */
  Traj2_B.Sum5 = Traj2_B.vd + Traj2_B.Gain2[0];

  /* RateLimiter: '<Root>/Rate Limiter' */
  if (Traj2_DW.LastMajorTime == (rtInf)) {
    /* RateLimiter: '<Root>/Rate Limiter' */
    Traj2_B.RateLimiter = Traj2_B.Sum5;
  } else {
    dy = Traj2_M->Timing.t[0];
    tau = dy - Traj2_DW.LastMajorTime;
    if (Traj2_DW.LastMajorTime == dy) {
      if (Traj2_DW.PrevLimited) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Traj2_B.RateLimiter = Traj2_DW.PrevY;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Traj2_B.RateLimiter = Traj2_B.Sum5;
      }
    } else {
      riseValLimit = tau * 0.1;
      dy = Traj2_B.Sum5 - Traj2_DW.PrevY;
      if (dy > riseValLimit) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        Traj2_B.RateLimiter = Traj2_DW.PrevY + riseValLimit;
        limitedCache = true;
      } else {
        tau *= -0.35;
        if (dy < tau) {
          /* RateLimiter: '<Root>/Rate Limiter' */
          Traj2_B.RateLimiter = Traj2_DW.PrevY + tau;
          limitedCache = true;
        } else {
          /* RateLimiter: '<Root>/Rate Limiter' */
          Traj2_B.RateLimiter = Traj2_B.Sum5;
          limitedCache = false;
        }
      }

      if (rtsiIsModeUpdateTimeStep(&Traj2_M->solverInfo)) {
        Traj2_DW.PrevLimited = limitedCache;
      }
    }
  }

  /* End of RateLimiter: '<Root>/Rate Limiter' */

  /* Saturate: '<Root>/Saturation' */
  tau = Traj2_B.RateLimiter;
  if (tau > 0.3) {
    /* Saturate: '<Root>/Saturation' */
    Traj2_B.Saturation = 0.3;
  } else if (tau < 0.0) {
    /* Saturate: '<Root>/Saturation' */
    Traj2_B.Saturation = 0.0;
  } else {
    /* Saturate: '<Root>/Saturation' */
    Traj2_B.Saturation = tau;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Sum: '<Root>/Sum2' */
  Traj2_B.Sum2 = Traj2_B.wd + Traj2_B.Gain2[1];

  /* Gain: '<Root>/Gain3' */
  Traj2_B.Gain3 = 0.2 * Traj2_B.Sum2;

  /* Sum: '<Root>/Sum6' */
  Traj2_B.vr_ref = Traj2_B.Saturation + Traj2_B.Gain3;

  /* Sum: '<Root>/Sum8' incorporates:
   *  Inport: '<Root>/In1'
   */
  Traj2_B.Sum8 = Traj2_B.vr_ref - Traj2_U.vr_actual;

  /* Gain: '<S44>/Proportional Gain' */
  Traj2_B.ProportionalGain = 4.0 * Traj2_B.Sum8;

  /* Integrator: '<S39>/Integrator' */
  Traj2_B.Integrator = Traj2_X.Integrator_CSTATE_i;

  /* Gain: '<S32>/Derivative Gain' */
  Traj2_B.DerivativeGain = 0.0 * Traj2_B.Sum8;

  /* Integrator: '<S34>/Filter' */
  Traj2_B.Filter = Traj2_X.Filter_CSTATE;

  /* Sum: '<S34>/SumD' */
  Traj2_B.SumD = Traj2_B.DerivativeGain - Traj2_B.Filter;

  /* Gain: '<S42>/Filter Coefficient' */
  Traj2_B.FilterCoefficient = 100.0 * Traj2_B.SumD;

  /* Sum: '<S48>/Sum' */
  Traj2_B.Sum = (Traj2_B.ProportionalGain + Traj2_B.Integrator) +
    Traj2_B.FilterCoefficient;

  /* Outport: '<Root>/Out1' */
  Traj2_Y.Out1 = Traj2_B.Sum;

  /* Gain: '<Root>/Gain4' */
  Traj2_B.Gain4 = 0.2 * Traj2_B.Sum2;

  /* Sum: '<Root>/Sum7' */
  Traj2_B.vl_ref = Traj2_B.Saturation - Traj2_B.Gain4;

  /* Sum: '<Root>/Sum9' incorporates:
   *  Inport: '<Root>/In2'
   */
  Traj2_B.Sum9 = Traj2_B.vl_ref - Traj2_U.vl_actual;

  /* Gain: '<S96>/Proportional Gain' */
  Traj2_B.ProportionalGain_p = 4.0 * Traj2_B.Sum9;

  /* Integrator: '<S91>/Integrator' */
  Traj2_B.Integrator_k = Traj2_X.Integrator_CSTATE_m;

  /* Gain: '<S84>/Derivative Gain' */
  Traj2_B.DerivativeGain_d = 0.0 * Traj2_B.Sum9;

  /* Integrator: '<S86>/Filter' */
  Traj2_B.Filter_e = Traj2_X.Filter_CSTATE_a;

  /* Sum: '<S86>/SumD' */
  Traj2_B.SumD_o = Traj2_B.DerivativeGain_d - Traj2_B.Filter_e;

  /* Gain: '<S94>/Filter Coefficient' */
  Traj2_B.FilterCoefficient_k = 100.0 * Traj2_B.SumD_o;

  /* Sum: '<S100>/Sum' */
  Traj2_B.Sum_d = (Traj2_B.ProportionalGain_p + Traj2_B.Integrator_k) +
    Traj2_B.FilterCoefficient_k;

  /* Outport: '<Root>/Out2' */
  Traj2_Y.Out2 = Traj2_B.Sum_d;
  limitedCache = rtmIsMajorTimeStep(Traj2_M);
  if (limitedCache) {
  }

  /* Integrator: '<Root>/Integrator3' */
  Traj2_B.Integrator3[0] = Traj2_X.Integrator3_CSTATE[0];
  Traj2_B.Integrator3[1] = Traj2_X.Integrator3_CSTATE[1];
  if (limitedCache) {
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   */
  Traj2_B.Sum_e = Traj2_U.vr_actual + Traj2_U.vl_actual;

  /* Gain: '<Root>/Gain' */
  Traj2_B.v_actual = 0.5 * Traj2_B.Sum_e;
  if (limitedCache) {
  }

  /* Sum: '<Root>/Sum1' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   */
  Traj2_B.Sum1 = Traj2_U.vr_actual - Traj2_U.vl_actual;

  /* Gain: '<Root>/Gain1' */
  Traj2_B.Gain1 = 2.5 * Traj2_B.Sum1;
  if (limitedCache) {
    /* ToWorkspace: '<Root>/To Workspace' */
    if (rtmIsMajorTimeStep(Traj2_M)) {
      rt_UpdateLogVar((LogVar *)(LogVar*) (Traj2_DW.ToWorkspace_PWORK.LoggedData),
                      &Traj2_B.y, 0);
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

  /* Gain: '<S36>/Integral Gain' */
  Traj2_B.IntegralGain = 10.0 * Traj2_B.Sum8;

  /* Gain: '<S88>/Integral Gain' */
  Traj2_B.IntegralGain_m = 10.0 * Traj2_B.Sum9;

  /* Product: '<Root>/Product' */
  Traj2_B.x_dot = Traj2_B.Cos * Traj2_B.v_actual;

  /* Product: '<Root>/Product1' */
  Traj2_B.y_dot = Traj2_B.u * Traj2_B.v_actual;
  if (rtmIsMajorTimeStep(Traj2_M)) {
    /* Matfile logging */
    rt_UpdateTXYLogVars(Traj2_M->rtwLogInfo, (Traj2_M->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Traj2_M)) {
    /* Update for RateLimiter: '<Root>/Rate Limiter' */
    Traj2_DW.PrevY = Traj2_B.RateLimiter;
    Traj2_DW.LastMajorTime = Traj2_M->Timing.t[0];
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(Traj2_M)) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal(Traj2_M)!=-1) &&
          !((rtmGetTFinal(Traj2_M)-(((Traj2_M->Timing.clockTick1+
               Traj2_M->Timing.clockTickH1* 4294967296.0)) * 1.3830000000000002))
            > (((Traj2_M->Timing.clockTick1+Traj2_M->Timing.clockTickH1*
                 4294967296.0)) * 1.3830000000000002) * (DBL_EPSILON))) {
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
      /* Update absolute timer for sample time: [1.3830000000000002s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 1.3830000000000002, which is the step size
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

  /* Derivatives for Integrator: '<S39>/Integrator' */
  _rtXdot->Integrator_CSTATE_i = Traj2_B.IntegralGain;

  /* Derivatives for Integrator: '<S34>/Filter' */
  _rtXdot->Filter_CSTATE = Traj2_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S91>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = Traj2_B.IntegralGain_m;

  /* Derivatives for Integrator: '<S86>/Filter' */
  _rtXdot->Filter_CSTATE_a = Traj2_B.FilterCoefficient_k;

  /* Derivatives for Integrator: '<Root>/Integrator3' */
  _rtXdot->Integrator3_CSTATE[0] = Traj2_B.Gain2[0];
  _rtXdot->Integrator3_CSTATE[1] = Traj2_B.Gain2[1];
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
  rtmSetTFinal(Traj2_M, 69.15);
  Traj2_M->Timing.stepSize0 = 1.3830000000000002;

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
      1.3830000000000002,
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
      1.3830000000000002,
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
      1.3830000000000002,
      1);
    if (Traj2_DW.ToWorkspace2_PWORK.LoggedData == (NULL))
      return;
  }

  /* InitializeConditions for Integrator: '<Root>/Integrator' */
  Traj2_X.Integrator_CSTATE = 9.0;

  /* InitializeConditions for Integrator: '<Root>/Integrator1' */
  Traj2_X.Integrator1_CSTATE = 1.0;

  /* InitializeConditions for Integrator: '<Root>/Integrator2' */
  Traj2_X.Integrator2_CSTATE = 1.3288976009265088;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  Traj2_DW.LastMajorTime = (rtInf);

  /* InitializeConditions for Integrator: '<S39>/Integrator' */
  Traj2_X.Integrator_CSTATE_i = 0.0;

  /* InitializeConditions for Integrator: '<S34>/Filter' */
  Traj2_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S91>/Integrator' */
  Traj2_X.Integrator_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S86>/Filter' */
  Traj2_X.Filter_CSTATE_a = 0.0;

  /* InitializeConditions for Integrator: '<Root>/Integrator3' */
  Traj2_X.Integrator3_CSTATE[0] = 0.0;
  Traj2_X.Integrator3_CSTATE[1] = 0.0;
}

/* Model terminate function */
void Traj2_terminate(void)
{
  /* (no terminate code required) */
}
