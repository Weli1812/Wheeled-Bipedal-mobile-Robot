/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Pure_Pursuit.c
 *
 * Code generated for Simulink model 'Pure_Pursuit'.
 *
 * Model version                  : 1.12
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Mon Mar 16 15:35:27 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Pure_Pursuit.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>
#include "math.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Continuous states */
X rtX;

/* Disabled State Vector */
XDis rtXDis;

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_atan2d_snf(real_T u0, real_T u1);

/* private model entry point functions */
extern void Pure_Pursuit_derivatives(void);

/* Forward declaration for local functions */
static real_T norm(const real_T x[2]);
static real_T closestPointOnLine(const real_T pt1[2], real_T pt2[2], const
  real_T refPt[2]);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)isinf(value);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
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
  int_T nXc = 10;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  Pure_Pursuit_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  Pure_Pursuit_step();
  Pure_Pursuit_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  Pure_Pursuit_step();
  Pure_Pursuit_derivatives();

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

static real_T norm(const real_T x[2])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

static real_T closestPointOnLine(const real_T pt1[2], real_T pt2[2], const
  real_T refPt[2])
{
  real_T refPt_0[2];
  real_T alpha;
  real_T distance;
  real_T v12;
  real_T v12_0;
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;

  /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(pt1[b_k] == pt2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  if (p) {
    pt2[0] = pt1[0];
    refPt_0[0] = refPt[0] - pt1[0];
    pt2[1] = pt1[1];
    refPt_0[1] = refPt[1] - pt1[1];
    distance = norm(refPt_0);
  } else {
    alpha = pt2[0] - pt1[0];
    v12 = (pt2[0] - refPt[0]) * alpha;
    v12_0 = alpha * alpha;
    alpha = pt2[1] - pt1[1];
    alpha = ((pt2[1] - refPt[1]) * alpha + v12) / (alpha * alpha + v12_0);
    if (alpha > 1.0) {
      pt2[0] = pt1[0];
      pt2[1] = pt1[1];
    } else if (!(alpha < 0.0)) {
      pt2[0] = (1.0 - alpha) * pt2[0] + alpha * pt1[0];
      pt2[1] = (1.0 - alpha) * pt2[1] + alpha * pt1[1];
    }

    refPt_0[0] = refPt[0] - pt2[0];
    refPt_0[1] = refPt[1] - pt2[1];
    distance = norm(refPt_0);
  }

  /* End of Start for MATLABSystem: '<Root>/Pure Pursuit' */
  return distance;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
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
void Pure_Pursuit_step(void)
{
  real_T rtb_TmpSignalConversionAtPurePu[3];
  real_T lookaheadStartPt[2];
  real_T lookaheadStartPt_0[2];
  real_T tmp_0[2];
  real_T dist;
  real_T lookaheadEndPt_idx_0;
  real_T lookaheadEndPt_idx_1;
  real_T lookaheadIdx;
  real_T minDistance;
  real_T tmp_1;
  int32_T b_i;
  int32_T b_k;
  int32_T tmp_2;
  int32_T tmp_size_idx_0;
  int32_T trueCount;
  int8_T tmp_data[12];
  boolean_T b[24];
  boolean_T tmp[12];
  boolean_T exitg1;
  boolean_T p;
  boolean_T searchFlag;
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  /* SignalConversion generated from: '<Root>/Pure Pursuit' incorporates:
   *  Integrator: '<Root>/Integrator'
   *  Integrator: '<Root>/Integrator2'
   *  Integrator: '<Root>/Integrator4'
   */
  rtb_TmpSignalConversionAtPurePu[0] = rtX.Integrator_CSTATE;
  rtb_TmpSignalConversionAtPurePu[1] = rtX.Integrator2_CSTATE;
  rtb_TmpSignalConversionAtPurePu[2] = rtX.Integrator4_CSTATE;

  /* MATLABSystem: '<Root>/Pure Pursuit' incorporates:
   *  Constant: '<Root>/Constant'
   *  Integrator: '<Root>/Integrator'
   *  Integrator: '<Root>/Integrator2'
   *  Integrator: '<Root>/Integrator4'
   */
  if (rtDW.obj.DesiredLinearVelocity != 0.2) {
    rtDW.obj.DesiredLinearVelocity = 0.2;
  }

  if (rtDW.obj.MaxAngularVelocity != 2.0) {
    rtDW.obj.MaxAngularVelocity = 2.0;
  }

  if (rtDW.obj.LookaheadDistance != 0.4) {
    rtDW.obj.LookaheadDistance = 0.4;
  }

  searchFlag = false;
  p = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 24)) {
    if ((rtDW.obj.WaypointsInternal[b_k] == rtConstP.Constant_Value[b_k]) ||
        (rtIsNaN(rtDW.obj.WaypointsInternal[b_k]) && rtIsNaN
         (rtConstP.Constant_Value[b_k]))) {
      b_k++;
    } else {
      p = false;
      exitg1 = true;
    }
  }

  if (p) {
    searchFlag = true;
  }

  if (!searchFlag) {
    memcpy(&rtDW.obj.WaypointsInternal[0], &rtConstP.Constant_Value[0], 24U *
           sizeof(real_T));
    rtDW.obj.ProjectionLineIndex = 0.0;
  }

  for (trueCount = 0; trueCount < 24; trueCount++) {
    b[trueCount] = !rtIsNaN(rtConstP.Constant_Value[trueCount]);
  }

  trueCount = 0;
  for (b_k = 0; b_k < 12; b_k++) {
    searchFlag = (b[b_k] && b[b_k + 12]);
    tmp[b_k] = searchFlag;
    if (searchFlag) {
      trueCount++;
    }
  }

  tmp_size_idx_0 = trueCount;
  trueCount = 0;
  for (b_k = 0; b_k < 12; b_k++) {
    if (tmp[b_k]) {
      tmp_data[trueCount] = (int8_T)b_k;
      trueCount++;
    }
  }

  if (tmp_size_idx_0 == 0) {
    dist = 0.0;
    minDistance = 0.0;
  } else {
    searchFlag = false;
    if (rtDW.obj.ProjectionLineIndex == 0.0) {
      searchFlag = true;
      rtDW.obj.ProjectionPoint[0] = rtConstP.Constant_Value[tmp_data[0]];
      rtDW.obj.ProjectionPoint[1] = rtConstP.Constant_Value[tmp_data[0] + 12];
      rtDW.obj.ProjectionLineIndex = 1.0;
    }

    if (tmp_size_idx_0 == 1) {
      lookaheadEndPt_idx_0 = rtConstP.Constant_Value[tmp_data[0]];
      rtDW.obj.ProjectionPoint[0] = lookaheadEndPt_idx_0;
      lookaheadEndPt_idx_1 = rtConstP.Constant_Value[tmp_data[0] + 12];
      rtDW.obj.ProjectionPoint[1] = lookaheadEndPt_idx_1;
    } else {
      b_k = tmp_data[(int32_T)(rtDW.obj.ProjectionLineIndex + 1.0) - 1];
      lookaheadStartPt[0] = rtConstP.Constant_Value[b_k];
      lookaheadStartPt[1] = rtConstP.Constant_Value[b_k + 12];
      minDistance = closestPointOnLine(rtDW.obj.ProjectionPoint,
        lookaheadStartPt, &rtb_TmpSignalConversionAtPurePu[0]);
      rtDW.obj.ProjectionPoint[0] = lookaheadStartPt[0];
      lookaheadStartPt_0[0] = lookaheadStartPt[0] -
        rtConstP.Constant_Value[tmp_data[(int32_T)(rtDW.obj.ProjectionLineIndex
        + 1.0) - 1]];
      rtDW.obj.ProjectionPoint[1] = lookaheadStartPt[1];
      lookaheadStartPt_0[1] = lookaheadStartPt[1] -
        rtConstP.Constant_Value[tmp_data[(int32_T)(rtDW.obj.ProjectionLineIndex
        + 1.0) - 1] + 12];
      dist = norm(lookaheadStartPt_0);
      lookaheadIdx = rtDW.obj.ProjectionLineIndex + 1.0;
      b_k = (int32_T)((1.0 - (rtDW.obj.ProjectionLineIndex + 1.0)) + ((real_T)
        tmp_size_idx_0 - 1.0)) - 1;
      b_i = 0;
      exitg1 = false;
      while ((!exitg1) && (b_i <= b_k)) {
        lookaheadEndPt_idx_0 = lookaheadIdx + (real_T)b_i;
        if ((!searchFlag) && (dist > rtDW.obj.LookaheadDistance)) {
          exitg1 = true;
        } else {
          trueCount = tmp_data[(int32_T)(lookaheadEndPt_idx_0 + 1.0) - 1];
          lookaheadEndPt_idx_1 = rtConstP.Constant_Value[trueCount];
          tmp_2 = tmp_data[(int32_T)lookaheadEndPt_idx_0 - 1];
          tmp_1 = rtConstP.Constant_Value[tmp_2];
          lookaheadStartPt_0[0] = tmp_1 - lookaheadEndPt_idx_1;
          lookaheadStartPt[0] = lookaheadEndPt_idx_1;
          tmp_0[0] = tmp_1;
          lookaheadEndPt_idx_1 = rtConstP.Constant_Value[trueCount + 12];
          tmp_1 = rtConstP.Constant_Value[tmp_2 + 12];
          lookaheadStartPt_0[1] = tmp_1 - lookaheadEndPt_idx_1;
          lookaheadStartPt[1] = lookaheadEndPt_idx_1;
          tmp_0[1] = tmp_1;
          dist += norm(lookaheadStartPt_0);
          lookaheadEndPt_idx_1 = closestPointOnLine(tmp_0, lookaheadStartPt,
            &rtb_TmpSignalConversionAtPurePu[0]);
          if (lookaheadEndPt_idx_1 < minDistance) {
            minDistance = lookaheadEndPt_idx_1;
            rtDW.obj.ProjectionPoint[0] = lookaheadStartPt[0];
            rtDW.obj.ProjectionPoint[1] = lookaheadStartPt[1];
            rtDW.obj.ProjectionLineIndex = lookaheadEndPt_idx_0;
          }

          b_i++;
        }
      }

      trueCount = tmp_data[(int32_T)(rtDW.obj.ProjectionLineIndex + 1.0) - 1];
      lookaheadEndPt_idx_0 = rtConstP.Constant_Value[trueCount];
      lookaheadStartPt_0[0] = rtDW.obj.ProjectionPoint[0] - lookaheadEndPt_idx_0;
      lookaheadEndPt_idx_1 = rtConstP.Constant_Value[trueCount + 12];
      lookaheadStartPt_0[1] = rtDW.obj.ProjectionPoint[1] - lookaheadEndPt_idx_1;
      dist = norm(lookaheadStartPt_0);
      lookaheadStartPt[0] = rtDW.obj.ProjectionPoint[0];
      lookaheadStartPt[1] = rtDW.obj.ProjectionPoint[1];
      minDistance = dist - rtDW.obj.LookaheadDistance;
      lookaheadIdx = rtDW.obj.ProjectionLineIndex;
      while ((minDistance < 0.0) && (lookaheadIdx < (real_T)tmp_size_idx_0 - 1.0))
      {
        lookaheadIdx++;
        b_k = tmp_data[(int32_T)lookaheadIdx - 1];
        minDistance = rtConstP.Constant_Value[b_k];
        lookaheadStartPt[0] = minDistance;
        trueCount = tmp_data[(int32_T)(lookaheadIdx + 1.0) - 1];
        lookaheadEndPt_idx_0 = rtConstP.Constant_Value[trueCount];
        lookaheadStartPt_0[0] = minDistance - lookaheadEndPt_idx_0;
        minDistance = rtConstP.Constant_Value[b_k + 12];
        lookaheadStartPt[1] = minDistance;
        lookaheadEndPt_idx_1 = rtConstP.Constant_Value[trueCount + 12];
        lookaheadStartPt_0[1] = minDistance - lookaheadEndPt_idx_1;
        dist += norm(lookaheadStartPt_0);
        minDistance = dist - rtDW.obj.LookaheadDistance;
      }

      lookaheadStartPt_0[0] = lookaheadStartPt[0] - lookaheadEndPt_idx_0;
      lookaheadStartPt_0[1] = lookaheadStartPt[1] - lookaheadEndPt_idx_1;
      dist = minDistance / norm(lookaheadStartPt_0);
      if (dist > 0.0) {
        lookaheadEndPt_idx_0 = (1.0 - dist) * lookaheadEndPt_idx_0 + dist *
          lookaheadStartPt[0];
        lookaheadEndPt_idx_1 = (1.0 - dist) * lookaheadEndPt_idx_1 + dist *
          lookaheadStartPt[1];
      }
    }

    rtDW.obj.LookaheadPoint[0] = lookaheadEndPt_idx_0;
    rtDW.obj.LookaheadPoint[1] = lookaheadEndPt_idx_1;
    dist = rt_atan2d_snf(rtDW.obj.LookaheadPoint[1] - rtX.Integrator2_CSTATE,
                         rtDW.obj.LookaheadPoint[0] - rtX.Integrator_CSTATE) -
      rtX.Integrator4_CSTATE;
    if (fabs(dist) > 3.1415926535897931) {
      if (rtIsNaN(dist + 3.1415926535897931) || rtIsInf(dist +
           3.1415926535897931)) {
        minDistance = (rtNaN);
      } else {
        minDistance = fabs((dist + 3.1415926535897931) / 6.2831853071795862);
        if (fabs(minDistance - floor(minDistance + 0.5)) >
            2.2204460492503131E-16 * minDistance) {
          minDistance = fmod(dist + 3.1415926535897931, 6.2831853071795862);
        } else {
          minDistance = 0.0;
        }

        if (minDistance == 0.0) {
          minDistance = 0.0;
        } else if (minDistance < 0.0) {
          minDistance += 6.2831853071795862;
        }
      }

      if ((minDistance == 0.0) && (dist + 3.1415926535897931 > 0.0)) {
        minDistance = 6.2831853071795862;
      }

      dist = minDistance - 3.1415926535897931;
    }

    minDistance = 2.0 * sin(dist) / rtDW.obj.LookaheadDistance;
    if (rtIsNaN(minDistance)) {
      minDistance = 0.0;
    }

    if (fabs(fabs(dist) - 3.1415926535897931) < 1.4901161193847656E-8) {
      if (rtIsNaN(minDistance)) {
        minDistance = (rtNaN);
      } else if (minDistance < 0.0) {
        minDistance = -1.0;
      } else {
        minDistance = (minDistance > 0.0);
      }
    }

    if (fabs(minDistance) > rtDW.obj.MaxAngularVelocity) {
      if (rtIsNaN(minDistance)) {
        lookaheadEndPt_idx_1 = (rtNaN);
      } else if (minDistance < 0.0) {
        lookaheadEndPt_idx_1 = -1.0;
      } else {
        lookaheadEndPt_idx_1 = (minDistance > 0.0);
      }

      minDistance = lookaheadEndPt_idx_1 * rtDW.obj.MaxAngularVelocity;
    }

    dist = rtDW.obj.DesiredLinearVelocity;
    rtDW.obj.LastPose[0] = rtX.Integrator_CSTATE;
    rtDW.obj.LastPose[1] = rtX.Integrator2_CSTATE;
    rtDW.obj.LastPose[2] = rtX.Integrator4_CSTATE;
  }

  lookaheadStartPt[0] = dist;

  /* Gain: '<Root>/Gain' incorporates:
   *  Gain: '<Root>/Gain1'
   *  MATLABSystem: '<Root>/Pure Pursuit'
   */
  lookaheadIdx = 0.1 * minDistance;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/encoder Vr-actual'
   *  MATLABSystem: '<Root>/Pure Pursuit'
   *  Sum: '<Root>/Sum'
   */
  dist = (lookaheadIdx + dist) - rtU.Vractual;

  /* Gain: '<S40>/Filter Coefficient' incorporates:
   *  Gain: '<S30>/Derivative Gain'
   *  Integrator: '<S32>/Filter'
   *  Sum: '<S32>/SumD'
   */
  rtDW.FilterCoefficient = (-0.0128072394159001 * dist - rtX.Filter_CSTATE) *
    84.6019689511989;

  /* Outport: '<Root>/Motor Vr Control signal' incorporates:
   *  Gain: '<S42>/Proportional Gain'
   *  Integrator: '<S37>/Integrator'
   *  Sum: '<S46>/Sum'
   */
  rtY.MotorVrControlsignal = (1.08351767141455 * dist + rtX.Integrator_CSTATE_m)
    + rtDW.FilterCoefficient;

  /* Gain: '<S34>/Integral Gain' */
  rtDW.IntegralGain = 20.5811272107881 * dist;

  /* Sum: '<Root>/Sum3' incorporates:
   *  Inport: '<Root>/encoder Vi-actual'
   *  MATLABSystem: '<Root>/Pure Pursuit'
   *  Sum: '<Root>/Sum1'
   * */
  dist = (lookaheadStartPt[0] - lookaheadIdx) - rtU.Vlactual;

  /* Gain: '<S92>/Filter Coefficient' incorporates:
   *  Gain: '<S82>/Derivative Gain'
   *  Integrator: '<S84>/Filter'
   *  Sum: '<S84>/SumD'
   */
  rtDW.FilterCoefficient_m = (-0.0128072394159001 * dist - rtX.Filter_CSTATE_f) *
    84.6019689511989;

  /* Outport: '<Root>/Motor Vi Control signal' incorporates:
   *  Gain: '<S94>/Proportional Gain'
   *  Integrator: '<S89>/Integrator'
   *  Sum: '<S98>/Sum'
   */
  rtY.MotorViControlsignal = (1.08351767141455 * dist + rtX.Integrator_CSTATE_n)
    + rtDW.FilterCoefficient_m;

  /* Gain: '<S86>/Integral Gain' */
  rtDW.IntegralGain_h = 20.5811272107881 * dist;

  /* Gain: '<Root>/Gain2' incorporates:
   *  Inport: '<Root>/encoder Vi-actual'
   *  Inport: '<Root>/encoder Vr-actual'
   *  Sum: '<Root>/Sum4'
   */
  dist = (rtU.Vractual + rtU.Vlactual) * 0.5;

  /* Product: '<Root>/Product' incorporates:
   *  Integrator: '<Root>/Integrator1'
   *  Trigonometry: '<Root>/cos'
   */
  rtDW.Product = dist * cos(rtX.Integrator1_CSTATE);

  /* Product: '<Root>/Product1' incorporates:
   *  Integrator: '<Root>/Integrator3'
   *  Trigonometry: '<Root>/sin'
   */
  rtDW.Product1 = dist * sin(rtX.Integrator3_CSTATE);

  /* Gain: '<Root>/Gain3' incorporates:
   *  Inport: '<Root>/encoder Vi-actual'
   *  Inport: '<Root>/encoder Vr-actual'
   *  Sum: '<Root>/Sum5'
   */
  rtDW.wactual = (rtU.Vractual - rtU.Vlactual) * 5.0;
  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void Pure_Pursuit_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<Root>/Integrator' */
  _rtXdot->Integrator_CSTATE = rtDW.Product;

  /* Derivatives for Integrator: '<Root>/Integrator2' */
  _rtXdot->Integrator2_CSTATE = rtDW.Product1;

  /* Derivatives for Integrator: '<Root>/Integrator4' */
  _rtXdot->Integrator4_CSTATE = rtDW.wactual;

  /* Derivatives for Integrator: '<S37>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = rtDW.IntegralGain;

  /* Derivatives for Integrator: '<S32>/Filter' */
  _rtXdot->Filter_CSTATE = rtDW.FilterCoefficient;

  /* Derivatives for Integrator: '<S89>/Integrator' */
  _rtXdot->Integrator_CSTATE_n = rtDW.IntegralGain_h;

  /* Derivatives for Integrator: '<S84>/Filter' */
  _rtXdot->Filter_CSTATE_f = rtDW.FilterCoefficient_m;

  /* Derivatives for Integrator: '<Root>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = rtDW.wactual;

  /* Derivatives for Integrator: '<Root>/Integrator3' */
  _rtXdot->Integrator3_CSTATE = rtDW.wactual;

  /* Derivatives for TransferFcn: '<Root>/Motor1' */
  _rtXdot->Motor1_CSTATE = -10.0 * rtX.Motor1_CSTATE;
}

/* Model initialize function */
void Pure_Pursuit_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&rtM->solverInfo, (boolean_T**)
      &rtM->contStateDisabled);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetIsContModeFrozen(&rtM->solverInfo, false);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->contStates = ((X *) &rtX);
  rtM->contStateDisabled = ((XDis *) &rtXDis);
  rtM->Timing.tStart = (0.0);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode3");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.001;

  {
    int32_T i;

    /* InitializeConditions for Integrator: '<Root>/Integrator' */
    rtX.Integrator_CSTATE = 1.0;

    /* InitializeConditions for Integrator: '<Root>/Integrator2' */
    rtX.Integrator2_CSTATE = 1.0;

    /* InitializeConditions for Integrator: '<Root>/Integrator4' */
    rtX.Integrator4_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S37>/Integrator' */
    rtX.Integrator_CSTATE_m = 0.0;

    /* InitializeConditions for Integrator: '<S32>/Filter' */
    rtX.Filter_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<S89>/Integrator' */
    rtX.Integrator_CSTATE_n = 0.0;

    /* InitializeConditions for Integrator: '<S84>/Filter' */
    rtX.Filter_CSTATE_f = 0.0;

    /* InitializeConditions for Integrator: '<Root>/Integrator1' */
    rtX.Integrator1_CSTATE = 0.0;

    /* InitializeConditions for Integrator: '<Root>/Integrator3' */
    rtX.Integrator3_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<Root>/Motor1' */
    rtX.Motor1_CSTATE = 0.0;

    /* Start for MATLABSystem: '<Root>/Pure Pursuit' */
    rtDW.obj.DesiredLinearVelocity = 0.2;
    rtDW.obj.MaxAngularVelocity = 2.0;
    rtDW.obj.LookaheadDistance = 0.4;
    rtDW.obj.isInitialized = 1;
    for (i = 0; i < 24; i++) {
      rtDW.obj.WaypointsInternal[i] = (rtNaN);
    }

    /* InitializeConditions for MATLABSystem: '<Root>/Pure Pursuit' */
    rtDW.obj.LookaheadPoint[0] = 0.0;
    rtDW.obj.LookaheadPoint[1] = 0.0;
    rtDW.obj.LastPose[0] = 0.0;
    rtDW.obj.LastPose[1] = 0.0;
    rtDW.obj.LastPose[2] = 0.0;
    rtDW.obj.ProjectionPoint[0] = (rtNaN);
    rtDW.obj.ProjectionPoint[1] = (rtNaN);
    rtDW.obj.ProjectionLineIndex = 0.0;
  }
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
