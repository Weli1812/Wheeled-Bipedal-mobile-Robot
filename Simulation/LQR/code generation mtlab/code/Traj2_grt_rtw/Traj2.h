/*
 * Traj2.h
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

#ifndef Traj2_h_
#define Traj2_h_
#ifndef Traj2_COMMON_INCLUDES_
#define Traj2_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* Traj2_COMMON_INCLUDES_ */

#include "Traj2_types.h"
#include <stddef.h>
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include <float.h>
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T t_vec[1276];
  real_T y_m[1276];
  real_T x_c[1276];
  real_T Clock;                        /* '<Root>/Clock' */
  real_T x;                            /* '<Root>/Integrator' */
  real_T y;                            /* '<Root>/Integrator1' */
  real_T theta;                        /* '<Root>/Integrator2' */
  real_T e[3];
  real_T Gain2[2];                     /* '<Root>/Gain2' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T Saturation;                   /* '<Root>/Saturation' */
  real_T Sum2;                         /* '<Root>/Sum2' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T vr_ref;                       /* '<Root>/Sum6' */
  real_T Sum8;                         /* '<Root>/Sum8' */
  real_T ProportionalGain;             /* '<S44>/Proportional Gain' */
  real_T Integrator;                   /* '<S39>/Integrator' */
  real_T DerivativeGain;               /* '<S32>/Derivative Gain' */
  real_T Filter;                       /* '<S34>/Filter' */
  real_T SumD;                         /* '<S34>/SumD' */
  real_T FilterCoefficient;            /* '<S42>/Filter Coefficient' */
  real_T Sum;                          /* '<S48>/Sum' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T vl_ref;                       /* '<Root>/Sum7' */
  real_T Sum9;                         /* '<Root>/Sum9' */
  real_T ProportionalGain_p;           /* '<S96>/Proportional Gain' */
  real_T Integrator_k;                 /* '<S91>/Integrator' */
  real_T DerivativeGain_d;             /* '<S84>/Derivative Gain' */
  real_T Filter_e;                     /* '<S86>/Filter' */
  real_T SumD_o;                       /* '<S86>/SumD' */
  real_T FilterCoefficient_k;          /* '<S94>/Filter Coefficient' */
  real_T Sum_d;                        /* '<S100>/Sum' */
  real_T Integrator3[2];               /* '<Root>/Integrator3' */
  real_T Sum_e;                        /* '<Root>/Sum' */
  real_T v_actual;                     /* '<Root>/Gain' */
  real_T Sum1;                         /* '<Root>/Sum1' */
  real_T Gain1;                        /* '<Root>/Gain1' */
  real_T u;                            /* '<Root>/ ' */
  real_T Cos;                          /* '<Root>/Cos' */
  real_T IntegralGain;                 /* '<S36>/Integral Gain' */
  real_T IntegralGain_m;               /* '<S88>/Integral Gain' */
  real_T x_dot;                        /* '<Root>/Product' */
  real_T y_dot;                        /* '<Root>/Product1' */
  real_T ex;                           /* '<Root>/MATLAB Function1' */
  real_T ey;                           /* '<Root>/MATLAB Function1' */
  real_T etheta;                       /* '<Root>/MATLAB Function1' */
  real_T xd;                           /* '<Root>/MATLAB Function' */
  real_T yd;                           /* '<Root>/MATLAB Function' */
  real_T thetad;                       /* '<Root>/MATLAB Function' */
  real_T vd;                           /* '<Root>/MATLAB Function' */
  real_T wd;                           /* '<Root>/MATLAB Function' */
} B_Traj2_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T PrevY;                        /* '<Root>/Rate Limiter' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<Root>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<Root>/To Workspace2' */

  boolean_T PrevLimited;               /* '<Root>/Rate Limiter' */
} DW_Traj2_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
  real_T Integrator_CSTATE_i;          /* '<S39>/Integrator' */
  real_T Filter_CSTATE;                /* '<S34>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S91>/Integrator' */
  real_T Filter_CSTATE_a;              /* '<S86>/Filter' */
  real_T Integrator3_CSTATE[2];        /* '<Root>/Integrator3' */
} X_Traj2_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
  real_T Integrator_CSTATE_i;          /* '<S39>/Integrator' */
  real_T Filter_CSTATE;                /* '<S34>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S91>/Integrator' */
  real_T Filter_CSTATE_a;              /* '<S86>/Filter' */
  real_T Integrator3_CSTATE[2];        /* '<Root>/Integrator3' */
} XDot_Traj2_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<Root>/Integrator' */
  boolean_T Integrator1_CSTATE;        /* '<Root>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<Root>/Integrator2' */
  boolean_T Integrator_CSTATE_i;       /* '<S39>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S34>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S91>/Integrator' */
  boolean_T Filter_CSTATE_a;           /* '<S86>/Filter' */
  boolean_T Integrator3_CSTATE[2];     /* '<Root>/Integrator3' */
} XDis_Traj2_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: P
   * Referenced by: '<Root>/Constant'
   */
  real_T Constant_Value[2552];

  /* Expression: theta_ref_unwrapped
   * Referenced by: '<Root>/Constant1'
   */
  real_T Constant1_Value[1276];

  /* Expression: vd_ref
   * Referenced by: '<Root>/Constant2'
   */
  real_T Constant2_Value[1276];

  /* Expression: wd_ref
   * Referenced by: '<Root>/Constant3'
   */
  real_T Constant3_Value[1276];

  /* Expression: -K
   * Referenced by: '<Root>/Gain2'
   */
  real_T Gain2_Gain[6];
} ConstP_Traj2_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T vr_actual;                    /* '<Root>/In1' */
  real_T vl_actual;                    /* '<Root>/In2' */
} ExtU_Traj2_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
  real_T Out2;                         /* '<Root>/Out2' */
} ExtY_Traj2_T;

/* Real-time Model Data Structure */
struct tag_RTM_Traj2_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_Traj2_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_Traj2_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[9];
  real_T odeF[3][9];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tStart;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_Traj2_T Traj2_B;

/* Continuous states (default storage) */
extern X_Traj2_T Traj2_X;

/* Disabled states (default storage) */
extern XDis_Traj2_T Traj2_XDis;

/* Block states (default storage) */
extern DW_Traj2_T Traj2_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Traj2_T Traj2_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Traj2_T Traj2_Y;

/* Constant parameters (default storage) */
extern const ConstP_Traj2_T Traj2_ConstP;

/* Model entry point functions */
extern void Traj2_initialize(void);
extern void Traj2_step(void);
extern void Traj2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Traj2_T *const Traj2_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Traj2'
 * '<S1>'   : 'Traj2/MATLAB Function'
 * '<S2>'   : 'Traj2/MATLAB Function1'
 * '<S3>'   : 'Traj2/PID Controller'
 * '<S4>'   : 'Traj2/PID Controller1'
 * '<S5>'   : 'Traj2/PID Controller/Anti-windup'
 * '<S6>'   : 'Traj2/PID Controller/D Gain'
 * '<S7>'   : 'Traj2/PID Controller/External Derivative'
 * '<S8>'   : 'Traj2/PID Controller/Filter'
 * '<S9>'   : 'Traj2/PID Controller/Filter ICs'
 * '<S10>'  : 'Traj2/PID Controller/I Gain'
 * '<S11>'  : 'Traj2/PID Controller/Ideal P Gain'
 * '<S12>'  : 'Traj2/PID Controller/Ideal P Gain Fdbk'
 * '<S13>'  : 'Traj2/PID Controller/Integrator'
 * '<S14>'  : 'Traj2/PID Controller/Integrator ICs'
 * '<S15>'  : 'Traj2/PID Controller/N Copy'
 * '<S16>'  : 'Traj2/PID Controller/N Gain'
 * '<S17>'  : 'Traj2/PID Controller/P Copy'
 * '<S18>'  : 'Traj2/PID Controller/Parallel P Gain'
 * '<S19>'  : 'Traj2/PID Controller/Reset Signal'
 * '<S20>'  : 'Traj2/PID Controller/Saturation'
 * '<S21>'  : 'Traj2/PID Controller/Saturation Fdbk'
 * '<S22>'  : 'Traj2/PID Controller/Sum'
 * '<S23>'  : 'Traj2/PID Controller/Sum Fdbk'
 * '<S24>'  : 'Traj2/PID Controller/Tracking Mode'
 * '<S25>'  : 'Traj2/PID Controller/Tracking Mode Sum'
 * '<S26>'  : 'Traj2/PID Controller/Tsamp - Integral'
 * '<S27>'  : 'Traj2/PID Controller/Tsamp - Ngain'
 * '<S28>'  : 'Traj2/PID Controller/postSat Signal'
 * '<S29>'  : 'Traj2/PID Controller/preInt Signal'
 * '<S30>'  : 'Traj2/PID Controller/preSat Signal'
 * '<S31>'  : 'Traj2/PID Controller/Anti-windup/Passthrough'
 * '<S32>'  : 'Traj2/PID Controller/D Gain/Internal Parameters'
 * '<S33>'  : 'Traj2/PID Controller/External Derivative/Error'
 * '<S34>'  : 'Traj2/PID Controller/Filter/Cont. Filter'
 * '<S35>'  : 'Traj2/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S36>'  : 'Traj2/PID Controller/I Gain/Internal Parameters'
 * '<S37>'  : 'Traj2/PID Controller/Ideal P Gain/Passthrough'
 * '<S38>'  : 'Traj2/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S39>'  : 'Traj2/PID Controller/Integrator/Continuous'
 * '<S40>'  : 'Traj2/PID Controller/Integrator ICs/Internal IC'
 * '<S41>'  : 'Traj2/PID Controller/N Copy/Disabled'
 * '<S42>'  : 'Traj2/PID Controller/N Gain/Internal Parameters'
 * '<S43>'  : 'Traj2/PID Controller/P Copy/Disabled'
 * '<S44>'  : 'Traj2/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S45>'  : 'Traj2/PID Controller/Reset Signal/Disabled'
 * '<S46>'  : 'Traj2/PID Controller/Saturation/Passthrough'
 * '<S47>'  : 'Traj2/PID Controller/Saturation Fdbk/Disabled'
 * '<S48>'  : 'Traj2/PID Controller/Sum/Sum_PID'
 * '<S49>'  : 'Traj2/PID Controller/Sum Fdbk/Disabled'
 * '<S50>'  : 'Traj2/PID Controller/Tracking Mode/Disabled'
 * '<S51>'  : 'Traj2/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S52>'  : 'Traj2/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S53>'  : 'Traj2/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S54>'  : 'Traj2/PID Controller/postSat Signal/Forward_Path'
 * '<S55>'  : 'Traj2/PID Controller/preInt Signal/Internal PreInt'
 * '<S56>'  : 'Traj2/PID Controller/preSat Signal/Forward_Path'
 * '<S57>'  : 'Traj2/PID Controller1/Anti-windup'
 * '<S58>'  : 'Traj2/PID Controller1/D Gain'
 * '<S59>'  : 'Traj2/PID Controller1/External Derivative'
 * '<S60>'  : 'Traj2/PID Controller1/Filter'
 * '<S61>'  : 'Traj2/PID Controller1/Filter ICs'
 * '<S62>'  : 'Traj2/PID Controller1/I Gain'
 * '<S63>'  : 'Traj2/PID Controller1/Ideal P Gain'
 * '<S64>'  : 'Traj2/PID Controller1/Ideal P Gain Fdbk'
 * '<S65>'  : 'Traj2/PID Controller1/Integrator'
 * '<S66>'  : 'Traj2/PID Controller1/Integrator ICs'
 * '<S67>'  : 'Traj2/PID Controller1/N Copy'
 * '<S68>'  : 'Traj2/PID Controller1/N Gain'
 * '<S69>'  : 'Traj2/PID Controller1/P Copy'
 * '<S70>'  : 'Traj2/PID Controller1/Parallel P Gain'
 * '<S71>'  : 'Traj2/PID Controller1/Reset Signal'
 * '<S72>'  : 'Traj2/PID Controller1/Saturation'
 * '<S73>'  : 'Traj2/PID Controller1/Saturation Fdbk'
 * '<S74>'  : 'Traj2/PID Controller1/Sum'
 * '<S75>'  : 'Traj2/PID Controller1/Sum Fdbk'
 * '<S76>'  : 'Traj2/PID Controller1/Tracking Mode'
 * '<S77>'  : 'Traj2/PID Controller1/Tracking Mode Sum'
 * '<S78>'  : 'Traj2/PID Controller1/Tsamp - Integral'
 * '<S79>'  : 'Traj2/PID Controller1/Tsamp - Ngain'
 * '<S80>'  : 'Traj2/PID Controller1/postSat Signal'
 * '<S81>'  : 'Traj2/PID Controller1/preInt Signal'
 * '<S82>'  : 'Traj2/PID Controller1/preSat Signal'
 * '<S83>'  : 'Traj2/PID Controller1/Anti-windup/Passthrough'
 * '<S84>'  : 'Traj2/PID Controller1/D Gain/Internal Parameters'
 * '<S85>'  : 'Traj2/PID Controller1/External Derivative/Error'
 * '<S86>'  : 'Traj2/PID Controller1/Filter/Cont. Filter'
 * '<S87>'  : 'Traj2/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S88>'  : 'Traj2/PID Controller1/I Gain/Internal Parameters'
 * '<S89>'  : 'Traj2/PID Controller1/Ideal P Gain/Passthrough'
 * '<S90>'  : 'Traj2/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S91>'  : 'Traj2/PID Controller1/Integrator/Continuous'
 * '<S92>'  : 'Traj2/PID Controller1/Integrator ICs/Internal IC'
 * '<S93>'  : 'Traj2/PID Controller1/N Copy/Disabled'
 * '<S94>'  : 'Traj2/PID Controller1/N Gain/Internal Parameters'
 * '<S95>'  : 'Traj2/PID Controller1/P Copy/Disabled'
 * '<S96>'  : 'Traj2/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S97>'  : 'Traj2/PID Controller1/Reset Signal/Disabled'
 * '<S98>'  : 'Traj2/PID Controller1/Saturation/Passthrough'
 * '<S99>'  : 'Traj2/PID Controller1/Saturation Fdbk/Disabled'
 * '<S100>' : 'Traj2/PID Controller1/Sum/Sum_PID'
 * '<S101>' : 'Traj2/PID Controller1/Sum Fdbk/Disabled'
 * '<S102>' : 'Traj2/PID Controller1/Tracking Mode/Disabled'
 * '<S103>' : 'Traj2/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S104>' : 'Traj2/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S105>' : 'Traj2/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S106>' : 'Traj2/PID Controller1/postSat Signal/Forward_Path'
 * '<S107>' : 'Traj2/PID Controller1/preInt Signal/Internal PreInt'
 * '<S108>' : 'Traj2/PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* Traj2_h_ */
