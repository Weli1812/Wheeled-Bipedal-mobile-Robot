/*
 * Traj2.h
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
  real_T t_vec[1012];
  real_T y_m[1012];
  real_T x_c[1012];
  real_T Clock;                        /* '<Root>/Clock' */
  real_T x;                            /* '<Root>/Integrator' */
  real_T y;                            /* '<Root>/Integrator1' */
  real_T theta;                        /* '<Root>/Integrator2' */
  real_T e[3];
  real_T Gain2[2];                     /* '<Root>/Gain2' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T Sum2;                         /* '<Root>/Sum2' */
  real_T Gain3;                        /* '<Root>/Gain3' */
  real_T vr_ref;                       /* '<Root>/Sum6' */
  real_T Sum8;                         /* '<Root>/Sum8' */
  real_T ProportionalGain;             /* '<S50>/Proportional Gain' */
  real_T Integrator;                   /* '<S45>/Integrator' */
  real_T DerivativeGain;               /* '<S38>/Derivative Gain' */
  real_T Filter;                       /* '<S40>/Filter' */
  real_T SumD;                         /* '<S40>/SumD' */
  real_T FilterCoefficient;            /* '<S48>/Filter Coefficient' */
  real_T Sum;                          /* '<S54>/Sum' */
  real_T Saturation;                   /* '<S52>/Saturation' */
  real_T Divide;                       /* '<Root>/Divide' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T vl_ref;                       /* '<Root>/Sum7' */
  real_T Sum9;                         /* '<Root>/Sum9' */
  real_T ProportionalGain_p;           /* '<S104>/Proportional Gain' */
  real_T Integrator_k;                 /* '<S99>/Integrator' */
  real_T DerivativeGain_d;             /* '<S92>/Derivative Gain' */
  real_T Filter_e;                     /* '<S94>/Filter' */
  real_T SumD_o;                       /* '<S94>/SumD' */
  real_T FilterCoefficient_k;          /* '<S102>/Filter Coefficient' */
  real_T Sum_d;                        /* '<S108>/Sum' */
  real_T Saturation_b;                 /* '<S106>/Saturation' */
  real_T Divide1;                      /* '<Root>/Divide1' */
  real_T Abs;                          /* '<Root>/Abs' */
  real_T Constant;                     /* '<S7>/Constant' */
  real_T Abs1;                         /* '<Root>/Abs1' */
  real_T Constant_f;                   /* '<S8>/Constant' */
  real_T u;                            /* '<Root>/ ' */
  real_T Cos;                          /* '<Root>/Cos' */
  real_T Sum_e;                        /* '<Root>/Sum' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T Sum1;                         /* '<Root>/Sum1' */
  real_T Gain1;                        /* '<Root>/Gain1' */
  real_T ZeroGain;                     /* '<S35>/ZeroGain' */
  real_T DeadZone;                     /* '<S37>/DeadZone' */
  real_T SignPreSat;                   /* '<S35>/SignPreSat' */
  real_T IntegralGain;                 /* '<S42>/Integral Gain' */
  real_T SignPreIntegrator;            /* '<S35>/SignPreIntegrator' */
  real_T Switch;                       /* '<S35>/Switch' */
  real_T ZeroGain_g;                   /* '<S89>/ZeroGain' */
  real_T DeadZone_n;                   /* '<S91>/DeadZone' */
  real_T SignPreSat_b;                 /* '<S89>/SignPreSat' */
  real_T IntegralGain_m;               /* '<S96>/Integral Gain' */
  real_T SignPreIntegrator_d;          /* '<S89>/SignPreIntegrator' */
  real_T Switch_n;                     /* '<S89>/Switch' */
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
  int8_T DataTypeConv1;                /* '<S35>/DataTypeConv1' */
  int8_T DataTypeConv2;                /* '<S35>/DataTypeConv2' */
  int8_T DataTypeConv1_c;              /* '<S89>/DataTypeConv1' */
  int8_T DataTypeConv2_h;              /* '<S89>/DataTypeConv2' */
  boolean_T Compare;                   /* '<S1>/Compare' */
  boolean_T Compare_a;                 /* '<S2>/Compare' */
  boolean_T NotEqual;                  /* '<S35>/NotEqual' */
  boolean_T Equal1;                    /* '<S35>/Equal1' */
  boolean_T AND3;                      /* '<S35>/AND3' */
  boolean_T Memory;                    /* '<S35>/Memory' */
  boolean_T NotEqual_m;                /* '<S89>/NotEqual' */
  boolean_T Equal1_k;                  /* '<S89>/Equal1' */
  boolean_T AND3_p;                    /* '<S89>/AND3' */
  boolean_T Memory_i;                  /* '<S89>/Memory' */
} B_Traj2_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T startTimeOfNextCycle;         /* '<S7>/Variable Pulse Generator' */
  real_T startTimeOfNextCycle_p;       /* '<S8>/Variable Pulse Generator' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<Root>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<Root>/To Workspace2' */

  boolean_T nextOutput;                /* '<S7>/Variable Pulse Generator' */
  boolean_T isStartOfNextCycle;        /* '<S7>/Variable Pulse Generator' */
  boolean_T isFirstWarningDCGreaterThanOne;/* '<S7>/Variable Pulse Generator' */
  boolean_T isFirstWarningDCLessThanZero;/* '<S7>/Variable Pulse Generator' */
  boolean_T nextOutput_p;              /* '<S8>/Variable Pulse Generator' */
  boolean_T isStartOfNextCycle_d;      /* '<S8>/Variable Pulse Generator' */
  boolean_T isFirstWarningDCGreaterThanOn_k;/* '<S8>/Variable Pulse Generator' */
  boolean_T isFirstWarningDCLessThanZero_f;/* '<S8>/Variable Pulse Generator' */
  boolean_T Memory_PreviousInput;      /* '<S35>/Memory' */
  boolean_T Memory_PreviousInput_o;    /* '<S89>/Memory' */
} DW_Traj2_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
  real_T Integrator_CSTATE_i;          /* '<S45>/Integrator' */
  real_T Filter_CSTATE;                /* '<S40>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S99>/Integrator' */
  real_T Filter_CSTATE_a;              /* '<S94>/Filter' */
} X_Traj2_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
  real_T Integrator_CSTATE_i;          /* '<S45>/Integrator' */
  real_T Filter_CSTATE;                /* '<S40>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S99>/Integrator' */
  real_T Filter_CSTATE_a;              /* '<S94>/Filter' */
} XDot_Traj2_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<Root>/Integrator' */
  boolean_T Integrator1_CSTATE;        /* '<Root>/Integrator1' */
  boolean_T Integrator2_CSTATE;        /* '<Root>/Integrator2' */
  boolean_T Integrator_CSTATE_i;       /* '<S45>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S40>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S99>/Integrator' */
  boolean_T Filter_CSTATE_a;           /* '<S94>/Filter' */
} XDis_Traj2_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T vr_actual;                    /* '<Root>/EncoderR' */
  real_T vl_actual;                    /* '<Root>/EncoderL' */
} ExtU_Traj2_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  boolean_T dirr;                      /* '<Root>/dirr' */
  boolean_T dirl;                      /* '<Root>/dirl' */
  real_T pwmr;                         /* '<Root>/pwmr' */
  real_T pwml;                         /* '<Root>/pwml' */
} ExtY_Traj2_T;

/* Parameters (default storage) */
struct P_Traj2_T_ {
  real_T K[6];                         /* Variable: K
                                        * Referenced by: '<Root>/Gain2'
                                        */
  real_T L;                            /* Variable: L
                                        * Referenced by:
                                        *   '<Root>/Gain1'
                                        *   '<Root>/Gain3'
                                        *   '<Root>/Gain4'
                                        */
  real_T P[2024];                      /* Variable: P
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Ts_ref;                       /* Variable: Ts_ref
                                        * Referenced by: '<Root>/Constant4'
                                        */
  real_T theta0;                       /* Variable: theta0
                                        * Referenced by: '<Root>/Integrator2'
                                        */
  real_T theta_ref_unwrapped[1012];    /* Variable: theta_ref_unwrapped
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T vd_ref[1012];                 /* Variable: vd_ref
                                        * Referenced by: '<Root>/Constant2'
                                        */
  real_T wd_ref[1012];                 /* Variable: wd_ref
                                        * Referenced by: '<Root>/Constant3'
                                        */
  real_T x0;                           /* Variable: x0
                                        * Referenced by: '<Root>/Integrator'
                                        */
  real_T y0;                           /* Variable: y0
                                        * Referenced by: '<Root>/Integrator1'
                                        */
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S38>/Derivative Gain'
                                        */
  real_T PIDController1_D;             /* Mask Parameter: PIDController1_D
                                        * Referenced by: '<S92>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S42>/Integral Gain'
                                        */
  real_T PIDController1_I;             /* Mask Parameter: PIDController1_I
                                        * Referenced by: '<S96>/Integral Gain'
                                        */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S40>/Filter'
                               */
  real_T PIDController1_InitialCondition;
                              /* Mask Parameter: PIDController1_InitialCondition
                               * Referenced by: '<S94>/Filter'
                               */
  real_T PIDController_InitialConditio_g;
                              /* Mask Parameter: PIDController_InitialConditio_g
                               * Referenced by: '<S45>/Integrator'
                               */
  real_T PIDController1_InitialConditi_c;
                              /* Mask Parameter: PIDController1_InitialConditi_c
                               * Referenced by: '<S99>/Integrator'
                               */
  real_T PIDController_LowerSaturationLi;
                              /* Mask Parameter: PIDController_LowerSaturationLi
                               * Referenced by:
                               *   '<S52>/Saturation'
                               *   '<S37>/DeadZone'
                               */
  real_T PIDController1_LowerSaturationL;
                              /* Mask Parameter: PIDController1_LowerSaturationL
                               * Referenced by:
                               *   '<S106>/Saturation'
                               *   '<S91>/DeadZone'
                               */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S48>/Filter Coefficient'
                                        */
  real_T PIDController1_N;             /* Mask Parameter: PIDController1_N
                                        * Referenced by: '<S102>/Filter Coefficient'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S50>/Proportional Gain'
                                        */
  real_T PIDController1_P;             /* Mask Parameter: PIDController1_P
                                        * Referenced by: '<S104>/Proportional Gain'
                                        */
  real_T PWM_Period;                   /* Mask Parameter: PWM_Period
                                        * Referenced by: '<S7>/Constant'
                                        */
  real_T PWM1_Period;                  /* Mask Parameter: PWM1_Period
                                        * Referenced by: '<S8>/Constant'
                                        */
  real_T PIDController_UpperSaturationLi;
                              /* Mask Parameter: PIDController_UpperSaturationLi
                               * Referenced by:
                               *   '<S52>/Saturation'
                               *   '<S37>/DeadZone'
                               */
  real_T PIDController1_UpperSaturationL;
                              /* Mask Parameter: PIDController1_UpperSaturationL
                               * Referenced by:
                               *   '<S106>/Saturation'
                               *   '<S91>/DeadZone'
                               */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Constant_Value_o;             /* Expression: 0
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<S35>/Constant1'
                                        */
  real_T Constant1_Value_g;            /* Expression: 0
                                        * Referenced by: '<S89>/Constant1'
                                        */
  real_T Constant6_Value;              /* Expression: 5
                                        * Referenced by: '<Root>/Constant6'
                                        */
  real_T Constant5_Value;              /* Expression: 5
                                        * Referenced by: '<Root>/Constant5'
                                        */
  real_T Gain_Gain;                    /* Expression: 0.5
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T ZeroGain_Gain;                /* Expression: 0
                                        * Referenced by: '<S35>/ZeroGain'
                                        */
  real_T ZeroGain_Gain_n;              /* Expression: 0
                                        * Referenced by: '<S89>/ZeroGain'
                                        */
  boolean_T Memory_InitialCondition;
                                  /* Computed Parameter: Memory_InitialCondition
                                   * Referenced by: '<S35>/Memory'
                                   */
  boolean_T Memory_InitialCondition_f;
                                /* Computed Parameter: Memory_InitialCondition_f
                                 * Referenced by: '<S89>/Memory'
                                 */
};

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
  real_T odeY[7];
  real_T odeF[3][7];
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
    struct {
      uint32_T TID[4];
    } TaskCounters;

    uint32_T CtrlRateNumTicksToNextHit[2];
    time_T tStart;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[4];
  } Timing;
};

/* Block parameters (default storage) */
extern P_Traj2_T Traj2_P;

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
 * '<S1>'   : 'Traj2/Compare To Zero'
 * '<S2>'   : 'Traj2/Compare To Zero1'
 * '<S3>'   : 'Traj2/MATLAB Function'
 * '<S4>'   : 'Traj2/MATLAB Function1'
 * '<S5>'   : 'Traj2/PID Controller'
 * '<S6>'   : 'Traj2/PID Controller1'
 * '<S7>'   : 'Traj2/PWM'
 * '<S8>'   : 'Traj2/PWM1'
 * '<S9>'   : 'Traj2/PID Controller/Anti-windup'
 * '<S10>'  : 'Traj2/PID Controller/D Gain'
 * '<S11>'  : 'Traj2/PID Controller/External Derivative'
 * '<S12>'  : 'Traj2/PID Controller/Filter'
 * '<S13>'  : 'Traj2/PID Controller/Filter ICs'
 * '<S14>'  : 'Traj2/PID Controller/I Gain'
 * '<S15>'  : 'Traj2/PID Controller/Ideal P Gain'
 * '<S16>'  : 'Traj2/PID Controller/Ideal P Gain Fdbk'
 * '<S17>'  : 'Traj2/PID Controller/Integrator'
 * '<S18>'  : 'Traj2/PID Controller/Integrator ICs'
 * '<S19>'  : 'Traj2/PID Controller/N Copy'
 * '<S20>'  : 'Traj2/PID Controller/N Gain'
 * '<S21>'  : 'Traj2/PID Controller/P Copy'
 * '<S22>'  : 'Traj2/PID Controller/Parallel P Gain'
 * '<S23>'  : 'Traj2/PID Controller/Reset Signal'
 * '<S24>'  : 'Traj2/PID Controller/Saturation'
 * '<S25>'  : 'Traj2/PID Controller/Saturation Fdbk'
 * '<S26>'  : 'Traj2/PID Controller/Sum'
 * '<S27>'  : 'Traj2/PID Controller/Sum Fdbk'
 * '<S28>'  : 'Traj2/PID Controller/Tracking Mode'
 * '<S29>'  : 'Traj2/PID Controller/Tracking Mode Sum'
 * '<S30>'  : 'Traj2/PID Controller/Tsamp - Integral'
 * '<S31>'  : 'Traj2/PID Controller/Tsamp - Ngain'
 * '<S32>'  : 'Traj2/PID Controller/postSat Signal'
 * '<S33>'  : 'Traj2/PID Controller/preInt Signal'
 * '<S34>'  : 'Traj2/PID Controller/preSat Signal'
 * '<S35>'  : 'Traj2/PID Controller/Anti-windup/Cont. Clamping Parallel'
 * '<S36>'  : 'Traj2/PID Controller/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S37>'  : 'Traj2/PID Controller/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S38>'  : 'Traj2/PID Controller/D Gain/Internal Parameters'
 * '<S39>'  : 'Traj2/PID Controller/External Derivative/Error'
 * '<S40>'  : 'Traj2/PID Controller/Filter/Cont. Filter'
 * '<S41>'  : 'Traj2/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S42>'  : 'Traj2/PID Controller/I Gain/Internal Parameters'
 * '<S43>'  : 'Traj2/PID Controller/Ideal P Gain/Passthrough'
 * '<S44>'  : 'Traj2/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S45>'  : 'Traj2/PID Controller/Integrator/Continuous'
 * '<S46>'  : 'Traj2/PID Controller/Integrator ICs/Internal IC'
 * '<S47>'  : 'Traj2/PID Controller/N Copy/Disabled'
 * '<S48>'  : 'Traj2/PID Controller/N Gain/Internal Parameters'
 * '<S49>'  : 'Traj2/PID Controller/P Copy/Disabled'
 * '<S50>'  : 'Traj2/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S51>'  : 'Traj2/PID Controller/Reset Signal/Disabled'
 * '<S52>'  : 'Traj2/PID Controller/Saturation/Enabled'
 * '<S53>'  : 'Traj2/PID Controller/Saturation Fdbk/Disabled'
 * '<S54>'  : 'Traj2/PID Controller/Sum/Sum_PID'
 * '<S55>'  : 'Traj2/PID Controller/Sum Fdbk/Disabled'
 * '<S56>'  : 'Traj2/PID Controller/Tracking Mode/Disabled'
 * '<S57>'  : 'Traj2/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S58>'  : 'Traj2/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S59>'  : 'Traj2/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S60>'  : 'Traj2/PID Controller/postSat Signal/Forward_Path'
 * '<S61>'  : 'Traj2/PID Controller/preInt Signal/Internal PreInt'
 * '<S62>'  : 'Traj2/PID Controller/preSat Signal/Forward_Path'
 * '<S63>'  : 'Traj2/PID Controller1/Anti-windup'
 * '<S64>'  : 'Traj2/PID Controller1/D Gain'
 * '<S65>'  : 'Traj2/PID Controller1/External Derivative'
 * '<S66>'  : 'Traj2/PID Controller1/Filter'
 * '<S67>'  : 'Traj2/PID Controller1/Filter ICs'
 * '<S68>'  : 'Traj2/PID Controller1/I Gain'
 * '<S69>'  : 'Traj2/PID Controller1/Ideal P Gain'
 * '<S70>'  : 'Traj2/PID Controller1/Ideal P Gain Fdbk'
 * '<S71>'  : 'Traj2/PID Controller1/Integrator'
 * '<S72>'  : 'Traj2/PID Controller1/Integrator ICs'
 * '<S73>'  : 'Traj2/PID Controller1/N Copy'
 * '<S74>'  : 'Traj2/PID Controller1/N Gain'
 * '<S75>'  : 'Traj2/PID Controller1/P Copy'
 * '<S76>'  : 'Traj2/PID Controller1/Parallel P Gain'
 * '<S77>'  : 'Traj2/PID Controller1/Reset Signal'
 * '<S78>'  : 'Traj2/PID Controller1/Saturation'
 * '<S79>'  : 'Traj2/PID Controller1/Saturation Fdbk'
 * '<S80>'  : 'Traj2/PID Controller1/Sum'
 * '<S81>'  : 'Traj2/PID Controller1/Sum Fdbk'
 * '<S82>'  : 'Traj2/PID Controller1/Tracking Mode'
 * '<S83>'  : 'Traj2/PID Controller1/Tracking Mode Sum'
 * '<S84>'  : 'Traj2/PID Controller1/Tsamp - Integral'
 * '<S85>'  : 'Traj2/PID Controller1/Tsamp - Ngain'
 * '<S86>'  : 'Traj2/PID Controller1/postSat Signal'
 * '<S87>'  : 'Traj2/PID Controller1/preInt Signal'
 * '<S88>'  : 'Traj2/PID Controller1/preSat Signal'
 * '<S89>'  : 'Traj2/PID Controller1/Anti-windup/Cont. Clamping Parallel'
 * '<S90>'  : 'Traj2/PID Controller1/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S91>'  : 'Traj2/PID Controller1/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S92>'  : 'Traj2/PID Controller1/D Gain/Internal Parameters'
 * '<S93>'  : 'Traj2/PID Controller1/External Derivative/Error'
 * '<S94>'  : 'Traj2/PID Controller1/Filter/Cont. Filter'
 * '<S95>'  : 'Traj2/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S96>'  : 'Traj2/PID Controller1/I Gain/Internal Parameters'
 * '<S97>'  : 'Traj2/PID Controller1/Ideal P Gain/Passthrough'
 * '<S98>'  : 'Traj2/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S99>'  : 'Traj2/PID Controller1/Integrator/Continuous'
 * '<S100>' : 'Traj2/PID Controller1/Integrator ICs/Internal IC'
 * '<S101>' : 'Traj2/PID Controller1/N Copy/Disabled'
 * '<S102>' : 'Traj2/PID Controller1/N Gain/Internal Parameters'
 * '<S103>' : 'Traj2/PID Controller1/P Copy/Disabled'
 * '<S104>' : 'Traj2/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S105>' : 'Traj2/PID Controller1/Reset Signal/Disabled'
 * '<S106>' : 'Traj2/PID Controller1/Saturation/Enabled'
 * '<S107>' : 'Traj2/PID Controller1/Saturation Fdbk/Disabled'
 * '<S108>' : 'Traj2/PID Controller1/Sum/Sum_PID'
 * '<S109>' : 'Traj2/PID Controller1/Sum Fdbk/Disabled'
 * '<S110>' : 'Traj2/PID Controller1/Tracking Mode/Disabled'
 * '<S111>' : 'Traj2/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S112>' : 'Traj2/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S113>' : 'Traj2/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S114>' : 'Traj2/PID Controller1/postSat Signal/Forward_Path'
 * '<S115>' : 'Traj2/PID Controller1/preInt Signal/Internal PreInt'
 * '<S116>' : 'Traj2/PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* Traj2_h_ */
