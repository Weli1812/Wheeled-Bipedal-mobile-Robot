/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Pure_Pursuit.h
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

#ifndef Pure_Pursuit_h_
#define Pure_Pursuit_h_
#ifndef Pure_Pursuit_COMMON_INCLUDES_
#define Pure_Pursuit_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* Pure_Pursuit_COMMON_INCLUDES_ */

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

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

#define Pure_Pursuit_M                 (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap
#define typedef_cell_wrap

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap;

#endif                                 /* typedef_cell_wrap */

#ifndef struct_tag_0oC10SALEMHxqV7kttu3wG
#define struct_tag_0oC10SALEMHxqV7kttu3wG

struct tag_0oC10SALEMHxqV7kttu3wG
{
  int32_T isInitialized;
  cell_wrap inputVarSize[2];
  real_T MaxAngularVelocity;
  real_T LookaheadDistance;
  real_T DesiredLinearVelocity;
  real_T ProjectionPoint[2];
  real_T ProjectionLineIndex;
  real_T LookaheadPoint[2];
  real_T LastPose[3];
  real_T WaypointsInternal[24];
};

#endif                                 /* struct_tag_0oC10SALEMHxqV7kttu3wG */

#ifndef typedef_nav_slalgs_internal_PurePursuit
#define typedef_nav_slalgs_internal_PurePursuit

typedef struct tag_0oC10SALEMHxqV7kttu3wG nav_slalgs_internal_PurePursuit;

#endif                             /* typedef_nav_slalgs_internal_PurePursuit */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  nav_slalgs_internal_PurePursuit obj; /* '<Root>/Pure Pursuit' */
  real_T FilterCoefficient;            /* '<S40>/Filter Coefficient' */
  real_T IntegralGain;                 /* '<S34>/Integral Gain' */
  real_T FilterCoefficient_m;          /* '<S92>/Filter Coefficient' */
  real_T IntegralGain_h;               /* '<S86>/Integral Gain' */
  real_T Product;                      /* '<Root>/Product' */
  real_T Product1;                     /* '<Root>/Product1' */
  real_T wactual;                      /* '<Root>/Gain3' */
} DW;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
  real_T Integrator4_CSTATE;           /* '<Root>/Integrator4' */
  real_T Integrator_CSTATE_m;          /* '<S37>/Integrator' */
  real_T Filter_CSTATE;                /* '<S32>/Filter' */
  real_T Integrator_CSTATE_n;          /* '<S89>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S84>/Filter' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator3_CSTATE;           /* '<Root>/Integrator3' */
  real_T Motor1_CSTATE;                /* '<Root>/Motor1' */
} X;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE;            /* '<Root>/Integrator' */
  real_T Integrator2_CSTATE;           /* '<Root>/Integrator2' */
  real_T Integrator4_CSTATE;           /* '<Root>/Integrator4' */
  real_T Integrator_CSTATE_m;          /* '<S37>/Integrator' */
  real_T Filter_CSTATE;                /* '<S32>/Filter' */
  real_T Integrator_CSTATE_n;          /* '<S89>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S84>/Filter' */
  real_T Integrator1_CSTATE;           /* '<Root>/Integrator1' */
  real_T Integrator3_CSTATE;           /* '<Root>/Integrator3' */
  real_T Motor1_CSTATE;                /* '<Root>/Motor1' */
} XDot;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE;         /* '<Root>/Integrator' */
  boolean_T Integrator2_CSTATE;        /* '<Root>/Integrator2' */
  boolean_T Integrator4_CSTATE;        /* '<Root>/Integrator4' */
  boolean_T Integrator_CSTATE_m;       /* '<S37>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S32>/Filter' */
  boolean_T Integrator_CSTATE_n;       /* '<S89>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S84>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<Root>/Integrator1' */
  boolean_T Integrator3_CSTATE;        /* '<Root>/Integrator3' */
  boolean_T Motor1_CSTATE;             /* '<Root>/Motor1' */
} XDis;

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
  real_T Constant_Value[24];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Vlactual;                     /* '<Root>/encoder Vi-actual' */
  real_T Vractual;                     /* '<Root>/encoder Vr-actual' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T MotorVrControlsignal;         /* '<Root>/Motor Vr Control signal' */
  real_T MotorViControlsignal;         /* '<Root>/Motor Vi Control signal' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[10];
  real_T odeF[3][10];
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
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Continuous states (default storage) */
extern X rtX;

/* Disabled states (default storage) */
extern XDis rtXDis;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Pure_Pursuit_initialize(void);
extern void Pure_Pursuit_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/To Workspace' : Unused code path elimination
 * Block '<Root>/To Workspace1' : Unused code path elimination
 * Block '<Root>/To Workspace2' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope4' : Unused code path elimination
 */

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
 * '<Root>' : 'Pure_Pursuit'
 * '<S1>'   : 'Pure_Pursuit/PID Controller'
 * '<S2>'   : 'Pure_Pursuit/PID Controller1'
 * '<S3>'   : 'Pure_Pursuit/PID Controller/Anti-windup'
 * '<S4>'   : 'Pure_Pursuit/PID Controller/D Gain'
 * '<S5>'   : 'Pure_Pursuit/PID Controller/External Derivative'
 * '<S6>'   : 'Pure_Pursuit/PID Controller/Filter'
 * '<S7>'   : 'Pure_Pursuit/PID Controller/Filter ICs'
 * '<S8>'   : 'Pure_Pursuit/PID Controller/I Gain'
 * '<S9>'   : 'Pure_Pursuit/PID Controller/Ideal P Gain'
 * '<S10>'  : 'Pure_Pursuit/PID Controller/Ideal P Gain Fdbk'
 * '<S11>'  : 'Pure_Pursuit/PID Controller/Integrator'
 * '<S12>'  : 'Pure_Pursuit/PID Controller/Integrator ICs'
 * '<S13>'  : 'Pure_Pursuit/PID Controller/N Copy'
 * '<S14>'  : 'Pure_Pursuit/PID Controller/N Gain'
 * '<S15>'  : 'Pure_Pursuit/PID Controller/P Copy'
 * '<S16>'  : 'Pure_Pursuit/PID Controller/Parallel P Gain'
 * '<S17>'  : 'Pure_Pursuit/PID Controller/Reset Signal'
 * '<S18>'  : 'Pure_Pursuit/PID Controller/Saturation'
 * '<S19>'  : 'Pure_Pursuit/PID Controller/Saturation Fdbk'
 * '<S20>'  : 'Pure_Pursuit/PID Controller/Sum'
 * '<S21>'  : 'Pure_Pursuit/PID Controller/Sum Fdbk'
 * '<S22>'  : 'Pure_Pursuit/PID Controller/Tracking Mode'
 * '<S23>'  : 'Pure_Pursuit/PID Controller/Tracking Mode Sum'
 * '<S24>'  : 'Pure_Pursuit/PID Controller/Tsamp - Integral'
 * '<S25>'  : 'Pure_Pursuit/PID Controller/Tsamp - Ngain'
 * '<S26>'  : 'Pure_Pursuit/PID Controller/postSat Signal'
 * '<S27>'  : 'Pure_Pursuit/PID Controller/preInt Signal'
 * '<S28>'  : 'Pure_Pursuit/PID Controller/preSat Signal'
 * '<S29>'  : 'Pure_Pursuit/PID Controller/Anti-windup/Passthrough'
 * '<S30>'  : 'Pure_Pursuit/PID Controller/D Gain/Internal Parameters'
 * '<S31>'  : 'Pure_Pursuit/PID Controller/External Derivative/Error'
 * '<S32>'  : 'Pure_Pursuit/PID Controller/Filter/Cont. Filter'
 * '<S33>'  : 'Pure_Pursuit/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S34>'  : 'Pure_Pursuit/PID Controller/I Gain/Internal Parameters'
 * '<S35>'  : 'Pure_Pursuit/PID Controller/Ideal P Gain/Passthrough'
 * '<S36>'  : 'Pure_Pursuit/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S37>'  : 'Pure_Pursuit/PID Controller/Integrator/Continuous'
 * '<S38>'  : 'Pure_Pursuit/PID Controller/Integrator ICs/Internal IC'
 * '<S39>'  : 'Pure_Pursuit/PID Controller/N Copy/Disabled'
 * '<S40>'  : 'Pure_Pursuit/PID Controller/N Gain/Internal Parameters'
 * '<S41>'  : 'Pure_Pursuit/PID Controller/P Copy/Disabled'
 * '<S42>'  : 'Pure_Pursuit/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S43>'  : 'Pure_Pursuit/PID Controller/Reset Signal/Disabled'
 * '<S44>'  : 'Pure_Pursuit/PID Controller/Saturation/Passthrough'
 * '<S45>'  : 'Pure_Pursuit/PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'Pure_Pursuit/PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'Pure_Pursuit/PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'Pure_Pursuit/PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'Pure_Pursuit/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'Pure_Pursuit/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'Pure_Pursuit/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'Pure_Pursuit/PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'Pure_Pursuit/PID Controller/preInt Signal/Internal PreInt'
 * '<S54>'  : 'Pure_Pursuit/PID Controller/preSat Signal/Forward_Path'
 * '<S55>'  : 'Pure_Pursuit/PID Controller1/Anti-windup'
 * '<S56>'  : 'Pure_Pursuit/PID Controller1/D Gain'
 * '<S57>'  : 'Pure_Pursuit/PID Controller1/External Derivative'
 * '<S58>'  : 'Pure_Pursuit/PID Controller1/Filter'
 * '<S59>'  : 'Pure_Pursuit/PID Controller1/Filter ICs'
 * '<S60>'  : 'Pure_Pursuit/PID Controller1/I Gain'
 * '<S61>'  : 'Pure_Pursuit/PID Controller1/Ideal P Gain'
 * '<S62>'  : 'Pure_Pursuit/PID Controller1/Ideal P Gain Fdbk'
 * '<S63>'  : 'Pure_Pursuit/PID Controller1/Integrator'
 * '<S64>'  : 'Pure_Pursuit/PID Controller1/Integrator ICs'
 * '<S65>'  : 'Pure_Pursuit/PID Controller1/N Copy'
 * '<S66>'  : 'Pure_Pursuit/PID Controller1/N Gain'
 * '<S67>'  : 'Pure_Pursuit/PID Controller1/P Copy'
 * '<S68>'  : 'Pure_Pursuit/PID Controller1/Parallel P Gain'
 * '<S69>'  : 'Pure_Pursuit/PID Controller1/Reset Signal'
 * '<S70>'  : 'Pure_Pursuit/PID Controller1/Saturation'
 * '<S71>'  : 'Pure_Pursuit/PID Controller1/Saturation Fdbk'
 * '<S72>'  : 'Pure_Pursuit/PID Controller1/Sum'
 * '<S73>'  : 'Pure_Pursuit/PID Controller1/Sum Fdbk'
 * '<S74>'  : 'Pure_Pursuit/PID Controller1/Tracking Mode'
 * '<S75>'  : 'Pure_Pursuit/PID Controller1/Tracking Mode Sum'
 * '<S76>'  : 'Pure_Pursuit/PID Controller1/Tsamp - Integral'
 * '<S77>'  : 'Pure_Pursuit/PID Controller1/Tsamp - Ngain'
 * '<S78>'  : 'Pure_Pursuit/PID Controller1/postSat Signal'
 * '<S79>'  : 'Pure_Pursuit/PID Controller1/preInt Signal'
 * '<S80>'  : 'Pure_Pursuit/PID Controller1/preSat Signal'
 * '<S81>'  : 'Pure_Pursuit/PID Controller1/Anti-windup/Passthrough'
 * '<S82>'  : 'Pure_Pursuit/PID Controller1/D Gain/Internal Parameters'
 * '<S83>'  : 'Pure_Pursuit/PID Controller1/External Derivative/Error'
 * '<S84>'  : 'Pure_Pursuit/PID Controller1/Filter/Cont. Filter'
 * '<S85>'  : 'Pure_Pursuit/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S86>'  : 'Pure_Pursuit/PID Controller1/I Gain/Internal Parameters'
 * '<S87>'  : 'Pure_Pursuit/PID Controller1/Ideal P Gain/Passthrough'
 * '<S88>'  : 'Pure_Pursuit/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S89>'  : 'Pure_Pursuit/PID Controller1/Integrator/Continuous'
 * '<S90>'  : 'Pure_Pursuit/PID Controller1/Integrator ICs/Internal IC'
 * '<S91>'  : 'Pure_Pursuit/PID Controller1/N Copy/Disabled'
 * '<S92>'  : 'Pure_Pursuit/PID Controller1/N Gain/Internal Parameters'
 * '<S93>'  : 'Pure_Pursuit/PID Controller1/P Copy/Disabled'
 * '<S94>'  : 'Pure_Pursuit/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S95>'  : 'Pure_Pursuit/PID Controller1/Reset Signal/Disabled'
 * '<S96>'  : 'Pure_Pursuit/PID Controller1/Saturation/Passthrough'
 * '<S97>'  : 'Pure_Pursuit/PID Controller1/Saturation Fdbk/Disabled'
 * '<S98>'  : 'Pure_Pursuit/PID Controller1/Sum/Sum_PID'
 * '<S99>'  : 'Pure_Pursuit/PID Controller1/Sum Fdbk/Disabled'
 * '<S100>' : 'Pure_Pursuit/PID Controller1/Tracking Mode/Disabled'
 * '<S101>' : 'Pure_Pursuit/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S102>' : 'Pure_Pursuit/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S103>' : 'Pure_Pursuit/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S104>' : 'Pure_Pursuit/PID Controller1/postSat Signal/Forward_Path'
 * '<S105>' : 'Pure_Pursuit/PID Controller1/preInt Signal/Internal PreInt'
 * '<S106>' : 'Pure_Pursuit/PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* Pure_Pursuit_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
