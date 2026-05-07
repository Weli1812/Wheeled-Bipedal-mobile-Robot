/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: LQRxSTM.h
 *
 * Code generated for Simulink model 'LQRxSTM'.
 *
 * Model version                  : 1.27
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu May  7 18:37:17 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef LQRxSTM_h_
#define LQRxSTM_h_
#ifndef LQRxSTM_COMMON_INCLUDES_
#define LQRxSTM_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* LQRxSTM_COMMON_INCLUDES_ */

#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define LQRxSTM_M                      (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: [-16.616882069954595	-1.118033988749899	3.162277660168384	-2.938330914554923	-2.061355185548641	1.622468529505869;
     -16.616882069954599	-1.118033988749900	-3.162277660168376	-2.938330914554922	-2.061355185548644	-1.622468529505866]
   * Referenced by: '<Root>/K_lqr'
   */
  real_T K_lqr_Value[12];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T P_b[200];                     /* '<Root>/P' */
  real_T theta_ref_unwrapped[100];     /* '<Root>/theta_ref_unwrapped' */
  real_T vd_path[100];                 /* '<Root>/vd_path' */
  real_T wd_path[100];                 /* '<Root>/wd_path' */
  real_T S_path[100];                  /* '<Root>/S_path' */
  real_T phi;                          /* '<Root>/phi' */
  real_T s;                            /* '<Root>/s' */
  real_T theta;                        /* '<Root>/theta' */
  real_T phi_dot;                      /* '<Root>/phi_dot' */
  real_T v;                            /* '<Root>/v' */
  real_T omega;                        /* '<Root>/omega' */
  real_T x_robot;                      /* '<Root>/x_robot' */
  real_T y_robot;                      /* '<Root>/y_robot' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T RPWM_R;                       /* '<Root>/RPWM_R' */
  real_T LPWM_R;                       /* '<Root>/LPWM_R' */
  real_T RPWM_L;                       /* '<Root>/RPWM_L' */
  real_T LPWM_L;                       /* '<Root>/LPWM_L' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void LQRxSTM_initialize(void);
extern void LQRxSTM_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
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
 * '<Root>' : 'LQRxSTM'
 * '<S1>'   : 'LQRxSTM/CombinedLQR'
 * '<S2>'   : 'LQRxSTM/MATLAB Function'
 * '<S3>'   : 'LQRxSTM/MATLAB Function1'
 */
#endif                                 /* LQRxSTM_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
