/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ReferenceGeneratorxLQRCombinedxPWMGenerator.h
 *
 * Code generated for Simulink model 'ReferenceGeneratorxLQRCombinedxPWMGenerator'.
 *
 * Model version                  : 1.30
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sun May 31 04:40:19 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef ReferenceGeneratorxLQRCombinedxPWMGenerator_h_
#define ReferenceGeneratorxLQRCombinedxPWMGenerator_h_
#ifndef ReferenceGeneratorxLQRCombinedxPWMGenerator_COMMON_INCLUDES_
#define ReferenceGeneratorxLQRCombinedxPWMGenerator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif        /* ReferenceGeneratorxLQRCombinedxPWMGenerator_COMMON_INCLUDES_ */

#include <stddef.h>
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define ReferenceGeneratorxLQRCombinedxPWMGenerator_M (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* External inputs (root inport signals with default storage) */
typedef struct {
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

/* Model entry point functions */
extern void ReferenceGeneratorxLQRCombinedxPWMGenerator_initialize(void);
extern void ReferenceGeneratorxLQRCombinedxPWMGenerator_step(void);

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
 * '<Root>' : 'ReferenceGeneratorxLQRCombinedxPWMGenerator'
 * '<S1>'   : 'ReferenceGeneratorxLQRCombinedxPWMGenerator/CombinedLQR'
 * '<S2>'   : 'ReferenceGeneratorxLQRCombinedxPWMGenerator/MATLAB Function'
 * '<S3>'   : 'ReferenceGeneratorxLQRCombinedxPWMGenerator/MATLAB Function1'
 */
#endif                      /* ReferenceGeneratorxLQRCombinedxPWMGenerator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
