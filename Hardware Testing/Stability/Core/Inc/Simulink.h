/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Simulink.h
 *
 * Code generated for Simulink model 'Simulink'.
 *
 * Model version                  : 1.0
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu May 14 02:49:46 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef Simulink_h_
#define Simulink_h_
#ifndef Simulink_COMMON_INCLUDES_
#define Simulink_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* Simulink_COMMON_INCLUDES_ */

#include <stddef.h>
// #include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define Simulink_M                     (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T state_x[6];                   /* '<Root>/state_x' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T RPWM_R;                       /* '<Root>/RPWM_R' */
  real_T RPWM_R1;                      /* '<Root>/RPWM_R1' */
  real_T RPWM_R2;                      /* '<Root>/RPWM_R2' */
  real_T RPWM_R3;                      /* '<Root>/RPWM_R3' */
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
extern void Simulink_initialize(void);
extern void Simulink_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

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
 * '<Root>' : 'Simulink'
 * '<S1>'   : 'Simulink/MATLAB Function'
 */
#endif                                 /* Simulink_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
