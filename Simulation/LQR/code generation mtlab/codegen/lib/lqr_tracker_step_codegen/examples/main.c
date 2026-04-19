/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 19-Apr-2026 13:12:28
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "lqr_tracker_step_codegen.h"
#include "lqr_tracker_step_codegen_emxAPI.h"
#include "lqr_tracker_step_codegen_initialize.h"
#include "lqr_tracker_step_codegen_terminate.h"
#include "lqr_tracker_step_codegen_types.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);

static void argInit_2x3_real_T(double result[6]);

static int argInit_d5000x1_real_T(double result_data[]);

static emxArray_real_T *argInit_d5000x2_real_T(void);

static double argInit_real_T(void);

/* Function Definitions */
/*
 * Arguments    : double result[3]
 * Return Type  : void
 */
static void argInit_1x3_real_T(double result[3])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 3; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

/*
 * Arguments    : double result[6]
 * Return Type  : void
 */
static void argInit_2x3_real_T(double result[6])
{
  int i;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 6; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[i] = argInit_real_T();
  }
}

/*
 * Arguments    : double result_data[]
 * Return Type  : int
 */
static int argInit_d5000x1_real_T(double result_data[])
{
  int idx0;
  int result_size;
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size = 2;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result_data[idx0] = argInit_real_T();
  }
  return result_size;
}

/*
 * Arguments    : void
 * Return Type  : emxArray_real_T *
 */
static emxArray_real_T *argInit_d5000x2_real_T(void)
{
  emxArray_real_T *result;
  double *result_data;
  int idx0;
  int idx1;
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result = emxCreate_real_T(2, 2);
  result_data = result->data;
  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0++) {
    for (idx1 = 0; idx1 < 2; idx1++) {
      /* Set the value of the array element.
Change this value to the value that the application requires. */
      result_data[idx0 + result->size[0] * idx1] = argInit_real_T();
    }
  }
  return result;
}

/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* Initialize the application.
You do not need to do this more than one time. */
  lqr_tracker_step_codegen_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_lqr_tracker_step_codegen();
  /* Terminate the application.
You do not need to do this more than one time. */
  lqr_tracker_step_codegen_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_lqr_tracker_step_codegen(void)
{
  emxArray_real_T *P;
  double theta_ref_data[5000];
  double vd_ref_data[5000];
  double wd_ref_data[5000];
  double dv[6];
  double dv1[3];
  double v_cmd;
  double w_cmd;
  double x_tmp;
  int theta_ref_size;
  boolean_T done;
  /* Initialize function 'lqr_tracker_step_codegen' input arguments. */
  x_tmp = argInit_real_T();
  /* Initialize function input argument 'P'. */
  P = argInit_d5000x2_real_T();
  /* Initialize function input argument 'theta_ref'. */
  argInit_d5000x1_real_T(theta_ref_data);
  /* Initialize function input argument 'vd_ref'. */
  argInit_d5000x1_real_T(vd_ref_data);
  /* Initialize function input argument 'wd_ref'. */
  argInit_d5000x1_real_T(wd_ref_data);
  /* Initialize function input argument 'K'. */
  /* Initialize function input argument 'goalPose'. */
  /* Call the entry-point 'lqr_tracker_step_codegen'. */
  argInit_2x3_real_T(dv);
  argInit_1x3_real_T(dv1);
  lqr_tracker_step_codegen(x_tmp, x_tmp, x_tmp, P, theta_ref_data,
                           &theta_ref_size, vd_ref_data, &theta_ref_size,
                           wd_ref_data, &theta_ref_size, dv, dv1, x_tmp, x_tmp,
                           &v_cmd, &w_cmd, &done, &theta_ref_size);
  emxDestroyArray_real_T(P);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
