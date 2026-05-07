/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 25.2
 * C/C++ source code generated on  : 20-Apr-2026 23:37:54
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
#include "robot_controller.h"
#include "robot_controller_initialize.h"
#include "robot_controller_terminate.h"

/* Function Declarations */
static void argInit_1x3_real_T(double result[3]);

static int argInit_d2000x1_real_T(double result_data[]);

static void argInit_d2000x3_real_T(double result_data[], int result_size[2]);

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
 * Arguments    : double result_data[]
 * Return Type  : int
 */
static int argInit_d2000x1_real_T(double result_data[])
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
 * Arguments    : double result_data[]
 *                int result_size[2]
 * Return Type  : void
 */
static void argInit_d2000x3_real_T(double result_data[], int result_size[2])
{
  int i;
  /* Set the size of the array.
Change this size to the value that the application requires. */
  result_size[0] = 2;
  result_size[1] = 3;
  /* Loop over the array to initialize each element. */
  for (i = 0; i < 6; i++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result_data[i] = argInit_real_T();
  }
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
  robot_controller_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_robot_controller();
  /* Terminate the application.
You do not need to do this more than one time. */
  robot_controller_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_robot_controller(void)
{
  double refPath_data[6000];
  double vd_ref_data[2000];
  double dv[3];
  double v_cmd;
  double w_cmd;
  int refPath_size[2];
  int vd_ref_size;
  boolean_T reached;
  /* Initialize function 'robot_controller' input arguments. */
  /* Initialize function input argument 'currentPose'. */
  /* Initialize function input argument 'refPath'. */
  argInit_d2000x3_real_T(refPath_data, refPath_size);
  /* Initialize function input argument 'vd_ref'. */
  argInit_d2000x1_real_T(vd_ref_data);
  /* Initialize function input argument 'wd_ref'. */
  vd_ref_size = argInit_d2000x1_real_T(vd_ref_data);
  /* Call the entry-point 'robot_controller'. */
  argInit_1x3_real_T(dv);
  robot_controller(dv, refPath_data, refPath_size, vd_ref_data, &vd_ref_size,
                   vd_ref_data, &vd_ref_size, argInit_real_T(), &v_cmd, &w_cmd,
                   &reached);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
