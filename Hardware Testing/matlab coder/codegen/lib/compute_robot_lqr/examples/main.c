/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
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

/* Include files */
#include "main.h"
#include "compute_robot_lqr.h"
#include "compute_robot_lqr_initialize.h"
#include "compute_robot_lqr_terminate.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void argInit_1x2_real_T(double result[2]);

static void argInit_1x6_real_T(double result[6]);

static double argInit_real_T(void);

/* Function Definitions */
static void argInit_1x2_real_T(double result[2])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 2; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static void argInit_1x6_real_T(double result[6])
{
  int idx1;
  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < 6; idx1++) {
    /* Set the value of the array element.
Change this value to the value that the application requires. */
    result[idx1] = argInit_real_T();
  }
}

static double argInit_real_T(void)
{
  return 0.0;
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* Initialize the application.
You do not need to do this more than one time. */
  compute_robot_lqr_initialize();
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_compute_robot_lqr();
  /* Terminate the application.
You do not need to do this more than one time. */
  compute_robot_lqr_terminate();
  return 0;
}

void main_compute_robot_lqr(void)
{
  creal_T cl_poles[6];
  double K[12];
  double dv[6];
  double dv1[2];
  double L_tmp;
  /* Initialize function 'compute_robot_lqr' input arguments. */
  L_tmp = argInit_real_T();
  /* Initialize function input argument 'Q_diag'. */
  /* Initialize function input argument 'R_diag'. */
  /* Call the entry-point 'compute_robot_lqr'. */
  argInit_1x6_real_T(dv);
  argInit_1x2_real_T(dv1);
  compute_robot_lqr(L_tmp, L_tmp, L_tmp, L_tmp, L_tmp, L_tmp, L_tmp, L_tmp, dv,
                    dv1, K, cl_poles);
}

/* End of code generation (main.c) */
