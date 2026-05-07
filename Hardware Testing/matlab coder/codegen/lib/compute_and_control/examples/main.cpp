//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// main.cpp
//
// Code generation for function 'main'
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
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

// Include files
#include "main.h"
#include "BipedController.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_1x2_real_T(double result[2]);

static void argInit_6x1_real_T(double result[6]);

static double argInit_real_T();

// Function Definitions
static void argInit_1x2_real_T(double result[2])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

static void argInit_6x1_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 6; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

int main(int, char **)
{
  BipedController *classInstance;
  classInstance = new BipedController;
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_compute_and_control(classInstance);
  delete classInstance;
  return 0;
}

void main_compute_and_control(BipedController *instancePtr)
{
  double K[12];
  double dv[6];
  double dv1[6];
  double dv2[2];
  double L_tmp;
  double pwm_L;
  double pwm_R;
  // Initialize function 'compute_and_control' input arguments.
  // Initialize function input argument 'state'.
  L_tmp = argInit_real_T();
  // Initialize function input argument 'Q_diag'.
  // Initialize function input argument 'R_diag'.
  // Call the entry-point 'compute_and_control'.
  argInit_6x1_real_T(dv);
  argInit_6x1_real_T(dv1);
  argInit_1x2_real_T(dv2);
  instancePtr->compute_and_control(dv, L_tmp, L_tmp, L_tmp, L_tmp, L_tmp, L_tmp,
                                   L_tmp, L_tmp, dv1, dv2, L_tmp, L_tmp, &pwm_R,
                                   &pwm_L, K);
}

// End of code generation (main.cpp)
