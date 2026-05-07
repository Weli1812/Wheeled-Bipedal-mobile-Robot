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
#include "LQRwheels.h"
#include "LQRwheels_terminate.h"
#include "rt_nonfinite.h"

// Function Declarations
static double argInit_real_T();

// Function Definitions
static double argInit_real_T()
{
  return 0.0;
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_LQRwheels();
  // Terminate the application.
  // You do not need to do this more than one time.
  LQRwheels_terminate();
  return 0;
}

void main_LQRwheels()
{
  double pwml;
  double pwmr;
  boolean_T dirl;
  boolean_T dirr;
  // Initialize function 'LQRwheels' input arguments.
  pwmr = argInit_real_T();
  // Call the entry-point 'LQRwheels'.
  LQRwheels(pwmr, pwmr, pwmr, pwmr, pwmr, pwmr, pwmr, pwmr, pwmr, pwmr, pwmr,
            &pwmr, &pwml, &dirr, &dirl);
}

// End of code generation (main.cpp)
