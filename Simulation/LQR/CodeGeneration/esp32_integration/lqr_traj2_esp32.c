#include "lqr_traj2_esp32.h"

#include <string.h>

#include "../codegen/lib/robot_controller/robot_controller.h"

static double s_refPathData[LQR_TRAJ2_REF_LEN * 3];
static int s_refPathSize[2] = {0, 3};
static double s_vdRefData[LQR_TRAJ2_REF_LEN];
static int s_vdRefSize[1] = {0};
static double s_wdRefData[LQR_TRAJ2_REF_LEN];
static int s_wdRefSize[1] = {0};
static int s_initialized = 0;

static int clamp_ref_len(int n)
{
  if (n < 1) {
    return 1;
  }
  if (n > LQR_TRAJ2_REF_LEN) {
    return LQR_TRAJ2_REF_LEN;
  }
  return n;
}

void lqr_traj2_init(const double *refPathN3,
                    int numPoints,
                    const double *vdRef,
                    const double *wdRef,
                    const double initialPose[3])
{
  int i;
  int n;

  n = clamp_ref_len(numPoints);
  s_refPathSize[0] = n;
  s_vdRefSize[0] = n;
  s_wdRefSize[0] = n;

  memset(s_refPathData, 0, sizeof(s_refPathData));
  memset(s_vdRefData, 0, sizeof(s_vdRefData));
  memset(s_wdRefData, 0, sizeof(s_wdRefData));

  for (i = 0; i < n; ++i) {
    const double x = refPathN3[i * 3 + 0];
    const double y = refPathN3[i * 3 + 1];
    const double theta = refPathN3[i * 3 + 2];

    s_refPathData[i] = x;
    s_refPathData[i + n] = y;
    s_refPathData[i + 2 * n] = theta;
    s_vdRefData[i] = vdRef[i];
    s_wdRefData[i] = wdRef[i];

    Traj2_P.P[i] = x;
    Traj2_P.P[LQR_TRAJ2_REF_LEN + i] = y;
    Traj2_P.theta_ref_unwrapped[i] = theta;
    Traj2_P.vd_ref[i] = vdRef[i];
    Traj2_P.wd_ref[i] = wdRef[i];
  }

  for (i = n; i < LQR_TRAJ2_REF_LEN; ++i) {
    Traj2_P.P[i] = Traj2_P.P[n - 1];
    Traj2_P.P[LQR_TRAJ2_REF_LEN + i] = Traj2_P.P[LQR_TRAJ2_REF_LEN + n - 1];
    Traj2_P.theta_ref_unwrapped[i] = Traj2_P.theta_ref_unwrapped[n - 1];
    Traj2_P.vd_ref[i] = 0.0;
    Traj2_P.wd_ref[i] = 0.0;
  }

  Traj2_P.x0 = initialPose[0];
  Traj2_P.y0 = initialPose[1];
  Traj2_P.theta0 = initialPose[2];

  Traj2_initialize();
  s_initialized = 1;
}

void lqr_traj2_step(double vrActual,
                    double vlActual,
                    const double currentPose[3],
                    uint32_t stepIndex0,
                    LqrTraj2Output *out)
{
  double v_cmd = 0.0;
  double w_cmd = 0.0;
  boolean_T reached = 0;
  double stepIndex1 = (double)stepIndex0 + 1.0;

  if (!out) {
    return;
  }

  if (!s_initialized) {
    memset(out, 0, sizeof(*out));
    return;
  }

  robot_controller(currentPose, s_refPathData, s_refPathSize, s_vdRefData,
                   s_vdRefSize, s_wdRefData, s_wdRefSize, stepIndex1, &v_cmd,
                   &w_cmd, &reached);

  Traj2_U.vr_actual = vrActual;
  Traj2_U.vl_actual = vlActual;

  /* Use externally estimated pose from localization at every control step. */
  Traj2_X.Integrator_CSTATE = currentPose[0];
  Traj2_X.Integrator1_CSTATE = currentPose[1];
  Traj2_X.Integrator2_CSTATE = currentPose[2];

  Traj2_step();

  out->v_cmd = v_cmd;
  out->w_cmd = w_cmd;
  out->reached = reached;
  out->dirr = Traj2_Y.dirr;
  out->dirl = Traj2_Y.dirl;
  out->pwmr = Traj2_Y.pwmr;
  out->pwml = Traj2_Y.pwml;
}

void lqr_traj2_terminate(void)
{
  if (s_initialized) {
    Traj2_terminate();
    s_initialized = 0;
  }
}
