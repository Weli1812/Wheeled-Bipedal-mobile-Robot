#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Traj2.h"
#include "lqr_tracker_step_codegen.h"
#include "lqr_tracker_step_codegen_initialize.h"
#include "lqr_tracker_step_codegen_types.h"

#define CONTROL_PERIOD_MS 50
#define CONTROL_DT_SECONDS (0.05)
#define WHEEL_BASE_M (0.40)

/* Switch to 1 if you want to directly output Simulink model wheel commands. */
#define USE_TRAJ2_OUTPUT 0

/* Internal test model keeps the control loop observable without motor hardware. */
#define USE_INTERNAL_FEEDBACK_MODEL 1

static const char *TAG = "WBMR_ESP";

typedef struct {
  double x;
  double y;
  double theta;
} Pose2D;

static Pose2D s_pose;
static double s_meas_vr;
static double s_meas_vl;
static double s_cmd_vr;
static double s_cmd_vl;

static int s_path_size[2] = {1276, 2};
static const int s_ref_size[1] = {1276};
static emxArray_real_T s_path;
static double s_goal_pose[3];
static double s_lqr_k[6];

__attribute__((weak)) double wbmr_read_vr_actual(void)
{
  return s_meas_vr;
}

__attribute__((weak)) double wbmr_read_vl_actual(void)
{
  return s_meas_vl;
}

__attribute__((weak)) void wbmr_write_wheel_commands(double vr_cmd, double vl_cmd)
{
  (void)vr_cmd;
  (void)vl_cmd;
}

static double wrap_pi(double angle)
{
  const double pi = 3.14159265358979323846;
  const double two_pi = 6.28318530717958647692;

  while (angle > pi) {
    angle -= two_pi;
  }

  while (angle < -pi) {
    angle += two_pi;
  }

  return angle;
}

static void vw_to_wheels(double v_cmd, double w_cmd, double *vr_cmd,
  double *vl_cmd)
{
  *vr_cmd = v_cmd + (0.5 * WHEEL_BASE_M * w_cmd);
  *vl_cmd = v_cmd - (0.5 * WHEEL_BASE_M * w_cmd);
}

static void integrate_pose_from_wheels(double vr_actual, double vl_actual,
  double dt)
{
  double v = 0.5 * (vr_actual + vl_actual);
  double w = (vr_actual - vl_actual) / WHEEL_BASE_M;

  s_pose.theta = wrap_pi(s_pose.theta + (w * dt));
  s_pose.x += cos(s_pose.theta) * v * dt;
  s_pose.y += sin(s_pose.theta) * v * dt;
}

static void init_controller_bindings(void)
{
  int i;

  s_path.data = (double *)&Traj2_ConstP.Constant_Value[0];
  s_path.size = s_path_size;
  s_path.allocatedSize = 2552;
  s_path.numDimensions = 2;
  s_path.canFreeData = false;

  s_goal_pose[0] = Traj2_ConstP.Constant_Value[1275];
  s_goal_pose[1] = Traj2_ConstP.Constant_Value[1275 + 1276];
  s_goal_pose[2] = Traj2_ConstP.Constant1_Value[1275];

  /* MATLAB LQR function expects +K and internally multiplies by -1. */
  for (i = 0; i < 6; ++i) {
    s_lqr_k[i] = -Traj2_ConstP.Gain2_Gain[i];
  }
}

static void warm_start_states(void)
{
  s_pose.x = Traj2_X.Integrator_CSTATE;
  s_pose.y = Traj2_X.Integrator1_CSTATE;
  s_pose.theta = Traj2_X.Integrator2_CSTATE;

  s_meas_vr = 0.0;
  s_meas_vl = 0.0;
  s_cmd_vr = 0.0;
  s_cmd_vl = 0.0;
}

static void run_control_iteration(uint32_t loop_idx)
{
  double vr_actual = wbmr_read_vr_actual();
  double vl_actual = wbmr_read_vl_actual();
  double v_cmd = 0.0;
  double w_cmd = 0.0;
  double vr_lqr = 0.0;
  double vl_lqr = 0.0;
  boolean_T done = false;
  int idx = 1;

  Traj2_U.vr_actual = vr_actual;
  Traj2_U.vl_actual = vl_actual;
  Traj2_step();

  lqr_tracker_step_codegen(s_pose.x, s_pose.y, s_pose.theta, &s_path,
    Traj2_ConstP.Constant1_Value, s_ref_size,
    Traj2_ConstP.Constant2_Value, s_ref_size,
    Traj2_ConstP.Constant3_Value, s_ref_size,
    s_lqr_k, s_goal_pose, 0.08, 0.08,
    &v_cmd, &w_cmd, &done, &idx);

  vw_to_wheels(v_cmd, w_cmd, &vr_lqr, &vl_lqr);

#if USE_TRAJ2_OUTPUT
  s_cmd_vr = Traj2_Y.Out1;
  s_cmd_vl = Traj2_Y.Out2;
#else
  s_cmd_vr = vr_lqr;
  s_cmd_vl = vl_lqr;
#endif

  wbmr_write_wheel_commands(s_cmd_vr, s_cmd_vl);

#if USE_INTERNAL_FEEDBACK_MODEL
  {
    const double alpha = 0.25;
    s_meas_vr += alpha * (s_cmd_vr - s_meas_vr);
    s_meas_vl += alpha * (s_cmd_vl - s_meas_vl);
  }
#endif

  integrate_pose_from_wheels(s_meas_vr, s_meas_vl, CONTROL_DT_SECONDS);

  if ((loop_idx % 20U) == 0U) {
    ESP_LOGI(TAG,
      "idx=%d done=%d pose=(%.3f, %.3f, %.3f) lqr_vw=(%.3f, %.3f) cmd_wheels=(%.3f, %.3f)",
      idx, (int)done, s_pose.x, s_pose.y, s_pose.theta, v_cmd, w_cmd,
      s_cmd_vr, s_cmd_vl);
  }
}

void app_main(void)
{
  uint32_t loop_idx = 0;

  Traj2_initialize();
  lqr_tracker_step_codegen_initialize();

  init_controller_bindings();
  warm_start_states();

  ESP_LOGI(TAG, "Controller ready. Upload once, then it starts automatically.");

  while (1) {
    run_control_iteration(loop_idx++);
    vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}