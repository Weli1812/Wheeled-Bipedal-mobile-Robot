/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.
 *
 * File: Simulink.c
 */
#include "main.h"
#include "Simulink.h"
#include <math.h>
#include "rtwtypes.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

extern real_T rt_roundd_snf(real_T u);

/* External tuning variables passed from main.c UART DMA */
extern volatile float esp_wheel_speed_r;
extern volatile float esp_wheel_speed_l;

/* Dynamic SMC Gains linked from main.c */
extern volatile float esp_smc_gains[6];
extern volatile float esp_smc_eta;

real_T rt_roundd_snf(real_T u)
{
    real_T y;
    if (fabs(u) < 4.503599627370496E+15) {
        if (u >= 0.5) {
            y = floor(u + 0.5);
        } else if (u > -0.5) {
            y = u * 0.0;
        } else {
            y = ceil(u - 0.5);
        }
    } else {
        y = u;
    }
    return y;
}

/* -----------------------------------------------------------------------
 * PERSISTENT CONTROLLER STATE
 * ----------------------------------------------------------------------- */
static uint32_T last_step_tick = 0;   /* For real dt measurements */

/* -----------------------------------------------------------------------
 * CONTROLLER CONSTRAINTS
 * ----------------------------------------------------------------------- */
#define INT_RESET_RAD   0.35    /* Reset sliding parameters if pitch > ~20 deg */
#define PHI_BOUNDARY    0.08    /* Boundary layer width to limit chattering noise */

/* Model step function */
void Simulink_step(void)
{
    real_T u_idx_0;
    real_T u_idx_1;
    int32_T pwm_l;
    int32_T pwm_r;

    /* ── Measure real dt from HAL tick (milliseconds → seconds) ── */
    uint32_T now     = HAL_GetTick();
    real_T   dt      = (real_T)(now - last_step_tick) * 0.001;
    last_step_tick   = now;

    /* Guard: clamp dt to sane range on first call or timer wrap */
    if (dt <= 0.0 || dt > 0.05) dt = 0.01;   /* Assume 100 Hz fallback */

    // Minimum PWM required to overcome 775 motor static friction.
    const int32_T DEADBAND_OFFSET = 130;

    /* ── SLIDING MODE CONTROL (SMC) IMPLEMENTATION ─────────────────────── */
    real_T phi = rtU.state_x[0];

    /* 1. Calculate the dynamic sliding surface (s = C^T * x) */
    real_T s_surface = 0.0;
    for (int i = 0; i < 6; i++) {
        s_surface += (real_T)esp_smc_gains[i] * rtU.state_x[i];
    }

    /* Guard boundary condition if completely out of bounds */
    if (fabs(phi) > INT_RESET_RAD) {
        s_surface = 0.0;
    }

    /* 2. Boundary Layer Evaluation (Continuous saturation replaces sign function to stop chattering) */
    real_T sat_s = s_surface / PHI_BOUNDARY;
    if (sat_s > 1.0)  sat_s = 1.0;
    if (sat_s < -1.0) sat_s = -1.0;

    /* 3. Sliding Mode Control Output: u = -eta * sat(s / Phi) */
    real_T smc_base_torque = -((real_T)esp_smc_eta * sat_s);

    /* 4. Multi-Input Decoupling/Mixing (Right and Left Outputs)
          Indices 0-4 handle uniform pitch/position mapping, index 5 handles differential steering yaw rate. */
    real_T differential_steering = (real_T)esp_smc_gains[5] * rtU.state_x[5];

    u_idx_0 = smc_base_torque + differential_steering; // Right wheel demand
    u_idx_1 = smc_base_torque - differential_steering; // Left wheel demand

    /* ── VOLTAGE/SPEED TO PWM CONVERSION ───────────────────────────────── */
    /* NFP-42GP-775-EN Motor Parameters */
    const real_T K_3     = 0.125;
    const real_T K_4     = 0.215;
    const real_T max_pwm = 4000;
    const real_T V_batt  = 22.5;

    /* 1. Fetch actual wheel speeds from ESP32 payload */
    real_T omega_wheel_r = (real_T)esp_wheel_speed_r;
    real_T omega_wheel_l = (real_T)esp_wheel_speed_l;

    /* 2. Calculate Voltages (V = I*R + Back_EMF) */
    real_T v_r = (u_idx_0 + K_3 * omega_wheel_r) / K_4;
    real_T v_l = (u_idx_1 + K_3 * omega_wheel_l) / K_4;

    /* 3. Convert to PWM */
    int32_T raw_pwm_r = (int32_T)rt_roundd_snf((v_r / V_batt) * max_pwm);
    int32_T raw_pwm_l = (int32_T)rt_roundd_snf((v_l / V_batt) * max_pwm);

    /* 4. Apply Deadband Compensation */
    if (raw_pwm_r > 0)       raw_pwm_r += DEADBAND_OFFSET;
    else if (raw_pwm_r < 0)  raw_pwm_r -= DEADBAND_OFFSET;

    if (raw_pwm_l > 0)       raw_pwm_l += DEADBAND_OFFSET;
    else if (raw_pwm_l < 0)  raw_pwm_l -= DEADBAND_OFFSET;

    /* 5. Clamp safely to hardware limits */
    pwm_r = (int32_T)fmax(fmin((real_T)raw_pwm_r, max_pwm), -max_pwm);
    pwm_l = (int32_T)fmax(fmin((real_T)raw_pwm_l, max_pwm), -max_pwm);

    /* 6. Route to Outports (Direction Control) */
    if (pwm_r >= 0) {
        rtY.RPWM_R  = (real_T)pwm_r;
        rtY.RPWM_R1 = 0.0;
    } else {
        rtY.RPWM_R  = 0.0;
        rtY.RPWM_R1 = fabs((real_T)pwm_r);
    }

    if (pwm_l >= 0) {
        rtY.RPWM_R2 = (real_T)pwm_l;
        rtY.RPWM_R3 = 0.0;
    } else {
        rtY.RPWM_R2 = 0.0;
        rtY.RPWM_R3 = fabs((real_T)pwm_l);
    }
}

void Simulink_initialize(void)
{
    last_step_tick = HAL_GetTick();
}

void Simulink_reset_integrator(void)
{
    // Integration parameters discarded for dynamic surface execution
}
