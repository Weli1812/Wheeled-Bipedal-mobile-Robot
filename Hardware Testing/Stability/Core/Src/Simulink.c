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
extern volatile float dynamic_K_gains[6];
extern volatile float dynamic_KI_PHI;
extern volatile float esp_wheel_speed_r;
extern volatile float esp_wheel_speed_l;

/* NEW: Variables to export to ESP32 */
volatile float export_tau_r = 0.0f;
volatile float export_tau_l = 0.0f;
volatile float export_pwm_r = 0.0f;
volatile float export_pwm_l = 0.0f;
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
 * INTEGRATOR STATE — persists between Simulink_step() calls
 * ----------------------------------------------------------------------- */
static real_T phi_integral    = 0.0;  /* accumulated pitch error          */
static uint32_T last_step_tick = 0;   /* for real dt measurement          */

/* -----------------------------------------------------------------------
 * INTEGRATOR TUNING — start here, change only these values
 *
 *  KI_PHI        how fast the integrator corrects a steady lean
 *                start at 0.0, increase by 0.3 each test run
 *                typical useful range: 0.3 – 1.5
 *
 *  INT_CLAMP_PWM max PWM counts the integrator term can ever add
 *                keep at ~15% of your 4000 limit = 600
 *                prevents windup from swamping the P+D terms
 *
 *  INT_RESET_RAD kill integrator if pitch exceeds this (robot falling)
 *                0.35 rad ≈ 20° — well inside your 0.52 fall-detect
 * ----------------------------------------------------------------------- */
#define KI_PHI          0.05f    /* SET TO 0 FIRST — confirm P+D still works */
#define INT_CLAMP_PWM   600     /* max integrator contribution in PWM counts */
#define INT_RESET_RAD   0.35    /* reset threshold in radians               */

/* Model step function */
void Simulink_step(void)
{
    real_T tmp;
    real_T u_idx_0;
    real_T u_idx_1;
    int32_T pwm_l;
    int32_T pwm_r;

    /* ── Measure real dt from HAL tick (milliseconds → seconds) ── */
    uint32_T now     = HAL_GetTick();
    real_T   dt      = (real_T)(now - last_step_tick) * 0.001;
    last_step_tick   = now;

    /* Guard: clamp dt to sane range on first call or timer wrap */
    if (dt <= 0.0 || dt > 0.05) dt = 0.01;   /* assume 100 Hz if bad */

    // TUNE THIS: Minimum PWM required to overcome 775 motor static friction.
    const int32_T DEADBAND_OFFSET = 130;

    /* Updated LQR Gain Matrix (K) */
    static const real_T a[12] = {
  		  -0.0 ,   -0.0 ,  /* State 1 (phi)    : tau_r, tau_l */
           -0.0,  -0.0,  /* State 2 (s)      : tau_r, tau_l */
            0.00,  -0.00,  /* State 3 (theta)  : tau_r, tau_l */
           -0.0,  -0.0,  /* State 4 (phi_dot): tau_r, tau_l */
           -0.0,  -0.0,  /* State 5 (v)      : tau_r, tau_l */
            0.000,  -0.000   /* State 6 (omega)  : tau_r, tau_l */
        };
    /* 1. Extract states and multiply by LQR gain matrix */
    // ... [Keep the dt and DEADBAND_OFFSET calculations as they were] ...

        /* 1. Extract states and multiply by dynamic LQR gain array */
        u_idx_0 = 0.0;
        u_idx_1 = 0.0;
        for (pwm_r = 0; pwm_r < 6; pwm_r++) {
            tmp = rtU.state_x[pwm_r];
            // Apply the same gain to both left and right wheels
            u_idx_0 += (real_T)dynamic_K_gains[pwm_r] * tmp;
            u_idx_1 += (real_T)dynamic_K_gains[pwm_r] * tmp;
        }

        /* ── INTEGRATOR ────────────────────────────────────────────────────── */
                real_T phi = rtU.state_x[0];

                /* Step A — reset on fall */
                if (fabs(phi) > INT_RESET_RAD) {
                    phi_integral = 0.0;
                }

                /* Step B — accumulate */
                phi_integral += phi * dt;

                /* Step C — anti-windup: clamp integral in torque units */
                const real_T int_clamp_torque = (real_T)INT_CLAMP_PWM * 6.0 / 4000.0;

                if (dynamic_KI_PHI > 0.0001f) {
                    if (phi_integral >  int_clamp_torque / (real_T)dynamic_KI_PHI)
                        phi_integral =  int_clamp_torque / (real_T)dynamic_KI_PHI;
                    if (phi_integral < -int_clamp_torque / (real_T)dynamic_KI_PHI)
                        phi_integral = -int_clamp_torque / (real_T)dynamic_KI_PHI;
                } else {
                    phi_integral = 0.0;
                }

                /* Add integrator contribution to both motor torque demands */
                        real_T ki_contribution = (real_T)dynamic_KI_PHI * phi_integral;

                        /* u_idx_0 and u_idx_1 now represent tau_r_des and tau_l_des */
                        u_idx_0 -= ki_contribution;
                        u_idx_1 -= ki_contribution;

                        /* NEW: Export Torque Values */
                        export_tau_r = (float)u_idx_0;
                        export_tau_l = (float)u_idx_1;

                /* ── VOLTAGE/SPEED TO PWM CONVERSION ───────────────────────────────── */

                        /* NFP-42GP-775-EN Motor Parameters */
                        const real_T K_3     = 0.125;
                        const real_T K_4     = 0.215;
                        const real_T max_pwm = 4000;
                        const real_T V_batt = 22.5;
                        /* 1. Fetch actual wheel speeds from ESP32 payload */
                        real_T omega_wheel_r = (real_T)esp_wheel_speed_r;
                        real_T omega_wheel_l = (real_T)esp_wheel_speed_l;

                        /* 2. Calculate Voltages (V = I*R + Back_EMF)
                              u_idx_0 and u_idx_1 are your desired torques (tau_r_des, tau_l_des) */
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
                        /* NEW: Export Raw PWM Values (After deadband/clamping) */
                                export_pwm_r = (float)pwm_r;
                                export_pwm_l = (float)pwm_l;
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
                        // ... end of Simulink_step function ...
                        } // This closes Simulink_step

                        void Simulink_initialize(void)
                        {
                            phi_integral   = 0.0;
                            last_step_tick = HAL_GetTick();
                        }

                        void Simulink_reset_integrator(void)
                        {
                            phi_integral = 0.0;
                        }
                        // DELETE ANY EXTRA BRACES HERE. There should be no more code or braces.
