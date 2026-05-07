# ESP32 Integration Notes

This folder integrates:
- `codegen/lib/robot_controller` (MATLAB codegen from `lq.m` flow)
- `Traj2_grt_rtw` (Simulink generated runtime from `Traj2.slx`)

## What is integrated

- `lqr_traj2_esp32.c` updates `Traj2` reference arrays (`P`, `theta_ref_unwrapped`, `vd_ref`, `wd_ref`) at startup from external path data.
- It calls `robot_controller()` every control step to produce high-level `v_cmd` and `w_cmd` for monitoring/debug.
- It calls `Traj2_step()` every control step to produce wheel PWM and direction outputs.
- The provided `currentPose = [x, y, theta]` is injected into Traj2 states every step, so localization can be sourced externally.

## API usage

1. Build path and reference arrays offline in MATLAB, then export as C arrays:
   - `refPathN3`: row-major `[x, y, theta]` for each point
   - `vdRef`: linear velocity profile
   - `wdRef`: angular velocity profile
2. Call `lqr_traj2_init(...)` once.
3. Call `lqr_traj2_step(...)` at fixed period `0.1 s` (matches generated Traj2 step size).
4. Apply `dirr/dirl/pwmr/pwml` to motor drivers.

## Required before compiling for ESP32

The current `Traj2_grt_rtw` artifact was generated with target `grt.tlc` for Windows x86-64 and depends on Simulink runtime support files not included in this repository, for example:
- `rtw_continuous.h`
- `rtw_solver.h`
- `rt_logging.h`

For real ESP32 deployment, regenerate `Traj2.slx` C code using `ert.tlc` and embedded target settings, then replace `Traj2_grt_rtw` with the generated embedded package.

## Control-loop timing

- Traj2 generated base sample time is `0.1 s` (`Traj2_M->Timing.stepSize0`).
- If your motor ISR is faster, run ISR fast and call `lqr_traj2_step()` from a 100 ms task.

## Safety checks to add on target

- Clamp PWM to driver limits before output.
- Add encoder timeout/fault detection.
- Add emergency stop override that forces both PWM outputs to zero.
