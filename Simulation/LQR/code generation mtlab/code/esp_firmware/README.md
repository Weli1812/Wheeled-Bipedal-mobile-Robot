# ESP Firmware (One Upload)

This folder contains an ESP-IDF app that compiles and runs the integrated generated code:

- Simulink model: `Traj2_grt_rtw`
- MATLAB Coder function: `lqr_tracker_step_codegen`
- Shared runtime: `common`

## One upload command

From this folder, run in an ESP-IDF terminal:

```
flash_one_upload.bat COM5 esp32
```

Replace `COM5` and `esp32` as needed.

## What the firmware does

- Initializes both generated modules.
- Runs a 50 ms loop.
- Calls `Traj2_step()` and `lqr_tracker_step_codegen()` every cycle.
- Converts LQR `v, w` commands to wheel commands.
- Sends wheel commands through weak hardware hooks.

## Hardware hook points

The file `main/main.c` provides weak functions you can override in your own component:

- `wbmr_read_vr_actual()`
- `wbmr_read_vl_actual()`
- `wbmr_write_wheel_commands(double vr_cmd, double vl_cmd)`

If not overridden, an internal first-order feedback model is used so the loop runs immediately after flashing.

## Notes

- The generated code uses desktop-target headers/intrinsics. Compatibility shims were added under `../common`:
  - `tmwtypes.h`
  - `rtw_continuous.h`
  - `rtw_solver.h`
  - `rt_logging.h`
  - `emmintrin.h`
- Logging APIs are stubbed as no-op for embedded runtime.