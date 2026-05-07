# Integrated Generated C Code

This folder merges:
- MATLAB Coder output from `codegen/lib/lqr_tracker_step_codegen`
- Simulink Coder output from `Traj2_grt_rtw`

## Folder layout

- `common/`
  - Shared runtime files used by both generators:
    - `rtwtypes.h`
    - `rt_nonfinite.c`, `rt_nonfinite.h`
    - `rtGetInf.c`, `rtGetInf.h`
    - `rtGetNaN.c`, `rtGetNaN.h`
    - `rt_defines.h`
- `lqr_tracker_step_codegen/`
  - LQR tracker generated sources and headers
  - Runtime duplicates removed to avoid linker symbol redefinition
- `Traj2_grt_rtw/`
  - Traj2 generated sources and headers
  - Runtime duplicates removed to use `common/` only

## Compile integration notes

When compiling, include these directories:
- `code/common`
- `code/lqr_tracker_step_codegen`
- `code/Traj2_grt_rtw`
- MATLAB/Simulink external include directory that provides:
  - `tmwtypes.h`
  - `rtw_continuous.h`
  - `rtw_solver.h`
  - `rt_logging.h`

Compile these C files (minimum):
- `common/rt_nonfinite.c`
- `common/rtGetInf.c`
- `common/rtGetNaN.c`
- `lqr_tracker_step_codegen/atan2.c`
- `lqr_tracker_step_codegen/lqr_tracker_step_codegen.c`
- `lqr_tracker_step_codegen/lqr_tracker_step_codegen_emxAPI.c`
- `lqr_tracker_step_codegen/lqr_tracker_step_codegen_emxutil.c`
- `lqr_tracker_step_codegen/lqr_tracker_step_codegen_initialize.c`
- `lqr_tracker_step_codegen/lqr_tracker_step_codegen_terminate.c`
- `Traj2_grt_rtw/Traj2.c`
- `Traj2_grt_rtw/Traj2_data.c`

Do not compile runtime files from module folders (they were intentionally removed).
