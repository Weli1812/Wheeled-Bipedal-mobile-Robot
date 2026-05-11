# Wheeled-Bipedal Mobile Robot

This repository is an end-to-end development workspace for a wheeled-bipedal robot, covering:

- modeling and control design in MATLAB/Simulink,
- path planning and trajectory tracking experiments,
- embedded implementation on STM32H750-based targets,
- motor communication bring-up (UART/RS485),
- PCB and schematic design.

It is organized as a research/prototyping project, with generated files and intermediate artifacts intentionally kept in version control.

## Repository overview

### 1) Simulation and control (`/Simulation`)

- **`Stability_LQR/main.m`**  
  Derives kinematic + dynamic equations for a self-balancing two-wheeled body and builds LQR controllers for longitudinal and lateral subsystems.
- **`LQR/`**  
  Trajectory-tracking experiments (`lq.m`, `Traj2.slx`) plus generated C code and integration material (including ESP32-oriented notes).
- **`TV_LQR/tvlqr.m`**  
  Path planning and tracking setup with occupancy maps and robotics planners.
- **`Pure Pursuit/`**  
  Pure-pursuit Simulink model plus embedded project exports (`.ioc`, MDK-ARM, generated code).
- **`Localization/` and `Localization_control/`**  
  Localization assets and wheel-level LQR/PID controller prototyping (`LQRwheels.m`, test benches).
- **`Motors Modelling/DCMotorModel.m`**  
  First-principles DC motor transfer-function estimation from rated parameters.

### 2) Hardware implementation (`/Hardware Testing`)

STM32CubeMX/CubeIDE project trees for board bring-up and communication:

- **`LQRxSTM/`**: LQR-oriented STM32 project with Simulink-generated integration assets.
- **`STM32H750VBT6-ESP32UARTCommunication/`**: UART communication test project.
- **`STM32H750VBT6-MAX485UARTCommunication/`**: RS485 motor-driver communication implementation.

> Note: a separate `/Hardware_Testing` folder exists and currently contains mainly IDE metadata snapshots.

### 3) Circuit design (`/Circuit Design`)

- **`Circuit Design/Robot/Circuit/`** contains Altium Designer source files:
  - `Circuit.PrjPcb`
  - `Sheet1.SchDoc`
  - `PCB1.PcbDoc`

### 4) References (`/Resources`)

- **`Papers/`**: control/path-tracking related literature.
- **`Darasheets/`**: component datasheets (folder name preserved as-is in repo).

## Typical development flow

1. Develop/validate dynamics and controllers in MATLAB scripts (`Stability_LQR`, `LQR`, `TV_LQR`).
2. Validate closed-loop behavior in Simulink models (`Traj2.slx`, `Pure_Pursuit.slx`, etc.).
3. Export generated code (MATLAB Coder / Simulink Coder).
4. Integrate and deploy to STM32/ESP32-targeted firmware projects.
5. Verify motor/control I/O and communications on hardware test projects.
6. Iterate hardware design in Altium if required.

## Toolchain and software requirements

### Modeling and simulation

- MATLAB
- Simulink
- Control System Toolbox
- Robotics System Toolbox
- MATLAB Coder / Simulink Coder / Embedded Coder (for code generation workflows)

### Embedded development

- STM32CubeMX / STM32CubeIDE
- ARM embedded toolchain (e.g., `arm-none-eabi-gcc`) for CMake-based embedded builds
- Keil MDK (for projects using `MDK-ARM` exports)
- ESP-IDF (for ESP32 integration folders where applicable)

### Hardware design

- Altium Designer

## Quick start

- Start with **`/Simulation/Stability_LQR/main.m`** to understand the baseline robot model and balancing control structure.
- Then inspect **`/Simulation/LQR/lq.m`** and **`/Simulation/TV_LQR/tvlqr.m`** for trajectory planning/tracking workflows.
- For embedded experiments, open an `.ioc` project under **`/Hardware Testing`** in STM32CubeMX/CubeIDE.
- For PCB/schematic review, open **`/Circuit Design/Robot/Circuit/Circuit.PrjPcb`** in Altium.

## Important notes

- The repository contains many generated files (`slprj`, `*_grt_rtw`, autosaves, IDE metadata). This is expected and reflects the project’s iterative workflow.
- There is no single root build/test pipeline for all subprojects; each simulation or firmware subfolder is executed with its native toolchain.
