# Wheeled-Bipedal Mobile Robot

This repository contains the design, modeling, simulation, and hardware work for a wheeled-bipedal mobile robot. It combines control theory, MATLAB and Simulink models, embedded target assets, and PCB design files in one place.

## What is in this repository

The project is organized around four main areas:

- Robot modeling and control simulations in MATLAB and Simulink.
- Path planning and trajectory tracking experiments for a mobile robot.
- Embedded target preparation for an STM32-based implementation.
- Circuit and PCB design files for the robot hardware.

## Main folders

- Simulation/LQR: LQR-related models, generated code artifacts, and trajectory/control experiments.
- Simulation/Stability_LQR: MATLAB scripts focused on the robot stability and state-space derivation.
- Simulation/TV_LQR: time-varying LQR and tracking-related Simulink models and scripts.
- Simulation/Pure Pursuit: path-planning and pure-pursuit workflow, including Simulink, STM32CubeMX, and Keil project files.
- Simulation/Motors Modelling: DC motor model used for drivetrain analysis.
- Circuit Design/Robot/Circuit: Altium Designer project files for schematic and PCB design.
- Resources: datasheets, papers, and reference material used during development.

## Key models and scripts

- Simulation/Stability_LQR/main.m builds the robot kinematic and dynamic model and designs an LQR controller.
- Simulation/TV_LQR/tvlqr.m sets up a sample occupancy map, plans a path with mobileRobotPRM, and prepares trajectory references.
- Simulation/Motors Modelling/DCMotorModel.m estimates a DC motor transfer function from rated electrical and mechanical parameters.
- Simulation/Pure Pursuit/untitled2.m contains visualization code for planned paths and simulation outputs.

## Requirements

The MATLAB and Simulink workflows typically require:

- MATLAB
- Simulink
- Control System Toolbox
- Robotics System Toolbox
- LQR and state-space related functionality
- For code generation and embedded workflows: MATLAB Coder, Simulink Coder, and Embedded Coder as applicable

The embedded and PCB parts additionally require:

- STM32CubeMX for the .ioc project
- Keil MDK or a compatible ARM toolchain for the MCU project
- Altium Designer for the circuit project files

## How to use the project

1. Open the repository in MATLAB or your preferred editor.
2. Start with Simulation/Stability_LQR/main.m to review the robot model and baseline LQR design.
3. Open the Simulink models under Simulation/ to inspect trajectory tracking, time-varying LQR, and pure pursuit workflows.
4. Use the Pure Pursuit folder if you want the embedded-target version of the navigation stack.
5. Open Circuit Design/Robot/Circuit/Circuit.PrjPcb in Altium Designer to inspect the schematic and PCB.

## Notes

- Several generated artifacts are committed on purpose, including Simulink cache folders and code generation outputs, so the repository preserves intermediate results from the design process.
- Some folders contain autosave or history files created by CAD and simulation tools.
- The repository is best treated as a design and research workspace rather than a polished software package.

## References

The Resources folder contains the main papers and datasheets used for the project, including work on wheeled-legged robots, trajectory tracking, and the STM32H750VB datasheet.