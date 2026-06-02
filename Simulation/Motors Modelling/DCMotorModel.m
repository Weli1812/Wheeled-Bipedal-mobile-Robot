%% Parameters (Motor side)
Rated_Voltage = 24;
Stall_Current = 24.5;
No_Load_Current = 700e-3;
No_load_speed = 330 * 2 * pi / 60; % rad/s
Rated_speed = 250 * 2 * pi / 60;   % rad/s
Rated_torque = 2.94; 

%% Gearbox Parameters
N = 25; % Gear ratio
Gearbox_Efficiency = 0.356338; % Standard estimate; use 1.0 if ideal

%% System Inertia (Assuming 0.02 is the load inertia at the wheel)
J_load = 0.02; 
J_rotor = 0.00005; % Example small rotor inertia; replace if known
Inertia_total = J_rotor + (J_load / (N^2)); 

%% Calculations
Armature_Resistance = Rated_Voltage / Stall_Current;

% Torque/Back-EMF constant (measured at the motor core)
Electric_Coefficient = (Rated_Voltage - No_Load_Current * Armature_Resistance) / No_load_speed;
Torque_Coefficient = Electric_Coefficient;

% Damping calculation based on no-load losses
Friction = (Torque_Coefficient * No_Load_Current) / No_load_speed;

%% Modelling

% 1. Motor Shaft Speed Transfer Function: w_motor(s) / V(s)
motor_sys = tf(Torque_Coefficient, [(Inertia_total * Armature_Resistance), (Friction * Armature_Resistance + Electric_Coefficient * Torque_Coefficient)]);

% 2. Gearbox Output Shaft Speed Transfer Function: w_output(s) / V(s)
% Since w_output = w_motor / N
output_sys = motor_sys * (1 / N);