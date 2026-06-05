% tb_LQRwheels.m
% Test bench for LQRwheels to establish input data types for MATLAB Coder

% Clear workspace and close figures to ensure a clean run
clear; clc;

% Define initial state and reference values 
x_curr     = 0.0;
y_curr     = 0.0;
theta_curr = 0.0;

% --- NEW PARAMETER ---
dt         = 0.1; % Sample time [s]

xd_k       = 1.0;
yd_k       = 1.0;
thetad_k   = pi/4;

vd_k       = 0.5; % Desired linear velocity [m/s]
wd_k       = 0.1; % Desired angular velocity [rad/s]

vr_actual  = 0.0; % Simulated initial right wheel speed
vl_actual  = 0.0; % Simulated initial left wheel speed

disp('Starting LQRwheels Test Bench...');

% Run the function in a loop to test the persistent integrators
for i = 1:10
    % 1. Call the function under test (Added dt)
    [pwmr, pwml, dirr, dirl] = LQRwheels(x_curr, y_curr, theta_curr, dt, ...
                                         xd_k, yd_k, thetad_k, ...
                                         vd_k, wd_k, ...
                                         vr_actual, vl_actual);
    
    % 2. Display the outputs
    fprintf('Step %2d | PWM R: %.3f (Dir: %d) | PWM L: %.3f (Dir: %d)\n', ...
            i, pwmr, dirr, pwml, dirl);
            
    % 3. Simulate a simple plant update (closing the loop loosely)
    % We assume the wheels speed up slightly based on the command
    vr_actual = vr_actual + 0.05 * (1 - 2*dirr);
    vl_actual = vl_actual + 0.05 * (1 - 2*dirl);
    
    % Update robot pose slightly to simulate movement
    x_curr = x_curr + 0.1;
    y_curr = y_curr + 0.1;
end

disp('Test Bench complete. Ready for code generation.');