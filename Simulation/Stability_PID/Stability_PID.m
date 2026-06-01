%% =========================================================
%  Wheel-Biped: Minimum Height Config + 2D PID (Damped)
%  Single Torque + Velocity & Recovery Time Metrics
%% =========================================================
clear; clc; close all;

%% 1. Parameters
Mw = 0.150; r = 0.060; d = 0.450; Iw = 2.757224e-4;
Mb = 10.0;  g = 9.81;
L  = 0.170; Iy = 0.1451; Iz = 0.1451;

%% 2. Mass matrix terms
m11 = Mb*L^2 + Iy;
m12 = Mb*L;
m22 = Mb + 2*Mw + 2*Iw/r^2;
det_sag = m11*m22 - m12^2;

%% 3. State-space matrices (4 States: phi, s, phi_dot, v)
a41 =  Mb*g*L*m22/det_sag;
a51 = -Mb*g*L*m12/det_sag;
b41 = -m12/(det_sag*r);
b51 =  m11/(det_sag*r);

A = [0,   0,   1,   0; 
     0,   0,   0,   1; 
     a41, 0,   0,   0; 
     a51, 0,   0,   0];
     
B = [0; 0; 2*b41; 2*b51]; % Single torque applied to both wheels

%% 4. PID Setup (Augmented State-Space)
% To implement an Integral (I) term for the tilt angle in state-space, 
% we augment the system with a 5th state: the integral of phi.
% New state vector: x_aug = [phi; s; phi_dot; v; int_phi]

A_aug = [A, zeros(4,1); 
         1, 0, 0, 0, 0]; % The derivative of int_phi is phi
B_aug = [B; 0];

% --- MANUAL PID TUNING ---
% 1. PID Gains for Tilt Angle (\phi)
Kp_tilt = -120;  % Proportional: Pushes back against tilt
Ki_tilt = -40;   % Integral: Eliminates steady-state tilt error
Kd_tilt = -25;   % Derivative: Dampens oscillations

% 2. PD Gains for Position (s) / Velocity (v)
% Strong position hold (-15) balanced by strong damping (-15)
Kp_pos = -15;     
Kd_pos = -15;     

% Gain matrix K (matches augmented states: [phi, s, phi_dot, v, int_phi])
K_pid = [Kp_tilt, Kp_pos, Kd_tilt, Kd_pos, Ki_tilt];

% Closed-loop system matrix
Acl = A_aug - B_aug * K_pid;

%% 5. Simulate: phi0 = 5.7 deg (0.1 rad)
t  = 0:0.005:5;
x0 = [0.1; 0; 0; 0; 0]; % Initial states (Notice the 5th state for int_phi is 0)

sys_cl = ss(Acl, B_aug, eye(5), zeros(5,1));
[y, t_out] = initial(sys_cl, x0, t);

%% 6. Calculate Metrics (Velocity & Recovery Time)
phi_deg = y(:,1) * 180/pi; % Convert tilt to degrees
v_out   = y(:,4);          % State 4 is the output velocity

% Calculate Recovery Time (Settling within +/- 0.1 degrees)
settle_threshold = 0.1; 
last_exceed_idx = find(abs(phi_deg) > settle_threshold, 1, 'last');

if ~isempty(last_exceed_idx) && last_exceed_idx < length(t_out)
    t_recovery = t_out(last_exceed_idx + 1);
else
    t_recovery = NaN; % Did not settle within 5 seconds
end

% Calculate Peak Velocity
v_peak = max(abs(v_out));

% Print Results to Console
fprintf('==============================================\n');
fprintf('  PID PERFORMANCE METRICS (5.7 deg init tilt)\n');
fprintf('==============================================\n');
fprintf('  Recovery Time (+/- %.1f deg) : %.3f seconds\n', settle_threshold, t_recovery);
fprintf('  Peak Output Velocity        : %.3f m/s\n', v_peak);
fprintf('  Final Settled Velocity      : %.3f m/s\n', v_out(end));
fprintf('==============================================\n');

%% 7. Plotting
figure('Name', 'PID Recovery Response', 'Position', [100, 100, 900, 400]);

% Subplot 1: Tilt Angle
subplot(1,2,1);
plot(t_out, phi_deg, 'b', 'LineWidth', 2); hold on;
yline(settle_threshold, 'k--', 'LineWidth', 1);
yline(-settle_threshold, 'k--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Tilt Angle Recovery');
grid on;

% Subplot 2: Output Velocity
subplot(1,2,2);
plot(t_out, v_out, 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Output Velocity');
grid on;