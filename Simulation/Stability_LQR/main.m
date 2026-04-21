%% ================================================
%  SIMPLE 6-STATE WHEEL-BIPED LQR
%  States:
%    x = [phi; s; Dh; phidot; v; Dhdot]
%
%  Inputs:
%    u = [tau_w; tau_h]
%
%  Goal:
%    Balance upright, stay near s = 0, and keep height near Dh = 0
% ================================================

clc;

%% -------------------------------
% 1) PARAMETERS
% -------------------------------
g     = 9.81;
Mb    = 11.3;          % body mass [kg]
Mw    = 0.150;         % one wheel mass [kg]
Iw    = 2.7572e-4;     % wheel inertia [kg.m^2]
r_w   = 0.060;         % wheel radius [m]
d     = 0.450;         % wheel separation [m]

% Hip / linkage parameters
b_hip = 0.05;          % hip damping estimate
L_arm = 0.35;          % linkage arm length [m]

% Fixed operating height
h_ref = 0.284;         % [m]

% Inertia fits from your larger model
IY_SLOPE =  1.2597;
IY_INT   = -0.0779;
IZ_SLOPE = -0.1085;
IZ_INT   =  0.1527;

%% -------------------------------
% 2) BUILD SIMPLE 6-STATE MODEL
% -------------------------------
[A, B, p] = build_simple_6state_model( ...
    h_ref, g, Mb, Mw, Iw, r_w, d, L_arm, b_hip, ...
    IY_SLOPE, IY_INT, IZ_SLOPE, IZ_INT);

disp('A ='); disp(A);
disp('B ='); disp(B);

%% -------------------------------
% 3) LQR DESIGN
% -------------------------------
% State order: [phi; s; Dh; phidot; v; Dhdot]
Q = diag([300, 20, 80, 15, 5, 5]);
R = diag([1.0, 0.5]);

K = lqr(A, B, Q, R);

disp('K ='); disp(K);

Acl = A - B*K;
ev  = eig(Acl);

disp('Closed-loop eigenvalues =');
disp(ev);

%% -------------------------------
% 4) SIMULATION SETTINGS
% -------------------------------
dt    = 0.005;
t_end = 6;
t     = 0:dt:t_end;
N     = numel(t);

% Initial condition:
% 5 deg tilt + 2 cm height error
x0 = [deg2rad(180);   % phi
      0;            % s
      0.02;         % Dh
      0;            % phidot
      0;            % v
      0];           % Dhdot

x = zeros(6, N);
u = zeros(2, N);
x(:,1) = x0;

%% -------------------------------
% 5) SIMULATION LOOP
% -------------------------------
for k = 1:N-1
    % LQR regulation to zero
    u(:,k) = -K * x(:,k);

    % Continuous-time simulation with simple Euler integration
    xdot = A*x(:,k) + B*u(:,k);
    x(:,k+1) = x(:,k) + dt*xdot;
end

u(:,N) = u(:,N-1);

%% -------------------------------
% 6) EXTRACT STATES
% -------------------------------
phi    = x(1,:);    % rad
s      = x(2,:);    % m
Dh     = x(3,:);    % m
phidot = x(4,:);    % rad/s
v      = x(5,:);    % m/s
Dhdot  = x(6,:);    % m/s

tau_w  = u(1,:);
tau_h  = u(2,:);

%% -------------------------------
% 7) PLOTS
% -------------------------------
figure('Color','w','Position',[100 100 1000 750]);

subplot(4,1,1);
plot(t, rad2deg(phi), 'LineWidth', 1.5);
grid on;
ylabel('\phi (deg)');
title('Simple 6-State LQR: Balance + Position + Height');

subplot(4,1,2);
plot(t, s, 'LineWidth', 1.5);
grid on;
ylabel('s (m)');

subplot(4,1,3);
plot(t, Dh*1000, 'LineWidth', 1.5);
grid on;
ylabel('\Deltah (mm)');

subplot(4,1,4);
plot(t, tau_w, 'LineWidth', 1.5); hold on;
plot(t, tau_h, 'LineWidth', 1.5);
grid on;
ylabel('Torque');
xlabel('Time (s)');
legend('\tau_w','\tau_h');

%% -------------------------------
% 8) PRINT SOME INFO
% -------------------------------
fprintf('Model parameters at h_ref = %.3f m\n', h_ref);
fprintf('Iy     = %.4f kg.m^2\n', p.Iy);
fprintf('Iz     = %.4f kg.m^2\n', p.Iz);
fprintf('Delta  = %.4f\n', p.Delta);
fprintf('m_eff  = %.4f\n', p.m_eff);
fprintf('b_eff  = %.4f\n', p.b_eff);

%% ================================================
% LOCAL FUNCTION
% ================================================
function [A, B, p] = build_simple_6state_model( ...
    h, g, Mb, Mw, Iw, r_w, d, L_arm, b_hip, ...
    IY_SLOPE, IY_INT, IZ_SLOPE, IZ_INT)

    % Geometry / inertia at chosen fixed height
    l  = h;
    Iy = IY_SLOPE*h + IY_INT;
    Iz = IZ_SLOPE*h + IZ_INT;

    % Safety clamp
    Iy = max(0.05, Iy);
    Iz = max(0.05, Iz);

    % Common dynamic terms
    a     = Mb*l^2 + Iy;
    b     = Mb*l;
    c     = Mb + 2*Mw + 2*Iw/r_w^2;
    Delta = a*c - b^2;

    % Pitch / forward-motion terms
    A41 =  c*Mb*g*l / Delta;
    A51 = -b*Mb*g*l / Delta;

    % Since left and right wheel torques are equal:
    % tau_L = tau_R = tau_w
    B41 = 2 * ( -b / (Delta*r_w) );
    B51 = 2 * (  a / (Delta*r_w) );

    % Hip vertical dynamics
    alpha  = asin(min(0.9999, h / L_arm));
    cosA   = cos(alpha);

    J_link = 0.05 + Mb*L_arm^2;
    m_eff  = J_link / (L_arm*cosA)^2;
    b_eff  = b_hip  / (L_arm*cosA)^2;

    % State order:
    % x = [phi; s; Dh; phidot; v; Dhdot]

    A = zeros(6,6);

    % Kinematics
    A(1,4) = 1;     % phi_dot = phidot
    A(2,5) = 1;     % s_dot   = v
    A(3,6) = 1;     % Dh_dot  = Dhdot

    % Dynamics
    A(4,1) = A41;               % phiddot from phi
    A(5,1) = A51;               % vdot from phi
    A(6,6) = -b_eff / m_eff;    % Dhdot damping

    B = zeros(6,2);

    % Input 1 = tau_w (common wheel torque)
    B(4,1) = B41;      % phiddot from wheel torque
    B(5,1) = B51;      % vdot from wheel torque

    % Input 2 = tau_h (hip torque)
    B(6,2) = 1 / m_eff;

    % Return useful values
    p.Iy    = Iy;
    p.Iz    = Iz;
    p.a     = a;
    p.b     = b;
    p.c     = c;
    p.Delta = Delta;
    p.m_eff = m_eff;
    p.b_eff = b_eff;
end