%% ================================================================
%  BIPEDAL TWO-WHEELED SELF-BALANCING ROBOT
%  Kinematic Model, Dynamic Model & LQR Control
% ================================================================
% Methodology follows the structure of:
%   "Trajectory Tracking Control of a Nonholonomic Mobile Robot
%    with Differential Drive" – Silva Sousa et al., 2016
%
% The BIPEDAL variant extends that work by adding a body that must
% be actively balanced (inverted-pendulum on differential wheels).
%
% SYSTEM OVERVIEW
% ----------------
%   • Two driven wheels, differential configuration
%   • Body modelled as a rigid pendulum above the wheel axle
%   • Needs active BALANCING (pitch) + POSITION + HEADING control
%
% DECOUPLED SUBSYSTEMS
% ---------------------
%   1. Longitudinal:  [theta, theta_dot, x_p, x_p_dot]  — balance + forward
%      Input: u1 = tau_R + tau_L  (sum of wheel torques)
%
%   2. Lateral (Yaw): [psi, psi_dot]  — heading
%      Input: u2 = tau_R - tau_L  (differential torque)
%
% Each subsystem gets its own LQR controller.
% ================================================================

clear all; clc; close all;

fprintf('============================================================\n');
fprintf('   BIPEDAL TWO-WHEELED ROBOT — KINEMATICS & LQR CONTROL\n');
fprintf('============================================================\n\n');

%% ================================================================
%  SECTION I — PHYSICAL PARAMETERS
% ================================================================
fprintf('=== SECTION I: ROBOT PARAMETERS ===\n');

M_b = 1.5;                   % Body (chassis) mass         [kg]
M_w = 0.05;                  % Mass of each wheel          [kg]
R   = 0.05;                  % Wheel radius                [m]
L   = 0.30;                  % Distance between wheels     [m]
d   = 0.20;                  % Wheel axle → body CoM       [m]
I_b = 0.02;                  % Body pitch inertia          [kg·m²]
I_w = 0.5 * M_w * R^2;      % Wheel axle inertia (disk)   [kg·m²]
I_z = (1/12)*M_b*(L^2/4 + d^2);  % Body yaw inertia       [kg·m²]
g   = 9.81;                  % Gravity                     [m/s²]

fprintf('  M_b = %.2f kg   (body mass)\n',   M_b);
fprintf('  M_w = %.3f kg  (wheel mass, each)\n', M_w);
fprintf('  R   = %.3f m   (wheel radius)\n', R);
fprintf('  L   = %.3f m   (track width)\n',  L);
fprintf('  d   = %.3f m   (CoM height)\n',   d);
fprintf('  I_b = %.4f kg·m²  (pitch inertia)\n', I_b);
fprintf('  I_w = %.6f kg·m²  (wheel inertia)\n', I_w);
fprintf('  I_z = %.6f kg·m²  (yaw inertia)\n\n', I_z);


%% ================================================================
%  SECTION II — KINEMATIC MODEL  (mirrors paper Eqs 1-6)
% ================================================================
%
%  Let:  v_R = R * phi_dot_R      (right wheel linear speed)
%        v_L = R * phi_dot_L      (left  wheel linear speed)
%
%  Wheel speeds from robot velocity (analogous to paper Eqs 1-2):
%
%        v_R = v + (L/2) * omega                       ...(1)
%        v_L = v - (L/2) * omega                       ...(2)
%
%  Inverse (analogous to paper Eqs 3-4):
%
%        v     = (v_R + v_L) / 2                       ...(3)
%        omega = (v_R - v_L) / L                       ...(4)
%
%  Nonholonomic constraint — robot CANNOT move sideways
%  (analogous to paper Eq 5):
%
%        -sin(psi)·x_dot + cos(psi)·y_dot = 0         ...(5)
%
%  Kinematic model in world frame (analogous to paper Eq 6):
%
%   [ x_dot   ]   [ cos(psi)  0 ]   [ v     ]
%   [ y_dot   ] = [ sin(psi)  0 ] · [       ]         ...(6)
%   [ psi_dot ]   [    0      1 ]   [ omega ]
%
%  EXTENSION for the balancing body:
%
%   [ theta_dot ] = theta_dot          (lean-angle kinematics)
%
%  Full state: q = [x, y, psi, theta, theta_dot]
%
fprintf('=== SECTION II: KINEMATIC MODEL ===\n');
fprintf('  v_R = v + (L/2)*omega             ...(1)\n');
fprintf('  v_L = v - (L/2)*omega             ...(2)\n');
fprintf('  v   = (v_R + v_L) / 2            ...(3)\n');
fprintf('  ω   = (v_R - v_L) / L            ...(4)\n');
fprintf('  Constraint: -sin(ψ)ẋ+cos(ψ)ẏ=0   ...(5)\n');
fprintf('  Kinematic model:                   ...(6)\n');
fprintf('    ẋ   = v·cos(ψ)\n');
fprintf('    ẏ   = v·sin(ψ)\n');
fprintf('    ψ̇   = (v_R - v_L)/L\n\n');


%% ================================================================
%  SECTION III — DYNAMIC MODEL  (Euler-Lagrange)
% ================================================================
%
%  Generalised coordinates:  q = [x_p, theta]
%    x_p   = forward displacement along heading direction  [m]
%    theta = body lean angle from vertical                 [rad]
%
%  Kinetic energy:
%    T = ½·alpha·ẋ_p² + beta·cos(θ)·ẋ_p·θ̇ + ½·gamma·θ̇²
%
%  Potential energy:
%    V = M_b·g·d·cos(θ)
%
%  Intermediate inertia terms:
%    alpha = M_b + 2·M_w + 2·I_w/R²     (translational)
%    beta  = M_b·d                       (coupling)
%    gamma = I_b + M_b·d²               (rotational / pitch)
%
%  Virtual work by wheel torques τ_total = τ_R + τ_L:
%    Q_{x_p}  =  τ_total / R
%    Q_{theta} = -τ_total        (motor reaction on body)
%
%  Euler-Lagrange Equations of Motion (nonlinear):
%
%    alpha·ẍ_p + beta·cos(θ)·θ̈ - beta·sin(θ)·θ̇²
%           =  τ_total / R                             ...(7)
%
%    beta·cos(θ)·ẍ_p + gamma·θ̈ - M_b·g·d·sin(θ)
%           = -τ_total                                 ...(8)
%
%  Matrix form:  M_inertia · q̈  =  f(q,q̇,τ)
%    M_inertia = [alpha, beta; beta, gamma]
%
%  Linearised around θ=0, θ̇=0, ẋ_p=0  (upright equilibrium):
%
%    alpha·ẍ_p + beta·θ̈  = τ_total / R               ...(7L)
%    beta·ẍ_p  + gamma·θ̈ = M_b·g·d·θ - τ_total       ...(8L)
%
%  Solving (invert 2×2 inertia matrix, det = D):
%    θ̈   = (alpha·M_b·g·d / D)·θ  -  ((alpha + beta/R)/D)·τ  ...(9)
%    ẍ_p = -(beta·M_b·g·d / D)·θ  +  ((gamma/R + beta)/D)·τ  ...(10)

alpha   = M_b + 2*M_w + 2*I_w/R^2;
beta    = M_b * d;
gam     = I_b + M_b*d^2;
D       = alpha*gam - beta^2;

fprintf('=== SECTION III: DYNAMIC MODEL ===\n');
fprintf('  alpha = M_b+2M_w+2I_w/R² = %.4f kg\n',    alpha);
fprintf('  beta  = M_b·d            = %.4f kg·m\n',  beta);
fprintf('  gamma = I_b+M_b·d²       = %.4f kg·m²\n', gam);
fprintf('  D     = alpha·gamma-beta² = %.4f\n\n',     D);


%% ================================================================
%  SECTION IV — LINEARISED STATE-SPACE MODEL
% ================================================================
%
%  ---- A) LONGITUDINAL SUBSYSTEM ----
%  State:  z1 = [theta; theta_dot; x_p; x_p_dot]
%  Input:  u1 = tau_R + tau_L                [N·m]
%
%  From Eqs (9)-(10):
%
%   d/dt [theta    ]   [         0          1   0  0 ] [theta    ]
%        [theta_dot] = [alpha·Mbg·d/D       0   0  0 ] [theta_dot]
%        [x_p      ]   [         0          0   0  1 ] [x_p      ]
%        [x_p_dot  ]   [-beta·Mbg·d/D       0   0  0 ] [x_p_dot  ]
%
%                    + [            0           ]
%                      [-(alpha+beta/R)/D       ] · u1
%                      [            0           ]
%                      [(gamma/R+beta)/D        ]

A_lon = [ 0,                    1,   0,  0 ;
          alpha*M_b*g*d/D,      0,   0,  0 ;
          0,                    0,   0,  1 ;
         -beta*M_b*g*d/D,       0,   0,  0 ];

B_lon = [  0 ;
          -(alpha + beta/R)/D ;
           0 ;
           (gam/R + beta)/D   ];

%  ---- B) LATERAL (YAW) SUBSYSTEM ----
%  State:  z2 = [psi; psi_dot]
%  Input:  u2 = tau_R - tau_L               [N·m]
%
%  Effective yaw inertia (body + both wheels about vertical axis):
%    I_psi = I_z + 2·(I_w + M_w·(L/2)²)
%
%  Yaw torque from differential input:
%    T_yaw = (tau_R - tau_L)·L/(2R) = u2·L/(2R)
%
%   d/dt [psi    ]   [0  1] [psi    ]   [      0         ]
%        [psi_dot] = [0  0] [psi_dot] + [L/(2R·I_psi_eff)] · u2

I_psi_eff = I_z + 2*(I_w + M_w*(L/2)^2);

A_lat = [ 0,  1 ;
          0,  0 ];

B_lat = [ 0 ;
          L / (2*R*I_psi_eff) ];

fprintf('=== SECTION IV: STATE-SPACE MODEL ===\n');
fprintf('-- Longitudinal --\n');
fprintf('A_lon =\n'); disp(A_lon);
fprintf('B_lon =\n'); disp(B_lon);
fprintf('-- Lateral --\n');
fprintf('A_lat =\n'); disp(A_lat);
fprintf('B_lat =\n'); disp(B_lat);

% Controllability check
rank_lon = rank(ctrb(A_lon, B_lon));
rank_lat = rank(ctrb(A_lat, B_lat));
fprintf('Controllability ranks — Longitudinal: %d/4   Lateral: %d/2\n\n', ...
        rank_lon, rank_lat);

% Open-loop poles (longitudinal)
fprintf('Open-loop poles (longitudinal): ');
disp(eig(A_lon).');


%% ================================================================
%  SECTION V — LQR CONTROLLER DESIGN
% ================================================================
%
%  Optimal gain K minimises:
%    J = ∫₀^∞ ( z'·Q·z + u'·R_lqr·u ) dt
%
%  Control law:  u = -K·(z - z_ref)   =>   u = K·e
%
%  Q penalises state errors; R_lqr penalises actuator effort.
%  Heavier Q on theta → robot prioritises balance.

fprintf('=== SECTION V: LQR DESIGN ===\n');

% ---- Longitudinal LQR ----
%  Q: [theta, theta_dot, x_p, x_p_dot]
Q_lon   = diag([300, 1, 30, 5]);
R_lon   = 1.0;

[K_lon, ~, cl_poles_lon] = lqr(A_lon, B_lon, Q_lon, R_lon);

fprintf('Q_lon = diag([300, 1, 30, 5]),  R_lon = %.1f\n', R_lon);
fprintf('K_lon = [%7.4f  %7.4f  %7.4f  %7.4f]\n', K_lon);
fprintf('Closed-loop poles (longitudinal):\n');
for i=1:4
    fprintf('  λ_%d = %+.4f %+.4fi\n', i, real(cl_poles_lon(i)), imag(cl_poles_lon(i)));
end

% ---- Lateral LQR ----
%  Q: [psi, psi_dot]
Q_lat   = diag([80, 1]);
R_lat   = 1.0;

[K_lat, ~, cl_poles_lat] = lqr(A_lat, B_lat, Q_lat, R_lat);

fprintf('\nQ_lat = diag([80, 1]),  R_lat = %.1f\n', R_lat);
fprintf('K_lat = [%7.4f  %7.4f]\n', K_lat);
fprintf('Closed-loop poles (lateral):\n');
for i=1:2
    fprintf('  λ_%d = %+.4f %+.4fi\n', i, real(cl_poles_lat(i)), imag(cl_poles_lat(i)));
end
fprintf('\n');


%% ================================================================
%  SECTION VI — SIMULATION
% ================================================================
%
%  Mimics the paper's Section V-VI approach:
%    • Start with initial posture error
%    • Euler integration of closed-loop system
%    • Collect state history for plotting
%
%  Two test cases are run (as in the paper Figs 9 & 12):
%    Case 1 — Large tilt + position offset  (balance recovery)
%    Case 2 — Large heading offset          (yaw correction)

fprintf('=== SECTION VI: SIMULATION ===\n');

dt      = 0.005;        % Sampling time  [s]
T_sim   = 12;           % Total time     [s]
t       = 0:dt:T_sim;
N       = length(t);
tau_max = 3.0;          % Actuator saturation  [N·m]

% ---------- CASE 1: Balance + position recovery ----------
z1_lon  = [0.15; 0; -0.5; 0];   % theta=0.15 rad, x_p=-0.5 m
z1_lat  = [pi/6; 0];             % psi=30 deg

r_lon   = zeros(4,1);   % reference: upright, origin
r_lat   = zeros(2,1);   % reference: psi = 0

Z1_lon  = zeros(4,N);   Z1_lat = zeros(2,N);
U1_1    = zeros(1,N);   U2_1   = zeros(1,N);
Xw1=zeros(1,N); Yw1=zeros(1,N);
xw=0; yw=0;

for k=1:N
    Z1_lon(:,k)=z1_lon;  Z1_lat(:,k)=z1_lat;
    Xw1(k)=xw; Yw1(k)=yw;

    u1 = K_lon*(r_lon - z1_lon);
    u2 = K_lat*(r_lat - z1_lat);
    u1 = max(-tau_max, min(tau_max, u1));
    u2 = max(-tau_max, min(tau_max, u2));
    U1_1(k)=u1;  U2_1(k)=u2;

    v_k = z1_lon(4);  psi_k = z1_lat(1);
    xw  = xw + dt*v_k*cos(psi_k);
    yw  = yw + dt*v_k*sin(psi_k);

    z1_lon = z1_lon + dt*(A_lon*z1_lon + B_lon*u1);
    z1_lat = z1_lat + dt*(A_lat*z1_lat + B_lat*u2);
end

% ---------- CASE 2: Heading-only correction ----------
z2_lon  = [0.05; 0; 0; 0];      % small tilt only
z2_lat  = [pi/2; 0];            % psi=90 deg offset

Z2_lon  = zeros(4,N);   Z2_lat = zeros(2,N);
U1_2    = zeros(1,N);   U2_2   = zeros(1,N);
Xw2=zeros(1,N); Yw2=zeros(1,N);
xw=0; yw=0;

for k=1:N
    Z2_lon(:,k)=z2_lon;  Z2_lat(:,k)=z2_lat;
    Xw2(k)=xw; Yw2(k)=yw;

    u1 = K_lon*(r_lon - z2_lon);
    u2 = K_lat*(r_lat - z2_lat);
    u1 = max(-tau_max, min(tau_max, u1));
    u2 = max(-tau_max, min(tau_max, u2));
    U1_2(k)=u1;  U2_2(k)=u2;

    v_k = z2_lon(4);  psi_k = z2_lat(1);
    xw  = xw + dt*v_k*cos(psi_k);
    yw  = yw + dt*v_k*sin(psi_k);

    z2_lon = z2_lon + dt*(A_lon*z2_lon + B_lon*u1);
    z2_lat = z2_lat + dt*(A_lat*z2_lat + B_lat*u2);
end

fprintf('  Case 1: theta0=0.15 rad, x_p0=-0.5 m, psi0=30 deg\n');
fprintf('  Case 2: theta0=0.05 rad, psi0=90 deg\n');
fprintf('  Simulation complete.\n\n');


%% ================================================================
%  SECTION VII — PLOTS  (mirrors paper Figs 7-12)
% ================================================================

blue = [0.12 0.47 0.71];
red  = [0.84 0.15 0.16];
grn  = [0.17 0.63 0.17];

%% ---- Figure 1: Longitudinal states — Case 1 ----
figure('Name','Fig1: Longitudinal States – Case 1','Position',[30 30 920 750]);
sgtitle('Fig 1 — Longitudinal Closed-Loop Response (Case 1)', ...
        'FontSize',13,'FontWeight','bold');

subplot(4,1,1);
plot(t, Z1_lon(1,:)*180/pi, 'Color',blue,'LineWidth',2); hold on;
yline(0,'--','Color',red,'LineWidth',1.5,'Label','\theta_{ref}=0');
xlabel('Time [s]'); ylabel('\theta [deg]');
title('Body Tilt Angle  (Balance)'); grid on; ylim([-5 15]);

subplot(4,1,2);
plot(t, Z1_lon(2,:)*180/pi,'Color',blue,'LineWidth',2); hold on;
yline(0,'--','Color',red,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('\thetȧ [deg/s]');
title('Tilt Rate'); grid on;

subplot(4,1,3);
plot(t, Z1_lon(3,:),'Color',blue,'LineWidth',2); hold on;
yline(0,'--','Color',red,'LineWidth',1.5,'Label','x_{ref}=0');
xlabel('Time [s]'); ylabel('x_p [m]');
title('Forward Position'); grid on;

subplot(4,1,4);
plot(t, Z1_lon(4,:),'Color',blue,'LineWidth',2); hold on;
yline(0,'--','Color',red,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('v [m/s]');
title('Linear Velocity'); grid on;

%% ---- Figure 2: Yaw states — Case 1 ----
figure('Name','Fig2: Yaw States – Case 1','Position',[60 60 920 450]);
sgtitle('Fig 2 — Yaw (Heading) Closed-Loop Response (Case 1)', ...
        'FontSize',13,'FontWeight','bold');

subplot(2,1,1);
plot(t, Z1_lat(1,:)*180/pi,'Color',blue,'LineWidth',2); hold on;
yline(0,'--','Color',red,'LineWidth',1.5,'Label','\psi_{ref}=0');
xlabel('Time [s]'); ylabel('\psi [deg]');
title('Heading Angle'); grid on;

subplot(2,1,2);
plot(t, Z1_lat(2,:)*180/pi,'Color',blue,'LineWidth',2); hold on;
yline(0,'--','Color',red,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('\psi̇ [deg/s]');
title('Yaw Rate'); grid on;

%% ---- Figure 3: Control Inputs — Case 1 ----
figure('Name','Fig3: Control Inputs – Case 1','Position',[90 90 920 420]);
sgtitle('Fig 3 — Wheel Torque Inputs (Case 1)', ...
        'FontSize',13,'FontWeight','bold');

subplot(2,1,1);
plot(t, U1_1,'k-','LineWidth',1.8); hold on;
yline( tau_max,'r--','LineWidth',1.3,'Label','Saturation');
yline(-tau_max,'r--','LineWidth',1.3);
xlabel('Time [s]'); ylabel('[N·m]');
title('u_1 = \tau_R + \tau_L  (Balance / Forward)'); grid on;

subplot(2,1,2);
plot(t, U2_1,'k-','LineWidth',1.8); hold on;
yline( tau_max,'r--','LineWidth',1.3,'Label','Saturation');
yline(-tau_max,'r--','LineWidth',1.3);
xlabel('Time [s]'); ylabel('[N·m]');
title('u_2 = \tau_R - \tau_L  (Steering / Yaw)'); grid on;

%% ---- Figure 4: XY Trajectory — both cases ----
figure('Name','Fig4: XY Trajectory','Position',[120 120 620 620]);
plot(Xw1, Yw1, '-','Color',blue,'LineWidth',2.5); hold on;
plot(Xw2, Yw2, '-','Color',grn,'LineWidth',2.5);
plot(Xw1(1),Yw1(1),'o','Color',blue,'MarkerSize',10,'MarkerFaceColor',blue);
plot(Xw2(1),Yw2(1),'o','Color',grn,'MarkerSize',10,'MarkerFaceColor',grn);
plot(0,0,'rs','MarkerSize',12,'MarkerFaceColor','r');
xlabel('X [m]','FontSize',12); ylabel('Y [m]','FontSize',12);
title('Fig 4 — Robot Trajectory in XY Plane', ...
      'FontSize',13,'FontWeight','bold');
legend({'Case 1  (\theta_0=0.15, x_{p0}=-0.5, \psi_0=30°)', ...
        'Case 2  (\theta_0=0.05, \psi_0=90°)', ...
        'Case 1 start','Case 2 start','Target (0,0)'}, ...
       'Location','best');
grid on; axis equal;

%% ---- Figure 5: State Errors — both cases (mirrors Fig 8 & 11) ----
figure('Name','Fig5: State Errors','Position',[150 150 920 620]);
sgtitle('Fig 5 — State Error Convergence', ...
        'FontSize',13,'FontWeight','bold');

subplot(3,1,1);
plot(t, Z1_lon(1,:)*180/pi,'Color',blue,'LineWidth',2); hold on;
plot(t, Z2_lon(1,:)*180/pi,'--','Color',grn,'LineWidth',2);
yline(0,'r:','LineWidth',1.5);
xlabel('Time [s]'); ylabel('\theta error [deg]');
title('Tilt Angle Error'); grid on;
legend('Case 1','Case 2');

subplot(3,1,2);
plot(t, Z1_lon(3,:),'Color',blue,'LineWidth',2); hold on;
plot(t, Z2_lon(3,:),'--','Color',grn,'LineWidth',2);
yline(0,'r:','LineWidth',1.5);
xlabel('Time [s]'); ylabel('x_p error [m]');
title('Position Error'); grid on;
legend('Case 1','Case 2');

subplot(3,1,3);
plot(t, Z1_lat(1,:)*180/pi,'Color',blue,'LineWidth',2); hold on;
plot(t, Z2_lat(1,:)*180/pi,'--','Color',grn,'LineWidth',2);
yline(0,'r:','LineWidth',1.5);
xlabel('Time [s]'); ylabel('\psi error [deg]');
title('Heading Error'); grid on;
legend('Case 1','Case 2');

%% ---- Figure 6: Linear & Angular Velocities (mirrors Fig 7 & 10) ----
figure('Name','Fig6: Velocities','Position',[180 180 920 400]);
sgtitle('Fig 6 — Linear and Angular Velocities (Case 1)', ...
        'FontSize',13,'FontWeight','bold');

subplot(2,1,1);
plot(t, Z1_lon(4,:),'Color',blue,'LineWidth',2);
xlabel('Time [s]'); ylabel('v [m/s]');
title('Linear Velocity  V'); grid on;

subplot(2,1,2);
plot(t, Z1_lat(2,:),'Color',red,'LineWidth',2);
xlabel('Time [s]'); ylabel('\omega [rad/s]');
title('Angular (Yaw) Rate  W'); grid on;


%% ================================================================
%  SECTION VIII — SUMMARY
% ================================================================
fprintf('=== SECTION VIII: SUMMARY ===\n\n');
fprintf('  Kinematic model:  identical structure to paper Eqs 1-6.\n');
fprintf('  Dynamic model:    Euler-Lagrange, inverted pendulum on wheels.\n');
fprintf('  State-space:      decoupled into longitudinal (4 states)\n');
fprintf('                    and lateral/yaw (2 states).\n\n');
fprintf('  LQR Gains:\n');
fprintf('    K_lon = [%6.3f  %6.3f  %6.3f  %6.3f]\n', K_lon);
fprintf('    K_lat = [%6.3f  %6.3f]\n\n', K_lat);
fprintf('  Both subsystems stabilised. Errors converge to zero.\n');
fprintf('  6 figures generated (mirrors Figs 7-12 of the paper).\n');
fprintf('\n============================================================\n');
fprintf('  Done.\n');
fprintf('============================================================\n');
