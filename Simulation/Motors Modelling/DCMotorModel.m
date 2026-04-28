%% =========================================================================
%  Wheeled Bipedal Robot (WBR) - Stabilization + Trajectory Tracking
%  Method  : Parameter-Varying LQR  (Zhang et al., 2022)
%  Equation: Full Eq.13 3x3 mass matrix  -- EXACTLY as in the paper
%  Robot   : Pow_WBR_Project (SeungbinOh / Your Design)
%  Params  : Values_UPDATED.docx
%  Limits  : Max tilt = 25 deg | Max torque = 3 Nm per wheel
%
%  STATE VECTOR  (6 states, matching paper Section 4.1):
%    x = [phi, s, theta, phi_dot, v, omega]
%         phi   = body tilt angle          (rad)
%         s     = traveled distance        (m)
%         theta = steering/yaw angle       (rad)
%         phi_dot = tilt rate              (rad/s)
%         v       = forward velocity       (m/s)
%         omega   = yaw rate               (rad/s)
%
%  INPUT VECTOR  (2 inputs):
%    u = [tau_r, tau_l]  (right and left wheel torques, Nm)
% ==========================================================================
clear; clc; close all;

%% =========================================================================
% SECTION 1 - PHYSICAL PARAMETERS  (from Values_UPDATED.docx)
% ==========================================================================

% --- Wheel parameters ---
Mw = 0.150;           % wheel mass          [kg]   (150 g)
R  = 0.060;           % wheel radius        [m]    (60 mm)
Iw = 2.757224e-4;     % wheel MOI about axle[kg.m^2]
dw = 0.450;           % wheel-to-wheel dist [m]    (mid of 430-470 mm)

% --- Body (upper body lumped) ---
Mb = 10.000;          % body mass           [kg]   (10000 g)
g  = 9.81;            % gravity             [m/s^2]

% --- Height-dependent parameters from docx (3 operating poses) ---
%                    Low-height   Mid-height   High-height
L_pts  = [0.170,     0.276,       0.303];  % pendulum length l(h)  [m]
Iy_pts = [0.1451,    0.2790,      0.3190]; % I_Y(h)  [kg.m^2]
Iz_pts = [0.1451,    0.1431,      0.1297]; % I_Z(h)  [kg.m^2]

% --- Safety limits ---
phi_max_deg = 25;                  % max tilt angle [deg]
phi_max     = deg2rad(phi_max_deg);
tau_max     = 3.0;                 % max wheel torque [Nm]

fprintf('============================================================\n');
fprintf(' WBR Parameter-Varying LQR Controller\n');
fprintf(' Based on: Zhang et al. (2022), Eq.13\n');
fprintf('============================================================\n\n');
fprintf('Robot parameters:\n');
fprintf('  Body mass Mb  = %.2f kg\n', Mb);
fprintf('  Wheel radius R = %.3f m\n', R);
fprintf('  Axle width  dw = %.3f m\n', dw);
fprintf('  Pendulum L (mid) = %.3f m\n', L_pts(2));
fprintf('  Tilt limit  = +/- %d deg\n', phi_max_deg);
fprintf('  Torque limit = +/- %.1f Nm\n\n', tau_max);

%% =========================================================================
% SECTION 2 - FULL EQ.13 MASS MATRIX & LINEARISATION
%
%  From Zhang et al. Eq.13, the full 3x3 wheeled dynamics are:
%
%  [M11  M12   0 ] [phi_ddot]   [-Mb*g*l*sin(phi)       ]
%  [M12  M22   0 ] [v_dot   ] + [-Mb*l*sin(phi)*phi_dot^2] = J^T * [tau_r]
%  [0    0   M33 ] [theta_ddot] [0                       ]         [tau_l]
%
%  Where:
%    M11 = Mb*l^2 + I_Y              (tilt inertia)
%    M12 = Mb*l*cos(phi)  -> at phi=0: M12 = Mb*l   (coupling term)
%    M22 = Mb + 2*Mw + 2*Iw/R^2     (translation inertia)
%    M33 = (dw^2/2)*Mw + (dw^2/2R^2)*Iw + I_Z  (yaw inertia)
%
%  Input Jacobian J^T (3x2):
%    [    0      0   ]   <- tilt equation: torque enters through coupling
%    [  1/R    1/R   ]   <- translation: both wheels drive forward
%    [ dw/2R  -dw/2R ]   <- yaw: differential drive turns the robot
%
%  LINEARISATION at equilibrium (phi=0, phi_dot=0):
%    sin(phi) -> phi,  cos(phi) -> 1,  phi_dot^2 -> 0
%
%  This gives the linear state-space: x_dot = A(h)*x + B(h)*u
%  where x = [phi, s, theta, phi_dot, v, omega]
% ==========================================================================

function [A, B] = build_full_AB(Mb, Mw, Iw, Iy, Iz, l, R, dw, g)
    %----------------------------------------------------------------------
    % Builds FULL 6x6 state matrix A and 6x2 input matrix B
    % exactly matching Eq.13 of Zhang et al. (2022)
    %
    % Inputs:
    %   Mb  - body mass [kg]
    %   Mw  - wheel mass [kg]
    %   Iw  - wheel MOI [kg.m^2]
    %   Iy  - body MOI about Y axis (wheel axis) [kg.m^2]
    %   Iz  - body MOI about Z axis (yaw) [kg.m^2]
    %   l   - pendulum length (CoM to wheel centre) [m]
    %   R   - wheel radius [m]
    %   dw  - wheel separation [m]
    %   g   - gravity [m/s^2]
    %
    % Outputs:
    %   A  - 6x6 system matrix
    %   B  - 6x2 input matrix
    %----------------------------------------------------------------------

    % ---- STEP 1: Build 3x3 Mass Matrix M from Eq.13 ----
    M11 = Mb*l^2 + Iy;                           % tilt inertia
    M12 = Mb*l;          % cos(0)=1 after linearisation -- coupling
    M22 = Mb + 2*Mw + 2*Iw/R^2;                 % translation inertia
    M33 = (dw^2/2)*Mw + (dw^2/(2*R^2))*Iw + Iz; % yaw inertia

    % Full 3x3 mass matrix (block diagonal after linearisation)
    M = [M11,  M12,   0  ;
         M12,  M22,   0  ;
          0,    0,   M33 ];

    % ---- STEP 2: Linearised gravity vector ----
    % From -Mb*g*l*sin(phi)  ->  linearise: sin(phi) ~ phi
    % This becomes a stiffness term: G_lin = [-Mb*g*l; 0; 0]
    % In state-space: appears as coefficient of phi in q_ddot equation
    G_lin = [Mb*g*l;   % acts on phi (destabilising -- positive = falls)
             0;
             0];

    % ---- STEP 3: Input Jacobian J^T from Eq.13 (3x2) ----
    % Rows = [tilt eq, translation eq, yaw eq]
    % Cols = [tau_r, tau_l]
    JT = [0,          0      ;   % tilt: torque enters only through M^-1*J
          1/R,        1/R    ;   % translation: sum of wheel torques
          dw/(2*R),  -dw/(2*R)]; % yaw: differential torque

    % ---- STEP 4: Compute M_inverse ----
    Minv = inv(M);  % 3x3 inversion (block diagonal -- efficient)

    % ---- STEP 5: Build position sub-matrix of A ----
    % q_ddot = Minv * (G_lin * q_pos - nothing from vel at linearisation)
    % => [phi_ddot; v_dot; theta_ddot] = Minv*G_lin * [phi; s; theta]
    %                                  + Minv*JT     * [tau_r; tau_l]
    %
    % NOTE: G_lin only has phi dependence (s and theta are integrators)
    % So the 3x3 position coefficient is:
    A_pos = Minv * [G_lin, zeros(3,2)];
    %              [phi,   s,    theta] columns
    %  A_pos(:,1) = Minv*G_lin  (phi dependence)
    %  A_pos(:,2) = 0           (s: pure integrator, no gravity)
    %  A_pos(:,3) = 0           (theta: pure integrator, no gravity)

    % Rearrange: A_pos is 3x3 but G_lin only fills first column
    Acont_pos = Minv * diag([Mb*g*l, 0, 0]);
    % This gives: [phi_ddot; v_dot; theta_ddot] from [phi; s; theta]

    % ---- STEP 6: Assemble full 6x6 A matrix ----
    % State order: [phi, s, theta, phi_dot, v, omega]
    %              [q1,  q2,  q3,  q1_dot, q2_dot, q3_dot]
    %
    %  x_dot = [q_dot    ]  =  [  0(3x3)   |   I(3x3)  ] * x
    %          [q_ddot   ]     [Acont_pos  |   0(3x3)  ]
    %                       +  [  0(3x2)   ] * u
    %                          [ Minv*JT   ]
    A = [zeros(3,3),   eye(3)   ;
         Acont_pos,    zeros(3,3)];

    % ---- STEP 7: Assemble full 6x2 B matrix ----
    B = [zeros(3,2);
         Minv * JT  ];
end

% ---- Verify at nominal (mid) height ----
L_nom  = L_pts(2);
Iy_nom = Iy_pts(2);
Iz_nom = Iz_pts(2);

[A_nom, B_nom] = build_full_AB(Mb, Mw, Iw, Iy_nom, Iz_nom, L_nom, R, dw, g);

fprintf('--- Full A matrix (Eq.13 linearised, mid height) ---\n');
disp(A_nom);
fprintf('--- Full B matrix (Eq.13, mid height) ---\n');
disp(B_nom);

ev = eig(A_nom);
fprintf('Open-loop eigenvalues of full system:\n');
disp(ev');
fprintf('=> Unstable eigenvalues confirm WIP instability (as expected)\n\n');

%% =========================================================================
% SECTION 3 - PARAMETER-VARYING LQR DESIGN  (Section 4.1 of paper)
%
%  The paper uses a single LQR on the full 6-state system.
%  State weights Q penalise: tilt (most), heading, then velocity.
%  Input weights R penalise torque usage (respect 3 Nm limit).
%
%  x = [phi,  s,  theta,  phi_dot,  v,  omega]
%  Q = diag([q1,  q2,  q3,   q4,    q5,  q6  ])
%
%  We pre-compute K at 3 heights and interpolate online (parameter-varying).
% ==========================================================================

% Weight matrix Q: state penalties
% phi      -> very high: robot MUST stay upright (10 kg robot, 27 Nm gravity)
% s        -> low: we track velocity, not position (avoids wind-up)
% theta    -> medium: follow heading reference
% phi_dot  -> high: damp tilt oscillations quickly
% v        -> low-medium: smooth speed tracking
% omega    -> medium: damp yaw oscillations
Q = diag([800,   1,   80,   100,   2,   15]);
%          phi   s  theta phi_dot  v  omega

% Weight matrix R: input penalties (2x2 for [tau_r, tau_l])
% Higher R = more conservative torque = smoother but slower response
R_lqr = diag([1.5,  1.5]);

% ---- Pre-compute LQR gains at 3 height operating points ----
K_tab = zeros(3, 2, 6);   % [height_idx, input_idx, state_idx]

fprintf('--- Computing LQR gains at 3 heights ---\n');
for i = 1:3
    [Ai, Bi] = build_full_AB(Mb, Mw, Iw, ...
                              Iy_pts(i), Iz_pts(i), L_pts(i), R, dw, g);
    Ki = lqr(Ai, Bi, Q, R_lqr);   % 2x6 gain matrix
    K_tab(i,:,:) = Ki;
    fprintf('Height %d (l=%.3f m): K =\n', i, L_pts(i));
    disp(Ki);
end

% ---- Online interpolation function ----
% Given current pendulum length L_cur, interpolate K between table values
get_K = @(L_cur) reshape( ...
    interp1(L_pts, reshape(K_tab, 3, 12), ...
            min(max(L_cur, L_pts(1)), L_pts(end)), 'linear'), ...
    2, 6);

% Verify nominal gain
K_nom = get_K(L_nom);
fprintf('Nominal gain K (mid height, 2x6 matrix):\n');
fprintf('  K = [tau_r gains; tau_l gains]\n');
fprintf('      [phi   s    theta  phi_dot  v    omega]\n');
disp(K_nom);

% Closed-loop stability check
A_cl = A_nom - B_nom * K_nom;
ev_cl = eig(A_cl);
fprintf('Closed-loop eigenvalues (all must have negative real part):\n');
disp(ev_cl');
if all(real(ev_cl) < 0)
    fprintf('=> System is STABLE with LQR. Robot will not fall.\n\n');
else
    fprintf('=> WARNING: Unstable closed-loop! Retune Q/R.\n\n');
end

%% =========================================================================
% SECTION 4 - PATH PLANNING  (Hybrid A* with 2-pass angle correction)
% ==========================================================================

resolution = 10;
map = binaryOccupancyMap(10, 10, resolution);

% Set obstacles (same as reference lq.m)
[X1,Y1]=meshgrid(1.0:0.1:2.0,1.0:0.1:3.8); setOccupancy(map,[X1(:) Y1(:)],1);
[X2,Y2]=meshgrid(1.5:0.1:3.2,5.2:0.1:7.8); setOccupancy(map,[X2(:) Y2(:)],1);
[X3,Y3]=meshgrid(4.0:0.1:5.8,2.0:0.1:4.2); setOccupancy(map,[X3(:) Y3(:)],1);
[X4,Y4]=meshgrid(4.8:0.1:6.4,5.3:0.1:7.2); setOccupancy(map,[X4(:) Y4(:)],1);
% [X5,Y5]=meshgrid(7.0:0.1:8.8,1.2:0.1:3.2); setOccupancy(map,[X5(:) Y5(:)],1);
% [X6,Y6]=meshgrid(7.2:0.1:8.9,6.2:0.1:8.8); setOccupancy(map,[X6(:) Y6(:)],1);
% [X7,Y7]=meshgrid(3.0:0.1:3.8,3.8:0.1:5.8); setOccupancy(map,[X7(:) Y7(:)],1);
% inflate(map, 0.20);   % inflate obstacles by 20 cm safety margin

startXY = [9, 1];
goalXY  = [2, 9];

ss_plan = stateSpaceSE2;
ss_plan.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
val = validatorOccupancyMap(ss_plan);
val.Map = map;
val.ValidationDistance = 0.1;

planner = plannerHybridAStar(val, ...
    'MinTurningRadius', 0.9, 'MotionPrimitiveLength', 0.8);
planner.ReverseCost = 1e20;

% Pass 1: plan with dummy angles to get path shape
disp('Hybrid A* pass 1 (dummy angles)...');
rp1   = plan(planner, [startXY, pi/2], [goalXY, pi/2]);
ps    = rp1.States;
th0   = atan2(ps(2,2)-ps(1,2), ps(2,1)-ps(1,1));
thG   = atan2(ps(end,2)-ps(end-1,2), ps(end,1)-ps(end-1,1));

% Pass 2: replan with corrected angles for smooth entry/exit
disp('Hybrid A* pass 2 (corrected angles)...');
rp2   = plan(planner, [startXY, th0], [goalXY, thG]);
poses = rp2.States;
fprintf('Path found: %d waypoints, length = %.2f m\n\n', ...
        size(poses,1), sum(sqrt(diff(poses(:,1)).^2+diff(poses(:,2)).^2)));

%% =========================================================================
% SECTION 5 - REFERENCE TRAJECTORY GENERATION
%
%  Converts discrete waypoints into a smooth time-stamped trajectory.
%  Produces: x_ref(t), y_ref(t), theta_ref(t), vd_ref(t), wd_ref(t)
% ==========================================================================

Ts    = 0.05;        % control step [s] -- 100 Hz, required for WIP stability
v_des = 0.20;        % desired cruise speed [m/s]
ds    = v_des * Ts;  % distance step per control cycle

% Arc-length reparameterise path
dxp  = diff(poses(:,1)); dyp = diff(poses(:,2));
S    = [0; cumsum(sqrt(dxp.^2 + dyp.^2))];
[S, ui] = unique(S,'stable');
poses   = poses(ui,:);

S_eq   = (0 : ds : S(end))';
if S_eq(end) < S(end), S_eq = [S_eq; S(end)]; end
N_ref  = length(S_eq);
t_ref  = (0:N_ref-1)' * Ts;
T_path = t_ref(end);

% Smooth spline interpolation of x,y along arc length
x_ref  = spline(S, poses(:,1), S_eq);
y_ref  = spline(S, poses(:,2), S_eq);
th_ref = unwrap(atan2(gradient(y_ref), gradient(x_ref)));

% Velocity and yaw-rate references
vd_ref = [diff(S_eq); ds] / Ts;
wd_ref = gradient(th_ref) / Ts;
vd_ref = smoothdata(vd_ref, 'gaussian', 15);
wd_ref = smoothdata(wd_ref, 'gaussian', 51);

% Smooth speed ramp up (3s) and down (2s) to avoid tipping at start/stop
ramp_u = min(t_ref / 3.0, 1);
ramp_d = max(min((T_path - t_ref) / 2.0, 1), 0);
sc     = min(ramp_u, ramp_d);
vd_ref = vd_ref .* sc;
wd_ref = wd_ref .* sc;
vd_ref = min(max(vd_ref, 0), 0.25);
wd_ref = min(max(wd_ref, -0.20), 0.20);
vd_ref([1 end]) = 0;
wd_ref([1 end]) = 0;

P = [x_ref, y_ref];   % reference path matrix (Nx2)

%% =========================================================================
% SECTION 6 - SIMULATION LOOP  (RK4 integration, 100 Hz)
%
%  Full 6-state controller: u = -K(h) * e
%  where e = x_ref - x_current  (state error vector)
%
%  The single 2x6 gain matrix K handles BOTH balance AND trajectory.
%  No decoupling needed -- the math shows yaw row is already decoupled
%  in the mass matrix (M33 block), so K naturally separates the tasks.
% ==========================================================================

T_sim = T_path + 4.0;       % extra 4 s after reaching path end
N_sim = ceil(T_sim / Ts) + 1;

% Initial full state: x = [phi, s, theta, phi_dot, v, omega]
x_state = [0; 0; th_ref(1); 0; 0; 0];

% Robot Cartesian position (separate from state -- for plotting)
X_r = x_ref(1);
Y_r = y_ref(1);

% Pre-allocate log arrays
log_t    = (0:N_sim-1)' * Ts;
log_X    = zeros(N_sim,1);
log_Y    = zeros(N_sim,1);
log_phi  = zeros(N_sim,1);
log_v    = zeros(N_sim,1);
log_taur = zeros(N_sim,1);
log_taul = zeros(N_sim,1);

goal_reached = false;
goalTol      = 0.10;   % [m]

% Pendulum length schedule: ramp from low (legs bent) to mid (nominal)
L_sched = @(t) interp1([0, 3, T_path+4], ...
                        [L_pts(1), L_pts(2), L_pts(2)], ...
                        min(max(t,0), T_path+4), 'pchip');

fprintf('--- Starting simulation: %d steps at Ts=%.3f s ---\n', N_sim, Ts);

for k = 1:N_sim
    t_now   = (k-1) * Ts;
    ref_idx = min(k, N_ref);

    % --- Current pendulum length (leg height) ---
    L_cur = L_sched(t_now);

    % --- Interpolate LQR gain for current height ---
    K = get_K(L_cur);   % 2x6 matrix

    % --- Build full reference state vector ---
    % Reference: upright (phi=0, phi_dot=0), track distance+heading+speed
    s_ref_k   = (ref_idx - 1) * ds;
    th_ref_k  = th_ref(ref_idx);
    vd_k      = vd_ref(ref_idx);
    wd_k      = wd_ref(ref_idx);

    x_ref_k = [0;          % phi_ref    = 0 (upright)
               s_ref_k;    % s_ref      = desired distance
               th_ref_k;   % theta_ref  = desired heading
               0;          % phi_dot_ref= 0 (no tilt rate)
               vd_k;       % v_ref      = desired speed
               wd_k];      % omega_ref  = desired yaw rate

    % --- State error vector (6x1) ---
    e = x_ref_k - x_state;

    % Wrap heading error to [-pi, pi] to avoid 360-degree jumps
    e(3) = atan2(sin(e(3)), cos(e(3)));

    % --- Control law: u = K * e  (LQR state feedback) ---
    % u = [tau_r; tau_l]  (2x1 vector)
    u = K * e;

    % --- Torque saturation (physical limit of your motor) ---
    tau_r = max(min(u(1), tau_max), -tau_max);
    tau_l = max(min(u(2), tau_max), -tau_max);
    u_sat = [tau_r; tau_l];

    % --- Log current values ---
    log_X(k)    = X_r;
    log_Y(k)    = Y_r;
    log_phi(k)  = x_state(1);
    log_v(k)    = x_state(5);
    log_taur(k) = tau_r;
    log_taul(k) = tau_l;

    % --- Build state-space matrices for current height ---
    [A_c, B_c] = build_full_AB(Mb, Mw, Iw, ...
        interp1(L_pts, Iy_pts, L_cur,'linear','extrap'), ...
        interp1(L_pts, Iz_pts, L_cur,'linear','extrap'), ...
        L_cur, R, dw, g);

    % --- RK4 Integration (4th-order Runge-Kutta) ---
    % Much more accurate than Euler -- essential for unstable WIP system
    f  = @(x_) A_c*x_ + B_c*u_sat;
    k1 = f(x_state);
    k2 = f(x_state + Ts/2 * k1);
    k3 = f(x_state + Ts/2 * k2);
    k4 = f(x_state + Ts   * k3);
    x_state = x_state + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);

    % --- Update Cartesian position from velocity + heading ---
    X_r = X_r + Ts * x_state(5) * cos(x_state(3));
    Y_r = Y_r + Ts * x_state(5) * sin(x_state(3));

    % --- Tilt safety clamp (fall recovery) ---
    % If tilt exceeds 25 deg, clamp and damp to allow recovery
    if abs(x_state(1)) > phi_max
        x_state(1) = sign(x_state(1)) * phi_max * 0.85;
        x_state(4) = x_state(4) * 0.20;   % damp tilt rate
        x_state(5) = x_state(5) * 0.40;   % slow down
    end

    % --- Goal detection ---
    if ~goal_reached && ...
            sqrt((X_r-goalXY(1))^2 + (Y_r-goalXY(2))^2) < goalTol
        goal_reached = true;
        fprintf('>>> Goal reached at t = %.2f s\n', t_now);
    end
end

fprintf('Simulation complete.\n');
fprintf('Max |phi| = %.2f deg\n\n', max(abs(rad2deg(log_phi))));

%% =========================================================================
% SECTION 7 - FIGURES
% ==========================================================================

ds_idx = 1:10:N_sim;   % plot every 10th point for speed

% ---- Figure 1: Map + Trajectory ----
figure('Name','Map + Trajectory','Color','w','Position',[50 50 680 580]);
show(map); hold on; axis equal; grid on;
plot(P(:,1), P(:,2), 'g--','LineWidth',2,'DisplayName','Reference Path');
plot(log_X(ds_idx), log_Y(ds_idx), 'b-','LineWidth',2,'DisplayName','Robot');
plot(startXY(1),startXY(2),'go','MarkerFaceColor','g','MarkerSize',10,'DisplayName','Start');
plot(goalXY(1), goalXY(2), 'r*','MarkerSize',14,'DisplayName','Goal');
xlabel('x [m]'); ylabel('y [m]');
title('WBR Trajectory on Occupancy Map (Full Eq.13 LQR)');
legend('Location','best'); xlim([0 10]); ylim([0 10]);

% ---- Figure 2: Tilt Angle ----
figure('Name','Tilt Angle','Color','w','Position',[740 50 600 320]);
plot(log_t(ds_idx), rad2deg(log_phi(ds_idx)),'b','LineWidth',1.5); hold on;
yline( phi_max_deg,'r--','LineWidth',2);
yline(-phi_max_deg,'r--','LineWidth',2);
yline(0,'k:','LineWidth',1);
xlabel('Time [s]'); ylabel('phi [deg]');
title(sprintf('Body Tilt Angle -- Limit = +/- %d deg', phi_max_deg));
legend('Tilt Angle','+ Limit','- Limit','Upright','Location','best');
ylim([-30 30]); grid on;

% ---- Figure 3: Velocity & Torque ----
figure('Name','Velocity & Torque','Color','w','Position',[50 430 680 450]);
subplot(2,1,1);
plot(log_t(ds_idx), log_v(ds_idx),'b','LineWidth',1.5); hold on;
N_pl = min(N_sim, N_ref);
plot(t_ref(1:N_pl), vd_ref(1:N_pl),'g--','LineWidth',1.2);
xlabel('Time [s]'); ylabel('v [m/s]');
title('Forward Velocity'); legend('Actual','Reference'); grid on;

subplot(2,1,2);
plot(log_t(ds_idx), log_taur(ds_idx),'b','LineWidth',1.2); hold on;
plot(log_t(ds_idx), log_taul(ds_idx),'r--','LineWidth',1.2);
yline( tau_max,'k:','LineWidth',1.5);
yline(-tau_max,'k:','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Torque [Nm]');
title(sprintf('Wheel Torques -- Limit = +/- %.0f Nm', tau_max));
legend('tau_R','tau_L','Limit','Location','best');
ylim([-3.5 3.5]); grid on;

% ---- Figure 4: Cross-track Error ----
err_ct = zeros(N_sim,1);
for k = ds_idx
    d = sqrt((P(:,1)-log_X(k)).^2 + (P(:,2)-log_Y(k)).^2);
    err_ct(k) = min(d);
end
figure('Name','Tracking Error','Color','w','Position',[740 430 600 320]);
plot(log_t(ds_idx), err_ct(ds_idx)*100,'b','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Cross-track Error [cm]');
title('Lateral Tracking Error'); grid on;

% ---- Figure 5: Animation ----
figure('Name','WBR Animation','Color','w','Position',[380 120 580 500]);
show(map); hold on; grid on; axis equal;
plot(P(:,1), P(:,2),'g--','LineWidth',1.5);
plot(startXY(1),startXY(2),'go','MarkerFaceColor','g','MarkerSize',8);
plot(goalXY(1), goalXY(2), 'r*','MarkerSize',12);
h_rob   = plot(log_X(1),log_Y(1),'bo','MarkerSize',9,'MarkerFaceColor','b');
log_psi = smoothdata(atan2(gradient(log_Y), gradient(log_X)),'gaussian',20);
h_dir   = quiver(log_X(1),log_Y(1), ...
                 cos(log_psi(1))*0.3, sin(log_psi(1))*0.3, ...
                 0,'b','LineWidth',2);
h_trail = plot(log_X(1),log_Y(1),'b-','LineWidth',1.5);
title('WBR Live Animation (Eq.13 Parameter-Varying LQR)');
xlabel('x [m]'); ylabel('y [m]');
xlim([0 10]); ylim([0 10]);

anim_step  = max(1, floor(N_sim/1500));  % more frames = smoother playback
anim_pause = 0.03;                        % seconds per frame — raise to go even slower
for k = 1:anim_step:N_sim
    set(h_rob,  'XData',log_X(k),'YData',log_Y(k));
    set(h_dir,  'XData',log_X(k),'YData',log_Y(k), ...
                'UData',cos(log_psi(k))*0.3, ...
                'VData',sin(log_psi(k))*0.3);
    set(h_trail,'XData',log_X(1:k),'YData',log_Y(1:k));
    drawnow;
    pause(anim_pause);
end

%% =========================================================================
% SECTION 8 - SUMMARY
% ==========================================================================
fprintf('========== Final Summary ==========\n');
fprintf('Max |tilt angle|  : %6.2f deg   (limit: %d deg)\n', ...
        max(abs(rad2deg(log_phi))), phi_max_deg);
fprintf('Max |torque|      : %6.3f Nm    (limit: %.0f Nm)\n', ...
        max(abs([log_taur; log_taul])), tau_max);
fprintf('Final position    : (%.3f, %.3f) m\n', log_X(end), log_Y(end));
fprintf('Goal position     : (%.3f, %.3f) m\n', goalXY(1), goalXY(2));
fprintf('Distance to goal  : %6.3f m\n', ...
        sqrt((log_X(end)-goalXY(1))^2+(log_Y(end)-goalXY(2))^2));
fprintf('Goal reached      : %s\n', mat2str(goal_reached));
fprintf('===================================\n');