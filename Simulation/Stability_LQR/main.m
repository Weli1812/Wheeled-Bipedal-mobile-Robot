%% =========================================================
%  WHEEL-BIPED ROBOT ANIMATION — SCENARIO A ONLY
%  Run this file directly. No other files needed.
%
%  What happens step by step:
%  1. Robot parameters are defined
%  2. LQR controller is designed
%  3. Robot response is simulated (Scenario A: tilt only)
%  4. If tilt exceeds the allowed maximum before stabilization,
%     the controller is switched OFF
%  5. Animation + torque plots are shown
%% =========================================================

clear; clc; close all;

fprintf('=== WHEEL-BIPED ROBOT ANIMATION ===\n');
fprintf('Step 1: Setting up parameters...\n');

%% =========================================================
%  STEP 1 — ROBOT PARAMETERS
%% =========================================================
g     = 9.81;
Mb    = 11.3;
Mw    = 0.150;
Iw    = 0.00027572;
r     = 0.060;
d     = 0.45;
b_hip = 0.05;

h_lut  = [0.170, 0.284, 0.311];
Iy_lut = [0.137, 0.276, 0.317];
Iz_lut = [0.134, 0.123, 0.118];
L_lut  = [0.170, 0.284, 0.311];

h_ref  = 0.284;

%% =========================================================
%  STEP 2 — BUILD LQR CONTROLLER
%% =========================================================
fprintf('Step 2: Building LQR controller...\n');

[A, B, ~] = get_model(h_ref, g, Mb, Mw, Iw, r, d, ...
                      h_lut, Iy_lut, Iz_lut, L_lut, b_hip);

Q = diag([200, 10, 20, 80, 5, 1, 1, 2]);
R = diag([1.0, 1.0, 0.5]);
K = lqr(A, B, Q, R);

Acl = A - B*K;

ev = eig(Acl);
if all(real(ev) < 0)
    fprintf('  Controller: STABLE (all eigenvalues negative)\n');
else
    fprintf('  WARNING: controller may be unstable!\n');
end

%% =========================================================
%  STEP 3 — SIMULATE SCENARIO A
%% =========================================================
fprintf('Step 3: Simulating robot response...\n');

dt    = 0.010;
t_end = 4.0;
t     = (0 : dt : t_end)';
N     = length(t);

tilt = 6;   % <-- change this value to test different angles

% --- SCENARIO A: robot tilted by 'tilt' degrees ---
% States in this model:
% x = [phi; s; theta; Dh; phidot; v; omega; Dhdot]
x0A = [tilt*pi/180; 0; 0; 0; 0; 0; 0; 0];

% Physical validity gate
phi_tip   = atan(r / h_ref) * 180/pi;   % about tipping geometry
phi0A_deg = abs(x0A(1)) * 180/pi;

if phi0A_deg >= 90
    error('STOPPED: %.1f° is physically impossible — robot is horizontal or upside down. Use an angle below 90°.', phi0A_deg);
elseif phi0A_deg > 3 * phi_tip
    error('STOPPED: %.1f° is beyond the linearization limit (~%.1f°). The simulation result would be meaningless. Use a smaller angle.', ...
          phi0A_deg, 3*phi_tip);
end

%% --- CONTROLLER SHUTDOWN SETTINGS ---
% If tilt exceeds max_tilt_deg before the robot settles, controller turns OFF.
max_tilt_deg    = 8.0;    % controller cutoff threshold
phi_tol_deg     = 0.5;    % considered settled if |phi| stays below this
phidot_tol_deg  = 5.0;    % deg/s
stable_time_req = 0.25;   % must remain settled for this long

max_tilt_rad   = deg2rad(max_tilt_deg);
phi_tol_rad    = deg2rad(phi_tol_deg);
phidot_tol_rad = deg2rad(phidot_tol_deg);
N_stable       = ceil(stable_time_req / dt);

%% --- DISCRETIZE THE OPEN-LOOP MODEL ---
% We simulate step-by-step so we can switch controller OFF during the run.
sysc = ss(A, B, eye(8), zeros(8,3));
sysd = c2d(sysc, dt);

Ad = sysd.A;
Bd = sysd.B;

%% --- STORAGE ---
xA     = zeros(N, 8);
u_hist = zeros(N, 3);

xA(1,:) = x0A.';
controller_on = true;
stabilized    = false;
stable_count  = 0;
stop_idx      = NaN;

for k = 1:N-1
    xk = xA(k,:).';

    % States
    phi_k    = xk(1);   % tilt angle
    phidot_k = xk(5);   % tilt rate

    % Check if robot has stabilized
    if ~stabilized
        if abs(phi_k) < phi_tol_rad && abs(phidot_k) < phidot_tol_rad
            stable_count = stable_count + 1;
            if stable_count >= N_stable
                stabilized = true;
                fprintf('  Robot stabilized at t = %.3f s\n', t(k));
            end
        else
            stable_count = 0;
        end
    end

    % Stop controller if tilt exceeds the allowed limit before stabilization
    if controller_on && ~stabilized
        if abs(phi_k) > max_tilt_rad
            controller_on = false;
            stop_idx = k;
            fprintf('  Controller switched OFF at t = %.3f s because |phi| = %.2f deg exceeded %.2f deg before stabilization.\n', ...
                    t(k), rad2deg(abs(phi_k)), max_tilt_deg);
        end
    end

    % Control law
    if controller_on
        uk = -K * xk;
    else
        uk = zeros(3,1);   % controller OFF
    end

    % Save input
    u_hist(k,:) = uk.';

    % State update
    xA(k+1,:) = (Ad * xk + Bd * uk).';
end

% Last input sample
u_hist(end,:) = u_hist(end-1,:);

% Torques for plotting
tau_L   = u_hist(:, 1);
tau_R   = u_hist(:, 2);
tau_hip = u_hist(:, 3);

fprintf('  Simulation complete.\n');
fprintf('Step 4: Starting animation...\n\n');

%% =========================================================
%  STEP 4 — ANIMATE
%% =========================================================
scenario_title = sprintf('Scenario A: %g degree tilt', tilt);
fprintf('--- %s ---\n', scenario_title);

% Pull out states
phi   = xA(:, 1);
s     = xA(:, 2);
Dh    = xA(:, 4);
h_now = h_ref + Dh;

%% --- CREATE FIGURE ---
fig = figure('Name', scenario_title, ...
             'NumberTitle', 'off', ...
             'Color', [0.08 0.08 0.12], ...
             'Position', [50 50 1400 750]);

%% --- 3D ANIMATION PANEL (left side) ---
ax3 = axes('Parent', fig, ...
           'Position', [0.02 0.10 0.44 0.82], ...
           'Color',    [0.08 0.08 0.12], ...
           'XColor', [0.7 0.7 0.7], ...
           'YColor', [0.7 0.7 0.7], ...
           'ZColor', [0.7 0.7 0.7], ...
           'GridAlpha', 0.3);
hold(ax3, 'on');
grid(ax3, 'on');
view(ax3, 35, 15);
xlabel(ax3, 'Forward (m)', 'Color', [0.8 0.8 0.8]);
ylabel(ax3, 'Lateral (m)', 'Color', [0.8 0.8 0.8]);
zlabel(ax3, 'Height (m)',  'Color', [0.8 0.8 0.8]);
title(ax3, scenario_title, 'Color', 'w', 'FontSize', 13, 'FontWeight', 'bold');

ax3.XLim = [-0.35  0.35];
ax3.YLim = [-0.35  0.35];
ax3.ZLim = [ 0     0.60];

%% --- STATIC GROUND ---
patch(ax3, [-0.5 0.5 0.5 -0.5], [-0.35 -0.35 0.35 0.35], [0 0 0 0], ...
      [0.30 0.28 0.22], 'EdgeColor', 'none', 'FaceAlpha', 0.6);

%% --- ROBOT PARTS ---
% Left wheel
[wV, wF]  = make_wheel(r, 0.038, 32);
h_tyre_L  = patch(ax3, 'Faces', wF, 'Vertices', wV, ...
                  'FaceColor', [0.12 0.12 0.12], 'EdgeColor', 'none');
h_hub_L   = patch(ax3, 'Faces', wF(1:32,:), 'Vertices', wV, ...
                  'FaceColor', [0.75 0.10 0.10], 'EdgeColor', 'none');

% Right wheel
h_tyre_R  = patch(ax3, 'Faces', wF, 'Vertices', wV, ...
                  'FaceColor', [0.80 0.76 0.68], 'EdgeColor', 'none');
h_hub_R   = patch(ax3, 'Faces', wF(1:32,:), 'Vertices', wV, ...
                  'FaceColor', [0.55 0.55 0.55], 'EdgeColor', 'none');

% Axle
h_axle = line(ax3, [0 0], [-d/2 d/2], [r r], ...
              'Color', [0.55 0.55 0.60], 'LineWidth', 4);

% Scissor linkage
h_link_L1 = line(ax3, [0 0], [-d/2 -d/2], [0 0], 'Color', [0.68 0.70 0.74], 'LineWidth', 4);
h_link_L2 = line(ax3, [0 0], [-d/2 -d/2], [0 0], 'Color', [0.68 0.70 0.74], 'LineWidth', 4);
h_link_R1 = line(ax3, [0 0], [ d/2  d/2], [0 0], 'Color', [0.68 0.70 0.74], 'LineWidth', 4);
h_link_R2 = line(ax3, [0 0], [ d/2  d/2], [0 0], 'Color', [0.68 0.70 0.74], 'LineWidth', 4);

% Pivot dots
h_piv = gobjects(8,1);
for pi = 1:8
    h_piv(pi) = line(ax3, 0, 0, 0, ...
                     'Marker', 'o', 'MarkerSize', 7, ...
                     'MarkerFaceColor', [0.40 0.40 0.45], ...
                     'MarkerEdgeColor', 'none', 'LineStyle', 'none');
end

% Platform
[pV, pF] = make_box(0.22, 0.38, 0.014);
h_platform = patch(ax3, 'Faces', pF, 'Vertices', pV, ...
                   'FaceColor', [0.58 0.60 0.64], ...
                   'EdgeColor', [0.75 0.75 0.80], 'LineWidth', 0.8);

% Body box
[bV, bF] = make_box(0.20, 0.28, 0.18);
h_body = patch(ax3, 'Faces', bF, 'Vertices', bV, ...
               'FaceColor', [0.22 0.24 0.28], ...
               'EdgeColor', [0.48 0.55 0.65], 'LineWidth', 1.0);

% Time label
h_time = text(ax3, -0.30, 0, 0.57, 't = 0.00 s', ...
              'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold', ...
              'FontName', 'Courier New');

% Controller status label
if isnan(stop_idx)
    status_text = 'Controller: ON';
    status_color = [0.2 1.0 0.2];
else
    status_text = sprintf('Controller OFF at %.2f s', t(stop_idx));
    status_color = [1.0 0.3 0.3];
end

h_status = text(ax3, -0.30, 0, 0.53, status_text, ...
                'Color', status_color, 'FontSize', 11, 'FontWeight', 'bold', ...
                'FontName', 'Courier New');

%% --- PLOTS PANEL (right side: 5 stacked plots) ---
col_plot = {[0.4 0.65 1.0],  [1.0 0.55 0.25],  [0.35 0.90 0.50], ...
            [0.90 0.35 0.75], [0.95 0.85 0.25]};
y_data   = {phi*180/pi,  Dh*1000,       s*1000,    tau_L,          tau_R};
y_label  = {'\phi (deg)', '\Deltah (mm)', 's (mm)', '\tau_L (N.m)', '\tau_R (N.m)'};
p_title  = {'Body tilt angle', 'Height deviation', 'Forward position', ...
            'Left wheel torque', 'Right wheel torque'};

n_plots  = 5;
h_cursor = gobjects(n_plots, 1);
h_cutoff = gobjects(n_plots, 1);

for pi = 1:n_plots
    ypos = 0.88 - (pi-1) * 0.175;
    axP  = axes('Parent', fig, 'Position', [0.49 ypos 0.49 0.14], ...
                'Color', [0.06 0.06 0.09], ...
                'XColor', 'w', 'YColor', 'w', 'FontSize', 7);
    hold(axP, 'on'); grid(axP, 'on');

    plot(axP, t, y_data{pi}, 'Color', col_plot{pi}, 'LineWidth', 1.5);
    ylabel(axP, y_label{pi}, 'Color', 'w', 'FontSize', 7);
    title(axP,  p_title{pi}, 'Color', 'w', 'FontSize', 8);
    xlim(axP, [0 t_end]);

    if pi == n_plots
        xlabel(axP, 'Time (s)', 'Color', 'w', 'FontSize', 7);
    else
        set(axP, 'XTickLabel', {});
    end

    h_cursor(pi) = xline(axP, 0, 'w--', 'LineWidth', 1.2, 'Alpha', 0.7);

    if ~isnan(stop_idx)
        h_cutoff(pi) = xline(axP, t(stop_idx), 'r--', 'LineWidth', 1.2, 'Alpha', 0.8);
    else
        h_cutoff(pi) = gobjects(1);
    end
end

%% --- ANIMATE FRAME BY FRAME ---
skip = 1;

for k = 1 : skip : N
    phi_k  = phi(k);
    s_k    = s(k);
    h_k    = h_now(k);
    roll_k = s_k / r;

    ax_x = s_k;
    ax_z = r;

    bc_x = ax_x + h_k * sin(phi_k);
    bc_z = ax_z + h_k * cos(phi_k);

    % Wheels
    wV_L = transform_wheel(wV, ax_x, -d/2, ax_z, roll_k);
    set(h_tyre_L, 'Vertices', wV_L);
    set(h_hub_L,  'Vertices', wV_L);

    wV_R = transform_wheel(wV, ax_x, d/2, ax_z, roll_k);
    set(h_tyre_R, 'Vertices', wV_R);
    set(h_hub_R,  'Vertices', wV_R);

    % Axle
    set(h_axle, 'XData', [ax_x ax_x], 'YData', [-d/2 d/2], 'ZData', [ax_z ax_z]);

    % Scissor linkage
    arm  = 0.10;
    BF_x = ax_x + arm;             BF_z = ax_z;
    BR_x = ax_x - arm;             BR_z = ax_z;
    TF_x = bc_x + arm*cos(phi_k);  TF_z = bc_z - arm*sin(phi_k);
    TR_x = bc_x - arm*cos(phi_k);  TR_z = bc_z + arm*sin(phi_k);

    set(h_link_L1, 'XData',[BF_x TR_x], 'YData',[-d/2 -d/2], 'ZData',[BF_z TR_z]);
    set(h_link_L2, 'XData',[BR_x TF_x], 'YData',[-d/2 -d/2], 'ZData',[BR_z TF_z]);
    set(h_link_R1, 'XData',[BF_x TR_x], 'YData',[ d/2  d/2], 'ZData',[BF_z TR_z]);
    set(h_link_R2, 'XData',[BR_x TF_x], 'YData',[ d/2  d/2], 'ZData',[BR_z TF_z]);

    pts = [BF_x BF_z; BR_x BR_z; TF_x TF_z; TR_x TR_z];
    for pi = 1:4
        set(h_piv(pi),   'XData', pts(pi,1), 'YData', -d/2, 'ZData', pts(pi,2));
        set(h_piv(pi+4), 'XData', pts(pi,1), 'YData',  d/2, 'ZData', pts(pi,2));
    end

    % Platform
    Rmat  = rot_y(phi_k);
    pV2   = (Rmat * pV')';
    pV2(:,1) = pV2(:,1) + bc_x;
    pV2(:,3) = pV2(:,3) + bc_z - 0.09 - 0.007;
    set(h_platform, 'Vertices', pV2);

    % Body
    bV2   = (Rmat * bV')';
    bV2(:,1) = bV2(:,1) + bc_x;
    bV2(:,3) = bV2(:,3) + bc_z;
    set(h_body, 'Vertices', bV2);

    % Labels
    set(h_time, 'String', sprintf('t = %.2f s', t(k)), ...
                'Position', [ax3.XLim(1)+0.02, 0, 0.57]);

    if isnan(stop_idx)
        set(h_status, 'String', 'Controller: ON', 'Color', [0.2 1.0 0.2]);
    else
        if k < stop_idx
            set(h_status, 'String', 'Controller: ON', 'Color', [0.2 1.0 0.2]);
        else
            set(h_status, 'String', sprintf('Controller OFF at %.2f s', t(stop_idx)), ...
                          'Color', [1.0 0.3 0.3]);
        end
    end

    % Cursors
    for pi = 1:n_plots
        h_cursor(pi).Value = t(k);
    end

    % Camera follows robot
    ax3.XLim = s_k + [-0.35 0.35];

    drawnow;
    pause(0.000003);
end

fprintf('  Animation complete.\n');

%% =========================================================
%%  HELPER FUNCTIONS
%% =========================================================

function [V, F] = make_wheel(R, hw, n)
    th = linspace(0, 2*pi, n+1)';
    th(end) = [];
    cx = R * cos(th);
    cz = R * sin(th);

    Vf = [cx, -hw*ones(n,1), cz];
    Vb = [cx,  hw*ones(n,1), cz];
    V  = [Vf; Vb; 0 -hw 0; 0 hw 0];

    Fside = zeros(n, 4);
    for i = 1:n
        j = mod(i, n) + 1;
        Fside(i,:) = [i, j, j+n, i+n];
    end

    cf     = 2*n + 1;
    nxt_f  = [2:n, 1]';
    Fcap_f = [cf*ones(n,1), (1:n)', nxt_f, nxt_f];

    cb     = 2*n + 2;
    nxt_b  = n + [2:n, 1]';
    Fcap_b = [cb*ones(n,1), n+(1:n)', nxt_b, nxt_b];

    F = [Fside; Fcap_f; Fcap_b];
end

function Vout = transform_wheel(V, tx, ty, tz, roll)
    Ry = [ cos(roll), 0, sin(roll);
           0,         1, 0;
          -sin(roll), 0, cos(roll) ];
    Vout = (Ry * V')';
    Vout(:,1) = Vout(:,1) + tx;
    Vout(:,2) = Vout(:,2) + ty;
    Vout(:,3) = Vout(:,3) + tz;
end

function [V, F] = make_box(W, D, H)
    hw = W/2; hd = D/2; hh = H/2;
    V = [-hw -hd -hh;
          hw -hd -hh;
          hw  hd -hh;
         -hw  hd -hh;
         -hw -hd  hh;
          hw -hd  hh;
          hw  hd  hh;
         -hw  hd  hh];
    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         3 4 8 7;
         1 4 8 5;
         2 3 7 6];
end

function R = rot_y(angle)
    R = [ cos(angle), 0, sin(angle);
          0,          1, 0;
         -sin(angle), 0, cos(angle) ];
end

function [A, B, p] = get_model(h, g, Mb, Mw, Iw, r, d, ...
                               h_lut, Iy_lut, Iz_lut, L_lut, b_hip)
    Iy = interp1(h_lut, Iy_lut, h, 'linear', 'extrap');
    Iz = interp1(h_lut, Iz_lut, h, 'linear', 'extrap');
    L  = interp1(h_lut, L_lut,  h, 'linear', 'extrap');

    l = h;
    if h > L
        h = 0.999 * L;
    end

    a     = Mb*l^2 + Iy;
    b_    = Mb*l;
    c     = Mb + 2*Mw + 2*Iw/r^2;
    Delta = a*c - b_^2;
    Jth   = (d^2/2)*Mw + (d^2/(2*r^2))*Iw + Iz;

    alpha  = asin(min(0.9999, h/L));
    cosA   = max(0.05, cos(alpha));
    m_eff  = Iy / (L*cosA)^2;
    b_eff  = b_hip / (L*cosA)^2;
    tau_h0 = Mb*g*L*cosA;

    A = zeros(8,8);
    A(1,5) = 1;   % phi_dot
    A(2,6) = 1;   % s_dot
    A(3,7) = 1;   % theta_dot
    A(4,8) = 1;   % Dh_dot

    A(5,1) =  c*Mb*g*l / Delta;
    A(6,1) = -b_*Mb*g*l / Delta;
    A(8,8) = -b_eff / m_eff;

    B = zeros(8,3);
    B(5,1) = -b_/(Delta*r);
    B(5,2) = -b_/(Delta*r);

    B(6,1) =  a/(Delta*r);
    B(6,2) =  a/(Delta*r);

    B(7,1) =  d/(2*r*Jth);
    B(7,2) = -d/(2*r*Jth);

    B(8,3) = 1/m_eff;

    p.Iy     = Iy;
    p.L      = L;
    p.Delta  = Delta;
    p.m_eff  = m_eff;
    p.tau_h0 = tau_h0;
end