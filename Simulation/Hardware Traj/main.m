clear; clc; close all;

%% =========================
% 1) Robot parameters
% ==========================
L_wb   = 0.4;      % wheelbase [m]
vd_nom = 0.2;      % constant desired forward cruise speed [m/s]

%% =========================
% 2) Goal stop parameters
% ==========================
goalPosTol   = 0.08;          % [m]
goalThetaTol = deg2rad(8);    % [rad]

%% =========================
% 3) Build occupancy map (Environment)
% ==========================
resolution = 10;
map = binaryOccupancyMap(10, 10, resolution);

% Obstacles (Note: The straight line will pass right through these, 
% but we are ignoring obstacles for this purely continuous test)
[X1, Y1] = meshgrid(1.0:0.1:2.0, 1.0:0.1:3.8); setOccupancy(map, [X1(:) Y1(:)], 1);
[X2, Y2] = meshgrid(1.5:0.1:3.2, 5.2:0.1:7.8); setOccupancy(map, [X2(:) Y2(:)], 1);
[X3, Y3] = meshgrid(4.0:0.1:5.8, 2.0:0.1:4.2); setOccupancy(map, [X3(:) Y3(:)], 1);
[X4, Y4] = meshgrid(4.8:0.1:6.4, 5.3:0.1:7.2); setOccupancy(map, [X4(:) Y4(:)], 1);
inflate(map, 0.2);

startXY = [9, 1];
goalXY  = [2, 9];

%% =========================
% 4) Straight Line Path Generation
% ==========================
disp('Generating straight line path...');
tic;
ds_spatial = 0.02;

% Calculate heading of the straight line
theta_line = atan2(goalXY(2) - startXY(2), goalXY(1) - startXY(1));

% Align initial and goal poses with the line
startPose = [startXY, theta_line];
goalPose  = [goalXY,  theta_line];

% Generate coarse points
num_coarse = 10;
x_coarse = linspace(startXY(1), goalXY(1), num_coarse)';
y_coarse = linspace(startXY(2), goalXY(2), num_coarse)';
poses = [x_coarse, y_coarse, repmat(theta_line, num_coarse, 1)];

% Generate fine spatial reference points
S_raw = [0; cumsum(sqrt(diff(poses(:,1)).^2 + diff(poses(:,2)).^2))];
S_path = (0 : ds_spatial : S_raw(end))';
if S_path(end) < S_raw(end)
    S_path = [S_path; S_raw(end)];
end

x_ref = interp1(S_raw, poses(:,1), S_path, 'linear');
y_ref = interp1(S_raw, poses(:,2), S_path, 'linear');
theta_ref = repmat(theta_line, size(S_path));
theta_ref_unwrapped = unwrap(theta_ref);
P = [x_ref y_ref];

fprintf('Straight line path generated in %.4f s\n', toc);

%% =========================
% 5) Build Realistic SPATIAL Reference Table
% ==========================
% For real-life hardware, we must ramp up and ramp down smoothly to 
% avoid infinite acceleration requests to the motors.
kappa_ref = gradient(theta_ref_unwrapped, ds_spatial);
kappa_ref = smoothdata(kappa_ref, 'gaussian', 51);

r_min     = 0.9;
kappa_max = 1 / r_min;
kappa_ref = max(min(kappa_ref, kappa_max), -kappa_max);

totalPathLength = S_path(end);
s_ramp_up   = min(0.4, totalPathLength * 0.15); % Ramp up over first 15% of path or 0.4m
s_ramp_down = min(0.4, totalPathLength * 0.15); % Ramp down over last 15% of path or 0.4m
a_ramp = vd_nom^2 / (2 * s_ramp_up);
v_kickstart = 0.02; % Tiny kickstart to overcome static friction

vd_path = zeros(size(S_path));
for k = 1:numel(S_path)
    s     = S_path(k);
    s_rem = totalPathLength - s;
    if s <= s_ramp_up
        vd_path(k) = max(sqrt(2 * a_ramp * s), v_kickstart);
    elseif s_rem <= s_ramp_down
        vd_path(k) = sqrt(2 * a_ramp * max(s_rem, 0));
    else
        vd_path(k) = vd_nom; % Constant cruise speed
    end
end
vd_path(end) = 0;

wd_path = vd_path .* kappa_ref;
wd_path = smoothdata(wd_path, 'gaussian', 31);

vd_max  = 0.3;
wd_max  = vd_max / r_min;
vd_path = min(vd_path, vd_max);
wd_path = max(min(wd_path, wd_max), -wd_max);

%% =========================
% 6) Fixed height & LQR Precomputation
% ==========================
h_ref = 0.17;
K_lqr = compute_lqr_gain(h_ref);

%% =========================
% 6.5) Calculate Dynamic Equilibrium Angle (Feedforward)
% ==========================
% 1. Define physical offsets for your CURRENT height (h_ref)
% Extract this directly from your SolidWorks X-COM graph.
% If h_ref = 0.17m, put the matching X-COM offset here in meters.
X_com_offset = 0.00; % [m] Update this based on your selected height!
g = 9.81;            % [m/s^2] Gravity

% 2. Calculate planned acceleration (kinematic: a = v * dv/ds)
dvd_ds = gradient(vd_path, ds_spatial);
ad_path = vd_path .* dvd_ds;
ad_path = smoothdata(ad_path, 'gaussian', 31); % Smooth to prevent jerky inputs

% 3. Calculate phi_ref for every single point on the track
% Formula: Static Offset (X_com) + Dynamic Offset (Acceleration)
phi_ref_path = atan(X_com_offset / h_ref) + (ad_path / g);

% Smooth the final reference angle to ensure the motors don't jitter
phi_ref_path = smoothdata(phi_ref_path, 'gaussian', 31); 

%% =========================
% 7) Initial conditions
% ==========================
% The robot must start leaning at the required initial angle!
phi0     = phi_ref_path(1); 
s0       = 0;
theta0   = theta_ref_unwrapped(1);
phi_dot0 = 0;
v0       = 0;
omega0   = 0;
x0       = P(1,1);
y0       = P(1,2);
x_init = [phi0; s0; theta0; phi_dot0; v0; omega0; x0; y0];

%% =========================
% 8) Simulation Execution
% ==========================
T_est = totalPathLength / vd_nom;
T_end = T_est * 1.5 + 5.0; % Add a buffer to ensure it reaches the goal

% Format variables for Simulink
S_path               = double(S_path(:));
x_ref                = double(x_ref(:));
y_ref                = double(y_ref(:));
theta_ref_unwrapped  = double(theta_ref_unwrapped(:));
kappa_ref            = double(kappa_ref(:));
phi_ref_path         = double(phi_ref_path(:)); % Added for Simulink
vd_path              = double(vd_path(:));
wd_path              = double(wd_path(:));
P                    = double(P);
h_ref                = double(h_ref);
x_init               = double(x_init);
K_lqr                = double(K_lqr);
Npath                = numel(S_path);

% Validation check to ensure safe export
assert(numel(phi_ref_path) == Npath, 'Size mismatch: phi_ref_path');

disp('Running Simulation (Continuous Drive)...');
out = sim('OneCombinedLQR', 'StopTime', num2str(T_end));

% Extract simulation data
sim_t  = out.tout(:);
x      = out.x_out(:);
y      = out.y_out(:);
theta  = out.theta_out(:);
phi    = out.phi_out(:);
v      = out.v_out(:);
omega  = out.omega_out(:);
vd_cmd = out.vd_cmd_out(:);
wd_cmd = out.wd_cmd_out(:);

disp('Simulation complete! Launching animation...');

%% =========================
% 9) Animated Figure
% ==========================
maxDriveFrames = 700;     
pause_drive    = 0.015;   

num_pts = min([numel(sim_t), numel(x), numel(y), numel(theta), ...
               numel(phi), numel(v), numel(omega), numel(vd_cmd), numel(wd_cmd)]);

sim_t   = sim_t(1:num_pts);
x       = x(1:num_pts);
y       = y(1:num_pts);
theta   = theta(1:num_pts);
phi     = phi(1:num_pts);
v       = v(1:num_pts);
omega   = omega(1:num_pts);
vd_cmd  = vd_cmd(1:num_pts);
wd_cmd  = wd_cmd(1:num_pts);

x_pad = 0.5; y_pad = 0.5;
map_xlim = [min(x) - x_pad,  max(x) + x_pad];
map_ylim = [min(y) - y_pad,  max(y) + y_pad];

fig_main = figure('Name', 'Robot Drive Animation', 'Color', 'w', 'Position', [80, 60, 1000, 820]);

% --- Top panel: 2-D map view ---
ax_map = subplot(4, 1, 1, 'Parent', fig_main);
hold(ax_map, 'on'); grid(ax_map, 'on'); axis(ax_map, 'equal');
title(ax_map, 'Robot Trajectory Tracking Animation', 'FontSize', 12, 'FontWeight', 'bold');
xlabel(ax_map, 'X (m)'); ylabel(ax_map, 'Y (m)');
ax_map.XLim = map_xlim; ax_map.YLim = map_ylim;

plot(ax_map, x_ref, y_ref, 'g--', 'LineWidth', 1.2); % Planned path
plot(ax_map, x(1), y(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(ax_map, x(end), y(end), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.84 0], 'MarkerEdgeColor', 'k');

h_trail = plot(ax_map, NaN, NaN, 'b-', 'LineWidth', 1.4);
h_dot = plot(ax_map, x(1), y(1), 'ko', 'MarkerSize', 11, 'MarkerFaceColor', [0.2 0.2 0.2]);
h_arrow = quiver(ax_map, x(1), y(1), 0.12*cos(theta(1)), 0.12*sin(theta(1)), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6);
st_label = text(ax_map, map_xlim(1)+0.08, map_ylim(2)-0.12, 'State: DRIVE', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.15 0.15 0.75]);

% --- Bottom subplot 1: phi ---
ax_phi = subplot(4, 1, 2, 'Parent', fig_main);
hold(ax_phi, 'on'); grid(ax_phi, 'on'); xlim(ax_phi, [sim_t(1) sim_t(end)]);
phi_pad = max(0.02, 0.1 * max(abs(phi))); ylim(ax_phi, [min(phi)-phi_pad, max(phi)+phi_pad]);
ylabel(ax_phi, '\phi (rad)'); title(ax_phi, 'Tilt angle \phi');
phi_line = animatedline(ax_phi, 'Color', '#0072BD', 'LineWidth', 1.5);

% --- Bottom subplot 2: velocity ---
ax_v = subplot(4, 1, 3, 'Parent', fig_main);
hold(ax_v, 'on'); grid(ax_v, 'on'); xlim(ax_v, [sim_t(1) sim_t(end)]);
v_min = min(min(v), min(vd_cmd)); v_max = max(max(v), max(vd_cmd));
v_pad = max(0.05, 0.1 * max(abs([v_min, v_max]))); ylim(ax_v, [v_min-v_pad, v_max+v_pad]);
ylabel(ax_v, 'v (m/s)'); title(ax_v, 'Forward velocity v');
v_line  = animatedline(ax_v, 'Color', '#0072BD', 'LineWidth', 1.5);
vd_line = animatedline(ax_v, 'Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);
legend(ax_v, 'Actual', 'Desired', 'Location', 'best');

% --- Bottom subplot 3: omega ---
ax_w = subplot(4, 1, 4, 'Parent', fig_main);
hold(ax_w, 'on'); grid(ax_w, 'on'); xlim(ax_w, [sim_t(1) sim_t(end)]);
w_min = min(min(omega), min(wd_cmd)); w_max = max(max(omega), max(wd_cmd));
w_pad = max(0.05, 0.1 * max(abs([w_min, w_max]))); ylim(ax_w, [w_min-w_pad, w_max+w_pad]);
ylabel(ax_w, '\omega (rad/s)'); xlabel(ax_w, 'Time (s)'); title(ax_w, 'Angular velocity \omega');
w_line  = animatedline(ax_w, 'Color', '#0072BD', 'LineWidth', 1.5);
wd_line = animatedline(ax_w, 'Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);
legend(ax_w, 'Actual', 'Desired', 'Location', 'best');

% --- Animation Loop ---
skip_factor = max(1, floor(num_pts / maxDriveFrames));
trail_x = []; trail_y = [];

for k = 1:skip_factor:num_pts
    if ~ishandle(fig_main) break; end
    cx = x(k); cy = y(k); ch = theta(k);
    
    trail_x(end+1) = cx; trail_y(end+1) = cy; 
    set(h_trail, 'XData', trail_x, 'YData', trail_y);
    set(h_dot, 'XData', cx, 'YData', cy);
    set(h_arrow, 'XData', cx, 'YData', cy, 'UData', 0.12*cos(ch), 'VData', 0.12*sin(ch));
    
    addpoints(phi_line, sim_t(k), phi(k));
    addpoints(v_line, sim_t(k), v(k)); addpoints(vd_line, sim_t(k), vd_cmd(k));
    addpoints(w_line, sim_t(k), omega(k)); addpoints(wd_line, sim_t(k), wd_cmd(k));
    
    drawnow; pause(pause_drive);
end

if ishandle(fig_main)
    addpoints(phi_line, sim_t(end), phi(end));
    addpoints(v_line, sim_t(end), v(end)); addpoints(vd_line, sim_t(end), vd_cmd(end));
    addpoints(w_line, sim_t(end), omega(end)); addpoints(wd_line, sim_t(end), wd_cmd(end));
    
    set(h_trail, 'XData', x, 'YData', y);
    set(h_dot, 'XData', x(end), 'YData', y(end));
    set(h_arrow, 'XData', x(end), 'YData', y(end), 'UData', 0.12*cos(theta(end)), 'VData', 0.12*sin(theta(end)));
    set(st_label, 'String', 'State: FINISHED', 'Color', [0.1 0.70 0.2]);
    text(ax_map, x(end) + 0.04, y(end) + 0.04, sprintf('Finished (%.2f, %.2f)', x(end), y(end)), 'FontSize', 10, 'Color', [0.1 0.65 0.2], 'FontWeight', 'bold');
    drawnow;
end

fprintf('\n[ANIM] Complete — drive-only animation finished at (%.3f, %.3f).\n', x(end), y(end));

%% ========================================================================
%  LOCAL FUNCTIONS
% ========================================================================
function K = compute_lqr_gain(h)
    Mw = 0.150; Mb = 11.3; r = 0.060; d = 0.450; Iw = 0.0002757224; g = 9.81;
    Lc =  -0.4748*h^2 + 1.2434*h - 0.0286;
    Iy =   1.4948*h^2 + 0.5774*h - 0.0053;
    Iz =  -0.5689*h^2 + 0.1583*h + 0.1236;
    M11   = Mb*Lc^2 + Iy; M12 = Mb*Lc;
    M22   = Mb + 2*Mw + 2*Iw/r^2;
    M33   = (d^2/2)*Mw + (d^2/(2*r^2))*Iw + Iz;
    Delta = M11*M22 - M12^2;
    A = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;
         (M22*Mb*g*Lc)/Delta  0 0 0 0 0; -(M12*Mb*g*Lc)/Delta  0 0 0 0 0; 0 0 0 0 0 0];
    B = [0 0; 0 0; 0 0; -M12/(r*Delta) -M12/(r*Delta); M11/(r*Delta) M11/(r*Delta); d/(2*r*M33) -d/(2*r*M33)];
    Q = diag([200, 5, 40, 20, 5, 10]); R = diag([2.0 2.0]);
    K = lqr(A, B, Q, R)
end