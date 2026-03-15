clear; clc; close all;

%% =========================
% 1) Robot parameters
% ==========================
L      = 0.2;
Ts_ref = 0.1;
vd_nom = 0.3;

%% =========================
% 2) Map and PRM path
% ==========================
map = binaryOccupancyMap(10,10,20);   % higher resolution

start = [1 1];
goal  = [8 8];

% -------- Rounded obstacles (easy to avoid) --------
[X,Y] = meshgrid(0:0.05:10, 0:0.05:10);

% Obstacle 1: upper-left circle
cx1 = 3.0; cy1 = 6.8; r1 = 0.9;
mask1 = (X-cx1).^2 + (Y-cy1).^2 <= r1^2;
setOccupancy(map, [X(mask1) Y(mask1)], 1);

% Obstacle 2: middle-lower circle
cx2 = 5.0; cy2 = 3.6; r2 = 1.0;
mask2 = (X-cx2).^2 + (Y-cy2).^2 <= r2^2;
setOccupancy(map, [X(mask2) Y(mask2)], 1);

% Obstacle 3: upper-right small circle
cx3 = 6.9; cy3 = 6.4; r3 = 0.75;
mask3 = (X-cx3).^2 + (Y-cy3).^2 <= r3^2;
setOccupancy(map, [X(mask3) Y(mask3)], 1);

% Small inflation only
inflate(map,0.18);

% PRM settings
prm = mobileRobotPRM(map,250);
prm.ConnectionDistance = 4.5;

P_raw = findpath(prm,start,goal);

if isempty(P_raw)
    error('No path found');
end

figure;
show(map);
hold on;
plot(start(1),start(2),'go','MarkerSize',10,'LineWidth',2);
plot(goal(1),goal(2),'rx','MarkerSize',10,'LineWidth',2);
plot(P_raw(:,1),P_raw(:,2),'b--','LineWidth',2);
title('Raw PRM Path with Rounded Obstacles');
axis equal;
grid on;
%% =========================
% 3) Make a smooth dense path
% ==========================

% Arc length of raw path
ds_raw = sqrt(sum(diff(P_raw).^2,2));
s_raw  = [0; cumsum(ds_raw)];

% Remove duplicate arc-length points
[s_raw_unique, idx_unique] = unique(s_raw);
P_unique = P_raw(idx_unique,:);

% Dense temporary sampling
n_temp = max(300, 25*size(P_unique,1));
s_temp = linspace(0, s_raw_unique(end), n_temp)';

% Use pchip first: smoother than raw PRM, safer than spline overshoot
x_temp = pchip(s_raw_unique, P_unique(:,1), s_temp);
y_temp = pchip(s_raw_unique, P_unique(:,2), s_temp);

% Mild smoothing only
x_temp = smoothdata(x_temp, 'gaussian', 21);
y_temp = smoothdata(y_temp, 'gaussian', 21);

% Keep exact start/end
x_temp(1)   = P_raw(1,1);
y_temp(1)   = P_raw(1,2);
x_temp(end) = P_raw(end,1);
y_temp(end) = P_raw(end,2);

% Recompute arc length after smoothing
ds_smooth = sqrt(diff(x_temp).^2 + diff(y_temp).^2);
s_smooth  = [0; cumsum(ds_smooth)];

% Final geometric spacing
ds_geom = 0.02;
s_geom  = (0:ds_geom:s_smooth(end))';

if s_geom(end) < s_smooth(end)
    s_geom = [s_geom; s_smooth(end)];
end

% Final resampled smooth path
x_geom = interp1(s_smooth, x_temp, s_geom, 'pchip');
y_geom = interp1(s_smooth, y_temp, s_geom, 'pchip');

% Keep exact start/end again
x_geom(1)   = P_raw(1,1);
y_geom(1)   = P_raw(1,2);
x_geom(end) = P_raw(end,1);
y_geom(end) = P_raw(end,2);

figure;
show(map);
hold on;
plot(P_raw(:,1), P_raw(:,2), 'k--', 'LineWidth', 1.5);
plot(x_geom, y_geom, 'b', 'LineWidth', 2);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
legend('Raw PRM Path','Smoothed Path','Start','Goal');
title('Raw Path vs Smoothed Path');
axis equal;
grid on;

%% =========================
% 4) Geometry of the path
% ==========================
% First derivatives wrt arc length
dx_ds = gradient(x_geom, ds_geom);
dy_ds = gradient(y_geom, ds_geom);

% Heading along geometric path
theta_geom = atan2(dy_ds, dx_ds);
theta_geom = unwrap(theta_geom);

% Second derivatives wrt arc length
d2x_ds2 = gradient(dx_ds, ds_geom);
d2y_ds2 = gradient(dy_ds, ds_geom);

% Curvature kappa(s)
den = (dx_ds.^2 + dy_ds.^2).^(3/2);
den(den < 1e-6) = 1e-6;

kappa = (dx_ds .* d2y_ds2 - dy_ds .* d2x_ds2) ./ den;
%% =========================
% 5) Make v decrease in curves
%    and start from zero smoothly
% ==========================

% Speed limits
v_max = 0.35;      % max speed on straight path [m/s]
v_min = 0.06;      % minimum speed in motion [m/s]

% Curvature-based speed limit
a_lat_max = 0.08;  % smaller value = slower in curves
kappa_eps = 1e-3;

vd_curve = sqrt(a_lat_max ./ (abs(kappa) + kappa_eps));

% Respect straight/low-speed limits
vd_geom = min(vd_curve, v_max);
vd_geom = max(vd_geom, v_min);

% Extra slowdown in very sharp curves
idx_sharp = abs(kappa) > 0.35;
vd_geom(idx_sharp) = min(vd_geom(idx_sharp), 0.10);

% Smooth speed profile
vd_geom = movmean(vd_geom, 21);

% ---------- Start from zero smoothly ----------
s_geom_path = s_geom;    % path coordinate from section 3
L_path = s_geom_path(end);

start_ramp_distance = 0.1;   % first 0.8 m: ramp from 0
idx_start = s_geom_path <= start_ramp_distance;

n_start = sum(idx_start);
if n_start > 1
    v_start_target = vd_geom(find(idx_start,1,'last'));
    ramp_start = linspace(0, v_start_target, n_start)';
    vd_geom(idx_start) = min(vd_geom(idx_start), ramp_start);
end

vd_geom(1) = 0;

% ---------- Slow down near the goal ----------
s_remain = L_path - s_geom_path;

stop_distance = 0.8;   % start braking in last 0.8 m
a_stop = 0.12;         % braking aggressiveness

idx_stop = s_remain <= stop_distance;
vd_stop = sqrt(2 * a_stop * max(s_remain,0));

vd_geom(idx_stop) = min(vd_geom(idx_stop), vd_stop(idx_stop));
vd_geom(end) = 0;

% Final smoothing
vd_geom = movmean(vd_geom, 11);

% Keep exact start/end after smoothing
vd_geom(1) = 0;
vd_geom(end) = 0;

% Allow zero only at start/end
vd_geom = min(vd_geom, v_max);
vd_geom = max(vd_geom, 0);

% ---------- Build time from this speed profile ----------
ds_seg = sqrt(diff(x_geom).^2 + diff(y_geom).^2);
v_mid  = 0.5 * (vd_geom(1:end-1) + vd_geom(2:end));
v_mid(v_mid < 1e-3) = 1e-3;

dt_seg = ds_seg ./ v_mid;
t_geom = [0; cumsum(dt_seg)];

% Optional: slow the whole trajectory more
time_scale = 1.2;   % >1 = slower overall
t_geom = time_scale * t_geom;

% Uniform time for Simulink
t_ref = (0:Ts_ref:t_geom(end))';
if t_ref(end) < t_geom(end)
    t_ref = [t_ref; t_geom(end)];
end

% Final time-based reference trajectory
x_ref     = interp1(t_geom, x_geom,     t_ref, 'pchip');
y_ref     = interp1(t_geom, y_geom,     t_ref, 'pchip');
theta_ref = interp1(t_geom, theta_geom, t_ref, 'pchip');
theta_ref = unwrap(theta_ref);

% Final reference path
P = [x_ref y_ref];

% Final desired speed and angular speed
vd_ref = interp1(t_geom, vd_geom, t_ref, 'pchip');
vd_ref = movmean(vd_ref, 9);
vd_ref(1) = 0;
vd_ref(end) = 0;

wd_ref = gradient(theta_ref, Ts_ref);
wd_ref = movmean(wd_ref, 9);
wd_ref(end) = 0;

% Limit angular speed
wd_max = 1.2;
wd_ref = max(min(wd_ref, wd_max), -wd_max);

% Initial heading
theta0 = theta_ref(1);

% Stop time
T_end = t_ref(end) + 5;
%% =========================
% 6) Constant LQR design
% ==========================
A = [0 0 0;
     0 0 vd_nom;
     0 0 0];

B = [1 0;
     0 0;
     0 1];

Q = diag([3 5 4]);
R = diag([6 6]);

K = lqr(A,B,Q,R);

%% =========================
% 7) Simulation time
% ==========================
T_end = (size(P,1)-1)*Ts_ref + 5;

%% =========================
% 8) Clear old simulation outputs
% ==========================
clear out x_out y_out theta_out

%% =========================
% 9) Run Simulink model
% ==========================
out = sim('Traj', 'StopTime', num2str(T_end));

%% =========================
% 10) Read fresh outputs
% ==========================
x = out.x_out;
y = out.y_out;
theta = out.theta_out;

%% =========================
% 11) Plot robot path
% ==========================
figure;
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);
hold on;
plot(x, y, 'b', 'LineWidth', 2);
plot(x(1), y(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(x(end), y(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
title('Reference Path vs Robot Path');
legend('Reference Path','Robot Path','Start','End');

%% =========================
% 12) Animate robot motion
% ==========================
figure;
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);
hold on;
grid on;
axis equal;
xlabel('x (m)');
ylabel('y (m)');
title('Robot Motion');

robot = plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
dirr = quiver(x(1), y(1), cos(theta(1)), sin(theta(1)), 0.4, 'r', 'LineWidth', 2);
traj = plot(x(1), y(1), 'b', 'LineWidth', 2);

for k = 1:length(x)
    set(robot, 'XData', x(k), 'YData', y(k));
    set(dirr, ...
        'XData', x(k), ...
        'YData', y(k), ...
        'UData', cos(theta(k)), ...
        'VData', sin(theta(k)));
    set(traj, 'XData', x(1:k), 'YData', y(1:k));
    drawnow;
end

%% =========================
% 13) v_d and w_d
% ==========================

figure;
subplot(2,1,1);
plot(t_ref, vd_ref, 'LineWidth', 1.5);
grid on;
ylabel('v_d [m/s]');
title('Reference Velocities');

subplot(2,1,2);
plot(t_ref, wd_ref, 'LineWidth', 1.5);
grid on;
ylabel('w_d [rad/s]');
xlabel('Time [s]');