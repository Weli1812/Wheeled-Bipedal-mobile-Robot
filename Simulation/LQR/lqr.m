clear; clc; close all;

%% =========================
% 1) Robot parameters
% ==========================
L      = 0.2;      % wheelbase [m]
Ts_ref = 0.05;     % reference sample time [s]
vd_nom = 0.3;      % nominal speed for LQR linearization
desiredVelocity = 0.25;  % planner trajectory speed target [m/s]

%% =========================
% 2) Build occupancy map for planner
% ==========================
gridSize = 100;
mapMatrix = zeros(gridSize, gridSize);
mapMatrix(10:22,15:20) = 1;
mapMatrix(35:45,55:65) = 1;
mapMatrix(55:70,18:28) = 1;
mapMatrix(72:85,72:78) = 1;
mapMatrix(25:32,78:90) = 1;

resolution = 10;   % cells per meter
map = binaryOccupancyMap(flipud(mapMatrix), resolution);
inflate(map, 0.45);

%% =========================
% 3) Plan path using Hybrid A*
% ==========================
stateSpace = stateSpaceSE2;
stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

validator = validatorOccupancyMap(stateSpace);
validator.Map = map;
validator.ValidationDistance = 0.1;

planner = plannerHybridAStar(validator, ...
    'MinTurningRadius', 0.9, ...
    'MotionPrimitiveLength', 0.8);
planner.ReverseCost = 1e20;

startPose = [9, 1, pi/2];
goalPose  = [5, 9, pi/2];

refPath = plan(planner, startPose, goalPose);
poses = refPath.States;

if isempty(poses)
    error('Hybrid A* failed to find a feasible path.');
end

%% =========================
% 4) Smooth planner output and build timed reference
% ==========================
controlFrequency = 1 / Ts_ref;
dt = 1 / controlFrequency;
ds_target = desiredVelocity * dt;

dx = diff(poses(:,1));
dy = diff(poses(:,2));
stepDistances = sqrt(dx.^2 + dy.^2);
S = [0; cumsum(stepDistances)];

S_equi = (0:ds_target:S(end))';
if S_equi(end) < S(end)
    S_equi = [S_equi; S(end)];
end

x_ref = spline(S, poses(:,1), S_equi);
y_ref = spline(S, poses(:,2), S_equi);

theta_unwrapped = unwrap(poses(:,3));
theta_ref = spline(S, theta_unwrapped, S_equi);
theta_ref = wrapToPi(theta_ref);

t_ref = S_equi / desiredVelocity;
P = [x_ref y_ref];

%% =========================
% 5) Derive v and w from timed reference
% ==========================
ds = sqrt(diff(x_ref).^2 + diff(y_ref).^2);
ds = [ds; ds(end)];

vd_ref = ds / Ts_ref;
vd_ref = smoothdata(vd_ref, 'gaussian', 9);

wd_ref = gradient(unwrap(theta_ref)) / Ts_ref;
wd_ref = smoothdata(wd_ref, 'gaussian', 21);

%% =========================
% 6) Start/end conditions and initial state
% ==========================

% Start/end conditions
vd_ref(1)   = 0;
vd_ref(end) = 0;
wd_ref(end) = 0;

% Limit angular speed
wd_max = 0.3;
wd_ref = max(min(wd_ref, wd_max), -wd_max);

% Initial conditions
theta0 = theta_ref(1);
x0     = x_ref(1);
y0     = y_ref(1);
start  = [x0 y0];

%% =========================
% 7) Collision check
% ==========================
occ = getOccupancy(map, P);
if any(occ > 0.5)
    warning('Reference trajectory intersects an obstacle.');
end

%% =========================
% 8) Constant LQR design
% ==========================
vd_pos = vd_ref(vd_ref > 0.02);
if ~isempty(vd_pos)
    vd_nom = mean(vd_pos);
end

A = [0 0 0;
     0 0 vd_nom;
     0 0 0];

B = [1 0;
     0 0;
     0 1];

Q = diag([20 30 10]);
R = diag([5 2]);

K = lqr(A,B,Q,R);

%% =========================
% 9) Simulation time
% ==========================
T_end = t_ref(end) + 6;

%% =========================
% 10) Prepare signals
% ==========================
P         = double(P);
theta_ref = double(theta_ref(:));
vd_ref    = double(vd_ref(:));
wd_ref    = double(wd_ref(:));
Ts_ref    = double(Ts_ref);
x0        = double(x0);
y0        = double(y0);
theta0    = double(theta0);
start     = double(start);

clear out x_out y_out theta_out

%% =========================
% 11) Run Simulink model
% ==========================
out = sim('Traj', 'StopTime', num2str(T_end));

%% =========================
% 12) Read outputs
% ==========================
x     = out.x_out(:);
y     = out.y_out(:);
theta = out.theta_out(:);

%% =========================
% 13) Animate robot motion only
% ==========================
figure;
show(map);
hold on;
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);
plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g');
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b');

robot = plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
dirr  = quiver(x(1), y(1), cos(theta(1)), sin(theta(1)), 0.4, 'r', 'LineWidth', 2);
traj  = plot(x(1), y(1), 'b', 'LineWidth', 2);

axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Robot Motion on Reference Path');

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