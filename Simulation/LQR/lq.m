clear; clc; close all;

%% =========================
% 0) Choose planner
% ==========================
% Options:
%   "HYBRID_ASTAR"
%   "RRTSTAR"
plannerType = "HYBRID_ASTAR";

%% =========================
% 1) Robot parameters
% ==========================
L      = 0.2;      % wheelbase [m]
Ts_ref = 0.05;     % reference sample time [s]
vd_nom = 0.3;      % nominal speed for LQR linearization
desiredVelocity = 0.25;  % target trajectory speed [m/s]

%% =========================
% 2) Goal stop parameters
% ==========================
goalPosTol   = 0.08;          % [m]
goalThetaTol = deg2rad(8);    % [rad]

%% =========================
% 3) Build occupancy map for planner
% ==========================
resolution = 10;
map = binaryOccupancyMap(10, 10, resolution);

% Obstacle 1: lower-left vertical block
[X1, Y1] = meshgrid(1.0:0.1:2.0, 1.0:0.1:3.8);
setOccupancy(map, [X1(:) Y1(:)], 1);

% Obstacle 2: upper-left block
[X2, Y2] = meshgrid(1.5:0.1:3.2, 5.2:0.1:7.8);
setOccupancy(map, [X2(:) Y2(:)], 1);

% Obstacle 3: center-lower block
[X3, Y3] = meshgrid(4.0:0.1:5.8, 2.0:0.1:4.2);
setOccupancy(map, [X3(:) Y3(:)], 1);

% Obstacle 4: center-upper block
[X4, Y4] = meshgrid(4.8:0.1:6.4, 5.3:0.1:7.2);
setOccupancy(map, [X4(:) Y4(:)], 1);

% Obstacle 5: right-lower block
[X5, Y5] = meshgrid(7.0:0.1:8.8, 1.2:0.1:3.2);
setOccupancy(map, [X5(:) Y5(:)], 1);

% Obstacle 6: right-upper block
[X6, Y6] = meshgrid(7.2:0.1:8.9, 6.2:0.1:8.8);
setOccupancy(map, [X6(:) Y6(:)], 1);

% Obstacle 7: narrow corridor maker
[X7, Y7] = meshgrid(3.0:0.1:3.8, 3.8:0.1:5.8);
setOccupancy(map, [X7(:) Y7(:)], 1);

inflate(map, 0.2);

startPose = [9, 1, pi/2];
goalPose  = [2, 9, pi/2];




%% =========================
% 4) Plan path using selected planner
% ==========================
switch upper(plannerType)

    case "HYBRID_ASTAR"
        stateSpace = stateSpaceSE2;
        stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

        validator = validatorOccupancyMap(stateSpace);
        validator.Map = map;
        validator.ValidationDistance = 0.1;

        planner = plannerHybridAStar(validator, ...
            'MinTurningRadius', 0.9, ...
            'MotionPrimitiveLength', 0.8);
        planner.ReverseCost = 1e20;

        if ~isStateValid(validator, startPose) || ~isStateValid(validator, goalPose)
            error('Hybrid A*: Start or Goal is invalid.');
        end

        disp('Running Hybrid A*...');
        tic;
        refPath = plan(planner, startPose, goalPose);
        planningTime = toc;

        if isempty(refPath) || isempty(refPath.States)
            error('Hybrid A* failed to find a feasible path.');
        end

        poses = refPath.States;

    case "RRTSTAR"
        ss = stateSpaceDubins;
        ss.MinTurningRadius = 0.9;
        ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

        validator = validatorOccupancyMap(ss);
        validator.Map = map;
        validator.ValidationDistance = 0.05;

        planner = plannerRRTStar(ss, validator);
        planner.MaxConnectionDistance = 0.25;
        planner.MaxIterations = 2500;

        if ~isStateValid(validator, startPose) || ~isStateValid(validator, goalPose)
            error('RRT*: Start or Goal is invalid.');
        end

        disp('Running RRT*...');
        rng('default');
        tic;
        [pthObj, solnInfo] = plan(planner, startPose, goalPose);
        planningTime = toc;

        if ~solnInfo.IsPathFound || isempty(pthObj) || isempty(pthObj.States)
            error('RRT* failed to find a feasible path.');
        end

        poses = pthObj.States;

    otherwise
        error('Unknown plannerType. Use "HYBRID_ASTAR" or "RRTSTAR".');
end

fprintf('Selected planner: %s\n', plannerType);
fprintf('Planning time: %.4f s\n', planningTime);

%% =========================
% 5) Smooth planner output and build timed reference
% ==========================
controlFrequency = 1 / Ts_ref;
dt = 1 / controlFrequency;
ds_target = desiredVelocity * dt;

dx = diff(poses(:,1));
dy = diff(poses(:,2));
stepDistances = sqrt(dx.^2 + dy.^2);
S = [0; cumsum(stepDistances)];

% Remove duplicate arc-length points
[S, uniqueIdx] = unique(S, 'stable');
poses = poses(uniqueIdx, :);

if numel(S) < 2
    error('Not enough unique path points after preprocessing.');
end

S_equi = (0:ds_target:S(end))';
if S_equi(end) < S(end)
    S_equi = [S_equi; S(end)];
end

x_ref = spline(S, poses(:,1), S_equi);
y_ref = spline(S, poses(:,2), S_equi);

% unwrap before interpolation, then keep unwrapped for reference generation
dx_ref = gradient(x_ref);
dy_ref = gradient(y_ref);

theta_ref = atan2(dy_ref, dx_ref);
theta_ref_unwrapped = unwrap(theta_ref);

P = [x_ref y_ref];

figure;
show(map);
hold on;

% Plot planned (smoothed) path
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);

% Plot start and goal
plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);

title(['Occupancy Map with Planned Path - ' char(plannerType)]);
xlabel('x (m)');
ylabel('y (m)');
grid on;
axis equal;

legend('Planned Path','Start','Goal');

%% =========================
% 6) Derive v and w from timed reference
% ==========================
ds = sqrt(diff(x_ref).^2 + diff(y_ref).^2);
ds = [ds; ds(end)];

vd_ref = ds / Ts_ref;
vd_ref = smoothdata(vd_ref, 'gaussian', 9);

wd_ref = gradient(theta_ref_unwrapped) / Ts_ref;
wd_ref = smoothdata(wd_ref, 'gaussian', 21);

% -------- Smooth speed ramp up/down --------
t_ramp_up   = 2.5;   % seconds
t_ramp_down = 3.0;   % seconds

t_vec = (0:length(vd_ref)-1)' * Ts_ref;
T_total = t_vec(end);

rampUpScale   = min(t_vec / t_ramp_up, 1);
rampDownScale = min((T_total - t_vec) / t_ramp_down, 1);
rampDownScale = max(rampDownScale, 0);

speedScale = min(rampUpScale, rampDownScale);

vd_ref = vd_ref .* speedScale;
wd_ref = wd_ref .* speedScale;

% -------- Enforce valid start/end values --------
vd_ref(vd_ref < 0) = 0;
vd_ref(1) = 0;
vd_ref(end) = 0;
wd_ref(1) = 0;
wd_ref(end) = 0;

% -------- Make tail monotonic decreasing --------
tailTime = 4.0;  % seconds before end
tailStartIdx = max(1, floor((T_total - tailTime)/Ts_ref) + 1);

for k = tailStartIdx+1:length(vd_ref)
    vd_ref(k) = min(vd_ref(k), vd_ref(k-1));
end
%% =========================
% 7) Start/end conditions and limits
% ==========================
vd_ref(1)   = 0;
vd_ref(end) = 0;
wd_ref(1)   = 0;
wd_ref(end) = 0;

wd_max = 0.25;
wd_ref = max(min(wd_ref, wd_max), -wd_max);

theta0 = theta_ref(1);
x0     = x_ref(1);
y0     = y_ref(1);
start  = [x0 y0];

%% =========================
% 8) Collision check
% ==========================
occ = getOccupancy(map, P);
if any(occ > 0.5)
    warning('Reference trajectory intersects an obstacle.');
end

%% =========================
% 9) Constant LQR design
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

% Softer than before to reduce oscillation
Q = diag([30 50 8]);
R = diag([6 3]);

K = lqr(A, B, Q, R);

%% =========================
% 10) Simulation time
% ==========================
T_end = t_vec(end) + 4;

%% =========================
% 11) Prepare signals for Simulink
% ==========================
P                    = double(P);
theta_ref_unwrapped  = double(theta_ref_unwrapped(:));
theta_ref            = double(theta_ref(:));
vd_ref               = double(vd_ref(:));
wd_ref               = double(wd_ref(:));
Ts_ref               = double(Ts_ref);
x0                   = double(x0);
y0                   = double(y0);
theta0               = double(theta0);
start                = double(start);
goalPose             = double(goalPose(:)');
goalPosTol           = double(goalPosTol);
goalThetaTol         = double(goalThetaTol);

clear out x_out y_out theta_out

%% =========================
% 12) Run Simulink model
% ==========================
out = sim('Traj', 'StopTime', num2str(T_end));

%% =========================
% 13) Read outputs
% ==========================
x     = out.x_out(:);
y     = out.y_out(:);
theta = out.theta_out(:);

%% =========================
% 14) Animate robot motion only
% ==========================
figure;
show(map);
hold on;

% Plot only the planned/reference path
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);

% Plot start and goal points
plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g');
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b');

% Robot body, heading arrow, and traced actual path
robot = plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
dirr  = quiver(x(1), y(1), cos(theta(1)), sin(theta(1)), 0.4, 'r', 'LineWidth', 2);
traj  = plot(x(1), y(1), 'b', 'LineWidth', 2);

axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title(['Robot Motion on Planned Path - ' char(plannerType)]);

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