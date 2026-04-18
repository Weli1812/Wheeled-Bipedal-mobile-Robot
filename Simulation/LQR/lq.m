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
L      = 0.4;      % wheelbase [m]
Ts_ref = 0.05;     % reference sample time [s]
vd_nom = 0.3;      % nominal speed for LQR linearization
desiredVelocity = 0.2;  % target trajectory speed [m/s]

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

startXY = [9, 1];
goalXY  = [2, 9];
startPose = [startXY, pi/2];   % dummy theta for first plan
goalPose  = [goalXY,  pi/2];   % dummy theta for first plan

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

        disp('Running Hybrid A* (first pass - dummy thetas)...');
        tic;
        refPath = plan(planner, startPose, goalPose);
        planningTime = toc;

        if isempty(refPath) || isempty(refPath.States)
            error('Hybrid A* first pass failed. Check start/goal are not in obstacles.');
        end

        poses = refPath.States;

        if size(poses, 1) < 2
            error('Too few path points returned by Hybrid A*.');
        end

        % --- Auto-correct thetas from path geometry ---
        theta_start_auto = atan2(poses(2,2)-poses(1,2), poses(2,1)-poses(1,1));
        theta_goal_auto  = atan2(poses(end,2)-poses(end-1,2), poses(end,1)-poses(end-1,1));
        startPose = [startXY, theta_start_auto];
        goalPose  = [goalXY,  theta_goal_auto];
        fprintf('Auto start theta: %.2f deg\n', rad2deg(theta_start_auto));
        fprintf('Auto goal  theta: %.2f deg\n', rad2deg(theta_goal_auto));

        % --- Replan with corrected thetas ---
        disp('Replanning Hybrid A* with corrected thetas...');
        tic;
        refPath = plan(planner, startPose, goalPose);
        planningTime = toc;
        if isempty(refPath) || isempty(refPath.States)
            error('Hybrid A* failed on replan.');
        end
        poses = refPath.States;

        % -------- Original smoothing pipeline for Hybrid A* --------
        controlFrequency = 1 / Ts_ref;
        dt = 1 / controlFrequency;
        ds_target = desiredVelocity * dt;

        dx = diff(poses(:,1));
        dy = diff(poses(:,2));
        stepDistances = sqrt(dx.^2 + dy.^2);
        S = [0; cumsum(stepDistances)];

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

        dx_ref = gradient(x_ref);
        dy_ref = gradient(y_ref);

        theta_ref = atan2(dy_ref, dx_ref);
        theta_ref_unwrapped = unwrap(theta_ref);

        P = [x_ref y_ref];

    case "RRTSTAR"
    samplingFreq = 1 / Ts_ref;
    dt_sim       = Ts_ref;
    minTurningRadius = 0.9;

    % Use the already-inflated main map directly
    mapInflated = map;

    ss = stateSpaceDubins;
    ss.MinTurningRadius = minTurningRadius;
    ss.StateBounds = [mapInflated.XWorldLimits; mapInflated.YWorldLimits; [-pi pi]];

    validator = validatorOccupancyMap(ss);
    validator.Map = mapInflated;
    validator.ValidationDistance = 0.05;

    planner = plannerRRTStar(ss, validator);
    planner.MaxConnectionDistance = 0.25;
    planner.MaxIterations = 2500;

    if ~isStateValid(validator, startPose) || ~isStateValid(validator, goalPose)
        error('RRT*: Start or Goal is invalid.');
    end

    disp('Running RRT* (first pass - dummy thetas)...');
    rng('default');
    tic;
    [pthObj, solnInfo] = plan(planner, startPose, goalPose);
    planningTime = toc;

    if ~solnInfo.IsPathFound || isempty(pthObj) || isempty(pthObj.States)
        error('RRT* failed to find a feasible path.');
    end

    rawStates = pthObj.States;
    poses = rawStates;

    % --- Auto-correct thetas from path geometry ---
    theta_start_auto = atan2(rawStates(2,2)-rawStates(1,2), rawStates(2,1)-rawStates(1,1));
    theta_goal_auto  = atan2(rawStates(end,2)-rawStates(end-1,2), rawStates(end,1)-rawStates(end-1,1));
    startPose = [startXY, theta_start_auto];
    goalPose  = [goalXY,  theta_goal_auto];
    fprintf('Auto start theta: %.2f deg\n', rad2deg(theta_start_auto));
    fprintf('Auto goal  theta: %.2f deg\n', rad2deg(theta_goal_auto));

    % --- Replan with corrected thetas ---
    disp('Replanning RRT* with corrected thetas...');
    rng('default');
    tic;
    [pthObj, solnInfo] = plan(planner, startPose, goalPose);
    planningTime = toc;

    if ~solnInfo.IsPathFound || isempty(pthObj.States)
        error('RRT* failed on replan.');
    end

    rawStates = pthObj.States;
    poses     = rawStates;

    numAnchors = min(16, max(6, size(rawStates,1)-2));

    if size(rawStates,1) < 4
        warning('RRT* returned too few states for anchor-based smoothing. Using raw path directly.');
        x_ref = rawStates(:,1);
        y_ref = rawStates(:,2);
        theta_ref = unwrap(rawStates(:,3));
        theta_ref_unwrapped = theta_ref;
        P = [x_ref y_ref];
    else
        idx = unique(round(linspace(2, size(rawStates,1)-1, numAnchors)));
        middlePoints = [rawStates(idx,1)'; rawStates(idx,2)'];

        departDist   = 0.2;
        approachDist = 0.2;

        startApp = [startPose(1) + departDist * cos(startPose(3));
                    startPose(2) + departDist * sin(startPose(3))];

        goalApp  = [goalPose(1) - approachDist * cos(goalPose(3));
                    goalPose(2) - approachDist * sin(goalPose(3))];

        anchorPoints = [[startPose(1); startPose(2)], ...
                        startApp, ...
                        middlePoints, ...
                        goalApp, ...
                        [goalPose(1); goalPose(2)]];

        tSamplesRaw = linspace(0, 1, 3000);
        [qRaw, ~] = bsplinepolytraj(anchorPoints, [0 1], tSamplesRaw);

        baseX = qRaw(1,:)';
        baseY = qRaw(2,:)';

        segLen = sqrt(diff(baseX).^2 + diff(baseY).^2);
        arcLen = [0; cumsum(segLen)];
        totalPathLength = arcLen(end);
        totalTravelTime = totalPathLength / desiredVelocity;

        ds = desiredVelocity * dt_sim;
        sQuery = (0:ds:totalPathLength)';

        if sQuery(end) < totalPathLength
            sQuery = [sQuery; totalPathLength];
        end

        sampledX = interp1(arcLen, baseX, sQuery, 'pchip');
        sampledY = interp1(arcLen, baseY, sQuery, 'pchip');

        dXs = gradient(sampledX);
        dYs = gradient(sampledY);
        sampledTheta = atan2(dYs, dXs);

        blendFrames = min(15, numel(sQuery)-1);
        if blendFrames >= 1
            startBlend = sampledTheta(end - blendFrames);
            angleDiff  = atan2(sin(goalPose(3) - startBlend), cos(goalPose(3) - startBlend));
            sampledTheta(end-blendFrames:end) = ...
                startBlend + linspace(0, angleDiff, blendFrames+1)';
        end

        x_ref = sampledX;
        y_ref = sampledY;
        theta_ref = sampledTheta;
        theta_ref_unwrapped = unwrap(theta_ref);
        P = [x_ref y_ref];

        fprintf('RRT* smoothed path length: %.3f m\n', totalPathLength);
        fprintf('Estimated travel time: %.3f s\n', totalTravelTime);
    end

    otherwise
        error('Unknown plannerType. Use "HYBRID_ASTAR" or "RRTSTAR".');
end

fprintf('Selected planner: %s\n', plannerType);
fprintf('Planning time: %.4f s\n', planningTime);

%% =========================
% 5) Compare raw vs final reference
% ==========================
figure;
show(map); hold on; grid on; axis equal;

plot(poses(:,1), poses(:,2), 'r-', 'LineWidth', 2);   % raw planner output
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);          % final path used by controller

plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g');
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b');

legend('Raw Path', 'Final Path', 'Start', 'Goal');
title('Raw Planner Output vs Final Path');

figure;
show(map);
hold on;

plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);

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
wd_ref = smoothdata(wd_ref, 'gaussian', 41);

% -------- Smooth speed ramp up/down --------
t_ramp_up   = 4.0;   % seconds
t_ramp_down = 1.0;   % seconds

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
vd_max = 0.3;  % [m/s] — set to your robot's actual limit
vd_ref = min(vd_ref, vd_max);

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
    vd_nom = quantile(vd_pos, 0.75);
end

A = [0 0 0;
     0 0 vd_nom;
     0 0 0];

B = [1 0;
     0 0;
     0 1];

Q = diag([30 50 15]);
R = diag([10 3]);

K = lqr(A, B, Q, R);

%% =========================
% 10) Simulation time
% ==========================
T_end = t_vec(end) + 5.4;

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

assert(size(P,2) == 2, 'P must be Nx2');
assert(length(theta_ref_unwrapped) == size(P,1), ...
    'Size mismatch: theta_ref_unwrapped vs P');
assert(length(vd_ref) == size(P,1), ...
    'Size mismatch: vd_ref vs P');
assert(length(wd_ref) == size(P,1), ...
    'Size mismatch: wd_ref vs P');

clear out x_out y_out theta_out

%% =========================
% 12) Run Simulink model
% ==========================
out = sim('Traj2', 'StopTime', num2str(T_end));

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
title(['Robot Motion on Planned Path - ' char(plannerType)]);

step = 10;
for k = 1:step:length(x)
    set(robot, 'XData', x(k), 'YData', y(k));
    set(dirr, ...
        'XData', x(k), ...
        'YData', y(k), ...
        'UData', cos(theta(k)), ...
        'VData', sin(theta(k)));
    set(traj, 'XData', x(1:k), 'YData', y(1:k));
    drawnow;
    pause(0.00001);
end

%%
