clear; clc; close all;

%% =========================
% 0) Choose planner
% ==========================
% Options:
%   "HYBRID_ASTAR"
%   "RRTSTAR"
%   "CUSTOM_KINEMATIC"
plannerType = "CUSTOM_KINEMATIC";

%% =========================
% 1) Robot parameters
% ==========================
L_wb   = 0.4;      % wheelbase [m]
vd_nom = 0.2;      % constant desired forward speed [m/s]

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

inflate(map, 0.2);

startXY = [9, 1];
goalXY  = [2, 9];
startPose = [startXY, pi/2];
goalPose  = [goalXY,  pi/2];

%% =========================
% 4) Plan path using selected planner
% ==========================
switch upper(plannerType)

    case "CUSTOM_KINEMATIC"
        disp('Running Custom Kinematic A* Planner...');
        tic;

        kinematics.min_turn      = 0.9;
        kinematics.step_distance = 0.4;
        max_steer = kinematics.step_distance / kinematics.min_turn;
        kinematics.steer_angles  = [-max_steer, 0.0, max_steer];
        kinematics.prim_costs    = [1.05, 1.0, 1.05];

        [raw_path, path_len] = PlanKinematicPath_World(map, startPose, goalPose, kinematics);
        planningTime1 = toc;

        if path_len < 2
            error('Custom Kinematic Planner failed to find a path.');
        end

        poses = SmoothPath_World(raw_path, 0.2, 0.2, 0.0001, map);
        planningTime2 = 0;

        disp('Custom Planner successful. Passing to Spatial Resampler...');

        ds_spatial = 0.02;
        dx = diff(poses(:,1));
        dy = diff(poses(:,2));
        stepDistances = sqrt(dx.^2 + dy.^2);
        S_raw = [0; cumsum(stepDistances)];
        [S_raw, uniqueIdx] = unique(S_raw, 'stable');
        poses = poses(uniqueIdx, :);

        if numel(S_raw) < 2
            error('Not enough unique path points after preprocessing.');
        end

        S_path = (0 : ds_spatial : S_raw(end))';
        if S_path(end) < S_raw(end)
            S_path = [S_path; S_raw(end)];
        end

        x_ref = spline(S_raw, poses(:,1), S_path);
        y_ref = spline(S_raw, poses(:,2), S_path);
        dx_ref = gradient(x_ref, ds_spatial);
        dy_ref = gradient(y_ref, ds_spatial);
        theta_ref = atan2(dy_ref, dx_ref);
        theta_ref_unwrapped = unwrap(theta_ref);
        P = [x_ref y_ref];

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
        planningTime1 = toc;

        if isempty(refPath) || isempty(refPath.States)
            error('Hybrid A* first pass failed.');
        end
        poses = refPath.States;
        if size(poses, 1) < 2
            error('Too few path points returned by Hybrid A*.');
        end

        theta_start_auto = atan2(poses(2,2)-poses(1,2), poses(2,1)-poses(1,1));
        theta_goal_auto  = atan2(poses(end,2)-poses(end-1,2), poses(end,1)-poses(end-1,1));
        startPose = [startXY, theta_start_auto];
        goalPose  = [goalXY,  theta_goal_auto];

        disp('Replanning Hybrid A* with corrected thetas...');
        tic;
        refPath = plan(planner, startPose, goalPose);
        planningTime2 = toc;

        if isempty(refPath) || isempty(refPath.States)
            error('Hybrid A* failed on replan.');
        end
        poses = refPath.States;

        ds_spatial = 0.02;
        dx = diff(poses(:,1));
        dy = diff(poses(:,2));
        stepDistances = sqrt(dx.^2 + dy.^2);
        S_raw = [0; cumsum(stepDistances)];
        [S_raw, uniqueIdx] = unique(S_raw, 'stable');
        poses = poses(uniqueIdx, :);

        if numel(S_raw) < 2
            error('Not enough unique path points after preprocessing.');
        end

        S_path = (0 : ds_spatial : S_raw(end))';
        if S_path(end) < S_raw(end)
            S_path = [S_path; S_raw(end)];
        end
        x_ref = spline(S_raw, poses(:,1), S_path);
        y_ref = spline(S_raw, poses(:,2), S_path);
        dx_ref = gradient(x_ref, ds_spatial);
        dy_ref = gradient(y_ref, ds_spatial);
        theta_ref = atan2(dy_ref, dx_ref);
        theta_ref_unwrapped = unwrap(theta_ref);
        P = [x_ref y_ref];

    case "RRTSTAR"
        minTurningRadius = 0.9;
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

        disp('Running RRT* (first pass)...');
        rng('default');
        tic;
        [pthObj, solnInfo] = plan(planner, startPose, goalPose);
        planningTime1 = toc;

        if ~solnInfo.IsPathFound || isempty(pthObj) || isempty(pthObj.States)
            error('RRT* failed to find a feasible path.');
        end

        rawStates = pthObj.States;
        poses = rawStates;

        theta_start_auto = atan2(rawStates(2,2)-rawStates(1,2), rawStates(2,1)-rawStates(1,1));
        theta_goal_auto  = atan2(rawStates(end,2)-rawStates(end-1,2), rawStates(end,1)-rawStates(end-1,1));
        startPose = [startXY, theta_start_auto];
        goalPose  = [goalXY,  theta_goal_auto];

        disp('Replanning RRT* with corrected thetas...');
        rng('default');
        tic;
        [pthObj, solnInfo] = plan(planner, startPose, goalPose);
        planningTime2 = toc;

        if ~solnInfo.IsPathFound || isempty(pthObj.States)
            error('RRT* failed on replan.');
        end

        rawStates = pthObj.States;
        poses     = rawStates;
        numAnchors = min(16, max(6, size(rawStates,1)-2));

        if size(rawStates,1) < 4
            warning('RRT* returned too few states. Using raw path directly.');
            x_base = rawStates(:,1);
            y_base = rawStates(:,2);
        else
            idx = unique(round(linspace(2, size(rawStates,1)-1, numAnchors)));
            middlePoints = [rawStates(idx,1)'; rawStates(idx,2)'];
            departDist   = 0.2;
            approachDist = 0.2;
            startApp = [startPose(1) + departDist * cos(startPose(3));
                        startPose(2) + departDist * sin(startPose(3))];
            goalApp  = [goalPose(1) - approachDist * cos(goalPose(3));
                        goalPose(2) - approachDist * sin(goalPose(3))];
            anchorPoints = [[startPose(1); startPose(2)], startApp, ...
                            middlePoints, goalApp, [goalPose(1); goalPose(2)]];
            tSamplesRaw = linspace(0, 1, 3000);
            [qRaw, ~] = bsplinepolytraj(anchorPoints, [0 1], tSamplesRaw);
            x_base = qRaw(1,:)';
            y_base = qRaw(2,:)';
        end

        ds_spatial = 0.02;
        segLen = sqrt(diff(x_base).^2 + diff(y_base).^2);
        S_raw  = [0; cumsum(segLen)];
        totalPathLength = S_raw(end);
        S_path = (0 : ds_spatial : totalPathLength)';
        if S_path(end) < totalPathLength
            S_path = [S_path; totalPathLength];
        end

        x_ref = interp1(S_raw, x_base, S_path, 'pchip');
        y_ref = interp1(S_raw, y_base, S_path, 'pchip');
        dXs = gradient(x_ref, ds_spatial);
        dYs = gradient(y_ref, ds_spatial);
        theta_ref = atan2(dYs, dXs);

        blendPoints = min(15, numel(S_path)-1);
        if blendPoints >= 1
            startBlend = theta_ref(end - blendPoints);
            angleDiff  = atan2(sin(goalPose(3) - startBlend), cos(goalPose(3) - startBlend));
            theta_ref(end-blendPoints:end) = ...
                startBlend + linspace(0, angleDiff, blendPoints+1)';
        end
        theta_ref_unwrapped = unwrap(theta_ref);
        P = [x_ref y_ref];
        fprintf('RRT* smoothed path length: %.3f m\n', totalPathLength);

    otherwise
        error('Unknown plannerType.');
end

fprintf('Selected planner: %s\n', plannerType);
fprintf('Planning time (pass 1): %.4f s\n', planningTime1);
fprintf('Planning time (pass 2): %.4f s\n', planningTime2);

%% =========================
% 5) Compare raw vs final reference
% ==========================
figure;
show(map); hold on; grid on; axis equal;
plot(poses(:,1), poses(:,2), 'r-', 'LineWidth', 2);
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);
plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g');
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b');
legend('Raw Path', 'Final Path', 'Start', 'Goal');
title('Raw Planner Output vs Final Path');

figure;
show(map); hold on;
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);
plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
title(['Occupancy Map with Planned Path - ' char(plannerType)]);
xlabel('x (m)'); ylabel('y (m)');
grid on; axis equal;
legend('Planned Path','Start','Goal');

%% =========================
% 6) Build SPATIAL reference table
% ==========================
kappa_ref = gradient(theta_ref_unwrapped, ds_spatial);
kappa_ref = smoothdata(kappa_ref, 'gaussian', 51);

dkappa_max = 0.05;
for k = 2:numel(kappa_ref)
    dkappa = kappa_ref(k) - kappa_ref(k-1);
    kappa_ref(k) = kappa_ref(k-1) + sign(dkappa)*min(abs(dkappa), dkappa_max);
end

r_min     = 0.9;
kappa_max = 1 / r_min;
kappa_ref = max(min(kappa_ref, kappa_max), -kappa_max);

totalPathLength = S_path(end);
s_ramp_up   = min(0.4, totalPathLength * 0.35);
s_ramp_down = min(0.4, totalPathLength * 0.35);
a_ramp = vd_nom^2 / (2 * s_ramp_up);
v_kickstart = 0.02;

vd_path = zeros(size(S_path));
for k = 1:numel(S_path)
    s     = S_path(k);
    s_rem = totalPathLength - s;
    if s <= s_ramp_up
        vd_path(k) = max(sqrt(2 * a_ramp * s), v_kickstart);
    elseif s_rem <= s_ramp_down
        vd_path(k) = sqrt(2 * a_ramp * max(s_rem, 0));
    else
        vd_path(k) = vd_nom;
    end
end
vd_path(end) = 0;

wd_path = vd_path .* kappa_ref;
wd_path = smoothdata(wd_path, 'gaussian', 31);

vd_max  = 0.3;
wd_max  = vd_max / r_min;
vd_path = min(vd_path, vd_max);
wd_path = max(min(wd_path, wd_max), -wd_max);

fprintf('Total path length      : %.3f m\n', totalPathLength);
fprintf('Number of path points  : %d\n',     numel(S_path));
fprintf('Spatial resolution ds  : %.4f m\n', ds_spatial);
fprintf('Max curvature          : %.4f rad/m\n', max(abs(kappa_ref)));
fprintf('Peak desired speed     : %.4f m/s\n',   max(vd_path));

%% =========================
% 7) Collision check
% ==========================
occ = getOccupancy(map, P);
if any(occ > 0.5)
    warning('Reference trajectory intersects an obstacle.');
end

%% =========================
% 9) Fixed height for simulation
% ==========================
h_ref = 0.17;

%% =========================
% 10) Precompute LQR gain
% ==========================
K_lqr = compute_lqr_gain(h_ref);
fprintf('LQR gain precomputed for h = %.3f m\n', h_ref);

%% =========================
% 11) Initial conditions
% ==========================
phi0     = deg2rad(0);
s0       = 0;
theta0   = theta_ref_unwrapped(1);
phi_dot0 = 0;
v0       = 0;
omega0   = 0;
x0       = P(1,1);
y0       = P(1,2);
x_init = [phi0; s0; theta0; phi_dot0; v0; omega0; x0; y0];

%% =========================
% 12) Simulation stop time
% ==========================
T_est = totalPathLength / vd_nom;
T_end = T_est * 1.5 + 5.0;

%% =========================
% 13) Prepare lookup tables for Simulink
% ==========================
S_path               = double(S_path(:));
x_ref                = double(x_ref(:));
y_ref                = double(y_ref(:));
theta_ref_unwrapped  = double(theta_ref_unwrapped(:));
kappa_ref            = double(kappa_ref(:));
vd_path              = double(vd_path(:));
wd_path              = double(wd_path(:));
P                    = double(P);
h_ref                = double(h_ref);
x_init               = double(x_init);
K_lqr                = double(K_lqr);

Npath = numel(S_path);
assert(numel(x_ref)               == Npath, 'Size mismatch: x_ref');
assert(numel(y_ref)               == Npath, 'Size mismatch: y_ref');
assert(numel(theta_ref_unwrapped) == Npath, 'Size mismatch: theta_ref_unwrapped');
assert(numel(kappa_ref)           == Npath, 'Size mismatch: kappa_ref');
assert(numel(vd_path)             == Npath, 'Size mismatch: vd_path');
assert(numel(wd_path)             == Npath, 'Size mismatch: wd_path');

fprintf('\n--- Spatial reference table ready ---\n');
fprintf('S_path range : 0 to %.3f m  (%d points)\n', S_path(end), Npath);
clear out x_out y_out theta_out phi_out v_out omega_out

%% =========================
% 14) Dynamic obstacle supervisor  <<<  UPDATED LOGIC  >>>
%
%  Flow:
%   (A) No obstacle seen       → continue on original path normally
%   (B) Obstacle seen, removed → decel until removal time → explicit 1 s constant-speed hold
%       before/at T_wait          → then ramp up → original path
%   (C) Obstacle seen, still   → decel until T_wait → try MOVING replan from current pose
%       after T_wait             → explicit 1 s constant-speed hold on new path → ramp up
%                                → if moving replan fails, continue braking to full stop → replan
% ==========================

% --- Save originals ---
map_original             = copy(map);
S_path_original          = S_path;
x_ref_original           = x_ref;
y_ref_original           = y_ref;
theta_ref_original       = theta_ref_unwrapped;
kappa_ref_original       = kappa_ref;
vd_path_original         = vd_path;
wd_path_original         = wd_path;
P_original               = P;

% --- Supervisor settings ---
T_detect        = 10.0;   % time when obstacle appears [s]
T_wait          = 2.0;    % planner/decision window [s]
T_obs_clear     = 1.5;    % obstacle is removed this many seconds after appearance [s]
obstacleAutoClear = false; % test case: obstacle disappears during braking
T_hold          = 1.0;    % speed-hold duration after obstacle clears [s]
sensorRange     = 1.5;    % detection range [m]
safeStopMargin  = 0.3;    % guaranteed stop distance before obstacle [m]
obsLookAhead    = 1.35;   % obstacle placed far enough for smooth braking [m]
obsHalfSize     = 0.35;   % obstacle half-size [m]

fprintf('\n====================================================\n');
fprintf('   Dynamic obstacle supervisor — SMOOTH DECEL\n');
fprintf('====================================================\n');

%% ---- Segment 1: Run freely until T_detect ----
out1 = sim('OneCombinedLQR', 'StopTime', num2str(T_detect));
seg1 = readSimSegment(out1, 0, false);

x10      = seg1.x(end);     y10     = seg1.y(end);
theta10  = seg1.theta(end); phi10   = seg1.phi(end);
v10      = seg1.v(end);     omega10 = seg1.omega(end);
fprintf('Seg1 done  | t=%.1f s | pos=(%.3f, %.3f) | v=%.4f m/s\n', ...
        T_detect, x10, y10, v10);

%% ---- Inject dynamic obstacle ----
mapDyn   = copy(map_original);
k_detect = nearestPathIndex(P_original, [x10, y10]);
s_obs    = min(S_path_original(end), S_path_original(k_detect) + obsLookAhead);
[~, k_obs] = min(abs(S_path_original - s_obs));
obsCenter  = P_original(k_obs, :);

[Xnew, Ynew] = meshgrid(obsCenter(1)-obsHalfSize : 0.05 : obsCenter(1)+obsHalfSize, ...
                         obsCenter(2)-obsHalfSize : 0.05 : obsCenter(2)+obsHalfSize);
setOccupancy(mapDyn, [Xnew(:) Ynew(:)], 1);
fprintf('Obstacle   | injected at (%.3f, %.3f)\n', obsCenter(1), obsCenter(2));
if obstacleAutoClear
    fprintf('Obstacle   | scheduled to be removed at t = %.1f s\n', T_detect + T_obs_clear);
end

%% ---- Sensor check at T_detect ----
laIdx1       = find(S_path_original >= S_path_original(k_detect) & ...
                    S_path_original <= S_path_original(k_detect) + sensorRange);
occ1         = getOccupancy(mapDyn, P_original(laIdx1, :));
obstacleSeen = any(occ1 > 0.5);

% Compute physics-based deceleration distance from detection point to obstacle
d_stop = safeStopMargin;   % fallback default
if obstacleSeen
    firstOccLocal = find(occ1 > 0.5, 1, 'first');
    firstOccIdx   = laIdx1(firstOccLocal);
    d_to_obs      = S_path_original(firstOccIdx) - S_path_original(k_detect);
    d_stop        = max(0.05, d_to_obs - safeStopMargin);
    fprintf('Distance to obstacle on path: %.3f m | Stop distance available: %.3f m\n', ...
            d_to_obs, d_stop);
end

% Pre-initialise empty segments (used in assembly regardless of path taken)
segDecel   = emptySegment();
segStop    = emptySegment();
segHold    = emptySegment();
usedReplan = false;
stoppedBeforeReplan = false;
obstacleRemoved = false;

% =========================================================
if ~obstacleSeen
% =========================================================
%  CASE A — No obstacle in sensor range → continue normally
% =========================================================
    fprintf('CASE A     | No obstacle detected — continuing original path.\n');

    % Trim the remaining original path
    [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
     vd_path, wd_path, P] = extractRemainingPath(...
         S_path_original, x_ref_original, y_ref_original, ...
         theta_ref_original, kappa_ref_original, ...
         vd_path_original, wd_path_original, P_original, k_detect);

    x_init = [phi10; 0; theta10; 0; v10; omega10; x10; y10];
    Npath  = numel(S_path);
    T_end2 = max(5.0, S_path(end)/vd_nom*1.5 + 5.0);

    out2 = sim('OneCombinedLQR', 'StopTime', num2str(T_end2));
    seg2 = readSimSegment(out2, T_detect, true);

% =========================================================
else
% =========================================================
%  CASES B & C — Obstacle detected → start smooth deceleration
% =========================================================
    fprintf('Obstacle!  | Detected — beginning smooth deceleration for %.1f s...\n', T_wait);

    % Build a ramp-down speed profile on the remaining original path
    % Robot continues tracking the same path but slows down smoothly.
    [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
     vd_path, wd_path, P] = buildDecelReference(...
         P_original, S_path_original, x_ref_original, y_ref_original, ...
         theta_ref_original, kappa_ref_original, k_detect, v10, d_stop);

    x_init = [phi10; 0; theta10; 0; v10; omega10; x10; y10];
    Npath  = numel(S_path);

    % Decide how long to keep decelerating before the next supervisor decision.
    % For this test, the obstacle is removed after T_obs_clear seconds.
    % If it clears before the 2-second planning window ends, we react at removal time.
    if obstacleAutoClear && T_obs_clear > 0 && T_obs_clear <= T_wait
        T_decelRun = T_obs_clear;
    else
        T_decelRun = T_wait;
    end

    % Simulate deceleration until either obstacle removal or the normal T_wait decision time
    outDecel = sim('OneCombinedLQR', 'StopTime', num2str(T_decelRun));
    segDecel = readSimSegment(outDecel, T_detect, true);

    x_aw     = segDecel.x(end);     y_aw     = segDecel.y(end);
    theta_aw = segDecel.theta(end); phi_aw   = segDecel.phi(end);
    v_aw     = segDecel.v(end);     omega_aw = segDecel.omega(end);
    t_aw     = T_detect + T_decelRun;
    fprintf('After decel| t=%.1f s | pos=(%.3f, %.3f) | v=%.4f m/s\n', ...
            t_aw, x_aw, y_aw, v_aw);

    % ---- Remove obstacle if its lifetime has finished ----
    if obstacleAutoClear && T_decelRun >= T_obs_clear
        mapDyn = copy(map_original); % remove the temporary dynamic obstacle from the map
        obstacleRemoved = true;
        fprintf('Obstacle   | removed at t = %.1f s\n', T_detect + T_obs_clear);
    end

    % ---- Re-check obstacle after deceleration/removal event ----
    k_aw   = nearestPathIndex(P_original, [x_aw, y_aw]);
    laIdx2 = find(S_path_original >= S_path_original(k_aw) & ...
                  S_path_original <= S_path_original(k_aw) + sensorRange);
    occ2   = getOccupancy(mapDyn, P_original(laIdx2, :));
    obstacleStillExists = any(occ2 > 0.5);

    % =====================================================
    if ~obstacleStillExists
    % =====================================================
    %  CASE B — Obstacle cleared during decel window
    %            → hold speed T_hold s, then ramp up → original path
    % =====================================================
        fprintf('CASE B     | Obstacle cleared/removed! Holding v=%.4f m/s for exactly %.1f s, then ramping up.\n', ...
                v_aw, T_hold);

        % -------------------------------------------------
        % Explicit recovery behavior requested:
        % 1) keep the speed constant at the speed reached after braking
        %    for exactly T_hold seconds
        % 2) only after that, ramp up again
        % -------------------------------------------------
        [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
         vd_path, wd_path, P] = buildConstantSpeedReference(...
             P_original, S_path_original, x_ref_original, y_ref_original, ...
             theta_ref_original, kappa_ref_original, k_aw, v_aw);

        x_init = [phi_aw; 0; theta_aw; 0; v_aw; omega_aw; x_aw; y_aw];
        Npath  = numel(S_path);

        outHold = sim('OneCombinedLQR', 'StopTime', num2str(T_hold));
        segHold = readSimSegment(outHold, t_aw, true);

        x_h     = segHold.x(end);     y_h     = segHold.y(end);
        theta_h = segHold.theta(end); phi_h   = segHold.phi(end);
        v_h     = segHold.v(end);     omega_h = segHold.omega(end);
        t_h     = t_aw + T_hold;
        fprintf('Hold done  | t=%.1f s | pos=(%.3f, %.3f) | v=%.4f m/s\n', ...
                t_h, x_h, y_h, v_h);

        k_h = nearestPathIndex(P_original, [x_h, y_h]);
        [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
         vd_path, wd_path, P] = buildRampUpReference(...
             P_original, S_path_original, x_ref_original, y_ref_original, ...
             theta_ref_original, kappa_ref_original, k_h, v_h, vd_nom);

        x_init = [phi_h; 0; theta_h; 0; v_h; omega_h; x_h; y_h];
        Npath  = numel(S_path);
        T_end2 = max(5.0, S_path(end)/vd_nom*1.5 + 5.0);

        out2 = sim('OneCombinedLQR', 'StopTime', num2str(T_end2));
        seg2 = readSimSegment(out2, t_h, true);

        usedReplan = false;

    % =====================================================
    else
    % =====================================================
    %  CASE C — Obstacle still present after T_wait
    %            → try MOVING replan from current pose/current speed
    %            → if moving replan fails, continue braking to full stop
    % =====================================================
        fprintf('CASE C     | Obstacle still present — trying moving replan from current pose...\n');

        % -------------------------------------------------
        % Realistic behavior:
        % T_wait is only the planning/decision window.
        % The robot does NOT need to be fully stopped after T_wait.
        % If a safe new path exists, switch smoothly to it using current speed.
        % -------------------------------------------------
        movingReplanSuccess = false;
        newStartPose = [x_aw, y_aw, theta_aw];

        try
            fprintf('Replanning | moving start from (%.3f, %.3f), v=%.4f m/s...\n', ...
                    x_aw, y_aw, v_aw);

            [~, S_replan, x_replan, y_replan, theta_replan, ...
             kappa_replan, vd_replan, wd_replan, P_replan] = ...
                replanCustomKinematic(mapDyn, newStartPose, goalPose, vd_nom, v_aw);

            fprintf('Recovery   | new path ready — holding current speed %.4f m/s for exactly %.1f s before ramp-up.\n', ...
                    v_aw, T_hold);

            % First hold the reached speed on the new path for exactly T_hold seconds.
            [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
             vd_path, wd_path, P] = buildConstantSpeedReference(...
                 P_replan, S_replan, x_replan, y_replan, ...
                 theta_replan, kappa_replan, 1, v_aw);

            x_init = [phi_aw; 0; theta_aw; 0; v_aw; omega_aw; x_aw; y_aw];
            Npath  = numel(S_path);

            outHold = sim('OneCombinedLQR', 'StopTime', num2str(T_hold));
            segHold = readSimSegment(outHold, t_aw, true);

            x_h     = segHold.x(end);     y_h     = segHold.y(end);
            theta_h = segHold.theta(end); phi_h   = segHold.phi(end);
            v_h     = segHold.v(end);     omega_h = segHold.omega(end);
            t_h     = t_aw + T_hold;
            fprintf('Hold done  | t=%.1f s | pos=(%.3f, %.3f) | v=%.4f m/s\n', ...
                    t_h, x_h, y_h, v_h);

            % Then ramp up smoothly from the speed reached after the hold.
            k_h = nearestPathIndex(P_replan, [x_h, y_h]);
            [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
             vd_path, wd_path, P] = buildRampUpReference(...
                 P_replan, S_replan, x_replan, y_replan, ...
                 theta_replan, kappa_replan, k_h, v_h, vd_nom);

            x_init = [phi_h; 0; theta_h; 0; v_h; omega_h; x_h; y_h];
            Npath  = numel(S_path);
            T_end2 = max(5.0, S_path(end)/vd_nom*1.5 + 5.0);

            out2 = sim('OneCombinedLQR', 'StopTime', num2str(T_end2));
            seg2 = readSimSegment(out2, t_h, true);

            usedReplan = true;
            movingReplanSuccess = true;
            fprintf('Moving replan successful | held speed for %.1f s, then ramped up on new path.\n', T_hold);

        catch ME
            fprintf('Moving replan failed: %s\n', ME.message);
            fprintf('Fallback   | Continue braking to full stop, then replan.\n');
        end

        if ~movingReplanSuccess
            % Continue decel from current speed v_aw → 0
            [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
             vd_path, wd_path, P] = buildFullStopReference(...
                 P_original, S_path_original, x_ref_original, y_ref_original, ...
                 theta_ref_original, kappa_ref_original, k_aw, v_aw);

            x_init = [phi_aw; 0; theta_aw; 0; v_aw; omega_aw; x_aw; y_aw];
            Npath  = numel(S_path);

            % Estimate time needed to reach full stop
            a_stop_est = max(0.03, v_aw * 0.5);
            T_stop_est = max(2.5, v_aw / a_stop_est + 1.5);

            outStop  = sim('OneCombinedLQR', 'StopTime', num2str(T_stop_est));
            segStop  = readSimSegment(outStop, t_aw, true);

            x_st     = segStop.x(end);     y_st     = segStop.y(end);
            theta_st = segStop.theta(end); phi_st   = segStop.phi(end);
            v_st     = segStop.v(end);     omega_st = segStop.omega(end);
            fprintf('Full stop  | pos=(%.3f, %.3f) | v=%.4f m/s\n', x_st, y_st, v_st);

            % ---- Replan from stopped position ----
            fprintf('Replanning | stopped start from (%.3f, %.3f)...\n', x_st, y_st);
            newStartPose = [x_st, y_st, theta_st];

            [~, S_replan, x_replan, y_replan, theta_replan, ...
             kappa_replan, vd_replan, wd_replan, P_replan] = ...
                replanCustomKinematic(mapDyn, newStartPose, goalPose, vd_nom, 0.0);

            timeOffset_hold = t_aw + T_stop_est;

            % Even after a full stop, keep the recovery rule consistent:
            % hold current speed (zero) for exactly T_hold seconds, then ramp up.
            [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
             vd_path, wd_path, P] = buildConstantSpeedReference(...
                 P_replan, S_replan, x_replan, y_replan, ...
                 theta_replan, kappa_replan, 1, 0.0);

            x_init = [phi_st; 0; theta_st; 0; 0; 0; x_st; y_st];
            Npath  = numel(S_path);

            outHold = sim('OneCombinedLQR', 'StopTime', num2str(T_hold));
            segHold = readSimSegment(outHold, timeOffset_hold, true);

            x_h     = segHold.x(end);     y_h     = segHold.y(end);
            theta_h = segHold.theta(end); phi_h   = segHold.phi(end);
            v_h     = segHold.v(end);     omega_h = segHold.omega(end);
            t_h     = timeOffset_hold + T_hold;
            fprintf('Hold done  | t=%.1f s | pos=(%.3f, %.3f) | v=%.4f m/s\n', ...
                    t_h, x_h, y_h, v_h);

            k_h = nearestPathIndex(P_replan, [x_h, y_h]);
            [S_path, x_ref, y_ref, theta_ref_unwrapped, kappa_ref, ...
             vd_path, wd_path, P] = buildRampUpReference(...
                 P_replan, S_replan, x_replan, y_replan, ...
                 theta_replan, kappa_replan, k_h, v_h, vd_nom);

            x_init = [phi_h; 0; theta_h; 0; v_h; omega_h; x_h; y_h];
            Npath  = numel(S_path);
            T_end2 = max(5.0, S_path(end)/vd_nom*1.5 + 5.0);

            out2   = sim('OneCombinedLQR', 'StopTime', num2str(T_end2));
            seg2   = readSimSegment(out2, t_h, true);

            usedReplan = true;
            stoppedBeforeReplan = true;
        end
    end
end

%% ---- Final variable cast for Simulink (safety) ----
S_path              = double(S_path(:));
x_ref               = double(x_ref(:));
y_ref               = double(y_ref(:));
theta_ref_unwrapped = double(theta_ref_unwrapped(:));
kappa_ref           = double(kappa_ref(:));
vd_path             = double(vd_path(:));
wd_path             = double(wd_path(:));
P                   = double(P);
Npath               = numel(S_path);

%% ---- Assemble all segments ----
sim_t  = [seg1.t;      segDecel.t;      segStop.t;      segHold.t;      seg2.t];
x      = [seg1.x;      segDecel.x;      segStop.x;      segHold.x;      seg2.x];
y      = [seg1.y;      segDecel.y;      segStop.y;      segHold.y;      seg2.y];
theta  = [seg1.theta;  segDecel.theta;  segStop.theta;  segHold.theta;  seg2.theta];
phi    = [seg1.phi;    segDecel.phi;    segStop.phi;    segHold.phi;    seg2.phi];
v      = [seg1.v;      segDecel.v;      segStop.v;      segHold.v;      seg2.v];
omega  = [seg1.omega;  segDecel.omega;  segStop.omega;  segHold.omega;  seg2.omega];
vd_cmd = [seg1.vd;     segDecel.vd;     segStop.vd;     segHold.vd;     seg2.vd];
wd_cmd = [seg1.wd;     segDecel.wd;     segStop.wd;     segHold.wd;     seg2.wd];

map = copy(mapDyn);

fprintf('\n====================================================\n');
if ~obstacleSeen
    fprintf('Result: CASE A — No obstacle, continued normally.\n');
elseif ~usedReplan
    fprintf('Result: CASE B — Obstacle removed after %.1f s, decelerated, held + ramped up.\n', T_obs_clear);
elseif stoppedBeforeReplan
    fprintf('Result: CASE C — Moving replan failed, decelerated to stop, then replanned.\n');
else
    fprintf('Result: CASE C — Decelerated during planning, held speed for 1 s, then switched/ramped on new path.\n');
end
fprintf('====================================================\n');

%% =========================
% Dynamic obstacle result plot
% ==========================
figure;
show(map); hold on; grid on; axis equal;

plot(P_original(:,1), P_original(:,2), 'g--', 'LineWidth', 2);
plot(x, y, 'b', 'LineWidth', 2);
if obstacleRemoved
    plot(obsCenter(1), obsCenter(2), 'mx', 'MarkerSize', 14, 'LineWidth', 3);
    obstacleLabel = 'Removed Obstacle Location';
else
    plot(obsCenter(1), obsCenter(2), 'rx', 'MarkerSize', 14, 'LineWidth', 3);
    obstacleLabel = 'New Obstacle';
end

if usedReplan
    plot(P(:,1), P(:,2), 'm--', 'LineWidth', 2);
    legend('Original Reference', 'Robot Path', obstacleLabel, 'Replanned Path');
    title('Dynamic Obstacle: Decel → Hold → Moving Replan / Fallback Stop  [Case C]');
elseif obstacleSeen
    legend('Original Reference', 'Robot Path', obstacleLabel);
    title('Dynamic Obstacle: Obstacle Removed → Hold → Ramp Up → Continue  [Case B]');
else
    legend('Original Reference', 'Robot Path', obstacleLabel);
    title('Dynamic Obstacle: No Obstacle Detected  [Case A]');
end

plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g');
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b');
xlabel('x (m)'); ylabel('y (m)');

%% =========================
% 16) Plots
% ==========================
figure;
show(map); hold on;
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);
plot(x, y, 'b', 'LineWidth', 2);
plot(startPose(1), startPose(2), 'go', 'MarkerFaceColor', 'g');
plot(goalPose(1), goalPose(2), 'bo', 'MarkerFaceColor', 'b');
legend('Reference Path','Robot Path','Start','Goal');
title('Spatial Reference — One Combined LQR');
grid on; axis equal;

figure;
subplot(3,1,1);
plot(sim_t, phi(1:length(sim_t)), 'LineWidth', 1.5);
grid on; ylabel('\phi (rad)'); title('Tilt angle \phi');
subplot(3,1,2);
plot(sim_t, v(1:length(sim_t)), 'LineWidth', 1.5); hold on;
plot(sim_t, vd_cmd(1:length(sim_t)), 'r--', 'LineWidth', 1);
grid on; ylabel('v (m/s)'); legend('Actual','Desired'); title('Forward velocity v');
subplot(3,1,3);
plot(sim_t, omega(1:length(sim_t)), 'LineWidth', 1.5); hold on;
plot(sim_t, wd_cmd(1:length(sim_t)), 'r--', 'LineWidth', 1);
grid on; ylabel('\omega (rad/s)'); legend('Actual','Desired');
title('Angular velocity \omega'); xlabel('Time (s)');

figure;
subplot(2,1,1);
plot(S_path, vd_path, 'b', 'LineWidth', 1.5);
grid on; xlabel('Arc-length s (m)'); ylabel('v_d (m/s)');
title('Desired speed profile — arc-length indexed');
subplot(2,1,2);
plot(S_path, kappa_ref, 'r', 'LineWidth', 1.5);
grid on; xlabel('Arc-length s (m)'); ylabel('\kappa (rad/m)');
title('Path curvature — arc-length indexed');

%% =========================
% Animated Figure: States & Physical Tilt
% ==========================
figure('Name', 'Animated States and Tilt', 'Position', [100, 100, 900, 800]);
s_act = cumtrapz(sim_t, v);

ax1 = subplot(4,1,1);
hold on; grid on; axis equal;
ylim([-0.1 0.6]);
title('Physical Robot Tilt (\phi) Animation');
ground_line = plot([-0.5 0.5], [0 0], 'k', 'LineWidth', 2);
wheel_dot   = plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
body_len    = 0.4;
body_line   = plot([0 0], [0 body_len], 'b-', 'LineWidth', 5);

subplot(4,1,2); hold on; grid on;
xlim([0 T_end]); ylim([min(phi)-0.02, max(phi)+0.02]);
ylabel('\phi (rad)'); title('Tilt angle \phi');
phi_line = animatedline('Color', '#0072BD', 'LineWidth', 1.5);

subplot(4,1,3); hold on; grid on;
xlim([0 T_end]);
ylim([min(min(v), min(vd_cmd))-0.05, max(max(v), max(vd_cmd))+0.05]);
ylabel('v (m/s)'); title('Forward velocity v');
v_line  = animatedline('Color', '#0072BD', 'LineWidth', 1.5);
vd_line = animatedline('Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);
legend('Actual', 'Desired', 'Location', 'best');

subplot(4,1,4); hold on; grid on;
xlim([0 T_end]);
ylim([min(min(omega), min(wd_cmd))-0.05, max(max(omega), max(wd_cmd))+0.05]);
ylabel('\omega (rad/s)'); xlabel('Time (s)'); title('Angular velocity \omega');
w_line  = animatedline('Color', '#0072BD', 'LineWidth', 1.5);
wd_line = animatedline('Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);

num_points  = length(sim_t);
skip_factor = max(1, floor(num_points / 500));
for k = 1:skip_factor:num_points
    current_phi = phi(k);
    current_s   = s_act(k);
    set(wheel_dot,  'XData', current_s, 'YData', 0);
    set(ground_line,'XData', [current_s - 0.5, current_s + 0.5]);
    x_body = current_s + body_len * sin(current_phi);
    y_body = body_len * cos(current_phi);
    set(body_line, 'XData', [current_s, x_body], 'YData', [0, y_body]);
    xlim(ax1, [current_s - 0.5, current_s + 0.5]);
    addpoints(phi_line, sim_t(k), current_phi);
    addpoints(v_line,   sim_t(k), v(k));
    addpoints(vd_line,  sim_t(k), vd_cmd(k));
    addpoints(w_line,   sim_t(k), omega(k));
    addpoints(wd_line,  sim_t(k), wd_cmd(k));
    drawnow;
end
addpoints(phi_line, sim_t(end), phi(end));
addpoints(v_line,   sim_t(end), v(end));
addpoints(vd_line,  sim_t(end), vd_cmd(end));
addpoints(w_line,   sim_t(end), omega(end));
addpoints(wd_line,  sim_t(end), wd_cmd(end));
final_s = s_act(end);
set(wheel_dot, 'XData', final_s, 'YData', 0);
set(body_line, 'XData', [final_s, final_s + body_len*sin(phi(end))], ...
               'YData', [0, body_len*cos(phi(end))]);
xlim(ax1, [final_s - 0.5, final_s + 0.5]);
drawnow;

%% ========================================================================
%  LOCAL FUNCTIONS
% ========================================================================

% -------------------------------------------------------------------------
%  LQR gain
% -------------------------------------------------------------------------
function K = compute_lqr_gain(h)
    Mw = 0.150; Mb = 11.3; r = 0.060; d = 0.450;
    Iw = 0.0002757224; g = 9.81;
    Lc =  -0.4748*h^2 + 1.2434*h - 0.0286;
    Iy =   1.4948*h^2 + 0.5774*h - 0.0053;
    Iz =  -0.5689*h^2 + 0.1583*h + 0.1236;
    M11   = Mb*Lc^2 + Iy;
    M12   = Mb*Lc;
    M22   = Mb + 2*Mw + 2*Iw/r^2;
    M33   = (d^2/2)*Mw + (d^2/(2*r^2))*Iw + Iz;
    Delta = M11*M22 - M12^2;
    A = [0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;
         (M22*Mb*g*Lc)/Delta  0 0 0 0 0;
        -(M12*Mb*g*Lc)/Delta  0 0 0 0 0;
         0 0 0 0 0 0];
    B = [0 0; 0 0; 0 0;
        -M12/(r*Delta)  -M12/(r*Delta);
         M11/(r*Delta)   M11/(r*Delta);
         d/(2*r*M33)    -d/(2*r*M33)];
    Q = diag([200, 5, 40, 20, 5, 10]);
    R = diag([2.0 2.0]);
    K = lqr(A, B, Q, R);
end

% -------------------------------------------------------------------------
%  Custom Kinematic A* planner
% -------------------------------------------------------------------------
function [path_out, path_len] = PlanKinematicPath_World(map, start, goal, kin)
    MAX_NODES   = 150000;
    THETA_BINS  = 36;
    spatial_res = 4;
    h_weight    = 2.0;
    x_max = map.XWorldLimits(2);
    y_max = map.YWorldLimits(2);
    grid_x_size = ceil(x_max * spatial_res) + 1;
    grid_y_size = ceil(y_max * spatial_res) + 1;
    closed_list = false(grid_x_size, grid_y_size, THETA_BINS);
    n_pose   = zeros(MAX_NODES, 3);
    n_g      = zeros(MAX_NODES, 1);
    n_f      = zeros(MAX_NODES, 1);
    n_parent = zeros(MAX_NODES, 1);
    n_open   = false(MAX_NODES, 1);
    n_pose(1,:) = start;
    n_g(1)      = 0;
    n_f(1)      = h_weight * CalcHeuristic(start, goal, kin.min_turn);
    n_parent(1) = 0;
    n_open(1)   = true;
    active_nodes    = 1;
    best_node_idx   = -1;
    path_found      = false;
    while active_nodes < MAX_NODES - 5
        open_idxs = find(n_open(1:active_nodes));
        if isempty(open_idxs), break; end
        [~, min_f_idx] = min(n_f(open_idxs));
        current_idx    = open_idxs(min_f_idx);
        n_open(current_idx) = false;
        curr_pose = n_pose(current_idx, :);
        dist       = norm(goal(1:2) - curr_pose(1:2));
        diff_theta = mod((curr_pose(3) - goal(3)) + pi, 2*pi) - pi;
        if dist < 0.8 && abs(diff_theta) < 0.8
            best_node_idx = current_idx;
            path_found = true; break;
        end
        cx   = round(curr_pose(1) * spatial_res) + 1;
        cy   = round(curr_pose(2) * spatial_res) + 1;
        c_th = GetThetaBin(curr_pose(3), THETA_BINS) + 1;
        if cx >= 1 && cx <= grid_x_size && cy >= 1 && cy <= grid_y_size
            closed_list(cx, cy, c_th) = true;
        end
        for i = 1:3
            next_pose    = zeros(1,3);
            next_pose(3) = mod(curr_pose(3) + kin.steer_angles(i), 2*pi);
            next_pose(1) = curr_pose(1) + kin.step_distance * cos(next_pose(3));
            next_pose(2) = curr_pose(2) + kin.step_distance * sin(next_pose(3));
            if next_pose(1)<0||next_pose(1)>=x_max||next_pose(2)<0||next_pose(2)>=y_max
                continue;
            end
            mid_x = curr_pose(1) + (kin.step_distance/2) * cos(next_pose(3));
            mid_y = curr_pose(2) + (kin.step_distance/2) * sin(next_pose(3));
            if mid_x<0||mid_x>=x_max||mid_y<0||mid_y>=y_max, continue; end
            if checkOccupancy(map,[next_pose(1),next_pose(2)])>0.5 || ...
               checkOccupancy(map,[mid_x,mid_y])>0.5, continue; end
            nx   = round(next_pose(1) * spatial_res) + 1;
            ny   = round(next_pose(2) * spatial_res) + 1;
            n_th = GetThetaBin(next_pose(3), THETA_BINS) + 1;
            if closed_list(nx, ny, n_th), continue; end
            active_nodes = active_nodes + 1;
            tentative_g  = n_g(current_idx) + kin.prim_costs(i);
            n_pose(active_nodes,:) = next_pose;
            n_g(active_nodes)      = tentative_g;
            n_f(active_nodes)      = tentative_g + h_weight*CalcHeuristic(next_pose,goal,kin.min_turn);
            n_parent(active_nodes) = current_idx;
            n_open(active_nodes)   = true;
        end
    end
    if ~path_found, path_out=[]; path_len=0; return; end
    current_trace = best_node_idx;
    raw_path = [];
    while current_trace > 0
        raw_path      = [n_pose(current_trace,:); raw_path];
        current_trace = n_parent(current_trace);
    end
    path_out = raw_path;
    path_len = size(path_out, 1);
end

function h = CalcHeuristic(curr, goal, min_turn_rad)
    dx   = goal(1) - curr(1);
    dy   = goal(2) - curr(2);
    dist = sqrt(dx^2 + dy^2);
    if dist < 0.1, h = 0.0; return; end
    angle_to_goal = atan2(dy, dx);
    diff_start    = mod((curr(3) - angle_to_goal) + pi, 2*pi) - pi;
    diff_end      = mod((goal(3) - angle_to_goal) + pi, 2*pi) - pi;
    turn_penalty  = min_turn_rad * (abs(diff_start) + abs(diff_end));
    h = dist + turn_penalty;
end

function bin = GetThetaBin(angle, bins)
    angle = mod(angle, 2*pi);
    if angle < 0, angle = angle + 2*pi; end
    bin = floor(angle / (2*pi / bins));
    if bin >= bins || bin < 0, bin = 0; end
end

function path = SmoothPath_World(path, d_weight, s_weight, tol, map)
    len = size(path, 1);
    if len <= 4, return; end
    orig = path; change = tol; iters = 0;
    while change >= tol && iters < 1000
        change = 0.0;
        for i = 2:(len - 1)
            aux   = path(i, 1:2);
            new_x = path(i,1) + d_weight*(orig(i,1)-path(i,1)) + ...
                    s_weight*(path(i-1,1)+path(i+1,1)-2*path(i,1));
            new_y = path(i,2) + d_weight*(orig(i,2)-path(i,2)) + ...
                    s_weight*(path(i-1,2)+path(i+1,2)-2*path(i,2));
            try
                if checkOccupancy(map, [new_x, new_y]) < 0.5
                    path(i,1) = new_x; path(i,2) = new_y;
                    change = change + abs(aux(1)-path(i,1)) + abs(aux(2)-path(i,2));
                end
            catch
            end
        end
        iters = iters + 1;
    end
    for i = 1:(len-1)
        path(i,3) = atan2(path(i+1,2)-path(i,2), path(i+1,1)-path(i,1));
    end
    path(len,3) = path(len-1,3);
end

% -------------------------------------------------------------------------
%  Utility
% -------------------------------------------------------------------------
function idx = nearestPathIndex(P, robotXY)
    dx = P(:,1) - robotXY(1);
    dy = P(:,2) - robotXY(2);
    [~, idx] = min(dx.^2 + dy.^2);
end

function seg = readSimSegment(out, timeOffset, removeFirstPoint)
    t     = out.tout(:)       + timeOffset;
    x     = out.x_out(:);
    y     = out.y_out(:);
    theta = out.theta_out(:);
    phi   = out.phi_out(:);
    v     = out.v_out(:);
    omega = out.omega_out(:);
    vd    = out.vd_cmd_out(:);
    wd    = out.wd_cmd_out(:);
    if removeFirstPoint && numel(t) > 1
        t=t(2:end); x=x(2:end); y=y(2:end); theta=theta(2:end);
        phi=phi(2:end); v=v(2:end); omega=omega(2:end);
        vd=vd(2:end); wd=wd(2:end);
    end
    seg.t=t; seg.x=x; seg.y=y; seg.theta=theta;
    seg.phi=phi; seg.v=v; seg.omega=omega; seg.vd=vd; seg.wd=wd;
end

function seg = emptySegment()
    seg.t=[]; seg.x=[]; seg.y=[]; seg.theta=[];
    seg.phi=[]; seg.v=[]; seg.omega=[]; seg.vd=[]; seg.wd=[];
end

% -------------------------------------------------------------------------
%  Reference builders  (NEW)
% -------------------------------------------------------------------------

function [S_path, x_ref, y_ref, theta_ref_uw, kappa_ref, vd_path, wd_path, P] = ...
    buildDecelReference(P_orig, S_orig, x_orig, y_orig, theta_orig, kappa_orig, ...
                        k_start, v0, d_stop)
% buildDecelReference — Smooth deceleration profile.
%
%  Deceleration 'a' is computed from kinematics so the robot reaches v=0
%  exactly at arc-length d_stop from the detection point.
%  This guarantees the robot always stops at least (safeStopMargin) metres
%  before the obstacle, regardless of detection speed.
%
%  Speed law:  v(s) = sqrt( max(0,  v0^2 - 2*a*s) )

    r_min  = 0.9;  vd_max = 0.3;  wd_max = vd_max / r_min;

    S_rem        = S_orig(k_start:end) - S_orig(k_start);
    x_ref        = x_orig(k_start:end);
    y_ref        = y_orig(k_start:end);
    theta_ref_uw = theta_orig(k_start:end);
    kappa_ref    = kappa_orig(k_start:end);
    P            = P_orig(k_start:end, :);
    S_path       = S_rem;

    % --- Physics-based deceleration ---
    % a = v0^2 / (2 * d_stop)  ensures v=0 at exactly s = d_stop
    a_decel = v0^2 / (2 * max(d_stop, 0.02));
    fprintf('Decel profile | a = %.4f m/s^2 | will stop in %.3f m\n', a_decel, d_stop);

    vd_path = zeros(size(S_rem));
    for k = 1:numel(S_rem)
        vd_path(k) = sqrt(max(0,  v0^2 - 2*a_decel*S_rem(k)));
    end
    vd_path(end) = 0;

    vd_path = min(vd_path, vd_max);
    wd_path = vd_path .* kappa_ref;
    wd_path = smoothdata(wd_path, 'gaussian', 31);
    wd_path = max(min(wd_path, wd_max), -wd_max);
end

% -------------------------------------------------------------------------
function [S_path, x_ref, y_ref, theta_ref_uw, kappa_ref, vd_path, wd_path, P] = ...
    buildFullStopReference(P_orig, S_orig, x_orig, y_orig, theta_orig, kappa_orig, k_start, v0)
% buildFullStopReference  — continue decelerating to complete standstill.
% Called in Case C after the T_wait window when obstacle is still there.
% Uses a more aggressive decel so the stop happens quickly.

    r_min  = 0.9;  vd_max = 0.3;  wd_max = vd_max / r_min;

    S_rem       = S_orig(k_start:end) - S_orig(k_start);
    x_ref       = x_orig(k_start:end);
    y_ref       = y_orig(k_start:end);
    theta_ref_uw= theta_orig(k_start:end);
    kappa_ref   = kappa_orig(k_start:end);
    P           = P_orig(k_start:end, :);
    S_path      = S_rem;

    % Aggressive stop: v reaches 0 within 30 % of remaining path
    stop_dist = max(0.2, min(S_rem(end) * 0.3, v0 * 1.5));
    if v0 < 1e-4
        vd_path = zeros(size(S_rem));
    else
        a_decel = v0^2 / (2 * stop_dist);
        vd_path = zeros(size(S_rem));
        for k = 1:numel(S_rem)
            vd_path(k) = sqrt(max(0, v0^2 - 2*a_decel*S_rem(k)));
        end
    end
    vd_path(end) = 0;

    vd_path = min(vd_path, vd_max);
    wd_path = vd_path .* kappa_ref;
    wd_path = smoothdata(wd_path, 'gaussian', 31);
    wd_path = max(min(wd_path, wd_max), -wd_max);
end

% -------------------------------------------------------------------------
function [S_path, x_ref, y_ref, theta_ref_uw, kappa_ref, vd_path, wd_path, P] = ...
    buildConstantSpeedReference(P_orig, S_orig, x_orig, y_orig, theta_orig, kappa_orig, ...
                                k_start, v_const)
% buildConstantSpeedReference
%   Explicit recovery hold profile.
%   The robot follows the remaining geometry while the desired forward speed
%   is kept constant. The supervisor runs this reference for exactly T_hold seconds.

    r_min  = 0.9;
    vd_max = 0.3;
    wd_max = vd_max / r_min;

    S_path       = S_orig(k_start:end) - S_orig(k_start);
    x_ref        = x_orig(k_start:end);
    y_ref        = y_orig(k_start:end);
    theta_ref_uw = theta_orig(k_start:end);
    kappa_ref    = kappa_orig(k_start:end);
    P            = P_orig(k_start:end, :);

    v_const = max(0.0, min(v_const, vd_max));
    vd_path = v_const * ones(size(S_path));

    % If v_const is zero, angular command must also be zero.
    wd_path = vd_path .* kappa_ref;
    wd_path = smoothdata(wd_path, 'gaussian', 31);
    wd_path = max(min(wd_path, wd_max), -wd_max);
end

% -------------------------------------------------------------------------
function [S_path, x_ref, y_ref, theta_ref_uw, kappa_ref, vd_path, wd_path, P] = ...
    buildRampUpReference(P_orig, S_orig, x_orig, y_orig, theta_orig, kappa_orig, ...
                         k_start, v_start, vd_nom)
% buildRampUpReference
%   Used after the explicit 1-second constant-speed hold.
%   Starts from the reached speed and then smoothly blends to vd_nom.

    r_min  = 0.9;
    vd_max = 0.3;
    wd_max = vd_max / r_min;

    S_path       = S_orig(k_start:end) - S_orig(k_start);
    x_ref        = x_orig(k_start:end);
    y_ref        = y_orig(k_start:end);
    theta_ref_uw = theta_orig(k_start:end);
    kappa_ref    = kappa_orig(k_start:end);
    P            = P_orig(k_start:end, :);

    totalRemaining = S_path(end);
    s_ramp_up      = min(0.8, max(0.25, totalRemaining * 0.25));
    s_ramp_down    = min(0.4, totalRemaining * 0.35);
    a_ramp_down    = vd_nom^2 / (2 * max(s_ramp_down, 0.01));

    v_start = max(0.0, min(v_start, vd_max));
    vd_path = zeros(size(S_path));

    for k = 1:numel(S_path)
        s     = S_path(k);
        s_rem = totalRemaining - s;

        % Smoothstep ramp from current speed to nominal speed.
        if s <= s_ramp_up
            lambda = s / max(s_ramp_up, 1e-6);
            smooth_lambda = lambda^2 * (3 - 2*lambda);
            vd_path(k) = (1 - smooth_lambda)*v_start + smooth_lambda*vd_nom;
        else
            vd_path(k) = vd_nom;
        end

        % Final goal stop ramp.
        if s_rem <= s_ramp_down
            v_end = sqrt(2 * a_ramp_down * max(s_rem, 0));
            vd_path(k) = min(vd_path(k), v_end);
        end
    end

    vd_path(end) = 0;
    vd_path = min(vd_path, vd_max);

    wd_path = vd_path .* kappa_ref;
    wd_path = smoothdata(wd_path, 'gaussian', 31);
    wd_path = max(min(wd_path, wd_max), -wd_max);
end

% -------------------------------------------------------------------------
function [S_path, x_ref, y_ref, theta_ref_uw, kappa_ref, vd_path, wd_path, P] = ...
    buildHoldAndRampReference(P_orig, S_orig, x_orig, y_orig, theta_orig, kappa_orig, ...
                               k_start, v_hold, vd_nom, T_hold)
% buildHoldAndRampReference  — Case B recovery profile:
%   1) hold current speed v_hold for T_hold seconds worth of distance
%   2) linear ramp from v_hold back up to vd_nom
%   3) cruise at vd_nom for the rest of the original path
%   4) ramp down to 0 at the very end (same as original goal profile)

    r_min  = 0.9;  vd_max = 0.3;  wd_max = vd_max / r_min;

    S_rem       = S_orig(k_start:end) - S_orig(k_start);
    x_ref       = x_orig(k_start:end);
    y_ref       = y_orig(k_start:end);
    theta_ref_uw= theta_orig(k_start:end);
    kappa_ref   = kappa_orig(k_start:end);
    P           = P_orig(k_start:end, :);
    S_path      = S_rem;

    % Distance boundaries for hold and ramp-up
    v_hold     = max(v_hold, 0.0);           % safety clamp
    s_hold_end = max(0.02, v_hold * T_hold); % distance covered at v_hold during T_hold s

    % Ramp-up distance: use same acceleration as original start ramp
    totalRemaining = S_rem(end);
    a_ramp_up  = vd_nom^2 / (2 * max(min(0.4, totalRemaining*0.35), 0.05));
    delta_v    = max(0, vd_nom - v_hold);
    s_ramp_end = s_hold_end + delta_v^2 / (2 * max(a_ramp_up, 0.01));

    % End-of-path ramp-down (same logic as main planner)
    s_ramp_down = min(0.4, totalRemaining * 0.2);
    a_ramp_down = vd_nom^2 / (2 * max(s_ramp_down, 0.01));

    vd_path = zeros(size(S_rem));
    for k = 1:numel(S_rem)
        s     = S_rem(k);
        s_rem = totalRemaining - s;

        % Phase 1 — hold
        if s <= s_hold_end
            vd_path(k) = v_hold;

        % Phase 2 — ramp up
        elseif s <= s_ramp_end
            s_into_ramp = s - s_hold_end;
            vd_path(k)  = sqrt(max(v_hold^2 + 2*a_ramp_up*s_into_ramp, v_hold^2));

        % Phase 3 — cruise
        else
            vd_path(k) = vd_nom;
        end

        % Phase 4 — ramp down at end (overrides above if close to goal)
        if s_rem <= s_ramp_down
            v_end = sqrt(2 * a_ramp_down * max(s_rem, 0));
            vd_path(k) = min(vd_path(k), v_end);
        end
    end
    vd_path(end) = 0;

    vd_path = min(vd_path, vd_max);
    wd_path = vd_path .* kappa_ref;
    wd_path = smoothdata(wd_path, 'gaussian', 31);
    wd_path = max(min(wd_path, wd_max), -wd_max);
end

% -------------------------------------------------------------------------
function [S_path, x_ref, y_ref, theta_ref_uw, kappa_ref, vd_path, wd_path, P] = ...
    extractRemainingPath(S_orig, x_orig, y_orig, theta_orig, kappa_orig, ...
                         vd_orig, wd_orig, P_orig, k_start)
% extractRemainingPath  — slice the original reference from k_start onward.
    S_path      = S_orig(k_start:end) - S_orig(k_start);
    x_ref       = x_orig(k_start:end);
    y_ref       = y_orig(k_start:end);
    theta_ref_uw= theta_orig(k_start:end);
    kappa_ref   = kappa_orig(k_start:end);
    vd_path     = vd_orig(k_start:end);
    wd_path     = wd_orig(k_start:end);
    P           = P_orig(k_start:end, :);
end

% -------------------------------------------------------------------------
%  Replan helpers
% -------------------------------------------------------------------------
function [poses, S_path, x_ref, y_ref, theta_ref_unwrapped, ...
          kappa_ref, vd_path, wd_path, P] = ...
          replanCustomKinematic(map, startPose, goalPose, vd_nom, v_start)
% replanCustomKinematic
%   v_start is the robot speed at the switching moment.
%   If the robot is still moving, the new reference starts near v_start
%   and blends smoothly to vd_nom. This avoids an unrealistic forced stop.

    if nargin < 5
        v_start = 0.0;
    end

    fprintf('Running Custom Kinematic replanner...\n');
    kinematics.min_turn      = 0.9;
    kinematics.step_distance = 0.4;
    max_steer = kinematics.step_distance / kinematics.min_turn;
    kinematics.steer_angles  = [-max_steer, 0.0, max_steer];
    kinematics.prim_costs    = [1.05, 1.0, 1.05];

    [raw_path, path_len] = PlanKinematicPath_World(map, startPose, goalPose, kinematics);
    if path_len < 2
        error('Replanner failed to find a new path after dynamic obstacle.');
    end

    poses = SmoothPath_World(raw_path, 0.2, 0.2, 0.0001, map);
    [S_path, x_ref, y_ref, theta_ref_unwrapped, ...
     kappa_ref, vd_path, wd_path, P] = ...
     buildSpatialReferenceFromPoses(poses, vd_nom, v_start);
end

function [S_path, x_ref, y_ref, theta_ref_unwrapped, ...
          kappa_ref, vd_path, wd_path, P] = ...
          buildSpatialReferenceFromPoses(poses, vd_nom, v_start)
% buildSpatialReferenceFromPoses
%   Builds a spatial reference after replanning.
%   The speed profile starts from v_start and blends smoothly to vd_nom.
%   This is used for moving replans so the robot does not need to stop first.

    if nargin < 3
        v_start = 0.0;
    end

    ds_spatial = 0.02;
    dx = diff(poses(:,1)); dy = diff(poses(:,2));
    stepDistances = sqrt(dx.^2 + dy.^2);
    S_raw = [0; cumsum(stepDistances)];
    [S_raw, uniqueIdx] = unique(S_raw, 'stable');
    poses = poses(uniqueIdx, :);

    if numel(S_raw) < 2
        error('Not enough unique path points after replanning.');
    end

    S_path = (0 : ds_spatial : S_raw(end))';
    if S_path(end) < S_raw(end)
        S_path = [S_path; S_raw(end)];
    end

    x_ref = spline(S_raw, poses(:,1), S_path);
    y_ref = spline(S_raw, poses(:,2), S_path);
    dx_ref = gradient(x_ref, ds_spatial);
    dy_ref = gradient(y_ref, ds_spatial);
    theta_ref            = atan2(dy_ref, dx_ref);
    theta_ref_unwrapped  = unwrap(theta_ref);
    P = [x_ref y_ref];

    kappa_ref = gradient(theta_ref_unwrapped, ds_spatial);
    kappa_ref = smoothdata(kappa_ref, 'gaussian', 51);

    dkappa_max = 0.05;
    for k = 2:numel(kappa_ref)
        dkappa       = kappa_ref(k) - kappa_ref(k-1);
        kappa_ref(k) = kappa_ref(k-1) + sign(dkappa)*min(abs(dkappa), dkappa_max);
    end

    r_min     = 0.9;
    vd_max    = 0.3;
    wd_max    = vd_max / r_min;
    kappa_max = 1 / r_min;
    kappa_ref = max(min(kappa_ref, kappa_max), -kappa_max);

    totalPathLength = S_path(end);
    s_blend     = min(0.8, max(0.25, totalPathLength * 0.25));
    s_ramp_down = min(0.4, totalPathLength * 0.35);
    a_ramp_down = vd_nom^2 / (2 * max(s_ramp_down, 0.01));

    % If starting from rest, give a tiny kickstart like the original profile.
    v_start = max(0.0, min(v_start, vd_max));
    if v_start < 1e-4
        v_start_cmd = 0.02;
    else
        v_start_cmd = v_start;
    end

    vd_path = zeros(size(S_path));
    for k = 1:numel(S_path)
        s     = S_path(k);
        s_rem = totalPathLength - s;

        % Smoothly blend from current robot speed to nominal path speed.
        if s <= s_blend
            lambda = s / max(s_blend, 1e-6);
            smooth_lambda = lambda^2 * (3 - 2*lambda);  % smoothstep
            vd_path(k) = (1 - smooth_lambda)*v_start_cmd + smooth_lambda*vd_nom;
        else
            vd_path(k) = vd_nom;
        end

        % Always slow down near the final goal.
        if s_rem <= s_ramp_down
            v_end = sqrt(2 * a_ramp_down * max(s_rem, 0));
            vd_path(k) = min(vd_path(k), v_end);
        end
    end

    vd_path(end) = 0;
    vd_path = min(vd_path, vd_max);

    wd_path = vd_path .* kappa_ref;
    wd_path = smoothdata(wd_path, 'gaussian', 31);
    wd_path = max(min(wd_path, wd_max), -wd_max);
end