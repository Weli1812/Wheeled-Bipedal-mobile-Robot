clear; clc; close all;

%% =========================
% 0) Choose planner
% ==========================
% Options:
%   "HYBRID_ASTAR"
%   "RRTSTAR"
plannerType = "HYBRID_ASTAR";

% =========================
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
% 
% % Obstacle 5: right-lower block
% [X5, Y5] = meshgrid(7.0:0.1:8.8, 1.2:0.1:3.2);
% setOccupancy(map, [X5(:) Y5(:)], 1);
% 
% % Obstacle 6: right-upper block
% [X6, Y6] = meshgrid(7.2:0.1:8.9, 6.2:0.1:8.8);
% setOccupancy(map, [X6(:) Y6(:)], 1);
% 
% % Obstacle 7: narrow corridor maker
% [X7, Y7] = meshgrid(3.0:0.1:3.8, 3.8:0.1:5.8);
% setOccupancy(map, [X7(:) Y7(:)], 1);

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
        planningTime1 = toc;

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
        planningTime2 = toc;
        if isempty(refPath) || isempty(refPath.States)
            error('Hybrid A* failed on replan.');
        end
        poses = refPath.States;

        % -------- Resample path at uniform spatial step ds --------
        ds_spatial = 0.02;   % [m] — spatial resolution, NOT time-linked

        dx = diff(poses(:,1));
        dy = diff(poses(:,2));
        stepDistances = sqrt(dx.^2 + dy.^2);
        S_raw = [0; cumsum(stepDistances)];

        [S_raw, uniqueIdx] = unique(S_raw, 'stable');
        poses = poses(uniqueIdx, :);

        if numel(S_raw) < 2
            error('Not enough unique path points after preprocessing.');
        end

        % Build uniform arc-length grid purely in spatial domain
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

        disp('Running RRT* (first pass - dummy thetas)...');
        rng('default');
        tic;
        [pthObj, solnInfo] = plan(planner, startPose, goalPose);
        planningTime1 = toc;

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

            anchorPoints = [[startPose(1); startPose(2)], ...
                            startApp, ...
                            middlePoints, ...
                            goalApp, ...
                            [goalPose(1); goalPose(2)]];

            tSamplesRaw = linspace(0, 1, 3000);
            [qRaw, ~] = bsplinepolytraj(anchorPoints, [0 1], tSamplesRaw);

            x_base = qRaw(1,:)';
            y_base = qRaw(2,:)';
        end

        % -------- Resample at uniform spatial step ds --------
        ds_spatial = 0.02;   % [m] — same spatial resolution as Hybrid A*

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

        % Blend final heading toward goal orientation
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
        error('Unknown plannerType. Use "HYBRID_ASTAR" or "RRTSTAR".');
end

fprintf('Selected planner: %s\n', plannerType);
fprintf('Planning time (pass 1): %.4f s\n', planningTime1);
fprintf('Planning time (pass 2 / replan): %.4f s\n', planningTime2);

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
% 6) Build SPATIAL reference table (arc-length parameterised)
%    Everything is a function of arc-length s, NOT time.
% ==========================

% --- Curvature kappa(s) = dtheta/ds ---
%     Computed spatially from the geometry, no velocity involved.
kappa_ref = gradient(theta_ref_unwrapped, ds_spatial);   % [rad/m]

% Much wider smoothing window
kappa_ref = smoothdata(kappa_ref, 'gaussian', 51);  

% Rate-limit kappa between consecutive stations to eliminate spikes
dkappa_max = 0.05;   % max curvature can change between two path points.
for k = 2:numel(kappa_ref)
    dkappa = kappa_ref(k) - kappa_ref(k-1);
    kappa_ref(k) = kappa_ref(k-1) + sign(dkappa)*min(abs(dkappa), dkappa_max); %% This prevents sudden curvature jumps.
end

% --- Enforce kinematic curvature limit  kappa_max = 1/r_min ---
r_min     = 0.9;                 % [m]  minimum turning radius
kappa_max = 1 / r_min;          % [1/m]
kappa_ref = max(min(kappa_ref, kappa_max), -kappa_max);

% --- Desired speed profile along arc-length (Square Root Ramp) ---
% Constant acceleration a = vd_nom^2 / (2 * s_ramp) keeps the ramp
% kinematically consistent: v(s) = sqrt(2*a*s) for the ramp-up,
% v(s) = sqrt(2*a*(totalPathLength - s)) for the ramp-down.
% Both ramps start and end exactly at 0 — no floor applied near the goal
% so the robot comes to a smooth, genuine stop without a torque spike.
totalPathLength = S_path(end);

% Ramp distance: use 0.8 m (4 s at 0.2 m/s average) for a gentle start/stop.
% If the path is very short, shrink the ramps so they don't overlap.
s_ramp_up   = min(0.4, totalPathLength * 0.35);
s_ramp_down = min(0.4, totalPathLength * 0.35);

% Acceleration magnitude derived from ramp distance and cruise speed
a_ramp = vd_nom^2 / (2 * s_ramp_up);

% Minimum speed applied ONLY during ramp-up so the robot breaks from rest.
% NOT applied during ramp-down — that region must reach true zero for a clean stop.
v_kickstart = 0.02;   % [m/s]  2 cm/s

vd_path = zeros(size(S_path));
for k = 1:numel(S_path)
    s     = S_path(k);
    s_rem = totalPathLength - s;
    if s <= s_ramp_up %%If robot is in the acceleration zone.
        vd_path(k) = max(sqrt(2 * a_ramp * s), v_kickstart); %%Desired speed follows square-root ramp, But it will not go below v_kickstart.

    elseif s_rem <= s_ramp_down
        % Ramp-down: sqrt profile reaching true zero — NO floor here
        vd_path(k) = sqrt(2 * a_ramp * max(s_rem, 0));
    else
        % Cruise at nominal speed
        vd_path(k) = vd_nom;
    end
end

% Force exact zero at the final sample only (eliminates sqrt floating-point residual)
vd_path(end) = 0;

% --- Desired angular speed from geometry: omega(s) = v(s) * kappa(s) ---
%     This is a PURE geometric relation — no time involved.
wd_path = vd_path .* kappa_ref;
wd_path = smoothdata(wd_path, 'gaussian', 31);   % smooth wd too

% --- Hard limits ---
vd_max  = 0.3;             % [m/s]
wd_max  = vd_max / r_min;  % [rad/s] ≈ 0.333

vd_path = min(vd_path, vd_max);
wd_path = max(min(wd_path, wd_max), -wd_max);  % [rad/s] as function of s

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
% 10) Precompute LQR gain once
% ==========================
K_lqr = compute_lqr_gain(h_ref);
fprintf('LQR gain precomputed for h = %.3f m\n', h_ref);

%% =========================a
% 11) Initial conditions for plant state
% State order: [phi; s; theta; phi_dot; v; omega; xpos; ypos]
% ==========================
phi0     = deg2rad(0);    % start perfectly balanced — avoids torque spike at t=0
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
%     Estimated from path length and nominal speed; independent of
%     the reference generator which is purely spatial.
% ==========================
   T_est = totalPathLength / vd_nom;   % rough travel time estimate [s]
   T_end = T_est * 1.5 + 5.0;   % generous margin

%% =========================
% 13) Prepare lookup tables for Simulink
%
%  The Simulink model must look up references by arc-length s, NOT by time.
%  Pass S_path as the breakpoint vector and all reference quantities
%  as the corresponding table data.
%  In Simulink use a 1-D Lookup Table block:
%      Breakpoints = S_path
%      Table data  = x_ref / y_ref / theta_ref_unwrapped / kappa_ref /
%                    vd_path / wd_path
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

% Verify all reference tables are consistently sized
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
% 14) Run Simulink
% ==========================
out = sim('OneCombinedLQR', 'StopTime', num2str(T_end));

%% =========================
% 15) Read outputs
% ==========================
sim_t  = out.tout(:);
x      = out.x_out(:);
y      = out.y_out(:);
theta  = out.theta_out(:);
phi    = out.phi_out(:);
v      = out.v_out(:);
omega  = out.omega_out(:);
vd_cmd = out.vd_cmd_out(:);
wd_cmd = out.wd_cmd_out(:);

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
grid on; ylabel('\phi (rad)');
title(['Tilt angle \phi - ']);

subplot(3,1,2);
plot(sim_t, v(1:length(sim_t)), 'LineWidth', 1.5); hold on;
plot(sim_t, vd_cmd(1:length(sim_t)), 'r--', 'LineWidth', 1);
grid on; ylabel('v (m/s)');
legend('Actual','Desired'); title('Forward velocity v');

subplot(3,1,3);
plot(sim_t, omega(1:length(sim_t)), 'LineWidth', 1.5); hold on;
plot(sim_t, wd_cmd(1:length(sim_t)), 'r--', 'LineWidth', 1);
grid on; ylabel('\omega (rad/s)');
legend('Actual','Desired'); title('Angular velocity \omega');
xlabel('Time (s)');

% --- Spatial reference diagnostics ---
figure;
subplot(2,1,1);
plot(S_path, vd_path, 'b', 'LineWidth', 1.5);
grid on; xlabel('Arc-length s (m)'); ylabel('v_d (m/s)');
title('Desired speed profile — indexed by arc-length');

subplot(2,1,2);
plot(S_path, kappa_ref, 'r', 'LineWidth', 1.5);
grid on; xlabel('Arc-length s (m)'); ylabel('\kappa (rad/m)');
title('Path curvature — indexed by arc-length');


%% =========================
% Animated Figure 3: States & Physical Tilt (Rolling)
% ==========================
figure('Name', 'Animated States and Tilt', 'Position', [100, 100, 900, 800]);

% --- Calculate Distance Traveled for the 1D Animation ---
s_act = cumtrapz(sim_t, v); % Integrate velocity to get 1D position

% --- Subplot 1: Physical Robot Tilt Animation ---
ax1 = subplot(4,1,1);
hold on; grid on; axis equal;
ylim([-0.1 0.6]);
title('Physical Robot Tilt (\phi) Animation');

% Draw Ground and Wheel (Initialize them at 0)
ground_line = plot([-0.5 0.5], [0 0], 'k', 'LineWidth', 2); % Ground
wheel_dot = plot(0, 0, 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % Wheel

% Initialize the Robot Body Line
body_len = 0.4; % Length of the stick figure body
body_line = plot([0 0], [0 body_len], 'b-', 'LineWidth', 5);

% --- Subplot 2: Phi Time Series ---
subplot(4,1,2); hold on; grid on;
xlim([0 T_end]); 
ylim([min(phi)-0.02, max(phi)+0.02]);
ylabel('\phi (rad)');
title('Tilt angle \phi');
phi_line = animatedline('Color', '#0072BD', 'LineWidth', 1.5);

% --- Subplot 3: Forward Velocity Time Series ---
subplot(4,1,3); hold on; grid on;
xlim([0 T_end]); 
ylim([min(min(v), min(vd_cmd))-0.05, max(max(v), max(vd_cmd))+0.05]);
ylabel('v (m/s)');
title('Forward velocity v');
v_line = animatedline('Color', '#0072BD', 'LineWidth', 1.5);
vd_line = animatedline('Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);
legend('Actual', 'Desired', 'Location', 'best');

% --- Subplot 4: Angular Velocity Time Series ---
subplot(4,1,4); hold on; grid on;
xlim([0 T_end]); 
ylim([min(min(omega), min(wd_cmd))-0.05, max(max(omega), max(wd_cmd))+0.05]);
ylabel('\omega (rad/s)'); xlabel('Time (s)');
title('Angular velocity \omega');
w_line = animatedline('Color', '#0072BD', 'LineWidth', 1.5);
wd_line = animatedline('Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);

% --- Animation Loop ---
num_points = length(sim_t);
skip_factor = max(1, floor(num_points / 500)); 

for k = 1:skip_factor:num_points
    
    current_phi = phi(k);
    current_s = s_act(k); % Get current horizontal position
    
    % 1. Update the Wheel and Ground Position
    set(wheel_dot, 'XData', current_s, 'YData', 0);
    set(ground_line, 'XData', [current_s - 0.5, current_s + 0.5]); % Move ground with robot
    
    % 2. Update the Stick Figure's Body
    x_body = current_s + body_len * sin(current_phi);
    y_body = body_len * cos(current_phi);
    set(body_line, 'XData', [current_s, x_body], 'YData', [0, y_body]);
    
    % 3. Track the Camera (Pan the axes to follow the robot)
    xlim(ax1, [current_s - 0.5, current_s + 0.5]);
    
    % 4. Add points to the rolling graphs
    addpoints(phi_line, sim_t(k), current_phi);
    addpoints(v_line, sim_t(k), v(k));
    addpoints(vd_line, sim_t(k), vd_cmd(k));
    addpoints(w_line, sim_t(k), omega(k));
    addpoints(wd_line, sim_t(k), wd_cmd(k));
    
    % Force MATLAB to render the frame
    drawnow;
end

% Ensure the very last point is plotted
addpoints(phi_line, sim_t(end), phi(end));
addpoints(v_line, sim_t(end), v(end));
addpoints(vd_line, sim_t(end), vd_cmd(end));
addpoints(w_line, sim_t(end), omega(end));
addpoints(wd_line, sim_t(end), wd_cmd(end));

% Final frame cleanup
final_s = s_act(end);
set(wheel_dot, 'XData', final_s, 'YData', 0);
set(body_line, 'XData', [final_s, final_s + body_len*sin(phi(end))], 'YData', [0, body_len*cos(phi(end))]);
xlim(ax1, [final_s - 0.5, final_s + 0.5]);
drawnow;

%% =========================
% Local function: precompute LQR gain
% ==========================
function K = compute_lqr_gain(h)
    Mw = 0.150;
    Mb = 11.3;
    r  = 0.060;
    d  = 0.450;
    Iw = 0.0002757224;
    g  = 9.81;

    Lc = -0.4748*h^2 + 1.2434*h - 0.0286;
    Iy =  1.4948*h^2 + 0.5774*h - 0.0053;
    Iz = -0.5689*h^2 + 0.1583*h + 0.1236;

    M11   = Mb*Lc^2 + Iy;
    M12   = Mb*Lc;
    M22   = Mb + 2*Mw + 2*Iw/r^2;
    M33   = (d^2/2)*Mw + (d^2/(2*r^2))*Iw + Iz;
    Delta = M11*M22 - M12^2;

    A = [0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1;
         (M22*Mb*g*Lc)/Delta   0 0 0 0 0;
        -(M12*Mb*g*Lc)/Delta   0 0 0 0 0;
         0 0 0 0 0 0];

    B = [0 0;
         0 0;
         0 0;
        -M12/(r*Delta)  -M12/(r*Delta);
         M11/(r*Delta)   M11/(r*Delta);
         d/(2*r*M33)    -d/(2*r*M33)];

    Q = diag([200, 5, 40, 20, 5, 10]);
    R = diag([2.0 2.0]); 

    K = lqr(A, B, Q, R);
end