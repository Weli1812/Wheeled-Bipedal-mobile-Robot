clear; clc; close all;

%% Model name
mdl = 'Pure_Pursuit';

%% Tunable parameters
L = 0.2;              % Distance between wheels [m]
vRef = 0.15;          % Lower speed gives easier tracking [m/s]
lookahead = 0.3;     % Smaller lookahead = tighter corner tracking [m]
maxW = 3.0;           % Maximum angular velocity [rad/s]
goalTol = 0.20;       % Goal tolerance for plotting/reporting [m]

%% Create map with obstacles
map = binaryOccupancyMap(10, 10, 5);

setOccupancy(map, [3 3; 3 4; 3 5; 4 5; 5 5], 1);
setOccupancy(map, [7 2; 7 3; 7 4; 7 5], 1);
setOccupancy(map, [5 7; 6 7; 7 7], 1);

inflate(map, 0.2);

%% Start and goal
start = [1 1];
goal  = [8 8];

%% Plan path
rng(10, 'twister');           % Fixed seed, so the path does not change every run

prm = mobileRobotPRM(map, 300);
prm.ConnectionDistance = 2.5;

path = findpath(prm, start, goal);

if isempty(path)
    prm.NumNodes = 700;
    prm.ConnectionDistance = 3.0;
    path = findpath(prm, start, goal);
end

if isempty(path)
    error('No path found. Increase NumNodes or ConnectionDistance.');
end

%% Make the path smoother for Pure Pursuit
P = densifyPath(path, 0.05);

% Initial heading must match the first path segment
% This is very important. Starting theta = 0 causes the robot to turn badly.
theta0 = atan2(P(2,2) - P(1,2), P(2,1) - P(1,1));

%% Send variables to Simulink base workspace
assignin('base', 'L', L);
assignin('base', 'P', P);
assignin('base', 'start', start);
assignin('base', 'theta0', theta0);

%% Load and configure Simulink model
load_system(mdl);

% This removes the STM32H7xx warning when you only want normal simulation.
% It is not needed for real STM32 deployment.
try
    set_param(mdl, 'HardwareBoard', 'None');
catch
    % Some MATLAB versions may not allow changing this parameter from script.
    % If so, do it manually from Model Settings > Hardware Implementation.
end

try
    set_param(mdl, 'SimulationMode', 'normal');
catch
end

% Avoid hardware/code-generation simulation mode inside the Pure Pursuit block.
try
    set_param([mdl '/Pure Pursuit'], 'SimulateUsing', 'Interpreted execution');
catch
end

% Pure Pursuit settings
set_param([mdl '/Pure Pursuit'], ...
    'DesiredLinearVelocity', num2str(vRef), ...
    'LookaheadDistance', num2str(lookahead), ...
    'MaxAngularVelocity', num2str(maxW));

% Initial condition of x and y
set_param([mdl '/Integrator'],  'InitialCondition', 'start(1)');
set_param([mdl '/Integrator2'], 'InitialCondition', 'start(2)');

% IMPORTANT FIX:
% Your model has three theta integrators: Integrator1, Integrator3, Integrator4.
% If only Integrator4 is changed, Pure Pursuit and robot motion use different headings.
% This creates the large wrong path / huge final error.
thetaIntegratorBlocks = {'Integrator1', 'Integrator3', 'Integrator4'};
for i = 1:numel(thetaIntegratorBlocks)
    set_param([mdl '/' thetaIntegratorBlocks{i}], 'InitialCondition', 'theta0');
end

% Simulation time: enough to reach goal, but not too long.
pathLength = sum(hypot(diff(P(:,1)), diff(P(:,2))));
stopTime = max(30, 1.35 * pathLength / vRef);
set_param(mdl, 'StopTime', num2str(stopTime));

%% Run simulation AFTER the path and initial conditions are set
out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

%% Read robot outputs
x_out     = getSimOut(out, 'x_out');
y_out     = getSimOut(out, 'y_out');
theta_out = getSimOut(out, 'theta_out');
tout      = getSimOut(out, 'tout');

x_out = x_out(:);
y_out = y_out(:);
theta_out = theta_out(:);
tout = tout(:);

numSamples = min([numel(x_out), numel(y_out), numel(theta_out), numel(tout)]);

if numSamples == 0
    error('Simulation output is empty. Check the To Workspace blocks.');
end

x_out = x_out(1:numSamples);
y_out = y_out(1:numSamples);
theta_out = theta_out(1:numSamples);
tout = tout(1:numSamples);

%% Check goal reach and avoid misleading final error after overshoot
distToGoal = hypot(x_out - goal(1), y_out - goal(2));
[minErr, idxMin] = min(distToGoal);
idxReach = find(distToGoal <= goalTol, 1, 'first');

if ~isempty(idxReach)
    idxEnd = idxReach;
    fprintf('Goal reached at t = %.2f s, error = %.3f m\n', tout(idxReach), distToGoal(idxReach));
else
    idxEnd = numSamples;
    fprintf('Goal was not reached within %.2f s. Closest error = %.3f m at t = %.2f s\n', ...
        tout(end), minErr, tout(idxMin));
    fprintf('Final position error at simulation end = %.3f m\n', distToGoal(end));
end

%% Plot map and planned path
fig = figure('Name', 'Robot Path Animation');
ax = axes(fig);
show(map, 'Parent', ax);
hold(ax, 'on');
title(ax, 'Robot Path Animation');

plot(ax, start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(ax, goal(1),  goal(2),  'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot(ax, P(:,1), P(:,2), 'b--', 'LineWidth', 2);

%% Create animated robot objects
hRobot = plot(ax, x_out(1), y_out(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hTrail = animatedline(ax, 'Color', 'r', 'LineWidth', 2);
addpoints(hTrail, x_out(1), y_out(1));

arrowLength = 0.3;
hArrow = quiver(ax, x_out(1), y_out(1), ...
    arrowLength*cos(theta_out(1)), arrowLength*sin(theta_out(1)), ...
    0, 'k', 'LineWidth', 2);

legend(ax, 'Start', 'Goal', 'Planned Path', 'Robot', 'Robot Trail', 'Heading', ...
    'Location', 'northeast');
grid(ax, 'on');
axis(ax, 'equal');
xlim(ax, [-1 10]);
ylim(ax, [0 10]);

%% Animation loop - finish in exactly about 10 seconds

animationDuration = 10;     % seconds
maxRenderFrames = 400;      % number of animation frames

idxAnim = unique(round(linspace(1, idxEnd, maxRenderFrames)));
N = numel(idxAnim);

tic;

for i = 1:N
    k = idxAnim(i);

    if ~isgraphics(fig) || ~isgraphics(ax)
        break;
    end

    % Update robot position
    hRobot.XData = x_out(k);
    hRobot.YData = y_out(k);

    % Update trail
    addpoints(hTrail, x_out(k), y_out(k));

    % Update heading arrow
    hArrow.XData = x_out(k);
    hArrow.YData = y_out(k);
    hArrow.UData = arrowLength*cos(theta_out(k));
    hArrow.VData = arrowLength*sin(theta_out(k));

    drawnow;

    % Force animation to match 10 seconds
    targetTime = (i-1) * animationDuration / (N-1);
    elapsedTime = toc;

    pause(max(0, targetTime - elapsedTime));
end

%% Local helper functions
function P_dense = densifyPath(P_raw, ds)
    P_raw = double(P_raw);

    d = [0; cumsum(hypot(diff(P_raw(:,1)), diff(P_raw(:,2))))];
    [d, idx] = unique(d, 'stable');
    P_raw = P_raw(idx, :);

    if d(end) <= 0
        error('Path length is zero.');
    end

    s = (0:ds:d(end))';
    if s(end) < d(end)
        s = [s; d(end)];
    end

    P_dense = [interp1(d, P_raw(:,1), s, 'linear'), ...
               interp1(d, P_raw(:,2), s, 'linear')];
end

function data = getSimOut(out, name)
    if strcmp(name, 'tout')
        data = out.tout;
        return;
    end

    try
        data = out.(name);
    catch
        data = evalin('base', name);
    end

    if isa(data, 'timeseries')
        data = data.Data;
    elseif isa(data, 'Simulink.SimulationData.Signal')
        vals = data.Values;
        if isa(vals, 'timeseries')
            data = vals.Data;
        else
            data = vals;
        end
    end
end
