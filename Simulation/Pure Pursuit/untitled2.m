L = 0.2;   % Distance between wheels


%% Create a simple map with obstacles
map = binaryOccupancyMap(10,10,5);

setOccupancy(map,[3 3; 3 4; 3 5; 4 5; 5 5],1);
setOccupancy(map,[7 2; 7 3; 7 4; 7 5],1);
setOccupancy(map,[5 7; 6 7; 7 7],1);

inflate(map,0.2);

%% Define start and goal
start = [1 1];
goal  = [8 8];

%% Path planner
prm = mobileRobotPRM(map,100);
prm.ConnectionDistance = 2;

path = findpath(prm,start,goal);

if isempty(path)
    error('No path found');
end

%% If you want to store the planned path in P
P = path;

%% Plot map and planned path
fig = figure;
ax = axes(fig);
show(map, 'Parent', ax)
hold(ax, 'on')
title('Robot Path Animation')

plot(ax, start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2)
plot(ax, goal(1),  goal(2),  'rx', 'MarkerSize', 10, 'LineWidth', 2)
plot(ax, path(:,1), path(:,2), 'b--', 'LineWidth', 2)

%% Get robot outputs from Simulink workspace object
x_out     = out.x_out;
y_out     = out.y_out;
theta_out = out.theta_out;
tout      = out.tout;

if isa(x_out, 'timeseries')
    x_out = x_out.Data;
end

if isa(y_out, 'timeseries')
    y_out = y_out.Data;
end

if isa(theta_out, 'timeseries')
    theta_out = theta_out.Data;
end

if isa(tout, 'timeseries')
    tout = tout.Data;
end

x_out = x_out(:);
y_out = y_out(:);
theta_out = theta_out(:);
tout = tout(:);

numSamples = min([numel(x_out), numel(y_out), numel(theta_out)]);

if numSamples == 0
    error('Simulation output is empty');
end

x_out = x_out(1:numSamples);
y_out = y_out(1:numSamples);
theta_out = theta_out(1:numSamples);

% Render fewer frames for faster visualization on dense trajectories.
maxRenderFrames = 500;
frameStep = max(1, ceil(numSamples / maxRenderFrames));


%% Create animated robot objects
hRobot = plot(ax, x_out(1), y_out(1), 'ro', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'r');

hTrail = animatedline(ax, 'Color', 'r', 'LineWidth', 2);
addpoints(hTrail, x_out(1), y_out(1));

% Heading arrow
arrowLength = 0.3;
hArrow = quiver(ax, ...
    x_out(1), y_out(1), ...
    arrowLength*cos(theta_out(1)), ...
    arrowLength*sin(theta_out(1)), ...
    0, 'k', 'LineWidth', 2);

legend('Start','Goal','Planned Path','Robot','Robot Trail','Heading')
grid on
axis equal

%% Animation loop
for k = 1:frameStep:numSamples
    if ~isgraphics(fig) || ~isgraphics(ax)
        break;
    end

    if ~isgraphics(hRobot)
        hRobot = plot(ax, x_out(k), y_out(k), 'ro', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'r');
    else
        hRobot.XData = x_out(k);
        hRobot.YData = y_out(k);
    end

    if ~isgraphics(hTrail)
        hTrail = animatedline(ax, 'Color', 'r', 'LineWidth', 2);
        addpoints(hTrail, x_out(1:k), y_out(1:k));
    else
        addpoints(hTrail, x_out(k), y_out(k));
    end

    if ~isgraphics(hArrow)
        hArrow = quiver(ax, ...
            x_out(k), y_out(k), ...
            arrowLength*cos(theta_out(k)), ...
            arrowLength*sin(theta_out(k)), ...
            0, 'k', 'LineWidth', 2);
    else
        hArrow.XData = x_out(k);
        hArrow.YData = y_out(k);
        hArrow.UData = arrowLength*cos(theta_out(k));
        hArrow.VData = arrowLength*sin(theta_out(k));
    end

    drawnow limitrate nocallbacks
end