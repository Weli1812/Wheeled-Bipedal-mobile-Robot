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
figure
show(map)
hold on
title('Robot Path Animation')

plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2)
plot(goal(1),  goal(2),  'rx', 'MarkerSize', 10, 'LineWidth', 2)
plot(path(:,1), path(:,2), 'b--', 'LineWidth', 2)

%% Get robot outputs from Simulink workspace object
x_out     = out.x_out;
y_out     = out.y_out;
theta_out = out.theta_out;
tout      = out.tout;


%% Create animated robot objects
hRobot = plot(x_out(1), y_out(1), 'ro', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'r');

hTrail = plot(x_out(1), y_out(1), 'r', 'LineWidth', 2);

% Heading arrow
arrowLength = 0.3;
hArrow = quiver( ...
    x_out(1), y_out(1), ...
    arrowLength*cos(theta_out(1)), ...
    arrowLength*sin(theta_out(1)), ...
    0, 'k', 'LineWidth', 2);

legend('Start','Goal','Planned Path','Robot','Robot Trail','Heading')
grid on
axis equal

%% Animation loop
for k = 1:length(x_out)
    % Update robot position
    set(hRobot, 'XData', x_out(k), 'YData', y_out(k));

    % Update robot trail
    set(hTrail, 'XData', x_out(1:k), 'YData', y_out(1:k));

    % Update heading arrow
    set(hArrow, ...
        'XData', x_out(k), ...
        'YData', y_out(k), ...
        'UData', arrowLength*cos(theta_out(k)), ...
        'VData', arrowLength*sin(theta_out(k)));

    drawnow
    pause(0.01)
end