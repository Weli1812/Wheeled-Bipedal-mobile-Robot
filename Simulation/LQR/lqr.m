% Robot parameters
L = 0.2;              % distance between wheels [m]
Ts_ref = 0.2;         % reference update time [s]

% Create occupancy map
map = binaryOccupancyMap(10,10,5);

% Obstacles
setOccupancy(map,[3 3;3 4;3 5;4 5;5 5],1);
setOccupancy(map,[7 2;7 3;7 4;7 5],1);
setOccupancy(map,[5 7;6 7;7 7],1);

inflate(map,0.2);

% Start and goal
start = [1 1];
goal  = [8 8];

% Path planner
prm = mobileRobotPRM(map,100);
prm.ConnectionDistance = 2;

path = findpath(prm,start,goal);

if isempty(path)
    error('No path found');
end

% Use the planned path as reference path
P = path;

% Compute heading along the path
dx = diff(P(:,1));
dy = diff(P(:,2));
theta_ref = atan2(dy,dx);
theta_ref = [theta_ref; theta_ref(end)];

% Nominal reference velocities
vd_ref = 0.3 * ones(size(P,1),1);
wd_ref = zeros(size(P,1),1);

% Initial robot heading from first segment
dx0 = P(2,1) - P(1,1);
dy0 = P(2,2) - P(1,2);
theta0 = atan2(dy0,dx0);

% Plot map and path
figure;
show(map);
hold on;
plot(start(1),start(2),'go','MarkerSize',10,'LineWidth',2);
plot(goal(1),goal(2),'rx','MarkerSize',10,'LineWidth',2);
plot(P(:,1),P(:,2),'b','LineWidth',2);
title('Planned Path');
axis equal;
grid on;