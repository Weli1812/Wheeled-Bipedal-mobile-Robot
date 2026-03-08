L = 0.2; %% Distance between wheels

%% Create a simple map with obstacles
map = binaryOccupancyMap(10,10,5);
setOccupancy(map,[3 3;3 4;3 5;4 5;5 5],1);
setOccupancy(map,[7 2;7 3;7 4;7 5],1);
setOccupancy(map,[5 7;6 7;7 7],1);
inflate(map,0.2);

figure
show(map)
hold on
title('Robot Map')

%% Define start and goal
start = [1 1];
start = [start(1)  start(2)];
goal  = [8 8];

%% Use a path planner
prm = mobileRobotPRM(map,100);
prm.ConnectionDistance = 2;
path = findpath(prm,start,goal);

if isempty(path)
    error('No path found');
end

%% Visualize the planned path
plot(start(1),start(2),'go','MarkerSize',10)
plot(goal(1),goal(2),'rx','MarkerSize',10)
plot(path(:,1),path(:,2),'b','LineWidth',2)

%% Identify path
P = path;
