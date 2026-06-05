% ==========================================================
% MATLAB VISUALIZER: BIPEDAL DIGITAL TWIN DASHBOARD
% ==========================================================
clear; clc; close all;

% 1. Load the Telemetry Log
try
    data = readmatrix('telemetry.csv');
catch
    error('Could not find telemetry.csv. Run your C code first!');
end

t_sim   = data(:, 1);
x       = data(:, 2);
y       = data(:, 3);
theta   = data(:, 4);
state   = data(:, 5); 
sensorR = data(:, 6);

% 2. Setup the Visualization Window
fig = figure('Name', 'Bipedal Path Planning Digital Twin', 'Position', [100, 100, 900, 700], 'Color', 'w');
ax = axes('Parent', fig, 'XLim', [0 50], 'YLim', [0 50]);
hold on; grid on; axis equal;

hTitle = title('System Initializing...', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Grid X (1 unit = 0.2m)'); ylabel('Grid Y (1 unit = 0.2m)');

% 3. Draw Static Obstacles and Inflations (Danger Zones)
obs_data = [
    10, 15, 2, 10;  
    40, 30, 5, 7;
    35, 10, 5, 5;
    15, 37, 5, 3;
    22, 22, 5, 5
];

% Draw Inflations
for i = 1:size(obs_data, 1)
    inf_x = obs_data(i, 1) - 3; inf_y = obs_data(i, 2) - 3;
    inf_w = obs_data(i, 3) + 6; inf_h = obs_data(i, 4) + 6;
    rectangle('Position', [inf_x, inf_y, inf_w, inf_h], 'FaceColor', [1 0.8 0.8], 'EdgeColor', 'none');
end

% Draw Solid Cores
for i = 1:size(obs_data, 1)
    rectangle('Position', obs_data(i, :), 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k', 'LineWidth', 1.5);
end

% =================================================================
% 4. DYNAMIC GOAL EXTRACTION & PATH DIFFERENTIATION
% =================================================================
goalX = 25; goalY = 10; % Fallback defaults

try
    initData = readmatrix('path_initial.csv');
    % Extract the true goal from the very last point of the initial path
    goalX = initData(end, 2);
    goalY = initData(end, 3);
    % Draw the initial plan as a faint Blue dashed line
    plot(initData(:,2), initData(:,3), 'Color', [0.4 0.6 1.0], 'LineWidth', 2, 'LineStyle', '--');
catch
end

try
    replanData = readmatrix('path_replan.csv');
    % Draw the replanned bypass as a faint Orange dashed line
    plot(replanData(:,2), replanData(:,3), 'Color', [1.0 0.6 0.2], 'LineWidth', 2, 'LineStyle', '--');
catch
end

% 5. Setup Live Animation Handles
hTrace = plot(NaN, NaN, 'k-', 'LineWidth', 3); 
hRobot = patch(NaN, NaN, 'b', 'EdgeColor', 'k', 'LineWidth', 1); 
hSensor = patch(NaN, NaN, [0.2 0.8 0.2], 'FaceAlpha', 0.4, 'EdgeColor', 'none'); 
hDynObs = rectangle('Position', [0 0 0 0], 'FaceColor', 'r', 'EdgeColor', 'k'); 

% Dynamically place the Goal Marker exactly where the C code wants it
hGoal = plot(goalX, goalY, 'g*', 'MarkerSize', 15, 'LineWidth', 2); 

robotBase = [-1, -1; 1.5, 0; -1, 1]; 
hasReplannedVisually = false; % Tracker for color changing

% 6. Start the Playback Loop
for i = 1:1:length(t_sim) 
    
    cx = x(i); cy = y(i); cth = theta(i); cState = state(i); r = sensorR(i);
    
    R_mat = [cos(cth), -sin(cth); sin(cth), cos(cth)];
    rotCorners = (R_mat * robotBase')';
    set(hRobot, 'XData', rotCorners(:,1) + cx, 'YData', rotCorners(:,2) + cy);
    
    % Check if the robot has entered the replanning state
    if cState == 2 
        hasReplannedVisually = true;
    end
    
    % Differentiate the live trace color! Black for initial, Purple for bypass.
    if hasReplannedVisually
        set(hTrace, 'XData', x(1:i), 'YData', y(1:i), 'Color', [0.6 0 0.8]);
    else
        set(hTrace, 'XData', x(1:i), 'YData', y(1:i), 'Color', 'k');
    end
    
    vis_r = min(r, 15.0); 
    coneAngles = linspace(-deg2rad(15), deg2rad(15), 10) + cth;
    coneX = [cx, cx + vis_r * cos(coneAngles), cx]; 
    coneY = [cy, cy + vis_r * sin(coneAngles), cy];
    set(hSensor, 'XData', coneX, 'YData', coneY);
    
    if cState == 0 % MOVING
        if hasReplannedVisually
            set(hTitle, 'String', sprintf('Time: %.2fs | State: BYPASSING OBSTACLE', t_sim(i)), 'Color', [0.6 0 0.8]);
        else
            set(hTitle, 'String', sprintf('Time: %.2fs | State: DRIVING INITIAL PATH', t_sim(i)), 'Color', 'k');
        end
        set(hSensor, 'FaceColor', [0.2 0.8 0.2]); 
        set(hDynObs, 'Position', [0 0 0 0]); 
        
    elseif cState == 1 % WAITING
        set(hTitle, 'String', sprintf('Time: %.2fs | State: BRAKING - OBSTACLE DETECTED!', t_sim(i)), 'Color', 'r');
        set(hSensor, 'FaceColor', 'r'); 
        
        ox = cx + r * cos(cth);
        oy = cy + r * sin(cth);
        set(hDynObs, 'Position', [ox-1, oy-1, 2, 2]); 
    end
    
    drawnow;       
    pause(0.025);  
end

title('Simulation Complete. Goal Reached!', 'Color', [0 0.6 0]);