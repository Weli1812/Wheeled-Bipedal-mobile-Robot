% =========================================================================
% Bipedal PC Test: MATLAB SIMULATION 
% Translation of C Path Planner
% =========================================================================
clc; clear; close all;

% --- Configuration & Constants ---
global MAP_WIDTH MAP_HEIGHT THETA_BINS GRID_RES_M MAX_NODES
MAP_WIDTH = 60; 
MAP_HEIGHT = 60;
THETA_BINS = 8;
GRID_RES_M = 0.2;
MAX_NODES = 20000;
MAX_PATH_LENGTH = 1000;

% --- Initialize Map ---
global_map = false(MAP_HEIGHT, MAP_WIDTH);
global_map = DrawObstacle(global_map, 10, 12, 15, 25);
global_map = DrawObstacle(global_map, 40, 45, 30, 37);
global_map = DrawObstacle(global_map, 35, 40, 10, 15);
global_map = DrawObstacle(global_map, 15, 20, 37, 40);
global_map = DrawObstacle(global_map, 22, 27, 22, 27);

% --- Initialize Kinematics ---
min_turn_rad_m = 0.5;
speed_m_s = 0.2;
kinematics = InitKinematics(min_turn_rad_m, speed_m_s);

% --- Goals and States ---
start_pose = [0.0, 0.0, 0.0];      % [x, y, theta]
goal_pose  = [25.0, 40.0, 1.5708];

inflated_map = InflateMap(global_map, 3);

% Approach pose (5 meters before goal)
approach_pose = zeros(1, 3);
approach_pose(1) = goal_pose(1) - 5.0 * cos(goal_pose(3));
approach_pose(2) = goal_pose(2) - 5.0 * sin(goal_pose(3));
approach_pose(3) = goal_pose(3);

fprintf('--- Bipedal PC Test: MATLAB SIMULATION ---\n');

% --- Initial Plan ---
[temp_path, path_len] = PlanKinematicPath(inflated_map, start_pose, approach_pose, kinematics);

if path_len > 0
    path_len = path_len - 1;
    
    % LAUNCH GATE (Initial Path)
    if path_len > 3
        for step = 1:3
            temp_path(step+1, 1) = temp_path(1, 1) + (step * 0.5 * cos(temp_path(1, 3)));
            temp_path(step+1, 2) = temp_path(1, 2) + (step * 0.5 * sin(temp_path(1, 3)));
            temp_path(step+1, 3) = temp_path(1, 3);
        end
    end
    
    % APPROACH GATE
    for i = 0:5
        temp_path(path_len+1, 1) = approach_pose(1) + (i * 1.0 * cos(goal_pose(3)));
        temp_path(path_len+1, 2) = approach_pose(2) + (i * 1.0 * sin(goal_pose(3)));
        temp_path(path_len+1, 3) = goal_pose(3);
        path_len = path_len + 1;
    end
    
    temp_path = temp_path(1:path_len, :);
    smoothed_path = SmoothPath(temp_path, 0.2, 0.2, 0.0001, inflated_map);
    active_traj = GenerateConstantVelocityTrajectory(smoothed_path, 1.0, 0.05, 2000);
    active_traj = FilterTrajectory(active_traj, 30, 0.05, kinematics);
else
    error('FAILED to find initial path!');
end

% --- Setup Plotting ---
figure('Name', 'Bipedal Path Planner Simulation', 'Color', 'w');
hold on; grid on; axis equal;
axis([-5 MAP_WIDTH -5 MAP_HEIGHT]);
colormap(flipud(gray));
imagesc([0 MAP_WIDTH-1], [0 MAP_HEIGHT-1], inflated_map);
set(gca, 'YDir', 'normal'); 
traj_plot = plot(active_traj(:,1), active_traj(:,2), 'b-', 'LineWidth', 2);
robot_plot = plot(start_pose(1), start_pose(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
title('Live Kinematic Trajectory & Re-Planning');

% --- Main State Machine Loop ---
STATE_MOVING = 0; STATE_WAITING = 1; STATE_REPLANNING = 2;
current_state = STATE_MOVING;

t_sim = 0.0;
wait_start_time = 0.0;
latest_r = 250.0;
current_path_idx = 1;
total_path_points = size(active_traj, 1);
last_print_time = 0.0;

while current_path_idx <= total_path_points
    
    % Mock Sensor Data (obstacle appears between 8.0s and 9.0s)
    if t_sim >= 8.0 && t_sim < 9.0
        latest_r = 6.0;
    else
        latest_r = 250.0;
    end
    
    t_sim = t_sim + 0.05;
    
    if (t_sim - last_print_time) >= 0.2
        fprintf('[t=%5.2f] Grid(X:%4.1f, Y:%4.1f) | Simulation Sensor Dist: %5.1f cm\n', ...
            t_sim, active_traj(current_path_idx, 1), active_traj(current_path_idx, 2), latest_r * 20.0);
        last_print_time = t_sim;
    end
    
    stopping_distance_meters = 1.5;
    stop_grids = stopping_distance_meters / GRID_RES_M;
    
    if latest_r < stop_grids && current_state == STATE_MOVING
        current_state = STATE_WAITING;
        wait_start_time = t_sim;
        fprintf('\n==================================================\n');
        fprintf('[!] OBSTACLE DETECTED! Braking at %.0fcm. Initiating 2-second patience wait...\n', latest_r * 20.0);
        fprintf('==================================================\n\n');
    end
    
    % Update Plot
    set(robot_plot, 'XData', active_traj(current_path_idx, 1), 'YData', active_traj(current_path_idx, 2));
    drawnow limitrate;
    
    switch current_state
        case STATE_MOVING
            current_path_idx = current_path_idx + 1;
            
        case STATE_WAITING
            if latest_r >= stop_grids
                fprintf('\n[t=%.2f] OBSTACLE CLEARED! Resuming current path...\n\n', t_sim);
                current_state = STATE_MOVING;
            elseif (t_sim - wait_start_time) >= 2.0
                current_state = STATE_REPLANNING;
                fprintf('\n[t=%.2f] Obstacle persists. Commencing Dubins Replan...\n', t_sim);
            end
            
        case STATE_REPLANNING
            ox = round(active_traj(current_path_idx, 1) + latest_r * cos(active_traj(current_path_idx, 3)));
            oy = round(active_traj(current_path_idx, 2) + latest_r * sin(active_traj(current_path_idx, 3)));
            
            % Draw dynamic obstacle in map
            global_map = DrawObstacle(global_map, ox-1, ox+1, oy-1, oy+1);
            inflated_map = InflateMap(global_map, 3);
            imagesc([0 MAP_WIDTH-1], [0 MAP_HEIGHT-1], inflated_map); % Update visual map
            
            curr_pose = active_traj(current_path_idx, 1:3);
            [temp_path, new_wp] = PlanKinematicPath(inflated_map, curr_pose, approach_pose, kinematics);
            
            % Erase the temporary dynamic obstacle
            global_map = EraseObstacle(global_map, ox-1, ox+1, oy-1, oy+1);
            
            if new_wp > 0
                new_wp = new_wp - 1;
                
                % LAUNCH GATE
                if new_wp > 3
                    for step = 1:3
                        temp_path(step+1, 1) = temp_path(1, 1) + (step * 0.5 * cos(temp_path(1, 3)));
                        temp_path(step+1, 2) = temp_path(1, 2) + (step * 0.5 * sin(temp_path(1, 3)));
                        temp_path(step+1, 3) = temp_path(1, 3);
                    end
                end
                
                % APPROACH GATE
                for i = 0:5
                    temp_path(new_wp+1, 1) = approach_pose(1) + (i * 1.0 * cos(goal_pose(3)));
                    temp_path(new_wp+1, 2) = approach_pose(2) + (i * 1.0 * sin(goal_pose(3)));
                    temp_path(new_wp+1, 3) = goal_pose(3);
                    new_wp = new_wp + 1;
                end
                
                temp_path = temp_path(1:new_wp, :);
                smoothed_path = SmoothPath(temp_path, 0.2, 0.2, 0.0001, inflated_map);
                active_traj = GenerateConstantVelocityTrajectory(smoothed_path, 1.0, 0.05, 2000);
                active_traj = FilterTrajectory(active_traj, 30, 0.05, kinematics);
                
                total_path_points = size(active_traj, 1);
                current_path_idx = 1;
                current_state = STATE_MOVING;
                
                % Update Plot Path
                set(traj_plot, 'XData', active_traj(:, 1), 'YData', active_traj(:, 2), 'Color', 'g');
                fprintf('[t=%.2f] REPLAN SUCCESS! Drive sequence engaged.\n\n', t_sim);
            else
                fprintf('[t=%.2f] Replan Failed! Waiting...\n', t_sim);
                current_state = STATE_WAITING;
                wait_start_time = t_sim;
            end
            latest_r = 250.0;
    end
end

fprintf('\nGoal Reached! Simulation Complete.\n');

% =========================================================================
% LOCAL FUNCTIONS (Core Path Planner Engine)
% =========================================================================

function map = DrawObstacle(map, start_x, end_x, start_y, end_y)
    global MAP_WIDTH MAP_HEIGHT
    for y = start_y:end_y
        for x = start_x:end_x
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
                map(y+1, x+1) = true; % MATLAB is 1-indexed
            end
        end
    end
end

function map = EraseObstacle(map, start_x, end_x, start_y, end_y)
    global MAP_WIDTH MAP_HEIGHT
    for y = start_y:end_y
        for x = start_x:end_x
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT)
                map(y+1, x+1) = false; 
            end
        end
    end
end

function kinematics = InitKinematics(min_turn_rad_m, speed_m_s)
    global GRID_RES_M
    step_distance = 1.5;
    step_dist_m = step_distance * GRID_RES_M;
    max_steer = step_dist_m / min_turn_rad_m;
    
    kinematics.velocity = speed_m_s;
    kinematics.min_turn = min_turn_rad_m;
    kinematics.max_yaw_rate = speed_m_s / min_turn_rad_m;
    kinematics.step_distance = step_distance;
    kinematics.steer_angles = [-max_steer, 0.0, max_steer];
    kinematics.prim_costs = [1.05, 1.0, 1.05];
end

function a = NormalizeAngle(a)
    a = mod(a, 2*pi);
    if a < 0
        a = a + 2*pi;
    end
end

function bin = GetThetaBin(angle)
    global THETA_BINS
    bin = floor(NormalizeAngle(angle) / 0.785398);
    if bin >= THETA_BINS || bin < 0
        bin = 0;
    end
end

function valid = IsValid(map, x, y)
    global MAP_WIDTH MAP_HEIGHT
    ix = round(x) + 1; 
    iy = round(y) + 1;
    if ix < 1 || ix > MAP_WIDTH || iy < 1 || iy > MAP_HEIGHT
        valid = false;
        return;
    end
    if map(iy, ix)
        valid = false;
        return;
    end
    valid = true;
end

function h = CalculateHeuristic(curr, goal, min_turn_rad)
    global GRID_RES_M
    dx = goal(1) - curr(1);
    dy = goal(2) - curr(2);
    dist = sqrt(dx^2 + dy^2);
    if dist < 0.1
        h = 0.0; return;
    end
    
    angle_to_goal = atan2(dy, dx);
    diff_start = curr(3) - angle_to_goal;
    diff_start = mod(diff_start + pi, 2*pi) - pi;
    
    diff_end = goal(3) - angle_to_goal;
    diff_end = mod(diff_end + pi, 2*pi) - pi;
    
    turning_radius_grids = min_turn_rad / GRID_RES_M;
    turn_penalty = turning_radius_grids * (abs(diff_start) + abs(diff_end));
    
    h = dist + turn_penalty;
end

function inflated = InflateMap(map, cells)
    global MAP_WIDTH MAP_HEIGHT
    inflated = map;
    [ys, xs] = find(map);
    for i = 1:length(xs)
        x = xs(i) - 1; y = ys(i) - 1;
        for dy = -cells:cells
            for dx = -cells:cells
                if dx*dx + dy*dy <= cells*cells
                    nx = x + dx; ny = y + dy;
                    if nx >= 0 && nx < MAP_WIDTH && ny >= 0 && ny < MAP_HEIGHT
                        inflated(ny+1, nx+1) = true;
                    end
                end
            end
        end
    end
end

function [path_out, path_len] = PlanKinematicPath(map, start, goal, kin)
    global MAP_WIDTH MAP_HEIGHT THETA_BINS MAX_NODES
    closed_list = false(MAP_HEIGHT, MAP_WIDTH, THETA_BINS);
    
    % Structure of Arrays for Nodes
    n_pose = zeros(MAX_NODES, 3);
    n_g = zeros(MAX_NODES, 1);
    n_f = zeros(MAX_NODES, 1);
    n_parent = zeros(MAX_NODES, 1);
    n_open = false(MAX_NODES, 1);
    
    % Init
    n_pose(1,:) = start;
    n_g(1) = 0;
    n_f(1) = CalculateHeuristic(start, goal, kin.min_turn);
    n_parent(1) = 0; % 0 represents -1 (null parent in MATLAB)
    n_open(1) = true;
    
    active_nodes = 1;
    best_node_idx = -1;
    path_found = false;
    
    while active_nodes < MAX_NODES - 5
        % Find min f
        open_idxs = find(n_open(1:active_nodes));
        if isempty(open_idxs)
            break;
        end
        [~, min_f_idx] = min(n_f(open_idxs));
        current_idx = open_idxs(min_f_idx);
        
        n_open(current_idx) = false;
        curr_pose = n_pose(current_idx, :);
        
        dx = goal(1) - curr_pose(1);
        dy = goal(2) - curr_pose(2);
        dist = sqrt(dx^2 + dy^2);
        
        diff_theta = curr_pose(3) - goal(3);
        diff_theta = mod(diff_theta + pi, 2*pi) - pi;
        
        if dist < 2.0 && abs(diff_theta) < 0.4
            best_node_idx = current_idx;
            path_found = true;
            break;
        end
        
        cx = round(curr_pose(1)) + 1;
        cy = round(curr_pose(2)) + 1;
        c_th = GetThetaBin(curr_pose(3)) + 1;
        
        % Ensure bounds to prevent crash
        if cx >= 1 && cx <= MAP_WIDTH && cy >= 1 && cy <= MAP_HEIGHT
            closed_list(cy, cx, c_th) = true;
        end
        
        for i = 1:3
            next_pose = zeros(1,3);
            next_pose(3) = NormalizeAngle(curr_pose(3) + kin.steer_angles(i));
            next_pose(1) = curr_pose(1) + kin.step_distance * cos(next_pose(3));
            next_pose(2) = curr_pose(2) + kin.step_distance * sin(next_pose(3));
            
            if ~IsValid(map, next_pose(1), next_pose(2))
                continue;
            end
            
            nx = round(next_pose(1)) + 1;
            ny = round(next_pose(2)) + 1;
            n_th = GetThetaBin(next_pose(3)) + 1;
            
            if closed_list(ny, nx, n_th)
                continue;
            end
            
            active_nodes = active_nodes + 1;
            tentative_g = n_g(current_idx) + kin.prim_costs(i);
            
            n_pose(active_nodes, :) = next_pose;
            n_g(active_nodes) = tentative_g;
            n_f(active_nodes) = tentative_g + CalculateHeuristic(next_pose, goal, kin.min_turn);
            n_parent(active_nodes) = current_idx;
            n_open(active_nodes) = true;
        end
    end
    
    if ~path_found
        path_out = []; path_len = 0; return;
    end
    
    % Traceback
    current_trace = best_node_idx;
    raw_path = [];
    while current_trace > 0
        raw_path = [n_pose(current_trace, :); raw_path]; 
        current_trace = n_parent(current_trace);
    end
    path_out = raw_path;
    path_len = size(path_out, 1);
end

function path = SmoothPath(path, d_weight, s_weight, tol, map)
    len = size(path, 1);
    if len <= 6, return; end
    orig = path;
    change = tol;
    iters = 0;
    while change >= tol && iters < 1000
        change = 0.0;
        for i = 3:(len - 5) % Protect Launch and Approach gates
            aux = path(i, 1:2);
            new_x = path(i,1) + d_weight * (orig(i,1) - path(i,1)) + ...
                s_weight * (path(i-1,1) + path(i+1,1) - 2*path(i,1));
            new_y = path(i,2) + d_weight * (orig(i,2) - path(i,2)) + ...
                s_weight * (path(i-1,2) + path(i+1,2) - 2*path(i,2));
            
            if IsValid(map, new_x, new_y)
                path(i,1) = new_x;
                path(i,2) = new_y;
                change = change + abs(aux(1) - path(i,1)) + abs(aux(2) - path(i,2));
            end
        end
        iters = iters + 1;
    end
    for i = 1:(len-1)
        path(i,3) = atan2(path(i+1,2) - path(i,2), path(i+1,1) - path(i,1));
    end
    path(len,3) = path(len-1,3);
end

function traj = GenerateConstantVelocityTrajectory(path, v, dt, max_len)
    len = size(path, 1);
    if len < 2, traj=[]; return; end
    S = zeros(len, 1);
    for i = 2:len
        S(i) = S(i-1) + sqrt((path(i,1)-path(i-1,1))^2 + (path(i,2)-path(i-1,2))^2);
    end
    total_time = S(end) / v;
    num_samples = min(floor(total_time / dt) + 1, max_len);
    
    traj = zeros(num_samples, 4); % [x, y, theta, t]
    seg = 1;
    for step = 1:num_samples
        t = (step - 1) * dt;
        target_s = t * v;
        while seg < (len - 1) && S(seg+1) < target_s
            seg = seg + 1;
        end
        seg_len = S(seg+1) - S(seg);
        if seg_len > 0.0001
            alpha = (target_s - S(seg)) / seg_len;
        else
            alpha = 0.0;
        end
        
        traj(step, 1) = path(seg,1) + alpha * (path(seg+1,1) - path(seg,1));
        traj(step, 2) = path(seg,2) + alpha * (path(seg+1,2) - path(seg,2));
        
        diff = path(seg+1,3) - path(seg,3);
        diff = mod(diff + pi, 2*pi) - pi;
        
        traj(step, 3) = path(seg,3) + alpha * diff;
        traj(step, 4) = t;
    end
end

function traj = FilterTrajectory(traj, max_win, dt, kin)
    len = size(traj, 1);
    if len <= 40, return; end
    temp = traj;
    
    p_start = 4; p_end = 4; % 1-indexed (equivalent to C protect_start=3)
    
    % Position Average Filter
    for i = p_start:(len - p_end)
        win = max_win;
        win = min([win, i - p_start, len - p_end - i]);
        
        idx_range = (i - win):(i + win);
        traj(i, 1) = mean(temp(idx_range, 1));
        traj(i, 2) = mean(temp(idx_range, 2));
    end
    
    % Update Theta from positions
    for i = 1:(len-1)
        traj(i, 3) = atan2(traj(i+1, 2) - traj(i, 2), traj(i+1, 1) - traj(i, 1));
    end
    traj(len, 3) = traj(len-1, 3);
    
    temp(:,3) = traj(:,3);
    
    % Theta Average Filter
    for i = p_start:(len - p_end)
        win = max_win;
        win = min([win, i - p_start, len - p_end - i]);
        
        idx_range = (i - win):(i + win);
        s_sin = sum(sin(temp(idx_range, 3)));
        s_cos = sum(cos(temp(idx_range, 3)));
        traj(i, 3) = atan2(s_sin, s_cos);
    end
    
    % STRICT BIPEDAL KINEMATIC CLAMPING
    max_dtheta = kin.max_yaw_rate * dt;
    for i = 2:len
        dtheta = traj(i, 3) - traj(i-1, 3);
        dtheta = mod(dtheta + pi, 2*pi) - pi;
        
        if dtheta > max_dtheta
            traj(i, 3) = traj(i-1, 3) + max_dtheta;
        elseif dtheta < -max_dtheta
            traj(i, 3) = traj(i-1, 3) - max_dtheta;
        end
        traj(i, 3) = mod(traj(i, 3) + pi, 2*pi) - pi;
    end
end