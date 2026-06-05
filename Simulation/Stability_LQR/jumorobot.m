% ---------------------------------------------------------
% 2D Animation: Wheeled Bipedal Robot Jump Sequence
% ---------------------------------------------------------
clc; clear; close all;

%% 1. Setup Physical Parameters
% Dimensions based on hardware specifications
r_wheel = 0.060;     % Wheel radius 60mm
L_min = 0.170;       % Minimum leg length 170mm
L_max = 0.311;       % Maximum leg length 311mm
body_w = 0.250;      % Visual width of the main body
body_h = 0.150;      % Visual height of the main body
leg_dist = 0.150;    % Distance between left and right legs visually

% Physics constants
g = 9.81;            % Gravity
v_max = 1.72;        % Estimated max liftoff velocity (m/s)

%% 2. Setup Figure and Axes
fig = figure('Name', 'Robot Jump Animation', 'Color', 'w', 'Position', [200, 200, 600, 700]);
ax = axes('Parent', fig, 'XLim', [-0.4 0.4], 'YLim', [-0.05 0.8]);
hold(ax, 'on');
grid(ax, 'on');
title('Bipedal Robot Jump Sequence', 'FontSize', 14);
xlabel('X Position (m)');
ylabel('Y Position (m)');

% Draw Ground
plot([-1 1], [0 0], 'k', 'LineWidth', 2);

%% 3. Initialize Graphics Objects
% Wheels (Left and Right)
wheel_L = rectangle('Position', [-leg_dist/2-r_wheel, 0, r_wheel*2, r_wheel*2], ...
                    'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2]);
wheel_R = rectangle('Position', [leg_dist/2-r_wheel, 0, r_wheel*2, r_wheel*2], ...
                    'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2]);

% Legs (Represented as thick lines connecting wheels to body)
leg_L = plot([-leg_dist/2, -leg_dist/2], [r_wheel, r_wheel + L_max], 'b', 'LineWidth', 6);
leg_R = plot([leg_dist/2, leg_dist/2], [r_wheel, r_wheel + L_max], 'b', 'LineWidth', 6);

% Main Body
body_rect = rectangle('Position', [-body_w/2, r_wheel + L_max, body_w, body_h], ...
                      'FaceColor', [0.7 0.7 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);

% Text indicator for State Machine
state_text = text(-0.35, 0.7, 'State: SQUAT', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'r');

%% 4. Initialize Simulation Variables
dt = 0.02;           % Time step for animation
t = 0;
state = 'SQUAT';
current_L = 0.250;   % Start at a mid-stance length
y_wheel = 0;         % Wheels start on the ground
v_y = 0;             % Initial vertical velocity

%% 5. Animation Loop
while t < 2.5 && ishandle(fig)
    
    % --- State Machine Logic ---
    switch state
        case 'SQUAT'
            % Compress legs to minimum length
            current_L = max(current_L - 0.2 * dt, L_min);
            if current_L <= L_min
                state = 'THRUST';
                pause(0.2); % Brief pause at bottom of squat
            end
            
        case 'THRUST'
            % Explosive extension
            current_L = min(current_L + 1.72 * dt, L_max);
            if current_L >= L_max
                state = 'FLIGHT';
                v_y = v_max; % Transfer velocity to the entire robot
            end
            
        case 'FLIGHT'
            % Projectile motion for the whole robot
            y_wheel = y_wheel + v_y * dt;
            v_y = v_y - g * dt; % Gravity pulls it down
            
            % Active leg retraction while in air
            current_L = max(current_L - 1.5 * dt, L_min + 0.03); 
            
            % Check for ground impact
            if y_wheel <= 0
                y_wheel = 0;
                state = 'LANDING';
            end
            
        case 'LANDING'
            % Absorb impact and settle to stance
            current_L = min(current_L + 0.8 * dt, 0.22); 
    end
    
    % --- Update Graphics ---
    % Calculate new Y position of the body based on wheels and legs
    body_y = y_wheel + r_wheel + current_L;
    
    % Update rectangles (Position array is [x, y, width, height])
    set(wheel_L, 'Position', [-leg_dist/2-r_wheel, y_wheel, r_wheel*2, r_wheel*2]);
    set(wheel_R, 'Position', [leg_dist/2-r_wheel, y_wheel, r_wheel*2, r_wheel*2]);
    set(body_rect, 'Position', [-body_w/2, body_y, body_w, body_h]);
    
    % Update leg lines
    set(leg_L, 'YData', [y_wheel + r_wheel, body_y]);
    set(leg_R, 'YData', [y_wheel + r_wheel, body_y]);
    
    % Update State Text
    set(state_text, 'String', ['State: ' state]);
    
    % Render the frame
    drawnow;
    
    % Step time
    t = t + dt;
    pause(0.01); % Controls playback speed visually
end