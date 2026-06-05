clc;
clear;
close all;

%% ==============================
%  Simple 2 cm Jump Simulation
%  Robot reaches endpoint at h = 0.17 m
%  Then jumps by extending hips to h = 0.311 m
%% ==============================

%% Robot parameters
g = 9.81;              % gravity [m/s^2]

m_body = 11.3;         % body mass [kg]
m_wheel = 0.15;        % one wheel mass [kg]
m = m_body + 2*m_wheel; % total mass [kg]

r_wheel = 0.06;        % wheel radius [m]

h_min = 0.170;         % lowest robot height [m]
h_max = 0.311;         % maximum robot height [m]

stroke = h_max - h_min; % available push stroke [m]

%% Jump target
jump_height = 0.02;    % target jump height [m] = 2 cm

% Required takeoff velocity for 2 cm jump
v_takeoff = sqrt(2*g*jump_height);

% Required upward acceleration during push
a_push = v_takeoff^2 / (2*stroke);

% Push time needed
T_push = v_takeoff / a_push;

fprintf('Required takeoff velocity = %.3f m/s\n', v_takeoff);
fprintf('Required push acceleration = %.3f m/s^2\n', a_push);
fprintf('Required push time = %.3f s\n', T_push);

% Required total ground force during push
F_push = m*(g + a_push);
fprintf('Average ground force during push = %.2f N\n', F_push);
fprintf('Average force per side = %.2f N\n', F_push/2);

%% Timing
T_stabilize = 0.5;     % stop and stabilize before jump
T_flight = 2*v_takeoff/g;
T_landing = 0.5;       % landing absorption time
T_stand = 0.5;

dt = 0.005;
t_end = T_stabilize + T_push + T_flight + T_landing + T_stand;
t = 0:dt:t_end;

%% Storage
N = length(t);

h_rel = zeros(1,N);       % body height relative to wheel axle
base_y = zeros(1,N);      % vertical displacement of whole robot
com_y = zeros(1,N);       % COM absolute height
wheel_clearance = zeros(1,N);

phase = strings(1,N);

%% Simulation
for i = 1:N

    ti = t(i);

    if ti <= T_stabilize
        %% Phase 1: Stabilize at endpoint
        h_rel(i) = h_min;
        base_y(i) = 0;
        phase(i) = "STABILIZE";

    elseif ti <= T_stabilize + T_push
        %% Phase 2: Jump push
        tau = ti - T_stabilize;

        % Accelerating hip extension:
        % h = h_min + 0.5*a*t^2
        h_rel(i) = h_min + 0.5*a_push*tau^2;

        if h_rel(i) > h_max
            h_rel(i) = h_max;
        end

        base_y(i) = 0;
        phase(i) = "PUSH";

    elseif ti <= T_stabilize + T_push + T_flight
        %% Phase 3: Flight
        tau = ti - T_stabilize - T_push;

        h_rel(i) = h_max;

        % Whole robot moves upward then downward
        base_y(i) = v_takeoff*tau - 0.5*g*tau^2;

        if base_y(i) < 0
            base_y(i) = 0;
        end

        phase(i) = "FLIGHT";

    elseif ti <= T_stabilize + T_push + T_flight + T_landing
        %% Phase 4: Landing absorption
        tau = ti - T_stabilize - T_push - T_flight;

        h_absorb = 0.250;  % bend after landing to absorb impact

        alpha = tau / T_landing;
        alpha = min(max(alpha,0),1);

        % Smooth movement from h_max to h_absorb
        smooth_alpha = 0.5 - 0.5*cos(pi*alpha);

        h_rel(i) = h_max + smooth_alpha*(h_absorb - h_max);
        base_y(i) = 0;
        phase(i) = "LANDING";

    else
        %% Phase 5: Stand stable
        h_rel(i) = 0.250;
        base_y(i) = 0;
        phase(i) = "STAND";
    end

    com_y(i) = base_y(i) + h_rel(i);
    wheel_clearance(i) = base_y(i);
end

%% ==============================
%  Plots
%% ==============================

figure;
plot(t, com_y, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('COM Height [m]');
title('Robot COM Height During 2 cm Jump');

figure;
plot(t, wheel_clearance*100, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Wheel Clearance [cm]');
title('Wheel Clearance From Ground');

figure;
plot(t, h_rel, 'LineWidth', 2);
grid on;
xlabel('Time [s]');
ylabel('Hip Height h [m]');
title('Hip Height Command');

%% ==============================
%  Simple Animation
%% ==============================

figure;
axis equal;
grid on;
hold on;

xlim([-0.7 0.7]);
ylim([-0.05 0.55]);

xlabel('X [m]');
ylabel('Y [m]');
title('Simple Robot Jump Animation');

% Ground
plot([-1 1], [0 0], 'k', 'LineWidth', 2);

% Robot drawing handles
leftWheel  = rectangle('Position',[-0.25-r_wheel, 0-r_wheel, 2*r_wheel, 2*r_wheel], ...
                       'Curvature',[1 1], 'LineWidth',2);
rightWheel = rectangle('Position',[ 0.25-r_wheel, 0-r_wheel, 2*r_wheel, 2*r_wheel], ...
                       'Curvature',[1 1], 'LineWidth',2);

body = rectangle('Position',[-0.18, h_min+0.05, 0.36, 0.12], ...
                 'Curvature',0.1, 'LineWidth',2);

leftLeg = plot([0 -0.25], [h_min 0], 'LineWidth', 3);
rightLeg = plot([0 0.25], [h_min 0], 'LineWidth', 3);

comPoint = plot(0, h_min, 'ko', 'MarkerFaceColor','k');

phaseText = text(-0.65, 0.50, '', 'FontSize', 12, 'FontWeight', 'bold');

for i = 1:8:N

    y_base = base_y(i);
    y_com = com_y(i);

    wheel_center_y = y_base + r_wheel;

    % Update wheels
    leftWheel.Position  = [-0.25-r_wheel, wheel_center_y-r_wheel, 2*r_wheel, 2*r_wheel];
    rightWheel.Position = [ 0.25-r_wheel, wheel_center_y-r_wheel, 2*r_wheel, 2*r_wheel];

    % Body position
    body_y = y_com + 0.04;
    body.Position = [-0.18, body_y, 0.36, 0.12];

    % Legs from body COM area to wheels
    set(leftLeg,  'XData', [0 -0.25], 'YData', [y_com wheel_center_y]);
    set(rightLeg, 'XData', [0  0.25], 'YData', [y_com wheel_center_y]);

    % COM point
    set(comPoint, 'XData', 0, 'YData', y_com);

    % Phase text
    set(phaseText, 'String', ['Mode: ', char(phase(i))]);

    drawnow;
end