clear; clc; close all;

%% =========================
% 1) Robot parameters
% ==========================
L      = 0.2;      % wheelbase [m]
Ts_ref = 0.05;     % reference sample time [s]
vd_nom = 0.3;      % nominal speed for LQR linearization

%% =========================
% 2) Map from mapMatrix
% ==========================
mapMatrix = zeros(100,100);
mapMatrix(30:50,20:25) = 1;
mapMatrix(60:75,80:90) = 1;
mapMatrix(20:30,70:80) = 1;
mapMatrix(75:80,30:40) = 1;
mapMatrix(45:55,45:55) = 1;

resolution = 10;   % cells per meter
map = binaryOccupancyMap(mapMatrix, resolution);

%% =========================
% 3) Read trajectory from CSV
% ==========================
excelFile = 'Path 3.csv';
data = readtable(excelFile);

t_excel = data.Time_s;
x_excel = data.X_m;
y_excel = data.Y_m;

t_excel = t_excel(:);
x_excel = x_excel(:);
y_excel = y_excel(:);

% Remove invalid rows
valid = ~(isnan(t_excel) | isnan(x_excel) | isnan(y_excel));
t_excel = t_excel(valid);
x_excel = x_excel(valid);
y_excel = y_excel(valid);

% Sort by time
[t_excel, idxSort] = sort(t_excel);
x_excel = x_excel(idxSort);
y_excel = y_excel(idxSort);

% Remove duplicate time values
[t_excel, idxUnique] = unique(t_excel, 'stable');
x_excel = x_excel(idxUnique);
y_excel = y_excel(idxUnique);

%% =========================
% 4) Uniform time for Simulink
% ==========================
t_ref = (t_excel(1):Ts_ref:t_excel(end))';
if t_ref(end) < t_excel(end)
    t_ref = [t_ref; t_excel(end)];
end

x_ref = interp1(t_excel, x_excel, t_ref, 'pchip');
y_ref = interp1(t_excel, y_excel, t_ref, 'pchip');

% Mild smoothing for path only
x_ref = smoothdata(x_ref, 'gaussian', 9);
y_ref = smoothdata(y_ref, 'gaussian', 9);

P = [x_ref y_ref];

%% =========================
% 5) Derive theta from path
% ==========================
dx_geom = gradient(x_ref);
dy_geom = gradient(y_ref);

theta_ref = atan2(dy_geom, dx_geom);
theta_ref = unwrap(theta_ref);

%% =========================
% 6) Derive v and w from path
% ==========================
% Linear speed from path spacing
ds = sqrt(diff(x_ref).^2 + diff(y_ref).^2);
ds = [ds; ds(end)];

vd_ref = ds / Ts_ref;
vd_ref = smoothdata(vd_ref, 'gaussian', 9);

% Optional overall scaling if robot is too fast
% vd_ref = 0.5 * vd_ref;

% Angular speed from heading
wd_ref = gradient(theta_ref) / Ts_ref;
wd_ref = smoothdata(wd_ref, 'gaussian', 21);

% Start/end conditions
vd_ref(1)   = 0;
vd_ref(end) = 0;
wd_ref(end) = 0;

% Limit angular speed
wd_max = 0.3;
wd_ref = max(min(wd_ref, wd_max), -wd_max);

% Initial conditions
theta0 = theta_ref(1);
x0     = x_ref(1);
y0     = y_ref(1);
start  = [x0 y0];

%% =========================
% 7) Collision check
% ==========================
occ = getOccupancy(map, P);
if any(occ > 0.5)
    warning('Reference trajectory intersects an obstacle.');
end

%% =========================
% 8) Constant LQR design
% ==========================
vd_pos = vd_ref(vd_ref > 0.02);
if ~isempty(vd_pos)
    vd_nom = mean(vd_pos);
end

A = [0 0 0;
     0 0 vd_nom;
     0 0 0];

B = [1 0;
     0 0;
     0 1];

Q = diag([20 30 10]);
R = diag([5 2]);

K = lqr(A,B,Q,R);

%% =========================
% 9) Simulation time
% ==========================
T_end = t_ref(end) + 6;

%% =========================
% 10) Prepare signals
% ==========================
P         = double(P);
theta_ref = double(theta_ref(:));
vd_ref    = double(vd_ref(:));
wd_ref    = double(wd_ref(:));
Ts_ref    = double(Ts_ref);
x0        = double(x0);
y0        = double(y0);
theta0    = double(theta0);
start     = double(start);

clear out x_out y_out theta_out

%% =========================
% 11) Run Simulink model
% ==========================
out = sim('Traj', 'StopTime', num2str(T_end));

%% =========================
% 12) Read outputs
% ==========================
x     = out.x_out(:);
y     = out.y_out(:);
theta = out.theta_out(:);

%% =========================
% 13) Animate robot motion only
% ==========================
figure;
show(map);
hold on;
plot(P(:,1), P(:,2), 'g--', 'LineWidth', 2);

robot = plot(x(1), y(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
dirr  = quiver(x(1), y(1), cos(theta(1)), sin(theta(1)), 0.4, 'r', 'LineWidth', 2);
traj  = plot(x(1), y(1), 'b', 'LineWidth', 2);

axis equal;
grid on;
xlabel('x (m)');
ylabel('y (m)');
title('Robot Motion on Reference Path');

for k = 1:length(x)
    set(robot, 'XData', x(k), 'YData', y(k));
    set(dirr, ...
        'XData', x(k), ...
        'YData', y(k), ...
        'UData', cos(theta(k)), ...
        'VData', sin(theta(k)));
    set(traj, 'XData', x(1:k), 'YData', y(1:k));
    drawnow;
end