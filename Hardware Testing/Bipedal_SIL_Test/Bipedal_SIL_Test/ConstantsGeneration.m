% 1. Load the raw C-generated trajectory
data = readmatrix('path_initial.csv');
t_raw = data(:, 1);
x_raw = data(:, 2);
y_raw = data(:, 3);
theta_raw = unwrap(data(:, 4)); % Unwrap to prevent 360-degree jumps

% 2. Resample to exactly 100 points for the embedded controller
N = 100;
t_100 = linspace(t_raw(1), t_raw(end), N)';

% Interpolate X, Y, and Theta to fit the 100-point timeline
X_100 = interp1(t_raw, x_raw, t_100, 'spline');
Y_100 = interp1(t_raw, y_raw, t_100, 'spline');
Theta_100 = interp1(t_raw, theta_raw, t_100, 'spline');

% -> POPULATE P [100, 2]
P = [X_100, Y_100];

% -> POPULATE theta_ref_unwrapped [100]
theta_ref_unwrapped = Theta_100;

% 3. Calculate Cumulative Distance (S_path)
S_path = zeros(N, 1);
for i = 2:N
    dx = X_100(i) - X_100(i-1);
    dy = Y_100(i) - Y_100(i-1);
    S_path(i) = S_path(i-1) + sqrt(dx^2 + dy^2);
end
% -> POPULATE S_path [100]

% 4. Calculate Velocities using central differences
vd_path = zeros(N, 1);
wd_path = zeros(N, 1);
dt = t_100(2) - t_100(1); % Constant time step for the 100 points

for i = 2:N-1
    % Linear Velocity (ds/dt)
    vd_path(i) = (S_path(i+1) - S_path(i-1)) / (2 * dt);
    % Angular Velocity (dtheta/dt)
    wd_path(i) = (Theta_100(i+1) - Theta_100(i-1)) / (2 * dt);
end

% Handle the endpoints
vd_path(1) = (S_path(2) - S_path(1)) / dt;
vd_path(N) = (S_path(N) - S_path(N-1)) / dt;
wd_path(1) = (Theta_100(2) - Theta_100(1)) / dt;
wd_path(N) = (Theta_100(N) - Theta_100(N-1)) / dt;

% -> POPULATE vd_path [100] and wd_path [100]

% =========================================================================
% 5. EXPORT VARIABLES TO TEXT FILE (MATLAB CODE FORMAT)
% =========================================================================
filename = 'reference_variables.txt';
fileID = fopen(filename, 'w');

fprintf(fileID, '%% --- PASTE THIS DIRECTLY INTO YOUR REFERENCE GENERATOR ---\n\n');

% Write S_path (Column vector)
fprintf(fileID, 'S_path = [\n');
fprintf(fileID, '    %.6f;\n', S_path);
fprintf(fileID, '];\n\n');

% Write P (N x 2 Matrix)
fprintf(fileID, 'P = [\n');
for i = 1:N
    fprintf(fileID, '    %.6f,  %.6f;\n', P(i,1), P(i,2));
end
fprintf(fileID, '];\n\n');

% Write theta_ref_unwrapped (Column vector)
fprintf(fileID, 'theta_ref_unwrapped = [\n');
fprintf(fileID, '    %.6f;\n', theta_ref_unwrapped);
fprintf(fileID, '];\n\n');

% Write vd_path (Column vector)
fprintf(fileID, 'vd_path = [\n');
fprintf(fileID, '    %.6f;\n', vd_path);
fprintf(fileID, '];\n\n');

% Write wd_path (Column vector)
fprintf(fileID, 'wd_path = [\n');
fprintf(fileID, '    %.6f;\n', wd_path);
fprintf(fileID, '];\n\n');

fclose(fileID);
disp(['All 100-point variables successfully generated and saved to: ', filename]);