% 1. Make sure all your variables are loaded into the workspace first
load('xd.mat', 'xd');
load('yd.mat', 'yd');
load('thetad.mat', 'thetad');
load('vd.mat', 'vd');
load('wd.mat', 'wd');

% 2. Open the file to write
fid = fopen('trajectory_data.h', 'w');

% 3. Write Header Guards and Length
fprintf(fid, '#ifndef TRAJECTORY_DATA_H\n');
fprintf(fid, '#define TRAJECTORY_DATA_H\n\n');
fprintf(fid, 'const int TRAJ_LENGTH = %d;\n\n', length(xd));

% 4. Write xd
fprintf(fid, 'const double xd_traj[] = {');
fprintf(fid, '%f, ', xd(1:end-1)); fprintf(fid, '%f};\n\n', xd(end));

% 5. Write yd
fprintf(fid, 'const double yd_traj[] = {');
fprintf(fid, '%f, ', yd(1:end-1)); fprintf(fid, '%f};\n\n', yd(end));

% 6. Write thetad
fprintf(fid, 'const double thetad_traj[] = {');
fprintf(fid, '%f, ', thetad(1:end-1)); fprintf(fid, '%f};\n\n', thetad(end));

% 7. Write vd
fprintf(fid, 'const double vd_traj[] = {');
fprintf(fid, '%f, ', vd(1:end-1)); fprintf(fid, '%f};\n\n', vd(end));

% 8. Write wd
fprintf(fid, 'const double wd_traj[] = {');
fprintf(fid, '%f, ', wd(1:end-1)); fprintf(fid, '%f};\n\n', wd(end));

% 9. Close the file
fprintf(fid, '#endif\n');
fclose(fid);

disp('Successfully generated the FULL trajectory_data.h!');