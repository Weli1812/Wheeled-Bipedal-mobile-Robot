function find_max_phi
clear; clc;

% -----------------------------
% User settings
% -----------------------------
phi_low_deg  = 0;     % definitely recoverable
phi_high_deg = 20;    % probably too large, adjust if needed
tol_deg      = 0.1;   % search precision
max_iter     = 20;

% Success test settings
phi_final_tol = 0.05;   % rad
phi_peak_fail = 0.8;    % rad, immediate fail if exceeded
sim_stop_time = 12;     % seconds

% Store best result
best_phi_deg = phi_low_deg;

for k = 1:max_iter
    phi_mid_deg = 0.5*(phi_low_deg + phi_high_deg);
    ok = run_phi_test(phi_mid_deg, sim_stop_time, phi_final_tol, phi_peak_fail);

    fprintf('Test %2d: phi0 = %.3f deg --> %s\n', ...
        k, phi_mid_deg, string(ok));

    if ok
        best_phi_deg = phi_mid_deg;
        phi_low_deg  = phi_mid_deg;
    else
        phi_high_deg = phi_mid_deg;
    end

    if (phi_high_deg - phi_low_deg) < tol_deg
        break;
    end
end

fprintf('\nEstimated maximum recoverable phi0 = %.3f deg\n', best_phi_deg);
fprintf('Estimated maximum recoverable phi0 = %.4f rad\n', deg2rad(best_phi_deg));
end


function ok = run_phi_test(phi0_deg, sim_stop_time, phi_final_tol, phi_peak_fail)

    % --------------------------------
    % Load your normal setup script data
    % --------------------------------
    run('main.m');

    % --------------------------------
    % Override the test for pure balance
    % --------------------------------
    testMode = "STRAIGHT_ONLY";   % or make a BALANCE_ONLY mode if you prefer

    % zero references for pure stabilization test
    vd_ref(:) = 0;
    wd_ref(:) = 0;
    s_ref(:)  = 0;
    theta_ref_unwrapped(:) = theta_ref_unwrapped(1);

    % initial condition override
    x_init(1) = deg2rad(phi0_deg);   % phi0
    x_init(2) = 0;                   % s
    x_init(4) = 0;                   % phi_dot
    x_init(5) = 0;                   % v
    x_init(6) = 0;                   % omega

    % make sure workspace sees updated values
    assignin('base','vd_ref',vd_ref);
    assignin('base','wd_ref',wd_ref);
    assignin('base','s_ref',s_ref);
    assignin('base','theta_ref_unwrapped',theta_ref_unwrapped);
    assignin('base','x_init',x_init);

    % run sim
    out = sim('OneCombinedLQR', 'StopTime', num2str(sim_stop_time));

    phi = out.phi_out(:);

    % success conditions
    finite_ok = all(isfinite(phi));
    peak_ok   = max(abs(phi)) < phi_peak_fail;
    final_ok  = abs(phi(end)) < phi_final_tol;

    ok = finite_ok && peak_ok && final_ok;
end