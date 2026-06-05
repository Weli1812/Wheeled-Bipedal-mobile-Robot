function [pwmr, pwml, dirr, dirl] = LQRwheels(x_curr, y_curr, theta_curr,dt, xd_k, yd_k, thetad_k, vd_k, wd_k, vr_actual, vl_actual) %#codegen
% lqr_pid_controller Calculates motor PWM commands for a differential drive robot
% Inputs:
%   x_curr, y_curr, theta_curr : Current estimated pose of the robot
%   xd_k, yd_k, thetad_k       : Desired reference pose
%   vd_k, wd_k                 : Desired reference velocities
%   vr_actual, vl_actual       : Actual wheel speeds from encoders
% Outputs:
%   pwmr, pwml                 : PWM duty cycles (0.0 to 1.0)
%   dirr, dirl                 : Direction booleans (0 = forward, 1 = backward)

    %% 1. Parameters
    L = 0.15;            % wheelbase [m]
    Ts_ref = dt;       % Sample time [s]
    pwm_limit = 5.0;    % Saturation limit
    
    % LQR Gain Matrix
    K = [2.2361, 0,      0;
         0,      4.0825, 2.6535];
    
    % PID Parameters
    Kp = 5.0;
    Ki = 0.07;
    
    %% 2. State Memory (Integrators)
    % Persistent variables keep their values between function calls
    persistent int_R int_L
    
    % Initialize integrators on the first run
    if isempty(int_R)
        int_R = 0;
        int_L = 0;
    end
    
    %% 3. Error Calculation (Kinematics)
    dx = x_curr - xd_k;
    dy = y_curr - yd_k;
    
    ex =  cos(thetad_k)*dx + sin(thetad_k)*dy;
    ey = -sin(thetad_k)*dx + cos(thetad_k)*dy;
    etheta = atan2(sin(theta_curr - thetad_k), cos(theta_curr - thetad_k));
    
    e = [ex; ey; etheta];
    
    %% 4. LQR Control Law
    u = -K * e;
    u1 = u(1);
    u2 = u(2);
    
    %% 5. Velocity Superposition & Inverse Kinematics
    v = vd_k + u1;
    w = wd_k + u2;
    
    vr_ref = v + (L/2)*w;
    vl_ref = v - (L/2)*w;
    
    %% 6. Motor PID Controllers with Clamping Anti-Windup
    err_R = vr_ref - vr_actual;
    err_L = vl_ref - vl_actual;
    
    % Proportional + Integral Calculation
    topwmr_pre = Kp * err_R + Ki * int_R;
    topwml_pre = Kp * err_L + Ki * int_L;
    
    % Saturation Clamping
    topwmr = max(min(topwmr_pre, pwm_limit), -pwm_limit);
    topwml = max(min(topwml_pre, pwm_limit), -pwm_limit);
    
    % Anti-Windup Logic: Only integrate if not pushing further into saturation
    if ~((topwmr_pre > pwm_limit && err_R > 0) || (topwmr_pre < -pwm_limit && err_R < 0))
        int_R = int_R + err_R * Ts_ref; 
    end
    
    if ~((topwml_pre > pwm_limit && err_L > 0) || (topwml_pre < -pwm_limit && err_L < 0))
        int_L = int_L + err_L * Ts_ref; 
    end
    
    %% 7. Outputs Mapping
    % Map absolute PWM to [0, 1] scale
    pwmr = abs(topwmr) / pwm_limit; 
    pwml = abs(topwml) / pwm_limit;
    
    % Direction: 1 if commanded voltage is negative, 0 otherwise
    dirr = topwmr < 0; 
    dirl = topwml < 0;

end