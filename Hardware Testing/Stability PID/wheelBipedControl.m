function [RPWM_right, LPWM_right, RPWM_left, LPWM_left] = wheelBipedControl(x_aug)
% WHEELBIPEDCONTROL Calculates BTS7960 PWM duty cycles using direct torque mapping.
%
% Inputs:
%   x_aug - 5x1 vector of augmented states: [phi; s; phi_dot; v; int_phi]
%           phi     : Tilt angle (rad)
%           s       : Position (m)
%           phi_dot : Tilt angular velocity (rad/s)
%           v       : Linear velocity (m/s)
%           int_phi : Integrated tilt error (rad*s)
%
% Outputs:
%   RPWM_right,   LPWM_right - PWM signals (0-4000) for the Right Motor Driver
%   RPWM_left,  LPWM_left  - PWM signals (0-4000) for the Left Motor Driver

    %% 1. Robot Parameters
    Mw = 0.150; r = 0.060; d = 0.450; Iw = 2.757224e-4;
    Mb = 10.0;  g = 9.81;
    L  = 0.170; Iy = 0.1451; 

    % Hardware Constraints
    MAX_TORQUE = 3.0;    % Maximum motor torque (Nm)
    MAX_PWM    = 4000;   % Maximum PWM duty cycle value

    %% 2. Mass Matrix & State-Space Terms
    m11 = Mb*L^2 + Iy;
    m12 = Mb*L;
    m22 = Mb + 2*Mw + 2*Iw/r^2;
    det_sag = m11*m22 - m12^2;

    a41 =  Mb*g*L*m22/det_sag;
    a51 = -Mb*g*L*m12/det_sag;
    b41 = -m12/(det_sag*r);
    b51 =  m11/(det_sag*r);

    A = [0,   0,   1,   0; 
         0,   0,   0,   1; 
         a41, 0,   0,   0; 
         a51, 0,   0,   0];
         
    B = [0; 0; 2*b41; 2*b51]; % Single torque applied to both wheels

    %% 3. PID Setup (Augmented State-Space)
    A_aug = [A, zeros(4,1); 
             1, 0, 0, 0, 0];
    B_aug = [B; 0];

    % Tuned Control Gains
    Kp_tilt = -120;  Ki_tilt = -40;  Kd_tilt = -25;   
    Kp_pos  = -15;                   Kd_pos  = -15;     

    K_pid = [Kp_tilt, Kp_pos, Kd_tilt, Kd_pos, Ki_tilt];

    %% 4. Calculate Control Action (Torque)
    % u = -K * x
    tau_total = -K_pid * x_aug;

    % Distribute total torque equally to both wheels
    tau_wheel = tau_total / 2;

    % Saturation limit based on maximum torque capability
    if tau_wheel > MAX_TORQUE
        tau_wheel = MAX_TORQUE;
    elseif tau_wheel < -MAX_TORQUE
        tau_wheel = -MAX_TORQUE;
    end

    %% 5. Direct Torque to 12-bit/Timer-Based PWM Mapping (0-4000)
    % Linearly map the magnitude of the torque relative to its maximum limit
    pwm_value = round((abs(tau_wheel) / MAX_TORQUE) * MAX_PWM);

    %% 6. Generate H-Bridge Directional Signals for BTS7960
    % For positive torque: Drive Forward (RPWM active, LPWM = 0)
    % For negative torque: Drive Backward (LPWM active, RPWM = 0)
    if tau_wheel >= 0
        RPWM_right = pwm_value;
        LPWM_right = 0;
        RPWM_left  = pwm_value; 
        LPWM_left  = 0;
    else
        RPWM_right = 0;
        LPWM_right = pwm_value;
        RPWM_left  = 0;
        LPWM_left  = pwm_value;
    end
end