function [RPWM_R, LPWM_R, RPWM_L, LPWM_L] = StabilityxSTM(state_x)
    %#codegen
    % Real-time LQR controller for Wheel-Biped Robot on STM32
    %
    % Inputs:
    %   state_x  : 6x1 state vector [phi; s; theta; phi_dot; v; omega]
    % Outputs:
    %   RPWM_R, LPWM_R : Right motor PWM signals (0 to 1000)
    %   RPWM_L, LPWM_L : Left motor PWM signals (0 to 1000)
    
    % 1. System Constants
    V_batt = 24.0;
    max_pwm = 1000;
    
    % Define the pre-computed K matrix using standard MATLAB syntax.
    % Simulink Coder will automatically convert this into a constant C array.
    K_matrix = [-12.4395, -0.7071,  0.7071, -2.9489, -1.3601,  0.7289;
                -12.4395, -0.7071, -0.7071, -2.9489, -1.3601, -0.7289];

    % 2. Calculate Control Effort (u = -K * x)
    % The output u will be a 2x1 vector [tau_r; tau_l]. 
    % We treat these commanded torques as commanded voltages (v_r, v_l) 
    % for the motor driver mapping.
    u = -K_matrix * state_x;
    
    v_r = u(1);
    v_l = u(2);
    
    % 3. Convert to PWM and Clamp
    pwm_r = round((v_r / V_batt) * max_pwm);
    pwm_l = round((v_l / V_batt) * max_pwm);
    
    % Clamping logic
    pwm_r = max(min(pwm_r, max_pwm), -max_pwm);
    pwm_l = max(min(pwm_l, max_pwm), -max_pwm);
    
    % 4. Directional Splitting for Right Motor
    if pwm_r >= 0
        RPWM_R = pwm_r;
        LPWM_R = 0;
    else
        RPWM_R = 0;
        LPWM_R = abs(pwm_r); 
    end
    
    % 5. Directional Splitting for Left Motor
    if pwm_l >= 0
        RPWM_L = pwm_l;
        LPWM_L = 0;
    else
        RPWM_L = 0;
        LPWM_L = abs(pwm_l); 
    end
end