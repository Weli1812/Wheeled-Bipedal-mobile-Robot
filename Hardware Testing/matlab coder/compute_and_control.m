function [pwm_R, pwm_L, K] = compute_and_control(state, L, Iy, Iz, Mw, r, d, Iw, Mb, Q_diag, R_diag, Kt, V_batt)
%#codegen

    g = 9.81;

    %% --- 1. Physics & State-Space ---
    m11 = Mb * L^2 + Iy;
    m12 = Mb * L;
    m22 = Mb + 2*Mw + 2*(Iw / r^2);
    m33 = (d^2/2)*Mw + (d^2/(2*r^2))*Iw + Iz;
    det_sag = m11*m22 - m12^2;
    
    A = zeros(6,6);
    A(1,4) = 1; A(2,5) = 1; A(3,6) = 1;
    A(4,1) = (Mb*g*L * m22) / det_sag;
    A(5,1) = (-Mb*g*L * m12) / det_sag;
    
    B = zeros(6,2);
    B(4,1) = -m12/(det_sag*r); B(4,2) = B(4,1);
    B(5,1) = m11/(det_sag*r);  B(5,2) = B(5,1);
    B(6,1) = d/(2*r*m33);      B(6,2) = -B(6,1);

    %% --- 2. Solve LQR ---
    H = [A, -(B*(diag(R_diag)\B')); -diag(Q_diag), -A'];
    [V, D] = eig(H);
    d_eig = diag(D);
    V_stable = complex(zeros(12, 6));
    count = 1;
    for i = 1:12
        if real(d_eig(i)) < 0 && count <= 6
            V_stable(:, count) = V(:, i);
            count = count + 1;
        end
    end
    % K = inv(R) * B' * P
    K = real(diag(R_diag) \ (B' * real(V_stable(7:12, :) / V_stable(1:6, :))));

    %% --- 3. Compute PWM ---
    % Control Law
    torque = -K * state;
    
    % Torque to PWM (0.0 to 1.0 scale)
    % PWM = (Torque / Kt) / V_batt
    % We multiply by a small safety factor if your driver has dead-zones
    raw_pwm_R = torque(1) / (Kt * V_batt);
    raw_pwm_L = torque(2) / (Kt * V_batt);

    % Saturation for 24V safety
    pwm_R = max(-1.0, min(1.0, raw_pwm_R));
    pwm_L = max(-1.0, min(1.0, raw_pwm_L));
end