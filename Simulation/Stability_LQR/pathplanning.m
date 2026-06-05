function K = compute_wip_lqr(Q, R, L, Iy, Iz)
    %#codegen
    % COMPUTE_WIP_LQR Computes the LQR gain for a transforming wheel-biped robot.
    % This function is optimized for C++ code generation.
    %
    % Inputs:
    %   Q  - 6x6 State weight matrix
    %   R  - 2x2 Input weight matrix
    %   L  - CoM distance from wheel axis [m] (varies with height)
    %   Iy - Moment of inertia about Y axis [kg.m^2] (varies with height)
    %   Iz - Moment of inertia about Z axis [kg.m^2] (varies with height)
    %
    % Output:
    %   K  - 2x6 LQR Feedback Gain Matrix [tau_r; tau_l]

    % --- 1. Fixed Physical Parameters ---
    Mw  = 0.150;                    % Wheel mass [kg]
    r   = 0.060;                    % Wheel radius [m]
    d   = 0.450;                    % Wheel-to-wheel distance [m]
    Iw  = 2.757224e-4;              % Wheel moment of inertia [kg.m^2]
    Mb  = 10.0;                     % Upper body lumped mass [kg]
    g   = 9.81;                     % Gravity [m/s^2]

    % --- 2. Mass Matrix Components ---
    m11 = Mb * L^2 + Iy;
    m12 = Mb * L;
    m22 = Mb + 2*Mw + 2*(Iw / r^2);
    m33 = (d^2 / 2)*Mw + (d^2 / (2*r^2))*Iw + Iz;

    det_sag = m11*m22 - m12^2;

    % --- 3. State-Space Matrix Construction ---
    % Pre-allocate arrays to fixed sizes (required for efficient C++ gen)
    A = zeros(6, 6);
    B = zeros(6, 2);

    % Gravity-driven accelerations
    a41 =  (Mb * g * L * m22) / det_sag;
    a51 = -(Mb * g * L * m12) / det_sag;

    % Input matrix entries
    b41 = -m12 / (det_sag * r);
    b51 =  m11 / (det_sag * r);
    b61 =  d   / (2 * r * m33);

    % Populate A matrix (State: phi, s, theta, phi_dot, v, omega)
    A(1, 4) = 1.0;
    A(2, 5) = 1.0;
    A(3, 6) = 1.0;
    A(4, 1) = a41;
    A(5, 1) = a51;

    % Populate B matrix (Input: tau_r, tau_l)
    B(4, 1) = b41;   B(4, 2) = b41;
    B(5, 1) = b51;   B(5, 2) = b51;
    B(6, 1) = b61;   B(6, 2) = -b61;

    % --- 4. LQR Gain Computation ---
    % Compute the optimal gain matrix
    [K, ~, ~] = lqr(A, B, Q, R);
end