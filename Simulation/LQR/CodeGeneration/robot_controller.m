function [v_cmd, w_cmd, reached] = robot_controller(currentPose, ...
                                                      refPath,     ...
                                                      vd_ref,      ...
                                                      wd_ref,      ...
                                                      stepIndex)
%#codegen
%
% Inputs:
%   currentPose  - [1x3] double: [x, y, theta]
%   refPath      - [Nx3] double: path from run_planner
%   vd_ref       - [Nx1] double: reference linear velocities
%   wd_ref       - [Nx1] double: reference angular velocities
%   stepIndex    - scalar double: current step (1-based)
%
% Outputs:
%   v_cmd   - linear  velocity command
%   w_cmd   - angular velocity command
%   reached - true when goal is reached

    % --- Declare sizes for codegen (CRITICAL) ---------------------------
    % Tell coder the upper bound on path length. Adjust MAX_N as needed.
    MAX_N   = 2000;
    refPath = coder.nullcopy(zeros(MAX_N, 3));  %#ok — size hint only
    % --------------------------------------------------------------------

    numPoints = size(refPath, 1);
    reached   = false;
    idx       = int32(stepIndex);

    % Goal / bounds check
    if idx >= int32(numPoints) || idx < int32(1)
        v_cmd = 0.0;  w_cmd = 0.0;  reached = true;
        return;
    end

    % Reference state at current step
    rx = refPath(idx, 1);
    ry = refPath(idx, 2);
    rt = refPath(idx, 3);
    rv = vd_ref(idx);
    rw = wd_ref(idx);

    % LQR gains
    K11 =  2.2361;
    K22 =  2.8868;
    K23 =  2.5456;

    % Pose errors (global frame)
    ex = rx - currentPose(1);
    ey = ry - currentPose(2);
    et = atan2(sin(rt - currentPose(3)), ...
               cos(rt - currentPose(3)));   % wrapped

    % Rotate errors to robot-local frame
    c =  cos(currentPose(3));
    s =  sin(currentPose(3));
    e_local_x =  c*ex + s*ey;
    e_local_y = -s*ex + c*ey;

    % Control law
    v_cmd = rv + K11 * e_local_x;
    w_cmd = rw + K22 * e_local_y + K23 * et;

    % Saturation
    v_cmd = max(0.0,  min(0.3,  v_cmd));
    w_cmd = max(-0.25, min(0.25, w_cmd));
end