function [v_cmd, w_cmd, reached] = robot_pipeline(currentPose, goalPose, stepIndex)
%#codegen

% 1. Use simple persistent arrays for NUMERIC data only.
% Do NOT store any 'map', 'validator', or 'planner' objects in persistent memory.
persistent refPath vd_ref wd_ref numPoints
    
if isempty(refPath)
    % Call a separate function to do the "heavy lifting"
    % This ensures the Map/Validator objects are local and temporary.
    [rawPath, v_refs, w_refs] = run_once_planner(currentPose, goalPose);
    
    refPath = rawPath;
    vd_ref  = v_refs;
    wd_ref  = w_refs;
    numPoints = size(rawPath, 1);
end

%% 2. RUNTIME TRACKING
reached = false;
idx = uint32(stepIndex);

if idx >= numPoints || idx < 1
    v_cmd = 0; w_cmd = 0; reached = true;
    return;
end

% Current targets
rx = refPath(idx,1); ry = refPath(idx,2); rt = refPath(idx,3);
rv = vd_ref(idx);    rw = wd_ref(idx);

% LQR Gains
K = [2.2361, 0, 0; 0, 2.8868, 2.5456];

% Local Error Transform
ex = rx - currentPose(1);
ey = ry - currentPose(2);
et = atan2(sin(rt - currentPose(3)), cos(rt - currentPose(3)));

c = cos(currentPose(3));
s = sin(currentPose(3));
e_local_x =  c * ex + s * ey;
e_local_y = -s * ex + c * ey;

% Control Law
v_cmd = rv + (K(1,1) * e_local_x);
w_cmd = rw + (K(2,2) * e_local_y + K(2,3) * et);

% Saturation
v_cmd = max(0, min(v_cmd, 0.3));
w_cmd = max(-0.25, min(w_cmd, 0.25));
end

%% --- HELPER FUNCTION: Only runs once ---
function [pth, v_ref, w_ref] = run_once_planner(start, goal)
    %#codegen
    % IMPORTANT: Use literal numbers for dimensions to lock the C-struct size
    m = binaryOccupancyMap(10, 10, 10); 
    
    % Obstacles
    [X1, Y1] = meshgrid(1.0:0.1:2.0, 1.0:0.1:3.8);
    setOccupancy(m, [X1(:) Y1(:)], 1);
    [X2, Y2] = meshgrid(1.5:0.1:3.2, 5.2:0.1:7.8);
    setOccupancy(m, [X2(:) Y2(:)], 1);

    % Planner setup
    ss = stateSpaceSE2;
    ss.StateBounds = [0 10; 0 10; -pi pi];
    sv = validatorOccupancyMap(ss);
    sv.Map = m; % Local assignment
    sv.ValidationDistance = 0.1;

    planner = plannerHybridAStar(sv, 'MinTurningRadius', 0.9);
    
    % Plan
    pathObj = plan(planner, start, goal);
    pth = pathObj.States;
    nP = size(pth, 1);
    
    % Velocity Profile
    Ts = 0.05;
    v_ref = zeros(nP, 1);
    w_ref = zeros(nP, 1);
    for k = 1:nP-1
        dist = sqrt(sum((pth(k+1,1:2)-pth(k,1:2)).^2));
        v_ref(k) = dist/Ts;
        dth = atan2(sin(pth(k+1,3)-pth(k,3)), cos(pth(k+1,3)-pth(k,3)));
        w_ref(k) = dth/Ts;
    end
end