function [refPath, vd_ref, wd_ref] = run_planner(startPose, goalPose)
% This function is NEVER compiled. Run it once in MATLAB to get the path.

    MAP_DIM = 10;
    MAP_RES = 10;
    Ts      = 0.05;

    % Build map
    m = binaryOccupancyMap(MAP_DIM, MAP_DIM, MAP_RES);

    [X1, Y1] = meshgrid(1.0:0.1:2.0, 1.0:0.1:3.8);
    setOccupancy(m, [X1(:) Y1(:)], 1);

    [X2, Y2] = meshgrid(1.5:0.1:3.2, 5.2:0.1:7.8);
    setOccupancy(m, [X2(:) Y2(:)], 1);

    % Build planner
    ss = stateSpaceSE2;
    ss.StateBounds = [0 MAP_DIM; 0 MAP_DIM; -pi pi];

    sv = validatorOccupancyMap(ss);
    sv.Map = m;
    sv.ValidationDistance = 0.1;

    planner = plannerHybridAStar(sv, ...
        'MinTurningRadius',      0.9, ...
        'MotionPrimitiveLength', 0.8);
    planner.ReverseCost = 1e10;

    % Plan and extract numeric data only
    pathObj  = plan(planner, startPose, goalPose);
    refPath  = pathObj.States;           % [N x 3] double
    N        = size(refPath, 1);

    vd_ref = zeros(N, 1);
    wd_ref = zeros(N, 1);

    for k = 1:N-1
        dx        = refPath(k+1,1) - refPath(k,1);
        dy        = refPath(k+1,2) - refPath(k,2);
        vd_ref(k) = sqrt(dx^2 + dy^2) / Ts;

        dth        = refPath(k+1,3) - refPath(k,3);
        wd_ref(k)  = atan2(sin(dth), cos(dth)) / Ts;
    end
end