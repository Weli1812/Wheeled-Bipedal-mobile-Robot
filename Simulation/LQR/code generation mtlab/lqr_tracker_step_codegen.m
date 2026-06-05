function [v_cmd, w_cmd, done, idx] = lqr_tracker_step_codegen( ...
    x, y, theta, ...
    P, theta_ref, vd_ref, wd_ref, ...
    K, goalPose, goalPosTol, goalThetaTol) %#codegen

% P            Nx2
% theta_ref    Nx1
% vd_ref       Nx1
% wd_ref       Nx1
% K            2x3
% goalPose     1x3   [xg yg thetag]

assert(size(P,2) == 2);
N = size(P,1);

assert(length(theta_ref) == N);
assert(length(vd_ref)    == N);
assert(length(wd_ref)    == N);
assert(all(size(K) == [2 3]));
assert(all(size(goalPose) == [1 3]));

% -------------------------------------------------
% 1) Find nearest reference point
% -------------------------------------------------
bestD2 = inf;
idx = int32(1);

for i = 1:N
    dxi = P(i,1) - x;
    dyi = P(i,2) - y;
    d2  = dxi*dxi + dyi*dyi;

    if d2 < bestD2
        bestD2 = d2;
        idx = int32(i);
    end
end

% -------------------------------------------------
% 2) Get reference at nearest point
% -------------------------------------------------
xr = P(idx,1);
yr = P(idx,2);
tr = theta_ref(idx);
vr = vd_ref(idx);
wr = wd_ref(idx);

% -------------------------------------------------
% 3) Tracking error in reference frame
% -------------------------------------------------
dx = xr - x;
dy = yr - y;

c = cos(tr);
s = sin(tr);

ex =  c*dx + s*dy;
ey = -s*dx + c*dy;
etheta = atan2(sin(tr - theta), cos(tr - theta));

e = [ex; ey; etheta];

% -------------------------------------------------
% 4) LQR correction
% -------------------------------------------------
u = -K * e;

v_cmd = vr + u(1);
w_cmd = wr + u(2);

% -------------------------------------------------
% 5) Saturation
% -------------------------------------------------
v_max = 0.30;
w_max = 0.25;

if v_cmd < 0
    v_cmd = 0;
elseif v_cmd > v_max
    v_cmd = v_max;
end

if w_cmd > w_max
    w_cmd = w_max;
elseif w_cmd < -w_max
    w_cmd = -w_max;
end

% -------------------------------------------------
% 6) Goal stop
% -------------------------------------------------
dxg = goalPose(1) - x;
dyg = goalPose(2) - y;
posErr = sqrt(dxg*dxg + dyg*dyg);

thetaErrGoal = atan2(sin(goalPose(3) - theta), cos(goalPose(3) - theta));

done = (posErr <= goalPosTol) && (abs(thetaErrGoal) <= goalThetaTol);

if done
    v_cmd = 0;
    w_cmd = 0;
end

end