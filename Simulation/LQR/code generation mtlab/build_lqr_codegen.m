function build_lqr_codegen()

cfg = coder.config('lib');
cfg.TargetLang = 'C';

% Variable-size reference arrays
P_ex  = coder.typeof(0, [5000 2], [1 0]);
th_ex = coder.typeof(0, [5000 1], [1 0]);
vd_ex = coder.typeof(0, [5000 1], [1 0]);
wd_ex = coder.typeof(0, [5000 1], [1 0]);

x_ex = 0;
y_ex = 0;
theta_ex = 0;
K_ex = zeros(2,3);
goalPose_ex = zeros(1,3);
goalPosTol_ex = 0;
goalThetaTol_ex = 0;

ARGS = { ...
    x_ex, y_ex, theta_ex, ...
    P_ex, th_ex, vd_ex, wd_ex, ...
    K_ex, goalPose_ex, goalPosTol_ex, goalThetaTol_ex};

codegen lqr_tracker_step_codegen -config cfg -args ARGS -report

end