s=5; %% Distance between wheels
% waypoints: [x y]
P = [...
    0   0
    0.5 0
    0.5 0.5
    1   1
    1.5 1.5
    2   1.5
    2.5 1.5
    2.5 2
    3 2
    3 2.5
    3 3];

ds = 0.1;  % desired spacing between points (change to 0.05 if you want denser)

% Distance along the polyline
d = [0; cumsum(sqrt(sum(diff(P).^2,2)))];

% New equally-spaced samples
d_new = (0:ds:d(end)).';

% Interpolate to get densified waypoints
x_new = interp1(d, P(:,1), d_new, 'linear');
y_new = interp1(d, P(:,2), d_new, 'linear');

P_dense = [x_new y_new];