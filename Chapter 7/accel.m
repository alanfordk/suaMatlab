function y_accel = accel(force_moment, x, P)
%ACCEL Summary of this function goes here
%   Detailed explanation goes here

% Unpack
f_b = force_moment(1:3);
ypr = x(9:-1:7);
T_v2b = v2b(ypr);
y_accel = 1/P.mass*f_b - T_v2b*[0,0,P.gravity]' + randn(3,1)*P.sigma_accel;

end

