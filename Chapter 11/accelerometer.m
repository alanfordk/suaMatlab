function y_accel = accelerometer( forces_moments, x, P )
%accelerometer synthesizes accelerometer measurements
%
% Inputs:
%   forces_moments = forces and moments in the body frame (mixed units)
%   x = state vector (mixed units)
%   P = simulation parameter structure (mixed units)
%
% Outputs
%   y_accel = synthesized accelerometer measurements (m/s^2)
%
% Example Usage
% y_accel = accelerometer( forces_moments, P )

% Author: Randy Christensen
% Date: 18-Feb-2019 23:22:06
% Reference: Small Unmanned Aircraft, Beard, section 7.1 and appendix H
% Copyright 2018 Utah State University

eta_accel = P.sig_accel * randn(3,1);
ypr = [x(9), x(8), x(7)]';
rvb = v2b(ypr);
f_b = forces_moments(1:3);
y_accel = 1/P.mass * f_b - rvb*[0, 0, P.gravity]' + eta_accel;
end
