function y_gyro = gyro( x, P )
%gyro synthesizes gyro measurements
%
% Inputs:
%   x = state vector (mixed units)
%   P = simulation parameter structure (mixed units)
%
% Outputs
%   y_gyro = synthesized gyro measurements (rad/s)
%
% Example Usage
% y_gyro = gyro( x, P )

% Author: Randy Christensen
% Date: 18-Feb-2019 23:46:41
% Reference: Small Unmanned Aircraft, Beard, section 7.2 and appendix H
% Copyright 2018 Utah State University

p = x(10);
q = x(11);
r = x(12);
eta_gyro = P.sig_gyro * randn(3,1);
y_gyro = [p, q, r]' + eta_gyro;
end
