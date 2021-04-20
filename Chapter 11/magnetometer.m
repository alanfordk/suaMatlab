function y_mag = magnetometer( x, P )
%magnetometer synthesizes magnetometer measurements
%
% Inputs:
%   x = state vector (mixed units)
%   P = simulation parameter structure (mixed units)
%
% Outputs
%   y_mag = description (units)
%   Output2 = description (units)
%
% Example Usage
% y_mag = magnetometer( x, P )

% Author: Randy Christensen
% Date: 19-Feb-2019 00:33:19
% Reference: Small Unmanned Aircraft, Beard, section 7.4 and appendix H
% Copyright 2018 Utah State University

beta_mag = -P.declination;
eta_mag = P.sig_mag * randn;
psi = x(9);
y_mag = psi + beta_mag + eta_mag;
end
