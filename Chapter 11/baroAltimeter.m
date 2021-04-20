function y_abspress = baroAltimeter( x, P)
%baroAltimeter synthesizes measurement from a barometric altimeter (i.e.
%absolute pressure sensor)
%
% Inputs:
%   x = state vector (mixed units)
%   P = simulation parameter structure (mixed units)
%
% Outputs
%   y_abspress = absolute pressure relative to ground (Pa)
%
% Example Usage
% y_abspress = baroAltimeter( x, P)

% Author: Randy Christensen
% Date: 18-Feb-2019 23:58:17
% Reference: Small Unmanned Aircraft, Beard, section 7.3.1 and appendix H
% Copyright 2018 Utah State University

h_agl = -x(3);
eta_abspres = P.sig_abspress*randn;
y_abspress = P.rho*P.gravity*h_agl + eta_abspres;
end
