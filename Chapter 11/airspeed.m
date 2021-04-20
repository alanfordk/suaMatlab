function y_diffpres = airspeed( airdata, P )
%airspeed synthesizes airspeed measurements
%
% Inputs:
%   airdata = airspeed (m/s)
%   P = simulation parameter structure (mixed units)
%
% Outputs
%   y_diffpres = differential pressure measurement (Pa)
%
% Example Usage
% y_diffpres = airspeed( airdata, P )

% Author: Randy Christensen
% Date: 19-Feb-2019 00:20:32
% Reference: Small Unmanned Aircraft, Beard, section 7.3.2 and appendix H
% Copyright 2018 Utah State University

Va = airdata(1);
eta_diff_pres = P.sig_diffpress * randn;
y_diffpres = P.rho*Va^2/2 + eta_diff_pres;
end
