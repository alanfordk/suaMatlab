function y = gps( x, t, P)
%gps synthesizes GPS measurements
%
% Inputs:
%   x = state vector (mixed units)
%   t = simulation time (s)
%   P = simulation parameter structure (mixed units)
%
% Outputs
%   y = synthesized gps measurements (mixed units)
%
% Example Usage
% y = gps( x, t, P)

% Author: Randy Christensen
% Date: 19-Feb-2019 00:42:00
% Reference: Small Unmanned Aircraft, Beard, section 7.5 and appendix H
% Copyright 2018 Utah State University

% Persistent variables that define random walk of GPS sensors
persistent eta_n;
persistent eta_e;
persistent eta_h;

if t==0  % initialize persistent variables
    eta_n = 0;
    eta_e = 0;
    eta_h = 0;
else      % propagate persistent variables
    eta_n = exp(-P.beta_gps*P.Ts_gps)*eta_n + P.sig_n_gps*randn;
    eta_e = exp(-P.beta_gps*P.Ts_gps)*eta_e + P.sig_e_gps*randn;
    eta_h = exp(-P.beta_gps*P.Ts_gps)*eta_h + P.sig_h_gps*randn;
end

% Synthesize North, East, and altitude GPS measurements
y_gps_n = x(1) + eta_n;
y_gps_e = x(2) + eta_e;
y_gps_h = -x(3) + eta_h;
y_gps = [y_gps_n, y_gps_e, y_gps_h]';

% Synthesize groundspeed and course measurements
v_b = [x(4), x(5), x(6)]';
ypr = [x(9), x(8), x(7)]';
b2v = v2b(ypr)';
v_v = b2v*v_b;
V_g = norm(v_v(1:2));
chi = calc_chi(x);
eta_V = P.sig_Vg_gps * randn;
eta_chi = P.sig_course_gps * randn;
y_gps_V_g = V_g + eta_V;
y_gps_chi = chi + eta_chi;

% Bundle measurements
y = [y_gps; y_gps_V_g; y_gps_chi];
end
