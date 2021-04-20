clearvars
close all
clc
%% Prelims
P.gravity = 9.8; % gravity (m/s^2)
P.Ts = 0.01;     % autopilot sample rate
%% Params for Aersonade UAV
aerosonde
%% Wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06*0;
P.sigma_v = 1.06*0;
P.sigma_w = .7*0;
%% Disable excitation flags for trim flight
elevator_impulse_enable = 0;
aileron_doublet_enable = 0;
rudder_doublet_enable = 0;
%% Trim Plots For Straight and Level Flight
% Desired initial conditions
Va = 17;     % desired airspeed
gamma = 0;   % desired flight path angle (radians)
R     = inf; % desired radius (m) - use (+) for right handed orbit, (-) for left handed orbit
% autopilot sample rate
P.Ts = 0.01;
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;
% set initial conditions to trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate
tsim = 10;
sim('mavsim_chap5',tsim);
generatePlots;
%% Trim Plots For Straight and Climbing Flight
% Desired initial conditions
Va = 17;                % desired airspeed
gamma = 5*pi/180;       % desired flight path angle (radians)
R     = inf;        % desired radius (m) - use (+) for right handed orbit, (-) for left handed orbit
% autopilot sample rate
P.Ts = 0.01;
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;
% set initial conditions to trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate
tsim = 10;
sim('mavsim_chap5',tsim);
generatePlots;
%% Trim Plots For Level Orbit Flight
% Desired initial conditions
Va = 17;                % desired airspeed
gamma = 0*pi/180;       % desired flight path angle (radians)
R     = 150;        % desired radius (m) - use (+) for right handed orbit, (-) for left handed orbit
% autopilot sample rate
P.Ts = 0.01;
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;
% set initial conditions to trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

tsim = (Va/R)^-1*270*pi/180;
sim('mavsim_chap5',tsim);
generatePlots;
%% Trim Plots For Climbing Orbit Flight
% Desired initial conditions
Va = 17;                % desired airspeed
gamma = 5*pi/180;       % desired flight path angle (radians)
R     = 150;        % desired radius (m) - use (+) for right handed orbit, (-) for left handed orbit
% autopilot sample rate
P.Ts = 0.01;
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;
% set initial conditions to trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

tsim = (Va/R)^-1*360*pi/180;
sim('mavsim_chap5',tsim);
generatePlots;
%% Linearized System Models
% Set desired initial conditions to straight and level
Va = 17;       % desired airspeed
gamma = 0;     % desired flight path angle (radians)
R = inf;       % desired radius (m) - use (+) for right handed orbit, (-) for left handed orbit
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = Va; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',Va,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;
% set initial conditions to trim conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -500;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% Compute system transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,...
    T_Va_delta_t,T_Va_theta,T_beta_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

figure;
bode(T_phi_delta_a, T_chi_phi, T_beta_delta_r); legend;
title('Lateral Transfer Functions');

figure;
bode(T_theta_delta_e, T_h_theta, T_h_Va); legend;
title('Longitudinal Transfer Functions Part 1')

figure;
bode(T_Va_delta_t, T_Va_theta); legend;
title('Longitudinal Transfer Functions Part 2')

% Linearize the equations of motion about the trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,...
    u_trim);

% Compute eigen values of the linearized system
eigen_values_lon = eig(A_lon);
eigen_values_lat = eig(A_lat);

% Compute natural frequencies and damping ratios of system modes.  Note
% that the computed natural frequencies and damping ratios are only
% meaningful for complex conjugate eigen values.
wn_lon = abs(eigen_values_lon);          %natural frequencies (rad/s)
fn_lon = wn_lon/2/pi;                    %natural frequencies (Hz)
zeta_lon = -cos(angle(eigen_values_lon));%damping ratio
fd_lon = fn_lon.*sqrt(1-zeta_lon.^2);    %damped natural frequency (Hz)
Td_lon = 1./fd_lon;                      %damped period (s)

wn_lat = abs(eigen_values_lat);			%natural frequencies (rad/s)
fn_lat = wn_lat/2/pi;                    %natural frequencies (Hz)
zeta_lat = -cos(angle(eigen_values_lat));%damping ratio
fd_lat = fn_lat.*sqrt(1-zeta_lat.^2);    %damped natural frequency (Hz)
Td_lat = 1./fd_lat;                      %damped period (s)
%% Excite the phugoid and short-period modes with an elevator impulse
elevator_impulse_enable = 1;
aileron_doublet_enable = 0;
rudder_doublet_enable = 0;
tsim = 60;
sim('mavsim_chap5',tsim);
generatePlots;
%% Excite the roll, dutch-roll, and spiral divergence modes with an aileron doublet.
elevator_impulse_enable = 0;
aileron_doublet_enable = 1;
rudder_doublet_enable = 0;
tsim = 60;
sim('mavsim_chap5',tsim);
generatePlots;
%% Excite the roll, dutch-roll, and spiral divergence modes with a rudder doublet.
elevator_impulse_enable = 0;
aileron_doublet_enable = 0;
rudder_doublet_enable = 1;
tsim = 60;
sim('mavsim_chap5',tsim);
generatePlots;