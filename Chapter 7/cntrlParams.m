closedLoopEnable = 1;
%% Linearized System Models
% Set desired initial conditions to straight and level
Va = 35;       % desired airspeed
gamma = 0;     % desired flight path angle (radians)
R = inf;       % desired radius (m) - use (+) for right handed orbit, (-) for left handed orbit
P.Va0 = Va;
h0 = 500;
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
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
P.pd0    = -h0;% initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate
%% Saturations
phi_c_sat = 45*pi/180;
theta_c_sat = 20*pi/180;
aileron_c_sat = 45*pi/180;
elevator_c_sat = 45*pi/180;
throttle_c_sat = 1;
%% Take-off autopilot parameters
P.takeoff_altitude = 10;
P.takeoff_throttle = throttle_c_sat;
P.takeoff_pitch = theta_c_sat;
P.altitudeHoldZone = 10;
P.stateMachineEnable = true;
%% Controller gains
% Roll controller parameters
P.k_pphi = 0.750;
P.k_iphi = 0;
P.k_dphi = -0.00441;
% Course controller parameters
P.k_pchi = 7.94;
P.k_ichi = 1;
% Pitch controller parameters
P.k_ptheta = -2.25;
P.k_dtheta = -0.503;
% Altitude from pitch controller parameters
P.k_ph = 0.0141;
P.k_ih = 5.00e-04;
% Velocity from throttle controller parameters
P.k_pv = 0.100;
P.k_iv = 0.050;