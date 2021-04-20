closedLoopEnable = 1;
gains = matfile('gains.mat');
cltfs = matfile('cltfs.mat');
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
P.psi0   = 0*pi/180;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate
% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',Va,gamma,R,P.psi0);
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
P.k_pphi = gains.k_pphi;
P.k_iphi = gains.k_iphi;
P.k_dphi = gains.k_dphi;
% Course controller parameters
P.k_pchi = gains.k_pchi;
P.k_ichi = gains.k_ichi;
% Pitch controller parameters
P.k_ptheta = gains.k_ptheta;
P.k_dtheta = gains.k_dtheta;
% Altitude from pitch controller parameters
P.k_ph = gains.k_ph;
P.k_ih = gains.k_ih;
% Velocity from throttle controller parameters
P.k_pv = gains.k_pv;
P.k_iv = gains.k_iv;
%% CLTFs from controller designs
cltf_chic_2_chi = cltfs.cltf_chic_2_chi;
cltf_hc_2_h = cltfs.cltf_hc_2_h;
cltf_vc_2_v = cltfs.cltf_vc_2_v;