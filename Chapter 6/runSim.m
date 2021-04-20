clearvars
close all
clc
%% Prelims
P.gravity = 9.8; % gravity (m/s^2)
P.Ts = 0.005;     % autopilot sample rate
tsim = 40;
s = tf('s');
closedLoopEnable = 1;
model = 'mavsim_chap6_week1';
phi_c_sat = inf;
aileron_c_sat = inf;
%% Params for Aersonade UAV
aerosonde
%% Wind parameters
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 0;
P.sigma_v = 0;
P.sigma_w = 0;
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
% Compute system transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,...
    T_Va_delta_t,T_Va_theta,T_beta_delta_r, lat, lon]...
    = compute_tf_model(x_trim,u_trim,P);
%% Design the Lateral Autopilot
% The following sections of code will design the controllers inside the 
% lateral autopilot.  Each controller design must have a phase margin of 
% > 60 degrees and a gain margin of >12 dB.  Make sure you change the
% default values of bode plots to Hz by calling "ctrlpref" and changing the
% appropriate settings.
%% Design the roll controller
% Design the roll controller using the procedure demonstrated in
% lateralControlDesignExample.m.  For the design of the roll controller, 
% you can assume the following:
% - The largest roll angle command you will issue is 60 degrees.
% - The aileron saturates at 45 degrees.
% After you are finished with your controller design, verify that you meet 
% the phase margin and gain margin requirements by create bode plots of the
% OLTF. Analyze the closed-loop performance of the system by create bode 
% plots of the CLTF and a step response plot.
close all
%% Choose k_{p_{\phi}} based on saturation of the ailerons and roll error
e_phi_max = 60*pi/180;   % Maximum roll error    60*pi/180
delta_a_max = 45*pi/180; % Max aileron angle    45*pi/180
k_pphi = delta_a_max/e_phi_max*sign(lat.a_phi2)*db2mag(-3);  %    delta_a_max/e_phi_max*sign(lat.a_phi2)
%% Calculate the natural frequency \omega_{n_{\phi}}of the roll control loop 
omega_nphi = sqrt(abs(lat.a_phi2)*delta_a_max/e_phi_max); %Roll bandwidth    sqrt(abs(lat.a_phi2)*delta_a_max/e_phi_max)
%% Choose the damping ratio \zeta_{\phi} of the roll control loop
zeta_phi = .8;          % Roll damping ratio    .8
%% Calculate k_{d_{\phi}}
k_dphi = (2*zeta_phi*omega_nphi - lat.a_phi1)/lat.a_phi2;      % (2*zeta_phi*omega_nphi - lat.a_phi1)/lat.a_phi2
k_iphi = 0;     % .5*sign(lat.a_phi2)*0

% Verify gain margins are > 60
g_deltaa_2_p = lat.a_phi2/(s+lat.a_phi1);
g_deltaaprime_2_phi = feedback(g_deltaa_2_p, k_dphi) * 1/s;
c_phi = k_iphi/s + k_pphi;

%% Verify that PM > 60 degrees and gain margin > 12 dB
OLTF_phi = c_phi * g_deltaaprime_2_phi;
figure; bode(OLTF_phi); grid on;
[gm, pm] = margin(OLTF_phi);
title(sprintf('Roll OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
        pm, 20*log10(gm)));
%% Analyze the closed loop response T_{\phi}
CLTF_phi_phi_c = feedback(OLTF_phi,1);
[gpeak, fpeak] = getPeakGain(CLTF_phi_phi_c);
figure; bode(CLTF_phi_phi_c); grid on;
title(sprintf('Roll CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
    bandwidth(CLTF_phi_phi_c)/2/pi, gpeak, fpeak/2/pi));

figure; step(CLTF_phi_phi_c);
sinfo = stepinfo(CLTF_phi_phi_c);
title(sprintf('Roll Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
    sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
%% Design the course hold controller
% Design the course controller to acheive 0 steady-state error to a
% constant commaned angle \chi_c. After you are finished with your 
% controller design, verify that you meet the phase margin and gain margin 
% requirements by create bode plots of the OLTF. Analyze the closed-loop 
% performance of the system by create bode plots of the CLTF and a step 
% response plot.
close all
%% Calculate the plant G_{\chi} for the course angle controller
v_g = Va;
G_chi = CLTF_phi_phi_c * P.gravity/v_g/s;
%% Temporarily set the course angle controller gains k_{p_{\chi}}=1 and k_{i_{\chi}}=0
k_pchi = 1; %db2mag(18);
k_ichi = 0;
c_chi = k_ichi/s + k_pchi;
%% Calculate the OLTF G_{\chi}H_{\chi}
OLTF_chi = G_chi * c_chi;
[gm, pm] = margin(OLTF_chi);
figure;
bode(OLTF_chi);
title(sprintf('Course OLTF\nPhase Margin = %.2f deg, nGain Margin = %.2f dB',...
    pm, 20*log10(gm)))
hold all;
grid on;
%% Tune the controller gains k_{p_{\chi}} and k_{i_{\chi}}
% Tune the value of k_{p_{\chi}} such that the phase margin is near and 
% within spec. Tune the value of k_{i_{\chi}} until the phase margin 
% starts to drop
k_pchi = db2mag(20);
k_ichi = 2;
c_chi = k_ichi/s + k_pchi;
OLTF_chi = G_chi * c_chi;
bode(OLTF_chi);
[gm, pm] = margin(OLTF_chi);
title(sprintf('Course OLTF\nPhase Margin = %.2f deg, nGain Margin = %.2f dB',...
    pm, 20*log10(gm)))
hold all;
grid on;
% Calculate the CLTF T_{\chi} and analyze the performance
CLTF_chi_chi_c = feedback(OLTF_chi,1);
[gpeak, fpeak] = getPeakGain(CLTF_chi_chi_c);
figure; bode(CLTF_chi_chi_c); grid on;
title(sprintf('Course CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
    gpeak, gpeak))
figure; step(CLTF_chi_chi_c);
sinfo = stepinfo(CLTF_chi_chi_c);
title(sprintf('Course Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
    sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
%% Implement the Roll and Course controllers
% At this point, you need to implement the controllers in your Simulink
% model.  It is recommended that you use Matlab's built-in PID Controller 
% block.
%% Define commanded values
% Set the inputs to zeros so that the model will build
chi_c_in = timeseries([0,0],[0,tsim]);
beta_c_in = timeseries(0,0);
h_c_in = timeseries(0,0);
V_a_c_in = timeseries(0,0);
%% Package controller parameters
% Roll controller parameters
P.k_pphi = k_pphi;
P.k_dphi = k_dphi;
% Course controller parameters
P.k_pchi = k_pchi;
P.k_ichi = k_ichi;
P.k_iphi = k_iphi;
%% Controller verification
% You designed the controllers using the approximate transfer functions from
% section 5.4.  Any nonlinear or cross-coupling terms were considered
% "disturbances".  Although the state space models are a linear
% approximation, they retain the cross-couping terms.  In this section, you
% will extract OLTFs from your Simulink model to verify adequate stability
% margins.  If the margins aren't sufficient, you must modify the
% controller gains to regain the necessary margins.  You can do this by
% naming the traces of the inputs and outputs of the transfer functions of 
% interest inside the sim, then
% making them "linear analysis point -> open-loop input/output".
% Hints: Use the "getlinio" command to determine the indices.  Use the
% "operpoint" command to define the trim condition about which to linearize.
% Use the "linearize" command to retrieve the OLTFs for the Roll and Course
% loops.  Create bode plots for each axis and verify sufficient gain margin
% and phase margin.
close all
io = getlinio(model);                   % Retrieve the analysis I/Os
op = operpoint(model);                  % Retrieve the operating point
% Check the roll control design
i_phi_c = 1;
i_phi = 4;
OLTF_phi_sim = linearize(model,io([i_phi_c,i_phi]),op); % Linearize the model
figure; bode(OLTF_phi, OLTF_phi_sim); grid on;
[gm, pm] = margin(OLTF_phi_sim);
title(sprintf('Roll OLTF Verification\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
        pm, 20*log10(gm)));
legend('design','sim'); xlim([0.01,100])
% Check the course angle control design
i_chi_c = 2;
i_chi = 3;
OLTF_chi_sim = linearize(model,io([i_chi_c, i_chi]),op); % Linearize the model
figure; bode(OLTF_chi, OLTF_chi_sim); grid on;
[gm, pm] = margin(OLTF_chi_sim);
title(sprintf('Course OLTF Verification\nPhase Margin = %.2f deg, nGain Margin = %.2f dB',...
    pm, 20*log10(gm)))
legend('design','sim'); xlim([0.01,100])
%% Controller Testing
% You are now ready to test the controller.  Define the course angle input 
% "chi_c_in" such that it is zero for 10 seconds, steps to 2 degrees and
% holds for 10 seconds, then ramps from 2 degrees to 180 degrees over 20
% seconds.  Make the units of the plot in degrees, but remember that the
% input to the simulation is in radians!
close all
chi_c_in = timeseries([0,0,2,2,180]*pi/180,[0,10,10,20,40]);
figure; plot(chi_c_in*180/pi); grid on
sim(model,tsim);
generatePlots
%% Bells and Whistles
% Now that the lateral autopilot is working correctly, you need to
% implement saturations so that the UAV response is more realistic.
% Saturate the ailerons to the value used in the design of the Roll
% controller.  To avoid asking too much of the Roll controller, saturate
% the output of the Course controller +/-45 degrees, using the "Clamping"
% method.  Modify the commanded course command you used in the previous
% step such that the commanded roll angle is saturated.
close all
phi_c_sat = 45*pi/180;
aileron_c_sat = delta_a_max;
chi_c_in = timeseries([0,0,90,90,180]*pi/180,[0,10,10,20,40]);
figure; plot(chi_c_in*180/pi); grid on
sim(model,tsim);
generatePlots