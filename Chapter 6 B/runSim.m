clearvars
close all
clc
%% Prelims
P.gravity = 9.8; % gravity (m/s^2)
P.Ts = 0.005;     % autopilot sample rate
tsim = 40;
s = tf('s');
closedLoopEnable = 1;
model = 'mavsim_chap6_week2';
phi_c_sat = inf;
aileron_c_sat = inf;
theta_c_sat = inf;
elevator_c_sat = inf;
throttle_c_sat = inf;
P.takeoff_altitude = 0;
P.takeoff_throttle = throttle_c_sat;
P.takeoff_pitch = theta_c_sat;
P.altitudeHoldZone = inf;
P.stateMachineEnable = false;  
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
% The lateral controller design from the previous assignment is implemented
% in the following script
lateral_controller_design
%% Design the Longitudinal Autopilot
% The following section of code will design the controllers inside the 
% longitudinal autopilot.  Each controller design must have a phase margin of 
% > 60 degrees and a gain margin of >12 dB.  Make sure you change the
% default values of bode plots to Hz by calling "ctrlpref" and changing the
% appropriate settings.
%% Design pitch controller
% For the design of the pitch controller, you can assume the following:
% - The largest pitch angle command you will issue is 20 degrees.
% - The elevator saturates at 45 degrees.
% After you are finished with your controller design, verify that you meet 
% the phase margin and gain margin requirements by creating bode plots of the
% OLTF. Analyze the closed-loop performance of the system by creating bode 
% plots of the CLTF and a step response plot. If a bode plot looks stable,
% but Matlab is telling you that it is not, you may need to use the
% minreal() function call on the associated OLTF to fix numerical issues.
close all
%% Choose k_{p_{\theta}} based on saturation of the elevator and the maximum expected pitch error
e_theta_max = 20*pi/180;  % Pitch command when delta_e_max is achieved
delta_e_max = 45*pi/180;  % maximum possible elevator command
k_ptheta = delta_e_max/e_theta_max*sign(lon.a_theta3);
%% Calculate the natural frequency \omega_{n_{\theta}}of the pitch control loop
omega_ntheta = sqrt(lon.a_theta2 + delta_e_max/e_theta_max*abs(lon.a_theta3));
%% Choose the damping ratio \zeta_{\theta} of the pitch control loop
zeta_theta = 0.9;
%% Calculate k_{d_{\theta}}
k_dtheta = (2*zeta_theta*omega_ntheta - lon.a_theta1)/lon.a_theta3;
g_deltaa_2_t = lon.a_theta3*s/(s^2 + (lon.a_theta1 + k_dtheta*lon.a_theta3)*s + lon.a_theta2);
% g_deltaaprime_2_theta = feedback(g_deltaa_2_t, k_dtheta) * 1/s;
% c_theta = k_dtheta/s + k_ptheta;

%% Verify that PM > 60 degrees and gain margin > 12 dB
%Note that it is recommended to use the minreal function in this case to
%avoid Matlab thinking your altitude loop is unstable.
OLTF_theta = minreal(k_ptheta * g_deltaa_2_t * 1/s);
% OLTF_theta = k_ptheta * g_deltaa_2_t * 1/s;
figure; bode(OLTF_theta); grid on;
[gm, pm] = margin(OLTF_theta);
title(sprintf('Pitch OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
        pm, 20*log10(gm)));
%% Analyze the closed loop response T_{\theta}
CLTF_thetac_2_theta = feedback(OLTF_theta,1);
[gpeak, fpeak] = getPeakGain(CLTF_thetac_2_theta);
figure; bode(CLTF_thetac_2_theta); grid on;
title(sprintf('Pitch CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
    bandwidth(CLTF_thetac_2_theta)/2/pi, gpeak, fpeak/2/pi));

figure; step(CLTF_thetac_2_theta);
sinfo = stepinfo(CLTF_thetac_2_theta);
title(sprintf('Pitch Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
    sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
%% Design the altitude hold using commanded pitch
% Design the altitude controller to acheive 0 steady-state error to a
% constant commanded altitude h_c. After you are finished with your 
% controller design, verify that you meet the phase margin and gain margin 
% requirements by creating bode plots of the OLTF. Analyze the closed-loop 
% performance of the system by createing bode plots of the CLTF and a step 
% response plot.
close all
%% Calculate the plant G_{h} for the altitude controller
v_g = Va;
k_theta_DC = k_ptheta*lon.a_theta3/(lon.a_theta2 + k_ptheta*lon.a_theta3);
G_h = CLTF_thetac_2_theta * v_g/s;
%% Temporarily set the altitude controller gains k_{p_{h}}=1 and k_{i_{h}}=0
k_ph = 1;
k_ih = 0;
C_h = k_ih/s + k_ph;
%% Calculate the OLTF G_{h}H_{h}
OLTF_h = minreal(G_h * C_h);  % G_h *k_theta_DC * C_h;
[gm, pm] = margin(OLTF_h);
figure;
bode(OLTF_h);
title(sprintf('Altitude OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
    pm, 20*log10(gm)))
hold all;
grid on;
% Tune the value of k_{p_{h}} such that the phase margin is near and within spec. Tune the value of k_{i_{h}} until the phase margin starts to drop.
% k_ph = db2mag(-22-15.5);
k_ph = db2mag(-22);
k_ih = .0005;
C_h = k_ih/s + k_ph;
OLTF_h = minreal(G_h * C_h);
bode(OLTF_h);
[gm, pm] = margin(OLTF_h);
title(sprintf('Altitude OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
    pm, 20*log10(gm)))
hold all;
grid on;
%% Calculate the CLTF T_{h} and analyze the performance
CLTF_hc_2_h = feedback(OLTF_h,1);
[gpeak, fpeak] = getPeakGain(CLTF_hc_2_h);
figure; bode(CLTF_hc_2_h); grid on;
title(sprintf('Altitude CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
    bandwidth(CLTF_hc_2_h)/2/pi, gpeak, fpeak/2/pi));

figure; step(CLTF_hc_2_h);
sinfo = stepinfo(CLTF_hc_2_h);
title(sprintf('Altitude Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
    sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
close all;
%% Design the airspeed hold using throttle controller
% Design the airspeed controller to acheive 0 steady-state error to a
% constant commanded airspeed V_c for the design of the airspeed controller, 
% you can assume the following:
% - The largest change in airspeed command you will issue is 10 m/s.
% - The throttle saturates at 1.
% After you are finished with your  controller design, verify that you meet 
% the phase margin and gain margin requirements by creating bode plots of 
% the OLTF. Analyze the closed-loop performance of the system by createing 
% bode plots of the CLTF and a step response plot.
%% Choose k_{p_{V}} based on saturation of the throttle and maximum expected airspeed error
delta_vamax = 10;
delta_tmax = 1;
% G_v = CLTF_thetac_2_theta * delta_tmax/(s+delta_vamax);
%% Temporarily set the airspeed controller gain k_{i_{V}}=0
k_pv = delta_tmax/delta_vamax;
k_iv = 0;
c_v = k_iv/s + k_pv;
OLTF_v = c_v*(lon.a_V2/(s+lon.a_V1));
[gm, pm] = margin(OLTF_v);
figure;
bode(OLTF_v);
title(sprintf('Airspeed Hold OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
    pm, 20*log10(gm)))
hold all;
grid on;
%% Tune the value of k_{i_{V}} until the phase margin starts to drop and step response is "good"
k_iv = .0005;
c_v = k_iv/s + k_pv;
OLTF_v = c_v*(lon.a_V2/(s+lon.a_V1));
[gm, pm] = margin(OLTF_v);
bode(OLTF_v);
title(sprintf('Airspeed Hold OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
    pm, 20*log10(gm)))
% Analyze the closed loop response T_{V}
CLTF_vc_2_v = feedback(OLTF_v,1);
[gpeak, fpeak] = getPeakGain(CLTF_vc_2_v);
figure; bode(CLTF_vc_2_v); grid on;
title(sprintf('Airspeed Hold CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
    bandwidth(CLTF_vc_2_v)/2/pi, gpeak, fpeak/2/pi));

figure; step(CLTF_vc_2_v);
sinfo = stepinfo(CLTF_vc_2_v);
title(sprintf('Airspeed Hold Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
    sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
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
% Pitch controller parameters
P.k_ptheta = k_ptheta;
P.k_dtheta = k_dtheta;
% Altitude from pitch controller parameters
P.k_ph = k_ph;
P.k_ih = k_ih;
% Velocity from throttle controller parameters
P.k_pv = k_pv;
P.k_iv = k_iv;
%% Controller verification
% You designed the controllers using the approximate transfer functions from
% section 5.4.  Any nonlinear or cross-coupling terms were considered
% "disturbances".  Although the state space models are a linear
% approximation, they retain the cross-couping terms.  In this section, you
% will extract OLTFs from your Simulink model to verify adequate stability
% margins.  If the margins aren't sufficient, you must modify the
% controller gains to regain the necessary margins.  You can do this by
% naming the traces of the inputs and outputs of the transfer functions of 
% interest inside the simulink model, then
% making them "linear analysis point -> open-loop input/output".
% Hints: Use the "getlinio" command to determine the indices.  Use the
% "operpoint" command to define the trim condition about which to linearize.
% Use the "linearize" command to retrieve the OLTFs for the Roll and Course
% loops.  Create bode plots for each axis and verify sufficient gain margin
% and phase margin.
close all
io = getlinio(model);                   % Retrieve the analysis I/Os
op = operpoint(model);                  % Retrieve the operating point
% Check the pitch angle controller design
i_theta_c = 3;
i_theta = 10;
OLTF_theta_sim = linearize(model,io([i_theta_c,i_theta]),op); % Linearize the model
figure; bode(OLTF_theta, OLTF_theta_sim); grid on;
title('Pitch Angle Controller Design Verification');
legend('design','sim'); xlim([0.01,100])
% Check the altitude controller design
i_h_c = 5;
i_h = 9;
OLTF_h_sim = linearize(model,io([i_h_c,i_h]),op); % Linearize the model
figure; bode(OLTF_h, OLTF_h_sim); grid on;
title('Altitude Controller Verification');
legend('design','sim'); xlim([0.01,100])
% Check the airspeed controller design
i_V_a_c = 4;
i_V_a = 8;
OLTF_v_sim = linearize(model,io([i_V_a_c,i_V_a]),op); % Linearize the model
figure; bode(OLTF_v, OLTF_v_sim); grid on;
title('Airspeed Controller Verification');
legend('design','sim'); xlim([0.01,100])
%% Controller Testing
% You are now ready to test the controller.  The following code creates a
% step change in commanded airspeed. Verify that the airspeed controller
% operates as expected.
close all
tsim = 70;
chi_c_in = timeseries([0,0,0,0,0]*pi/180,[0,10,10,20,70]);
h_c_in = timeseries([h0, h0, h0, h0],[0, 10, 10, 70]);
V_a_c_in = timeseries([Va, Va, Va + 10, Va+10],[0, 10, 10, 70]);
figure; plot(chi_c_in*180/pi); grid on; title('Commanded Course Angle')
figure; plot(h_c_in); grid on; title('Commanded Altitude')
figure; plot(V_a_c_in); grid on; title('Commanded Airspeed')
sim(model,tsim);
generatePlots
%% Saturations
% Now that the longitudinal autopilot is working correctly, you need to
% implement saturations so that the UAV response is more realistic.
% Saturate the elevator to the values used in the design of the pitch
% controller.  To avoid asking too much of the Pitch controller,
% saturate the output of the altitude controller to +/-20 degrees using
% the "Clamping" anti-windup method.  Also saturate the throttle to [0, 1]
% using the "Clamping" anti-windup method.  The commanded trajectory is
% modified for you below, so as to exercise these saturations.
close all
tsim = 40;
phi_c_sat = 45*pi/180;
theta_c_sat = 20*pi/180;
aileron_c_sat = delta_a_max;
elevator_c_sat = delta_e_max;
throttle_c_sat = delta_tmax;
chi_c_in = timeseries([0,0]*pi/180,[0,40]);
h_c_in = timeseries([h0, h0, h0 + 100, h0 + 100],[0, 2, 2, 40]);
V_a_c_in = timeseries([Va, Va, Va+15, Va+15],[0, 22, 22, 40]);
figure; plot(chi_c_in*180/pi); grid on; title('Commanded Course Angle')
figure; plot(h_c_in); grid on; title('Commanded Altitude')
figure; plot(V_a_c_in); grid on; title('Commanded Airspeed')
sim(model,tsim);
generatePlots
%% Take-off
% You will now implement logic to handle takeoff from a (strong) hand
% launch, and climb to altitude using the take simplified altitude state
% machine.  To do this, you need to insert the saturation logic into the
% "Altitude Hold" block.  You also need to insert logic in the "Takeoff
% Autopilot" block.
close all
u_trim = zeros(4,1);
x_trim = [0,0,-2, 3,0,-0.5, 0,0,0, 0,0,0];
P.x_trim = x_trim;
P.u_trim = u_trim;
% set initial conditions to stationary on the ground
P.pn0    = x_trim(1);  % initial North position
P.pe0    = x_trim(2);  % initial East position
P.pd0    = x_trim(3);% initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle 
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate
% Specify the parameters of the take-off autopilot
P.takeoff_altitude = 10;
P.takeoff_throttle = throttle_c_sat;
P.takeoff_pitch = theta_c_sat;
P.altitudeHoldZone = 10;
P.stateMachineEnable = true;
% Specify trajectory
tsim = 80;
chi_c_in = timeseries([0,0]*pi/180,[0,40]);
h_c_in = timeseries([100, 100, 25, 25],[0, 40, 40, 80]);
V_a_c_in = timeseries([Va, Va],[0, 40]);
figure; plot(chi_c_in*180/pi); grid on; title('Commanded Course Angle')
figure; plot(h_c_in); grid on; title('Commanded Altitude')
figure; plot(V_a_c_in); grid on; title('Commanded Airspeed')
sim(model,tsim);
generatePlots