clearvars
close all
clc
%% Prelims
tsim = 60;
model = 'mavsim_chap9';
P.Ts = 0.005;     % autopilot sample rate
P.gravity = 9.8;  % gravity (m/s^2)
s = tf('s');
%% Params for Aersonade UAV
aerosonde
%% Params for autopilot controllers
cntrlParams
%% Wind parameters
windParams
%% Commanded trajectory
trajectoryParams
%% Sensor parameters
sensorParams
%% Airspeed guidance model parameters
% Determine the parameters of the design model that will acheive <10%
% mismatch on the rise time, settling time, and bandwidth.
% w = 2*pi*.477*1.15
b_va = 3.4; % w
cltf_vc_2_v_guidance = b_va/(s + b_va);
% Generate plots
figure(1);
bode(cltf_vc_2_v, cltf_vc_2_v_guidance);
legend('controller design','guidance')
title('Airspeed CLTF')
figure(2);
step(cltf_vc_2_v, cltf_vc_2_v_guidance);
legend('controller design','guidance')
title('Course Angle Step Response')
% Calculate metrics
stepinfo_control = stepinfo(cltf_vc_2_v);
stepinfo_guidance = stepinfo(cltf_vc_2_v_guidance);
bw_control = bandwidth(cltf_vc_2_v)/2/pi;
bw_guidance = bandwidth(cltf_vc_2_v_guidance)/2/pi;
% Build a table to summarize results
airspeed_results = genResultsTable(stepinfo_control, stepinfo_guidance, ...
    bw_control, bw_guidance);
disp(airspeed_results)
%% Altitude guidance model parameters
% Determine the parameters of the design model that will acheive <10%
% mismatch on the rise time, settling time, and bandwidth.
wn = 2*pi*.07*.55;
zeta = .32;

% wn = 2*pi*.07*.29;
% zeta = 1.4;

b_hdot = 2*zeta*wn;
b_h = wn^2;
cltf_hc_2_h_guidance = (b_hdot/b_h*s + 1)*b_h/(s^2 + b_hdot*s + b_h);
% Generate plots
figure(3);
bode(cltf_hc_2_h, cltf_hc_2_h_guidance);
legend('controller design','guidance')
title('Altitude CLTF')
figure(4);
step(cltf_hc_2_h, cltf_hc_2_h_guidance);
legend('controller design','guidance')
title('Altitude Step Response')
% Calculate metrics
stepinfo_control = stepinfo(cltf_hc_2_h);
stepinfo_guidance = stepinfo(cltf_hc_2_h_guidance);
bw_control = bandwidth(cltf_hc_2_h)/2/pi;
bw_guidance = bandwidth(cltf_hc_2_h_guidance)/2/pi;
% Build a table to summarize results
altitude_results = genResultsTable(stepinfo_control, stepinfo_guidance, ...
    bw_control, bw_guidance);
disp(altitude_results)
%% Course angle guidance model parameters
% Determine the parameters of the design model that will acheive <10%
% mismatch on the rise time and bandwidth.
w = 2 * pi * .34;
zeta = .2;
b_chidot = 2*zeta*w;
b_chi = w^2;
cltf_chic_2_chi_guidance = (b_chidot *s + b_chi)/(s^2 + b_chidot*s + b_chi);
% Generate plots
figure(5);
bode(cltf_chic_2_chi, cltf_chic_2_chi_guidance);
legend('controller design','guidance')
title('Course Angle CLTF')
figure(6);
step(cltf_chic_2_chi, cltf_chic_2_chi_guidance);
legend('controller design','guidance')
title('Course Angle Step Response')
stepinfo_chi = stepinfo(cltf_chic_2_chi);
stepinfo_chi_guidance = stepinfo(cltf_chic_2_chi);
% Calculate metrics
stepinfo_control = stepinfo(cltf_chic_2_chi);
stepinfo_guidance = stepinfo(cltf_chic_2_chi_guidance);
bw_control = bandwidth(cltf_chic_2_chi)/2/pi;
bw_guidance = bandwidth(cltf_chic_2_chi_guidance)/2/pi;
% Build a table to summarize results
altitude_results = genResultsTable(stepinfo_control, stepinfo_guidance, ...
    bw_control, bw_guidance);
disp(altitude_results)

%% Guidance model initial conditions
% After implementing the guidance models in the Simulink model, specify the
% initial condition of these models to correspond to the initial condition
% of the truth models.
x0_design = [0;
             0;
             0;
             1;
             500;
             1;
             35];
%% Simulate and generate plots
% Simulation the guidance and truth models, plot the results, and verify
% that guidance states track the truth states.
sim(model,tsim);
generatePlots