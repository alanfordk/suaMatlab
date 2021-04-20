clearvars
close all
clc
%% Prelims
tsim = 100;
model = 'mavsim_chap7';
P.Ts = 0.005;     % autopilot sample rate
P.gravity = 9.8;  % gravity (m/s^2)
%% Params for Aersonade UAV
aerosonde
%% Params for autopilot controllers
cntrlParams
%% Wind parameters
windParams
%% Commanded trajectory
trajectoryParams
%% High-rate sensor params
P.Ts_highrate = P.Ts;       %Sampling period of high-rate sensors (s)

% Gyro
P.sigma_gyro = 0.13*pi/180;

% Accelerometer
P.sigma_accel = .0025 * P.gravity;

% Pressure
P.press_beta_abs = 0.125;
P.press_sigma_abs = 0.01;
P.press_beta_diff = 0.020;
P.press_sigma_diff = 0.002;


% Enter noise parameters of high-rate sensors here

%% Low-rate sensor params
P.Ts_lowrate = 1;       %Sampling period of low-rate sensors (s)

% Magnetic
P.declination = (11 + 25/60)*pi/180; %Magnetic declination for Logan, Utah (rad)
P.sigma_mag = 0.3;
P.beta_mag = 1;

% GPS
P.Ts_gps = 1.0;
P.K_gps = 1/1100;
P.sigma_n_gps = 0.21;
P.sigma_e_gps = 0.21;
P.sigma_d_gps = 0.40;

P.sigma_velocity = .05;


% Enter noise parameters of low-rate sensors here


%% Simulate and generate plots
sim(model,tsim);
generatePlots