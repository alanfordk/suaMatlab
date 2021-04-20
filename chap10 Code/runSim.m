clearvars
close all
clc
%% Prelims
tsim = 60;
model = 'mavsim_chap10';
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
%% Guidance model parameters
guidanceParams
%% Straight line following without wind
% The following code defines a desired path oriented NE and a incline of 5
% degrees. Implement the straight line follower in your Simulink model.
% Tune the parameters chi_inf and k_path such that the MAV flies at ~90
% degrees to the desired path when far away and converges to within 10
% meters of the path in less than 30 seconds.
[qn, qe, qd] = sph2cart(45*pi/180, -5*pi/180, 1);
path_struct.type = 'lin';
path_struct.origin = [P.pn0-1000, P.pe0, P.pd0]';
path_struct.direction = [qn, qe, qd]';
path_struct.chi_inf = pi/2;
path_struct.k_path = .02;
path_vector = pathstruct2pathvec(path_struct);
sim(model,tsim);
skipStatePlots = true;
generatePlots;
%% Orbit following without wind
% The following code defines a 200 meter level orbit.  Implement the orbit
% follower in your Simulink model. Tune the parameter k_orbit to converge
% to within 10 meters of the orbit in less than 30 seconds.
path_struct.type = 'orb';
path_struct.center = [P.pn0, P.pe0 + 1000, P.pd0-100]';
path_struct.radius = 200;
path_struct.orbit_direction = 1;
path_struct.k_orbit = 5;
path_vector = pathstruct2pathvec(path_struct);
sim(model,tsim);
skipStatePlots = true;
generatePlots;
%% Orbit following, minimum radius
% Using the k_orbit value from the previous step, determine the minimum
% achievable radius for orbit following. Look through the generated plots
% to investigate causes of the limitation and include your findings below:

% <With a k_orbit value of 5 I got the radius down to around 135. When I
% got any smaller than this the plane had a hard time making the turns to
% match the desired orbit. When looking at the graphs the airspeed and
% pitch angle were the plots that stood out to me. These plots stood out
% casue the acutal vs commanded were very different. Because the altitude
% didn't seem to be a problem though I think it was more of the airseed.
% The plane couldn't make that sharp of turns with that air speed.>

path_struct.type = 'orb';
path_struct.center = [P.pn0, P.pe0 + 300, P.pd0]';
path_struct.radius = 135;
path_struct.orbit_direction = 1;
path_struct.k_orbit = 5;
path_vector = pathstruct2pathvec(path_struct);
sim(model,tsim);
skipStatePlots = false;
generatePlots;
%% Straight line following with wind
% Using the design parameters for line following from above, determine how 
% much (positive) north wind can be tolerated before the design 
% requirements are not met.  Include your findings below:

% <Insert comments here, i.e. what percentage of the airspeed? As I stated
% to get the wind speed above 30 the plane went way off course. At 30 the
% plane was getting to the needed path, but having a hard time. The desired
% airspeed was 35 so that probably has something to do with it>

P.wind_n = 35;
[qn, qe, qd] = sph2cart(45*pi/180, -5*pi/180, 1);
path_struct.type = 'lin';
path_struct.origin = [P.pn0-1000, P.pe0, P.pd0]';
path_struct.direction = [qn, qe, qd]';
path_struct.chi_inf = pi/2;
path_struct.k_path = .02;
path_vector = pathstruct2pathvec(path_struct);
sim(model,tsim);
skipStatePlots = false;
generatePlots;
%% Orbit following with wind
% Using the design parameters for orbit following from above, determine how 
% much (positive) north wind can be tolerated before the design 
% requirements are not met.  Include your findings below:
% <Insert comments here, i.e. what percentage of the airspeed?  Again when
% I had the wind at 30 the plane was making progress, but having a hard
% time getting to the needed path. When the wind was set to 35 the plane
% was way off couse. The reason has something to do with the commanded
% airspeed also being 35 I beilieve>
P.wind_n = 25;
path_struct.type = 'orb';
path_struct.center = [P.pn0, P.pe0 + 1000, P.pd0-100]';
path_struct.radius = 200;
path_struct.orbit_direction = 1;
path_struct.k_orbit = 5;
path_vector = pathstruct2pathvec(path_struct);
sim(model,tsim);
skipStatePlots = false;
generatePlots;