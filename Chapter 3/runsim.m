clear
close all
clc
format long
format compact
%% Prelims
model = 'mavsim_chap3';
tsim = 20;
P.Ts = 0.005;     % autopilot sample rate
P.gravity = 9.81;
timevals = 0:P.Ts:tsim;
nsamp = length(timevals);
%% Problem 0: Implementation
% Follow the guidelines in problem 3.1 and 3.2 of the textbook to implement
% the mav_dynamics.m S-function.  For the aircraft parameters, use 
% Table E.2 for the Aerosonde UAV.

%% Problem 1: No Inputs, Zero Initial Conditions
% For this problem, you will test the mav_dynamics.m S-function with no
% input forces and torques, and all initial velocities set to zero.
%
% Implement define_inertia_symmetric.m according to Table E.2 but for a
% fully symmetric body.
%
% Implement define_ic_1.m such that the vehicle starts at zero north and
% east position, and an altitude of 5 meters.  Set the initial attitude to
% 45 degrees NE and zero roll and pitch.  Set all velocities to zero.
%
% What behavior to you expect for EVERY state?
%
% Insert answer here
% The plane shouldn't move at all. There are no forces or velocities so
% there should be no movement. The plane should be at a altitude of 5 and
% facing the north east direction.
% Run the simulation and verify the expected behavior.
% Run the simulation and verify the expected behavior.
define_inertia_symmetric;
define_ic_1;
force = zeros(3,nsamp);
torque = zeros(3,nsamp);
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;
%% Problem 2: No Inputs, Non-Zero Initial Translational Velocities
% For this problem, you will test the mav_dynamics.m S-function with no
% input forces and torques, but with initial translational velocities
%
% Continue to use the vehicle inertias for a for a fully symmetric body.
%
% Implement define_ic_2.m such that the vehicle starts at zero north and
% east position, and an altitude of 5 meters.  Set the initial attitude to
% 45 degrees NE and zero roll and pitch.  Set initial velocities such that
% the vehicle is flying northeast at 1 meter per second and ascending at 
% 0.5 meters per second.
%
% What behavior to you expect for EVERY state?
%
% Insert answer here
% The plane should be facing the north east direction. It will start at a
% altitude of 5 and ries up during flight due to the innitial velocity in
% the z axis. It will also be moving in the north east direction.
close all;
define_inertia_symmetric;
define_ic_2;
force = zeros(3,nsamp);
torque = zeros(3,nsamp);
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;
%% Problem 3: No Inputs, Non-Zero Initial Rotational Velocities
% For this problem, you will test the mav_dynamics.m S-function with no
% input forces and torques, but with initial rotational velocities
%
% Continue to use the vehicle inertias for a for a fully symmetric body.
%
% Implement define_ic_3.m such that the vehicle starts at zero north and
% east position, and an altitude of 5 meters.  Set the initial attitude to
% 45 degrees NE and zero roll and pitch.  Set initial translational 
% velocities such that the vehicle is at rest.  Set the initial rotational
% velocities such that the vehicle is rotating about the nose of the
% aircraft by 0.5 rad/s.
%
% What behavior to you expect for EVERY state?
%
% Insert answer here
%
% Run the simulation and verify the expected behavior.
close all;
define_inertia_symmetric;
define_ic_3;
force = zeros(3,nsamp);
torque = zeros(3,nsamp);
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;
%% Problem 4: No Inputs, Non-Zero Initial Rotational Velocities
% For this problem, you will test the mav_dynamics.m S-function with no
% input forces and torques, with initial rotational velocities, and with an
% axisymmetric body (as opposed to a full-symmetric body)
%
% Implement define_inertia_axisymmetric.m according to Table E.2
%
% Use the same initial conditions as problem 3.
%
% What behavior to you expect for EVERY state?
% Insert answer here
%% Insert answer here
% The plane will not be moving in any direction because velocities are all
% zero. There will be some rotation that is rotating about the nose with a
% roll rate, but it will be differen't because the plane is not fully
% symetric.
% Run the simulation and verify the expected behavior.
close all;
define_inertia_axisymmetric;
define_ic_3;
force = zeros(3,nsamp);
torque = zeros(3,nsamp);
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;
%% Problem 5: Constant Force Inputs
% For this problem, you will test the mav_dynamics.m S-function with zero
% initial conditions and constant force inputs.
%
% Continue to use define_inertia_axisymmetric.m
% 
% Use the same initial conditions as problem 1
%
% Define the input forces to achieve 2 m/s NE velocity and 1 m/s up
% velocity by the end of the simulation
%
% What behavior to you expect for EVERY state?
% The plane will start not moving, but from the forces the plane will be
% accelerating upwards and northeast.
% Run the simulation and verify the expected behavior.
close all;
define_inertia_axisymmetric;
define_ic_1;
force = zeros(3,nsamp);
force(1,:) = 1.35;
force(3,:) = -0.675;
torque = zeros(3,nsamp);
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;
%% Problem 6: Constant Torque Inputs
% For this problem, you will test the mav_dynamics.m S-function with zero
% initial conditions and constant torques.
%
% Continue to use define_inertia_axisymmetric.m
% 
% Use the same initial conditions as problem 1
%
% Define the input torques to achieve 1 rad/s of angular velocity about the
% body Y axis by the end of the simulation.
%
% What behavior to you expect for EVERY state?
% Insert answer here
%Again the plane will not be moving at first. It will not accelerate in any
%direction, but the torque will cause accleration which will cause the
%plane to start to roll
% Run the simulation and verify the expected behavior.
close all;
define_inertia_axisymmetric;
define_ic_1;
force = zeros(3,nsamp);
torque = zeros(3,nsamp);
torque(2,:) = .05675;
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;
% display(states.Data(:,11));
%% Problem 7: Constant Torque Inputs
% For this problem, you will test the mav_dynamics.m S-function with zero
% initial conditions and constant torques.
%
% Continue to use define_inertia_axisymmetric.m
% 
% Use the same initial conditions as problem 1
%
% Define the input torques to 0.1 Newtons about the body Z axis
%
% What behavior to you expect for EVERY state?
% Insert answer here
% The plane will not have any translational movement due to zero
% velocities. The plane will spin in a circleabout the z axis
% Run the simulation and verify the expected behavior.
close all;
define_inertia_axisymmetric;
define_ic_1;
force = zeros(3,nsamp);
torque = zeros(3,nsamp);
torque(3,:) = .1;
datavals = [force;torque];
forces_torques = timeseries(datavals,timevals);
sim(model);
generatePlots;