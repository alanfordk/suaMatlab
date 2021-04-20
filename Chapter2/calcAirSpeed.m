function [ V_a_b ] = calcAirSpeed(v_ned, w_ned, ypr)
%calcAirSpeedBodyFrame Calculates the airspeed expressed in the body frame
%using ground velocity, wind velocity, and attitude.
%
% Inputs:
%   v_ned = ground speed expressed in NED frame (m/s)
%   w_ned = wind speed expressed in NED frame (m/s)
%   ypr = yaw, pitch, roll euler angles (rad)
%
% Outputs
%   V_a_b = airspeed vector expressed in the body frame (m/s)
%
% Example Usage
% [ V_a_b ] = calcAirSpeedBodyFrame(alpha, beta, v_ned, w_ned, ypr)
%
% See also FUNC1, FUNC2

% Author: Randy Christensen
% Date: 27-Jun-2018 23:58:34
% Reference: Beard, Mclain, Small Unmanned Aicraft page 18-19
% Copyright 2018 Utah State University

V_a_b = 

end
