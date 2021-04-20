function [ r, q_out ] = followWpp( w, p, newpath )
%followWpp implements waypoint following via connected straight-line
%paths.
%
% Inputs:
%   W = 3xn matrix of waypoints in NED (m)
%   p = position of MAV in NED (m)
%   newpath = flag to initialize the algorithm or define new waypoints
%
% Outputs
%   r_out = origin of straight-line path in NED (m)
%   q_out = direction of straight-line path in NED (m)
%
% Example Usage
% [ r_out, q_out ] = followWpp( w, p, newpath )

% Author: Randy Christensen
% Date: 13-Mar-2019 23:15:27
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, Algorithm 5
% Copyright 2018 Utah State University

persistent i
if isempty(i)
    i = 0;
end
if newpath
    % Initialize index
    i = 2;
    % Check sizes
    [m,N] = size(w);
    assert(N >=3);
    assert(m == 3);
else
    [m,N] = size(w);
    assert(N >=3);
    assert(m == 3);
end
% Calculate all the q vector
wi = [w(1,i), w(2,i), w(3,i)]';
wiMinus = [w(1,i-1), w(2,i-1), w(3,i-1)]';
wiPlus = [w(1,i+1), w(2,i+1), w(3,i+1)]';

qiMinus = (wi - wiMinus)/(norm(wi-wiMinus));
qi = (wiPlus - wi)/(norm(wiPlus - wi));

% Calculate the origin of the current path
r = wiMinus;

% Calculate the unit normal to define the half plane
n_i = (qiMinus+qi)/norm(qiMinus+qi);
q_out = qiMinus;

% Check if the MAV has crossed the half-plane
if in_half_plane(p,w(:,i),n_i)
    if (i ~= N-1)
        i = i+1;
    end
end

