function [flag, r, q, c, rho, lambda] = followWppFillet( w, p, R, newpath )
%followWppFillet implements waypoint following via straightline paths
%connected by fillets
%
% Inputs:
%   W = 3xn matrix of waypoints in NED (m)
%   p = position of MAV in NED (m)
%   R = fillet radius (m)
%   newpath = flag to initialize the algorithm or define new waypoints
%
% Outputs
%   flag = flag for straight line path (1) or orbit (2)
%   r = origin of straight-line path in NED (m)
%   q = direction of straight-line path in NED (m)
%   c = center of orbit in NED (m)
%   rho = radius of orbit (m)
%   lambda = direction or orbit, 1 clockwise, -1 counter clockwise
%
% Example Usage
% [flag, r, q, c, rho, lambda] = followWppFillet( w, p, R, newpath )
%

% Author: Randy Christensen
% Date: 14-Mar-2019 23:51:41
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, Algorithm 6
% Copyright 2018 Utah State University

persistent i state
if isempty(i)
    i = 0;
    state = 0;
end
if newpath
    % Initialize index
    i = 2;
    state = 1;
    % Check sizes
    [m,N] = size(w);
    assert(N >=3);
    assert(m == 3);
else
    [m,N] = size(w);
    assert(N >=3);
    assert(m == 3);
end
% Calculate all the q vector and fillet angle
wi = [w(1,i), w(2,i), w(3,i)]';
wiMinus = [w(1,i-1), w(2,i-1), w(3,i-1)]';
wiPlus = [w(1,i+1), w(2,i+1), w(3,i+1)]';

qiMinus = (wi - wiMinus)/(norm(wi-wiMinus));
qi = (wiPlus - wi)/(norm(wiPlus - wi));

e = acos(-qiMinus' * qi);

% Determine if the MAV is on a straight or orbit path
if state == 1
    flag = 1;
    r = wiMinus;
    q = qiMinus;
    z = wi - (R/tan(e/2))*qiMinus;
    
    c = nan(3,1);
    rho = nan;
    lambda = sign(nan);
    
    if in_half_plane(p,z,qiMinus)
        state = 2;
    end
    
elseif state == 2
    flag = 2;
    c = wi - (R/sin(e/2)) * ((qiMinus - qi)/norm(qiMinus - qi));
    rho = R;
    lambda = sign(qiMinus(1)*qi(2) - qiMinus(2)*qi(1));
    z = wi + (R/tan(e/2))*qi;
    
    r = p;
    q = nan(3,1);
    
    if in_half_plane(p,z,qi)
        if (i ~= N-1)
            i = i+1;
        end
        state = 1;
    end

else
    %Fly north as a default
    flag = -1;
    r = p;
    q = [1, 0, 0]';
    c = nan(3,1);
    rho = nan;
    lambda = nan;
end
end
