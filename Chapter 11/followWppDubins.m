function [ flag, r, q, c, rho, lambda ] = followWppDubins( w, Chi, p, R, newpath)
%followWppDubins implements waypoint following via Dubins paths
%
% Inputs:
%   P = description (units)
%   p = description (units)
%   R = description (units)
%   newpath = description (units)
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
%  [ flag, r, q, c, rho, lambda ] = followWppDubins( P, p, R, newpath)
%

% Author: Randy Christensen
% Date: 15-Mar-2019 23:11:19
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, Algorithm 8
% Copyright 2018 Utah State University

persistent i state;
if isempty(i)
    i = 0;
    state = 0;
end
if newpath
    i = 2;
    state = 1;
    [m,N] = size(w);
    assert(N >= 3,'Non enough vehicle configurations.');
    assert(m == 3);
else
    [m,N] = size(w);
    assert(N >= 3,'Non enough vehicle configurations.');
    assert(m == 3);
end

wi = [w(1,i), w(2,i), w(3,i)]';
wiMinus = [w(1,i-1), w(2,i-1), w(3,i-1)]';
wiPlus = [w(1,i+1), w(2,i+1), w(3,i+1)]';

chi_i = Chi(i);
chi_minus = Chi(i-1);
chi_plus = Chi(3);

% Determine the Dubins path parameters
dp = findDubinsParameters( wiMinus, chi_minus, wi, chi_i, R );

if state == 1
    %Follow start orbit until on the correct side of H1
    flag = 2;
    c = dp.c_s;
    rho = R;
    lambda = dp.lambda_s;
    r = nan(3,1);
    q = [1; 0; 0];

    if in_half_plane(p,dp.z_1,-dp.q_1)
        state = 2;
    end
elseif state == 2
    flag = 2;
    r = nan(3,1);
    q = nan(3,1);
    c = dp.c_s;
    rho = R;
    lambda = dp.lambda_s;
    %Continue following the start orbit until in H1
    if in_half_plane(p,dp.z_1, dp.q_1)
        state = 3;
    end
elseif state == 3
    %Transition to straight-line path until in H2
    flag = 1;
    r = dp.z_1;
    q = dp.q_1;
    c = nan(3,1);
    rho = nan;
    lambda = nan;
    if in_half_plane(p,dp.z_2,dp.q_1)
        state = 4;
    end
elseif state == 4
    %Follow the end orbit until on the correct side of H3
    flag = 2;
    r = nan(3,1);
    q = nan(3,1);
    c = dp.c_e;
    rho = R;
    lambda = dp.lambda_e;
    if in_half_plane(p,dp.z_3,dp.q_3)
        state = 5;
    end
elseif state == 5 %state == 5
    flag = 2;
    r = nan(3,1);
    q = nan(3,1);
    c = dp.c_e;
    rho = R;
    lambda = dp.lambda_e;
    %Continue following the end orbit until in H3
    if in_half_plane(p,dp.z_3,dp.q_3)
        i = i+1;
        state = 1;
%         if (i ~= N-1)
%             i = i+1;
%         end

        findDubinsParameters( wiMinus, chi_minus, wi, chi_i, R );
    end
else
    flag = nan;
    r = nan(3,1);
    q = nan(3,1);
    c = nan(3,1);
    rho = nan;
    lambda = nan;
end
end