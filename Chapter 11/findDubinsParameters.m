function [ dp ] = findDubinsParameters( p_s, chi_s, p_e, chi_e, R )
%findDubinsParameters determines the dubins path parameters
%
% Inputs:
%   p_s = start position (m)
%   chi_s = start course angle (rad)
%   p_e = end position (m)
%   chi_e = end course angle (rad)
%   R = turn radius (m)
%
% Outputs
%   dp.L = path length (m)
%   dp.c_s = start circle origin (m)
%   dp.lambda_s = start circle direction (unitless)
%   dp.c_e = end circle origin (m)
%   dp.lambda_e = end circle direction (unitless)
%   dp.z_1 = Half-plane H_1 location (m)
%   dp.q_12 = Half-planes H_1 and H_2 unit normals (unitless)
%   dp.z_2 = Half-plane H_2 location (m)
%   dp.z_3 = Half-plane H_3 location  (m)
%   dp.q_3 = Half-plane H_3 unit normal (m)
%   dp.case = case (unitless)
%
% Example Usage
% [ dp ] = findDubinsParameters( p_s, chi_s, p_e, chi_e, R )

% Author: Randy Christensen
% Date: 15-Mar-2019 23:10:52
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, Algorithm 7
% Copyright 2018 Utah State University

%Ensure that the configurations are far enough apart
assert(norm(p_s(1:2) - p_e(1:2)) >= 3*R,'Start and end configurations are too close!');
%Misc calcs
e_1 = [1,0,0]';
%Compute circle centers
c_rs = p_s + R*Rz(pi/2)*[cos(chi_s), sin(chi_s), 0]';
c_ls = p_s + R*Rz(-pi/2)*[cos(chi_s), sin(chi_s), 0]';
c_re = p_e + R*Rz(pi/2)*[cos(chi_e), sin(chi_e), 0]';
c_le = p_e + R*Rz(-pi/2)*[cos(chi_e), sin(chi_e), 0]';

%Compute path lengths
%Case 1: R-S-R
th = angle(c_re - c_rs);
L1 = norm(c_rs-c_re) + R * mod(2*pi + mod(th-pi/2, 2*pi) - mod(chi_s-pi/2, 2*pi), 2*pi) + R * mod(2*pi + mod(chi_e-pi/2, 2*pi) - mod(th-pi/2, 2*pi), 2*pi);

%Case 2: R-S-L
th = angle(c_le - c_rs);
ell = norm(c_le-c_rs);
th2 = th - pi/2 + asin(2*R/ell); % (pi - (pi/2 + cos((ell/2)/sqrt(ell^2 - 4*R^2))));

%<insert code here>
if ~isreal(th2)
    L2 = nan; %Will not be selected
else
    L2 = sqrt(ell^2 - 4*R^2) ...
        + R * mod(2*pi + mod(th2, 2*pi) - mod(chi_s-pi/2, 2*pi), 2*pi) ...
        + R * mod(2*pi + mod(th2 + pi, 2*pi) - mod(chi_e + pi/2, 2*pi), 2*pi);
end
%Case 3: L-S-R
th = angle(c_re - c_ls);
ell = norm(c_re - c_ls);
th2 = acos(2*R/ell); % pi - (pi/2 + cos(sqrt(ell^2 - 4*R^2)/(ell/2)));
%<insert code here>
if ~isreal(th2)
    L3 = nan; %Will not be selected
else
    L3 = sqrt(ell^2 - 4*R^2) ...
        + R*mod(2*pi + mod(chi_s+pi/2, 2*pi) - mod(th+th2, 2*pi), 2*pi) ...
        + R*mod(2*pi + mod(chi_e-pi/2, 2*pi) - mod(th+th2-pi, 2*pi), 2*pi);
end
%Case 4: L-S-L
th = angle(c_le - c_ls);
%<insert code here>
L4 = norm(c_ls-c_le) ...
    + R * mod(2*pi + mod(chi_s+pi/2, 2*pi) - mod(th+pi/2, 2*pi), 2*pi) ...
    + R * mod(2*pi + mod(th+pi/2, 2*pi) - mod(chi_e+pi/2, 2*pi), 2*pi);
%Define the parameters for the minimum length path (i.e. Dubins path)
[L,i_min] = min([L1, L2, L3, L4]);
if i_min == 1
    c_s = c_rs;
    lambda_s = +1;
    c_e = c_re;
    lambda_e = 1;
    q_1 = (c_e-c_s)/norm(c_e-c_s);
    z_1 = c_s + R*Rz(-pi/2)*q_1;
    z_2 = c_e + R*Rz(-pi/2)*q_1;
elseif i_min == 2
    c_s = c_rs;
    lambda_s = +1;
    c_e = c_le;
    lambda_e = -1;
    ell = norm(c_e-c_s);
    th = angle(c_e-c_s);
    th2 = th-pi/2 + asin(2*R/ell);
    q_1 = Rz(th2+pi/2)*e_1;
    z_1 = c_s + R*Rz(th2)*e_1;
    z_2 = c_e + R*Rz(th2+pi)*e_1;
elseif i_min == 3
   c_s = c_ls;
   c_e = c_re;
   lambda_s = -1;
   lambda_e = +1;
   ell = norm(c_e-c_s);
   th = angle(c_e-c_s);
   th2 = acos(2*R/ell);
   q_1 = Rz(th+th2-pi/2)*e_1;
   z_1 = c_s + R*Rz(th+th2)*e_1;
   z_2 = c_e + R*Rz(th+th2-pi)*e_1;
else% i_min == 4
    c_s = c_ls;
    c_e = c_le;
    lambda_s = -1;
    lambda_e = -1;
    q_1 = (c_e-c_s)/norm(c_e-c_s);
    z_1 = c_s + R*Rz(pi/2)*q_1;
    z_2 = c_e + R*Rz(pi/2)*q_1;
end
z_3 = p_e;
q_3 = Rz(chi_e)*e_1;

%Package outputs into struct
dp.L = L;
dp.c_s = c_s;
dp.lambda_s = lambda_s;
dp.c_e = c_e;
dp.lambda_e = lambda_e;
dp.z_1 = z_1;
dp.q_1 = q_1;
dp.z_2 = z_2;
dp.z_3 = z_3;
dp.q_3 = q_3;
dp.case = i_min;
dp.lengths = [L1, L2, L3, L4];
dp.theta = th;
dp.ell = ell;
dp.c_rs = c_rs;
dp.c_ls = c_ls;
dp.c_re = c_re;
dp.c_le = c_le;
end

function out = Rz(th)
out = [cos(th), -sin(th), 0;
    sin(th), cos(th), 0;
    0,       0, 1];
end

function out = angle(v)
out = atan2(v(2),v(1));
end