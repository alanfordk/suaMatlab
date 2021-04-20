function [ path_vector ] = pathstruct2pathvec( path_struct )
%path_pack places the elements of the path structure in the appropriate
%index in the path vector
%
% Inputs:
%   path_struct = structure containing path parameters (mixed units)
%
% Outputs
%   path_vector = vector containing path parameters (mixed units)
%
% Example Usage
% [ path_vector ] = pathstruct2pathvec( path_struct )

% Author: Randy Christensen
% Date: 12-Mar-2019 00:32:19
% Reference: 
% Copyright 2018 Utah State University

if strcmp(path_struct.type,'lin')
    flag = 1;
    r = path_struct.origin;
    q = path_struct.direction;
    chi_inf = path_struct.chi_inf;
    k_path = path_struct.k_path;
    c = nan(3,1);
    rho = nan;
    lambda = nan;
    k_orbit = nan;
elseif strcmp(path_struct.type,'orb')
    flag = 2;
    r = nan(3,1);
    q = nan(3,1);
    chi_inf = nan;
    k_path = nan;
    c = path_struct.center;
    rho = path_struct.radius;
    lambda = path_struct.orbit_direction;
    k_orbit = path_struct.k_orbit;
else
    flag = nan;
    r = nan(3,1);
    q = nan(3,1);
    chi_inf = nan;
    k_path = nan;
    c = nan(3,1);
    rho = nan;
    lambda = nan;
    k_orbit = nan;
    assert(false,'Non-supported path type!')
end
path_vector = [flag; r; q; chi_inf; k_path; c; rho; lambda; k_orbit];
end
