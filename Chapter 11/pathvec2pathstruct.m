function [ path_struct ] = pathvec2pathstruct( path_vector )
%path_unpack retrieves the path parameters from the path vector and places
%them in the path structure
%
% Inputs:
%   path_vector = vector containing path parameters (mixed units)
%
% Outputs
%   path_struct = structure containing path parameters (mixed units)
%
% Example Usage
% [ path_struct ] = pathvec2pathstruct( path_vector )

% Author: Randy Christensen
% Date: 12-Mar-2019 00:32:24
% Reference:
% Copyright 2018 Utah State University

% Unpack the input
flag = path_vector(1);
r = path_vector(2:4);
q = path_vector(5:7);
chi_inf = path_vector(8);
k_path = path_vector(9);
c = path_vector(10:12);
rho = path_vector(13);
lambda = path_vector(14);
k_orbit = path_vector(15);
% Create the output structure
path_struct.origin = r;
path_struct.direction = q;
path_struct.chi_inf = chi_inf;
path_struct.k_path = k_path;
path_struct.center = c;
path_struct.radius = rho;
path_struct.orbit_direction = lambda;
path_struct.k_orbit = k_orbit;
if flag == 1
    path_struct.type = 'lin';
elseif flag == 2
    path_struct.type = 'orb';
else
    path_struct.type = 'non_supported';
    assert(false,'Non-supported path type!')
end
end
