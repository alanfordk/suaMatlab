function [ in ] = in_half_plane( p, r, n )
%in_half_plane returns a flag indicating if the position p is in the half
%plane defined by the point r and unit normal n.
%
% Inputs:
%   p = position of the MAV (m)
%   r = point on the half plane (m)
%   n = unit normal of the half plane
%
% Outputs
%   in = flag indicating that p is in the half plane
%
% Example Usage
% [ in ] = in_half_plane( p,r,n )
%

% Author: Randy Christensen
% Date: 15-Mar-2019 00:00:35
% Reference: Beard, Small Unmanned Aircraft, Chapter 11, page 188
% Copyright 2018 Utah State

% Determine whether p is in the half plane be calculating the sign of the
% dot product.

in = (p - r)'*n >= 0;
end
