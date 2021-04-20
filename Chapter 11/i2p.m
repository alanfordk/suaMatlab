function [ rip ] = i2p( chi_q )
%i2p Summary of the function goes here
%   Detailed explanation goes here
%
% Inputs:
%   chi_q = description (units)
%
% Outputs
%   rip = description (units)
%
% Example Usage
% [ rip ] = i2p( q_i )

% Author: Randy Christensen
% Date: 12-Mar-2019 14:54:00
% Reference: 
% Copyright 2018 Utah State University

rip = [cos(chi_q), sin(chi_q), 0;
      -sin(chi_q), cos(chi_q), 0;
      0, 0, 1];
end
