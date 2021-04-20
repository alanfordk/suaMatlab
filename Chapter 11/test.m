% Author: Randy Christensen
% Date: 11-Mar-2019 23:07:46
% Reference: 
% Copyright 2018 Utah State University
%% Prelims
clearvars
close all
clc
%% Define the Dubins path
p_s = [0,0,0]';
chi_s = 45*pi/180;
p_e = [-1000, 1000,0]';
chi_e = 160*pi/180;
R = 150;
[ dp ] = findDubinsParameters( p_s, chi_s, p_e, chi_e, R );
%% Plot the Dubins path
h_fig = figure;
h_ax = plotDubinsPath( p_s, chi_s, p_e, chi_e, R , h_fig );
axis equal
grid on;
legend('location','northeastoutside')
xlabel('east(m)');
ylabel('north(m)')
title(sprintf('Dubins Path Test\nCase %d Selected',dp.case))