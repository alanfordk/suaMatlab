function h_ax = plotDubinsPath( p_s, chi_s, p_e, chi_e, R , h_fig )
%plotDubinsPath plots the provided Dubins path
%
% Inputs:
%   p_s = start position (m)
%   chi_s = start course angle (rad)
%   p_e = end position (m)
%   chi_e = end course angle (rad)
%   R = turn radius (m)
%   h_fig = figure handle
%
% Outputs
%   h_ax = array of axis handles for each segment of Dubins path
%
% Example Usage
% h_ax = plotDubinsPath( p_s, chi_s, p_e, chi_e, R, h_fig )
%

% Author: Randy Christensen
% Date: 16-Mar-2019 00:23:06
% Reference:
% Copyright 2019 Utah State University

%% Define misc parameters
dth = 0.01;
linewidth = 2;

%% Create the Dubins path
[ dp ] = findDubinsParameters( p_s, chi_s, p_e, chi_e, R );
%% Plot candidate circles
th = 0:dth:2*pi;
c_rs = dp.c_rs + R*[cos(th);sin(th);zeros(1,length(th))];
c_ls = dp.c_ls + R*[cos(th);sin(th);zeros(1,length(th))];
c_re = dp.c_re + R*[cos(th);sin(th);zeros(1,length(th))];
c_le = dp.c_le + R*[cos(th);sin(th);zeros(1,length(th))];
figure(h_fig); hold on;
h_ax(1) = plot3(c_rs(2,:),c_rs(1,:),-c_rs(3,:),'r--','DisplayName','c_{rs}');
h_ax(2) = plot3(c_ls(2,:),c_ls(1,:),-c_ls(3,:),'b--','DisplayName','c_{ls}');
h_ax(3) = plot3(c_re(2,:),c_re(1,:),-c_re(3,:),'r--','DisplayName','c_{re}');
h_ax(4) = plot3(c_le(2,:),c_le(1,:),-c_le(3,:),'b--','DisplayName','c_{le}');
h_ax(5) = plot3([p_s(2),p_s(2) + R*sin(chi_s)],...
    [p_s(1),p_s(1) + R*cos(chi_s)],...
    -[p_s(3),p_s(3)],'k','LineWidth',linewidth,...
    'DisplayName','\chi_s');
h_ax(6) = plot3([p_e(2),p_e(2) + R*sin(chi_e)],...
    [p_e(1),p_e(1) + R*cos(chi_e)],...
    -[p_e(3),p_e(3)],'k','LineWidth',linewidth,...
    'DisplayName','\chi_e');
if dp.case == 1
    set(h_ax(1),'LineWidth',linewidth);
    set(h_ax(3),'LineWidth',linewidth);
elseif dp.case == 2
    set(h_ax(1),'LineWidth',linewidth);
    set(h_ax(4),'LineWidth',linewidth);
elseif dp.case == 3
    set(h_ax(2),'LineWidth',linewidth);
    set(h_ax(3),'LineWidth',linewidth);
else % dp.case == 4
    set(h_ax(2),'LineWidth',linewidth);
    set(h_ax(4),'LineWidth',linewidth);
end