% Author: Randy Christensen
% Date: 11-Mar-2019 23:07:46
% Reference: 
% Copyright 2018 Utah State University
%% Prelims
clearvars
close all
clc
%% UAV position
p_i = [50,10,-120]';
%% Line to follow
r_i = [0,0,-100]';
th = 0*pi/180;
q_i = [cos(th),0,-sin(th)]';
%% Common calcs
k_i = [0,0,1]';
n_i = cross(k_i,q_i)/norm(cross(k_i,q_i));
e_p_i = p_i - r_i;
%% My calcs
beta = e_p_i'*n_i
A = [       1, k_i'*q_i;
     q_i'*k_i,        1];
y = [e_p_i'*q_i, e_p_i'*k_i]';
x = A\y;
alpha = x(1)
gamma = x(2)
h_d = -(r_i + alpha*q_i)'*k_i
%% Book calcs
r_d = r_i(3);
q_n = q_i(1);
q_e = q_i(2);
q_d = q_i(3);
chi_q = atan2(q_e,q_n);
R_i_p = [cos(chi_q), sin(chi_q), 0;
        -sin(chi_q), cos(chi_q), 0;
                  0,          0, 1];
s_i = e_p_i - (e_p_i'*n_i)*n_i;
s_n = s_i(1);
s_e = s_i(2);
s_d = s_i(3);
h_d = -r_d - sqrt(s_n^2 + s_e^2)*q_d/sqrt(q_n^2 + q_e^2)