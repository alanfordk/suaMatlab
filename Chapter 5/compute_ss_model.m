function [A_lon,B_lon,A_lat,B_lat] = compute_ss_model(filename,x_trim,u_trim)
% x_trim is the trimmed state,
% u_trim is the trimmed input
% Use the linmod function to extract the linearized state space system from
% the Simulink model

% See appendix F.3 in book, or see section 5.5

[A,B,C,D] = linmod(filename,x_trim,u_trim);

E1 = zeros(5,12);
E1(1,5) = 1;
E1(2,10) = 1;
E1(3,12) = 1;
E1(4,6) = 1;
E1(5,8) = 1;

E2 = zeros(2,4);
E2(1,2)=1;
E2(2,3)=1;

E3 = zeros(5,12);
E3(1,4) = 1;
E3(2,6) = 1;
E3(3,11) = 1;
E3(4,8) = 1;
E3(5,3) = -1;

E4 = zeros(2,4);
E4(1,1) = 1;
E4(2,4) = 1;


A_lat = E1*A*E1';
B_lat = E1*B*E2';
A_lon = E3*A*E3';
B_lon = E3*B*E4';

% A_lon = [5 5 5 5 5;
%          5 5 5 5 5;
%          5 5 5 5 5;
%          5 5 5 5 5;
%          5 5 5 5 5];
% B_lon = [5 5;
%          5 5;
%          5 5;
%          5 5;
%          5 5];