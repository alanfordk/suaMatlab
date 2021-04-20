function [x_trim,u_trim] = compute_trim(filename, Va, gamma, R)
% Va is the desired airspeed (m/s)
% gamma is the desired flight path angle (radians)
% R is the desired radius (m) - use (+) for right handed orbit, 
%                                   (-) for left handed orbit
% See appendixe F.1 for details on the inputs to the 'trim' function

% Define the inputs to the trim function

x0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
u0 = [0; 0; 0; 1];
y0 = [Va; gamma; 0];
ix = [];
iu = [];
iy = [1,3];
dx0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R; 0; 0; 0];
idx = [3; 4; 5; 6; 7; 8; 9; 10; 11; 12];

% Compute trim conditions
[x_trim,u_trim,~,dx_trim] = trim(filename,x0,u0,y0,ix,iu,iy,dx0,idx);
% Check to make sure that the linearization worked (should be small)
fprintf('Trim converged to %g\n', norm(dx_trim(3:end)-dx0(3:end)));

