function [ rsw ] = s2w( beta )
%S2W returns the stability frame to wind frame DCM

% Compute the DCM
rsw = [cos(beta) sin(beta) 0;
       -sin(beta) cos(beta) 0;
       0 0 1];
   
assert(all(all(abs(rsw*rsw' - eye(3)) <1e-12)), 'Assert Failed on s2w');
assert(abs(det(rsw) - 1) <= 1e-12);
end