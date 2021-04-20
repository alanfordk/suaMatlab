function [ rbw ] = b2w( alpha, beta )
%B2W returns the body frame to wind frame DCM

rbw = [cos(beta)*cos(alpha) sin(beta) cos(beta)*sin(alpha);
       -sin(beta)*cos(alpha) cos(beta) -sin(beta)*sin(alpha);
       -sin(alpha) 0 cos(alpha)];
   
assert(all(all(abs(rbw*rbw' - eye(3)) <1e-12)), 'Assert Failed on b2w');
assert(abs(det(rbw) - 1) <= 1e-12);
end

