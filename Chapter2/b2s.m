function [ rbs ] = b2s( alpha )
%B2S returns the body frame to stability frame DCM

% Compute the DCM
rbs = [cos(alpha) 0 sin(alpha);
       0 1 0;
       -sin(alpha) 0 cos(alpha)];
assert(all(all(abs(rbs*rbs' - eye(3)) <1e-12)), 'Assert Failed on b2s');
assert(abs(det(rbs) - 1) <= 1e-12);
end

