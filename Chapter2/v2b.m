function [ rvb ] = v2b(ypr)
%V2B returns the vehicle frame to body frame DCM

%getting data
psi = ypr(1);
theta = ypr(2);
phi = ypr(3);

%Trig stuff
cth = cos(theta);
sth = sin(theta);
cpsi = cos(psi);
spsi = sin(psi);
cphi = cos(phi);
sphi = sin(phi);

% Compute DCM
rvb = [cth*cpsi cth*spsi -sth;
       sphi*sth*cpsi-cphi*spsi sphi*sth*spsi+cphi*cpsi sphi*cth;
       cphi*sth*cpsi+sphi*spsi cphi*sth*spsi-sphi*cpsi cphi*cth];

assert(all(all(abs(rvb*rvb' - eye(3)) <1e-12)), 'Assert Failed on v2b');
assert(abs(det(rvb) - 1) <= 1e-12);
   
end

