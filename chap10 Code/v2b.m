function [ rvb ] = v2b( ypr )
%V2B returns the vehicle frame to body frame DCM
% Source: Beard, Mclain, Small Unmanned Aicraft, page 15

% Unpack inputs
phi = ypr(3);
th = ypr(2);
psi = ypr(1);
% Compute trig terms
cth = cos(th);
sth = sin(th);
cphi = cos(phi);
sphi = sin(phi);
cpsi = cos(psi);
spsi = sin(psi);
% Compute DCM
rvb = zeros(3,3);
rvb(1,1) = cth*cpsi;
rvb(1,2) = cth*spsi;
rvb(1,3) = -sth;
rvb(2,1) = sphi*sth*cpsi - cphi*spsi;
rvb(2,2) = sphi*sth*spsi + cphi*cpsi;
rvb(2,3) = sphi*cth;
rvb(3,1) = cphi*sth*cpsi + sphi*spsi;
rvb(3,2) = cphi*sth*spsi - sphi*cpsi;
rvb(3,3) = cphi*cth;
end