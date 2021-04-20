function y_mag = magnetic(x, P)
%MAGNETIC Summary of this function goes here
%   Detailed explanation goes here

psi = x(9);
y_mag = psi - P.declination + P.sigma_mag * randn;

end

