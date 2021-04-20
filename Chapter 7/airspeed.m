function y_diff_press = airspeed(x,P)
%AIRSPEED Summary of this function goes here
%   Detailed explanation goes here

Va = x(1);

y_diff_press = P.rho * Va^2/2 + randn*P.press_sigma_diff;

end

