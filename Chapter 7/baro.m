function y_abs_press = baro(x, P)
%BARO Summary of this function goes here
%   Detailed explanation goes here


h = -x(3);
y_abs_press = P.rho * P.gravity * h + randn * P.press_sigma_abs + randn * P.press_beta_abs;

end

