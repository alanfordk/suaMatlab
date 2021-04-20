function y_gyro = gyro(x, P)
%GYRO Summary of this function goes here
%   Detailed explanation goes here

p = x(10);
q = x(11);
r = x(12);

y_gyro = [p;
          q;
          r] + randn(3,1)*P.sigma_gyro;


end

