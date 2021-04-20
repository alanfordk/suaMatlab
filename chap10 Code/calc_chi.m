function chi = calc_chi(x)
%CALC_CHI calculates the course angle
v_bwrti_b = x(4:6);
phi     = x(7);
theta   = x(8);
psi     = x(9);
ypr = [psi, theta, phi]';
R_v2b = v2b(ypr);
R_b2v = R_v2b';
v_bwrti_v = R_b2v * v_bwrti_b;
chi = atan2(v_bwrti_v(2),v_bwrti_v(1));
end

