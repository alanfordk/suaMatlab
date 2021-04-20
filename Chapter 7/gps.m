function y = gps(x, time, P)
%GPS Summary of this function goes here
%   Detailed explanation goes here

persistent nu_n;
persistent nu_e;
persistent nu_d;


if time==0
    nu_n = 0;
    nu_e = 0;
    nu_d = 0;
else
    nu_n = exp(-P.K_gps*P.Ts_gps)*nu_n + randn*P.sigma_n_gps;
    nu_e = exp(-P.K_gps*P.Ts_gps)*nu_e + randn*P.sigma_e_gps;
    nu_d = exp(-P.K_gps*P.Ts_gps)*nu_d + randn*P.sigma_d_gps;
end
y_gps_n = x(1) + nu_n;
y_gps_e = x(2) + nu_e;
y_gps_d = -x(3) + nu_d;

y_gps = [y_gps_n, y_gps_e, y_gps_d]';

% Synthesize groundspeed and course measurements
v_b = [x(4), x(5), x(6)]';
ypr = [x(9), x(8), x(7)]';
b2v = v2b(ypr);
v_v = b2v*v_b;
V_g = norm(v_v(1:2));
chi = calc_chi(x);
eta_V = P.sigma_n_gps * randn;
sigma_chi = P.sigma_velocity/V_g;
eta_chi = sigma_chi * randn;
y_gps_V_g = V_g + eta_V;
y_gps_chi = chi + eta_chi;

y = [y_gps; y_gps_V_g; y_gps_chi];

end

