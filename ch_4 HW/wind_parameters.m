% wind parameters
gustEnable = 0;
P.L_u = 200.0;
P.L_v = 200.0;
P.L_w = 50.0;
P.sigma_u = 1.06*gustEnable;
P.sigma_v = 1.06*gustEnable;
P.sigma_w = 0.7*gustEnable;
P.wind_n = 0;
P.wind_e = 0;
P.wind_d = 0;
V_g_b = [P.u0, P.v0, P.w0]';
ypr = [P.psi0, P.theta0, P.phi0]';
V_w_v = [P.wind_n, P.wind_e, P.wind_d]';
V_a_b = V_g_b - v2b(ypr) * V_w_v;
P.Va0 = V_a_b(1); % reference airspeed for gust model