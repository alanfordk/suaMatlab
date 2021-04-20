% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)
    % relabel the inputs
    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % prelims
    ypr = [psi, theta, phi]';
    rvb = v2b(ypr);
    rbv = rvb';
    % compute wind data in NED
    w_steady_v = [w_ns, w_es, w_ds]';
    w_gust_b = [u_wg, v_wg, w_wg]';
    V_w_b = rvb * w_steady_v + w_gust_b; %equation 4.4.8
    V_w_v = rbv * V_w_b;
    V_g_b = [u, v, w]';
    V_a_b = V_g_b - V_w_b; %equation 2.6
    u_r = V_a_b(1);
    v_r = V_a_b(2);
    w_r = V_a_b(3);
    % compute air data per equation 2.8
    Va = norm(V_a_b);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/norm(V_a_b));
    % compute force of gravity in the body frame
    f_g_v = [0, 0 ,P.mass*P.gravity]';
    f_g_b = rvb * f_g_v;
    % compute aerodynamic force coefficients per equation 4.19
    C_L = calc_C_L(alpha, P);
    C_D = calc_C_D(alpha, P);
    C_X = -C_D*cos(alpha) + C_L*sin(alpha);
    C_Xq = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    C_Xdeltae = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e * sin(alpha);
    C_Z = -C_D*sin(alpha) - C_L*cos(alpha);
    C_Zq = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    C_Zdeltae = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
    % compute the aerodynamic force per equation 4.18
    C_aeroforce_x = C_X + C_Xq*P.c/2/Va*q + C_Xdeltae*delta_e;
    C_aeroforce_y = P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/2/Va*p ...
        + P.C_Y_r*P.b/2/Va*r + P.C_Y_delta_a*delta_a ...
        + P.C_Y_delta_r*delta_r;
    C_aeroforce_z = C_Z + C_Zq*P.c/2/Va*q + C_Zdeltae*delta_e;
    f_a_b = 0.5*P.rho*Va^2*P.S_wing*[C_aeroforce_x;
                                     C_aeroforce_y;
                                     C_aeroforce_z];
    % compute propulsion forces in the body frame
    f_prop = 0.5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2 - Va^2); %Equation 4.3.1.4
    f_p_b = [f_prop, 0, 0]';
    % compute aerodyamic moments in the body frame per equation 4.20
    C_aeromoment_x = P.b*(P.C_ell_0 + P.C_ell_beta*beta ...
        + P.C_ell_p*P.b/2/Va*p + P.C_ell_r*P.b/2/Va*r ...
        + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
    C_aeromoment_y = P.c*(P.C_m_0 + P.C_m_alpha*alpha ...
        + P.C_m_q*P.c/2/Va*q + P.C_m_delta_e*delta_e);
    C_aeromoment_z = P.b*(P.C_n_0 + P.C_n_beta*beta ...
        + P.C_n_p*P.b/2/Va*p + P.C_n_r*P.b/2/Va*r ...
        + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
    m_a_b = 0.5*P.rho*Va^2*P.S_wing*[C_aeromoment_x;
                                     C_aeromoment_y;
                                     C_aeromoment_z];
    % compute propulsion moments in the body frame
    m_prop = -P.k_T_P*(P.k_Omega*delta_t)^2; %Equation 4.3.2.1
    m_p_b = [m_prop, 0, 0]';
    % compute external forces and torques on the aircraft in the body frame
    f_b = f_g_b + f_a_b + f_p_b;
    m_b = m_a_b + m_p_b;
    % package up results
    Force(1) =  f_b(1);
    Force(2) =  f_b(2);
    Force(3) =  f_b(3);
    Torque(1) = m_b(1);
    Torque(2) = m_b(2);
    Torque(3) = m_b(3);
    w_n = V_w_v(1);
    w_e = V_w_v(2);
    w_d = V_w_v(3);
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end

function out = calc_C_L(alpha, P)
%Equation 4.9
sig_alpha = sig(alpha, P);
out = (1-sig_alpha)*(P.C_L_0 + P.C_L_alpha*alpha) ...
    + sig_alpha*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
end

function out = sig(alpha, P)
%Equation 4.10
temp1 = exp(-P.M*(alpha - P.alpha0));
temp2 = exp( P.M*(alpha + P.alpha0));
out = (1 + temp1 + temp2)/((1 + temp1)*(1 + temp2));
end

function out = calc_C_D(alpha, P)
%Equation 4.11
AR = P.b^2/P.S_wing;
out = P.C_D_p + (P.C_L_0 + P.C_L_alpha*alpha)^2 ...
    /(pi*P.e*AR);
end
