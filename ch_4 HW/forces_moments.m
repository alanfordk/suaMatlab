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
    
    %TODO: Enter code here to compute all quantities in "out".  A few 
    %utility functions are provided below if you find them useful.

    ypr = [phi, theta, psi];
    rvb = v2b(ypr);
    V_wb = rvb*[w_ns;
                w_es;
                w_ds];
    V_wb = V_wb + [u_wg; v_wg; w_wg];

    V_ab = [u;
            v;
            w] - V_wb;

    Va = sqrt((V_ab(1)^2) + (V_ab(2)^2) + (V_ab(3)^2));
    alpha = (atan(V_ab(3)/V_ab(1)));
    beta = (asin(V_ab(2)/Va));
    
    b2v = transpose(rvb);
    wind_gust = b2v*[u_wg;
                     v_wg;
                     w_wg];
    wind = [w_ns;
            w_es;
            w_ds] + wind_gust;
   
%     wind_uvw = b2v*[u;
%                     v;
%                     w];
%     wind = wind_uvw - wind;
        
    w_n = wind(1);
    w_e = wind(2);
    w_d = wind(3);

    % Calculate Some C values
    Cx = -calc_C_D(alpha, P)*cos(alpha) + calc_C_L(alpha, P)*sin(alpha);
    Cx_q = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    Cx_delta_e = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    Cz = -calc_C_D(alpha, P)*sin(alpha) - calc_C_L(alpha, P)*cos(alpha);
    Cz_q = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    Cz_delta_e = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
                                      
    Force_one = [-P.mass*P.gravity*sin(theta);
             P.mass*P.gravity*cos(theta)*sin(phi);
             P.mass*P.gravity*cos(theta)*cos(phi)];
    
    Force_two = .5*P.rho*(Va^2)*P.S_wing * [Cx + Cx_q*P.c/(2*Va)*q + Cx_delta_e*delta_e;
                                            P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*P.b/(2*Va)*p + P.C_Y_r*P.b/(2*Va)*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r;
                                            Cz + Cz_q*P.c/(2*Va)*q + Cz_delta_e*delta_e];


    Force_three = .5*P.rho*P.S_prop*P.C_prop * [((P.k_motor*delta_t)^2) - (Va^2);
                                                0;
                                                0];
    Force = Force_one + Force_two + Force_three;
    
    Torque_one = .5*P.rho*(Va^2)*P.S_wing * [P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*P.b/(2*Va)*p + P.C_ell_r*P.b/(2*Va)*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r);
                                             P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*P.c/(2*Va)*q + P.C_m_delta_e*delta_e);
                                             P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*P.b/(2*Va)*p + P.C_n_r*P.b/(2*Va)*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r)];
                                      
    Torque_two = [-P.k_T_P*(P.k_Omega*delta_t)^2;
                  0;
                  0];
              
    Torque = Torque_one + Torque_two;

    % out = [Force, Torque, Va/Air Speed, Angle of Attack, Side Slip, Wind
    % North, East, Down]
%     disp("Force");
%     disp(Force');
%     disp("Torque");
%     disp(Torque');
%     disp("Va");
%     disp(Va);
%     disp("alpha");
%     disp(alpha);
%     disp("beta");
%     disp(beta);
%     disp("w_n");
%     disp(w_n);
%     disp("w_e");
%     disp(w_e);
%     disp("w_d");
%     disp(w_d);
    
%     airdata = [Va, alpha, beta];
%     out = [Force'; Torque'; airdata; wind'];
    
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];
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
