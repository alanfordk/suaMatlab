function calc_SS_matrices(x_trim, u_trim, Va_star, P)
%CALC_SS_MATRICES calculates some elements of the state space matrices
%Longitudinal state-space coefficients
q_star = x_trim(11);
% w_star = x_trim(6);
% alpha_star = 
% delta_e_star = 
% u_star = 
% X_w = 

%Lateral state-space coefficients
gamma = P.Jx * P.Jz - P.Jxz^2;
gamma_1 = (P.Jxz*(P.Jx - P.Jy + P.Jz))/gamma;
gamma_3 = P.Jz/gamma;
gamma_4 = P.Jxz/gamma;
C_P_P = gamma_3*P.C_ell_p + gamma_4*P.C_n_p;
L_p = gamma_1*q_star + P.rho*Va_star*P.S_wing*P.b^2/4*C_P_P;
fprintf('L_p = %g\n',L_p);
end

