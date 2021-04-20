%% Airspeed guidance model parameters
% Determine the parameters of the design model that will acheive <10%
% mismatch on the rise time, settling time, and bandwidth.
b_va = 2*pi*0.55;
cltf_vc_2_v_guidance = b_va/(s + b_va);
%% Altitude guidance model parameters
% Determine the parameters of the design model that will acheive <10%
% mismatch on the rise time, settling time, and bandwidth.
wn_altitude = 0.00007*2.1;
zeta_altitude = 1300.0;
b_hdot = 2*zeta_altitude*wn_altitude;
b_h = sqrt(wn_altitude);
cltf_hc_2_h_guidance = (b_hdot*s + b_h)/(s^2 + b_hdot * s + b_h);
%% Course angle guidance model parameters
% Determine the parameters of the design model that will acheive <10%
% mismatch on the rise time and bandwidth.
wn_chi = 2*pi*0.583/4;
zeta_chi = 2;
b_chidot = 2*zeta_chi*wn_chi;
b_chi = sqrt(wn_chi);
cltf_chic_2_chi_guidance = (b_chidot*s + b_chi)/(s^2 + b_chidot*s + b_chi);