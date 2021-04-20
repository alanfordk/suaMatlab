%% Choose k_{p_{\phi}} based on saturation of the ailerons and roll error
e_phi_max = 60*pi/180;   % Maximum roll error
delta_a_max = 45*pi/180; % Max aileron angle
k_pphi = delta_a_max/e_phi_max*sign(lat.a_phi2)*db2mag(-1.74);
%% Calculate the natural frequency \omega_{n_{\phi}}of the roll control loop 
omega_nphi = sqrt(abs(lat.a_phi2)*delta_a_max/e_phi_max); %Roll bandwidth
%% Choose the damping ratio \zeta_{\phi} of the roll control loop
zeta_phi = 0.8;          % Roll damping ratio
%% Calculate k_{d_{\phi}}
k_dphi = (2*zeta_phi*omega_nphi - lat.a_phi1)/lat.a_phi2;
k_pchi = db2mag(19.4);
k_ichi = 0.5;