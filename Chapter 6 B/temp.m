%% Design the roll controller
% Design the roll controller using the procedure demonstrated in
% lateralControlDesignExample.m.  For the design of the roll controller,
% you can assume the following:
% - The largest roll angle command you will issue is 60 degrees.
% - The aileron saturates at 45 degrees.
% After you are finished with your controller design, verify that you meet
% the phase margin and gain margin requirements by create bode plots of the
% OLTF. Analyze the closed-loop performance of the system by create bode
% plots of the CLTF and a step response plot.
close all
plot_lat_enable = false;
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
%% Verify that PM > 60 degrees and gain margin > 12 dB
G_p_dela = lat.a_phi2/(s + lat.a_phi1 + k_dphi*lat.a_phi2);
OLTF_phi = k_pphi * G_p_dela * 1/s;
if plot_lat_enable
    figure; bode(OLTF_phi); grid on;
    [gm, pm] = margin(OLTF_phi);
    title(sprintf('Roll OLTF\nPhase Margin = %.2f deg, Gain Margin = %.2f dB',...
        pm, 20*log10(gm)));
end
%% Analyze the closed loop response T_{\phi}
CLTF_phi_phi_c = k_pphi*lat.a_phi2...
    /(s^2 + (lat.a_phi1 + lat.a_phi2*k_dphi) * s + k_pphi*lat.a_phi2);
if plot_lat_enable
    [gpeak, fpeak] = getPeakGain(CLTF_phi_phi_c);
    figure; bode(CLTF_phi_phi_c); grid on;
    title(sprintf('Roll CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
        bandwidth(CLTF_phi_phi_c)/2/pi, gpeak, fpeak/2/pi));
    
    figure; step(CLTF_phi_phi_c);
    sinfo = stepinfo(CLTF_phi_phi_c);
    title(sprintf('Roll Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
        sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
end
%% Design the course hold controller
% Design the course controller to acheive 0 steady-state error to a
% constant commaned angle \chi_c. After you are finished with your
% controller design, verify that you meet the phase margin and gain margin
% requirements by create bode plots of the OLTF. Analyze the closed-loop
% performance of the system by create bode plots of the CLTF and a step
% response plot.
close all
%% Calculate the plant G_{\chi} for the course angle controller
V_g = Va;
G_chi = CLTF_phi_phi_c * P.gravity/V_g/s;
%% Temporarily set the course angle controller gains k_{p_{\chi}}=1 and k_{i_{\chi}}=0
k_pchi = 1;
k_ichi = 0;
c_chi = k_pchi + k_ichi/s;
%% Calculate the OLTF G_{\chi}H_{\chi}
OLTF_chi = c_chi * G_chi;
if plot_lat_enable
    [gm, pm] = margin(OLTF_chi);
    figure;
    bode(OLTF_chi);
    title(sprintf('Course OLTF\nPhase Margin = %.2f deg, nGain Margin = %.2f dB',...
        pm, 20*log10(gm)))
    hold all;
    grid on;
end
%% Tune the controller gains k_{p_{\chi}} and k_{i_{\chi}}
% Tune the value of k_{p_{\chi}} such that the phase margin is near and
% within spec. Tune the value of k_{i_{\chi}} until the phase margin
% starts to drop
k_pchi = db2mag(20);
k_ichi = 0.1;
c_chi = k_pchi + k_ichi/s;
OLTF_chi = c_chi * G_chi;
if plot_lat_enable
    bode(OLTF_chi);
    [gm, pm] = margin(OLTF_chi);
    title(sprintf('Course OLTF\nPhase Margin = %.2f deg, nGain Margin = %.2f dB',...
        pm, 20*log10(gm)))
    hold all;
    grid on;
end
%% Calculate the CLTF T_{\chi} and analyze the performance
CLTF_chi_chi_c = feedback(OLTF_chi,1);
if plot_lat_enable
    [gpeak, fpeak] = getPeakGain(CLTF_chi_chi_c);
    figure; bode(CLTF_chi_chi_c); grid on;
    title(sprintf('Course CLTF\nBandwidth = %.2f Hz, Peak Gain = %.2f at %.f Hz',...
        gpeak, gpeak))
end
if plot_lat_enable
    figure; step(CLTF_chi_chi_c);
    sinfo = stepinfo(CLTF_chi_chi_c);
    title(sprintf('Course Step\nPercent Overshoot = %.2f %%\n Rise Time = %.2f sec, Settling Time = %.2f sec',...
        sinfo.Overshoot, sinfo.RiseTime, sinfo.SettlingTime));
end