%% High-rate sensor params
P.Ts_highrate = P.Ts;       %Sampling period of high-rate sensors (s)
P.sig_gyro = 0.13*pi/180;   %Standard deviation of gyro measurement (rad/s)
P.sig_accel = 0.0025*P.gravity; %Standard deviation of accel measurement (m/s^2)
P.sig_abspress = 0.01*1000;   %Standard deviation of absolute presssure sensor (Pa)
P.sig_diffpress = 0.002*1000; %Standard deviation of differential presssure sensor (Pa)
%% Low-rate sensor params
P.Ts_lowrate = 1;       %Sampling period of low-rate sensors (s)
P.sig_mag = 0.3*pi/180; %Standard deviation of magnetometer (rad)
P.declination = (11 + 25/60)*pi/180; %Magnetic declination for Logan, Utah (rad)
P.Ts_gps = P.Ts_lowrate;%sample rate of GPS in s
P.beta_gps = 1/1100;    %GPS gauss-markov process time constant (1/s)
P.sig_n_gps = 0.21;     %GPS north position standard deviation (m)
P.sig_e_gps = 0.21;     %GPS east position standard deviation (m)
P.sig_h_gps = 0.40;     %GPS down position standard deviation (m)
P.sig_Vg_gps = 0.05;    %GPS velocity standard deviation (m)
P.sig_course_gps = P.sig_Vg_gps/P.Va0; %Course angle standard deviation (rad)