%% Commanded trajectory
plottraj = 0;
chi_c_in = timeseries([0,0,2,2,120,120]*pi/180,[0,10,10,20,40,100]);
h_c_in = timeseries([h0, h0, h0 - 50, h0 - 50],[0, 10, 40, 100]);
V_a_c_in = timeseries([Va, Va, Va*0.90],[0, 80, 100]);
beta_c_in = timeseries([0,0],[0,100]);
if plottraj
    figure; plot(chi_c_in*180/pi); grid on; title('Commanded Course Angle')
    figure; plot(h_c_in); grid on; title('Commanded Altitude')
    figure; plot(V_a_c_in); grid on; title('Commanded Airspeed')
end