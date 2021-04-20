%% Commanded trajectory
plottraj = 0;
chi_c_in = timeseries([0,0,120,120]*pi/180,[0,20,40,60]);
h_c_in = timeseries([h0, h0+200],[0, 60]);
V_a_c_in = timeseries([Va, Va],[0, 60]);
beta_c_in = timeseries([0,0],[0,60]);
if plottraj
    figure; plot(chi_c_in*180/pi); grid on; title('Commanded Course Angle')
    figure; plot(h_c_in); grid on; title('Commanded Altitude')
    figure; plot(V_a_c_in); grid on; title('Commanded Airspeed')
end