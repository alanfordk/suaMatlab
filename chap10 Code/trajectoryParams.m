%% Commanded trajectory
plottraj = 0;
V_a_c_in = timeseries([Va, Va]',[0, 60]');
beta_c_in = timeseries([0,0]',[0,60]');
if plottraj
    figure; plot(V_a_c_in); grid on; title('Commanded Airspeed')
end