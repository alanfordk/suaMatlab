%Intermediate calcs
Va_buff = airdata.Data(:,1);
we = airdata.Data(:,5);
wn = airdata.Data(:,4);
psi = states.Data(:,9);
chi_calced = atan2(Va_buff.*sin(psi)+we, Va_buff.*cos(psi)+wn);

%Plot position vs. time
figure; plot(states.Time, [states.Data(:,1),...
    states.Data(:,2),...
    -states.Data(:,3)]);
xlabel('time(s)');
ylabel('position(m)');
legend('North','East','Altitude')
grid on;

%Plot velocity vs. time
figure; plot(states.Time, [states.Data(:,4),...
    states.Data(:,5),...
    states.Data(:,6)]);
xlabel('time(s)');
ylabel('velocity(m/s)');
legend('u','v','w')
grid on;

%Plot attitude
figure; plot(states.Time, 180/pi*[states.Data(:,7),...
    states.Data(:,8),...
    states.Data(:,9),...
    chi_calced]);
xlabel('time(s)');
ylabel('attitude(deg)');
legend('Roll','Pitch','Yaw','\chi')
grid on;

%Plot angular rates
figure; plot(states.Time, [states.Data(:,10),...
    states.Data(:,11),...
    states.Data(:,12)]);
xlabel('time(s)');
ylabel('angular rates(rad/s)');
legend('p','q','r')
grid on;

%Plot N vs E position
figure; plot(states.Data(:,2), states.Data(:,1));
xlabel('East Position (m)');
ylabel('North Position (m)');
grid on;
axis equal

%Plot NED position
figure; plot3(states.Data(:,2), states.Data(:,1), -states.Data(:,3));
xlabel('East Position (m)');
ylabel('North Position (m)');
zlabel('Altitude (m)');
grid on;
axis equal

%Plot control surfaces
figure; plot(delta.Time, delta.Data(:,1)*180/pi);
xlabel('time(s)')
ylabel('elevator(deg)')
grid on;

figure; plot(delta.Time, delta.Data(:,2)*180/pi);
xlabel('time(s)')
ylabel('aileron(deg)')
grid on;

figure; plot(delta.Time, delta.Data(:,3)*180/pi);
xlabel('time(s)')
ylabel('rudder(deg)')
grid on;

figure; plot(delta.Time, delta.Data(:,4));
xlabel('time(s)')
ylabel('throttle(unitless)')
grid on;

%Plot airspeed data
figure; plot(airdata.Time, airdata.Data(:,1));
xlabel('time(s)')
ylabel('airspeed(m/s)')
grid on;

figure; plot(airdata.Time, airdata.Data(:,2)*180/pi);
xlabel('time(s)')
ylabel('angle of attack(deg)')
grid on;

figure; plot(airdata.Time, airdata.Data(:,3)*180/pi);
xlabel('time(s)')
ylabel('side slip(deg)')
grid on;

%Plot controller errors
figure; plot(chi.Time, chi_c.Data*180/pi); hold all;
plot(chi.Time, chi.Data*180/pi,'--');
xlabel('time(s)')
ylabel('course angle(deg)')
legend('commanded', 'actual');
grid on;

figure; plot(chi.Time, (chi_c.Data-chi.Data)*180/pi);
xlabel('time(s)')
ylabel('course angle error(deg)')
grid on;

figure; plot(phi.Time, phi_c.Data*180/pi); hold all;
plot(phi.Time, phi.Data*180/pi,'--')
xlabel('time(s)')
ylabel('roll angle(deg)')
legend('commanded', 'actual');
grid on;

figure; plot(phi.Time, (phi_c.Data-phi.Data)*180/pi)
xlabel('time(s)')
ylabel('roll angle error(deg)')
grid on;

figure; plot(h_c.Time, h_c.Data); hold all;
plot(h.Time, h.Data,'--')
xlabel('time(s)')
ylabel('altitude(m)')
legend('commanded', 'actual');
grid on;

figure; plot(h.Time, h_c.Data-h.Data)
xlabel('time(s)')
ylabel('altitude error(m)')
grid on;

figure; plot(theta.Time, theta_c.Data*180/pi); hold all;
plot(theta.Time, theta.Data*180/pi,'--')
xlabel('time(s)')
ylabel('pitch angle(deg)')
legend('commanded', 'actual');
grid on;

figure; plot(theta.Time, (theta_c.Data-theta.Data)*180/pi)
xlabel('time(s)')
ylabel('pitch angle error(deg)')
grid on;

figure; plot(Vbar_a_c.Time, Vbar_a_c.Data); hold all;
plot(Vbar_a.Time, Vbar_a.Data,'--')
xlabel('time(s)')
ylabel('airspeed(m/s)')
legend('commanded', 'actual');
grid on;

figure; plot(Vbar_a.Time, Vbar_a_c.Data-Vbar_a.Data)
xlabel('time(s)')
ylabel('airspeed error(m/s)')
grid on;

%% Plots sensor measurements
%Calculate some truth values
V_g_true = zeros(length(states.Time),1);
chi_true = zeros(length(states.Time),1);
nu_true = zeros(3,length(states.Time));
for i=1:length(states.Time)
    rvb = v2b(states.Data(i,[9,8,7]));
    rbv = rvb';
    f_b = forces_moments_logged.Data(i,1:3)';
    nu_true(:,i) = 1/P.mass * f_b - rvb*[0, 0, P.gravity]';
    v_b = states.Data(i,[4,5,6])';
    v_v = rbv * v_b;
    V_g_true(i) = norm(v_v(1:2));
    chi_true(i) = atan2(v_v(2),v_v(1));
end

figure; stairs(high_rate_sensors.accel.Time, high_rate_sensors.accel.Data);
hold all;
plot(forces_moments_logged.Time, nu_true','--');
xlabel('time(s)')
ylabel('m/s^2')
legend('x_b','y_b','z_b','\nu_x','\nu_y','\nu_z')
grid on;
title('Accelerometer Measurements')

figure; stairs(high_rate_sensors.gyro.Time, high_rate_sensors.gyro.Data);
hold all;
plot(states.Time, states.Data(:,10:12),'--')
xlabel('time(s)')
ylabel('rad/s')
legend('x_b','y_b','z_b','p','q','r')
grid on;
title('Gyro Measurements')

figure; stairs(high_rate_sensors.airspeed.Time, ...
    high_rate_sensors.airspeed.Data);
hold all;
plot(airdata.Time, P.rho*airdata.Data(:,1).^2/2,'--r')
xlabel('time(s)')
ylabel('Pa')
legend('measured','true')
grid on;
title('Airspeed Measurements')

figure; stairs(high_rate_sensors.baro.Time, high_rate_sensors.baro.Data);
hold all;
plot(airdata.Time, -P.rho*P.gravity*states.Data(:,3),'--r')
xlabel('time(s)')
ylabel('P_{ground} - P (Pa)')
legend('measured','true')
grid on;
title('Barometric Altimeter Measurements')

figure; stairs(low_rate_sensors.gps.Time, low_rate_sensors.gps.Data(:,1:3));
hold all;
plot(states.Time, [states.Data(:,1:2), -states.Data(:,3)],'--');
xlabel('time(s)')
ylabel('m')
legend('North','East','Altitude','North_{true}','East_{true}',...
    'Altitude_{true}')
grid on;
title('GPS Position Measurements')


i_gps_match = states.Time == 0;
for j = 1:length(low_rate_sensors.gps.Time)
    i_gps_match = i_gps_match | states.Time == low_rate_sensors.gps.Time(j);
end
figure; stairs(low_rate_sensors.gps.Time, ...
    low_rate_sensors.gps.Data(:,1:3) ...
    - [states.Data(i_gps_match,1:2), -states.Data(i_gps_match,3)]);
hold all;
xlabel('time(s)')
ylabel('m')
legend('North','East','Altitude')
grid on;
title('GPS Position Gauss Markov Noise')

figure; stairs(low_rate_sensors.gps.Time, low_rate_sensors.gps.Data(:,4));
hold all;
plot(states.Time, V_g_true,'--r')
xlabel('time(s)')
ylabel('m/s')
legend('measured','true')
grid on;
title('GPS Ground Speed Measurements')

figure; stairs(low_rate_sensors.gps.Time, ...
    low_rate_sensors.gps.Data(:,5)*180/pi);
hold all;
plot(states.Time, chi_true*180/pi, '--r')
xlabel('time(s)')
ylabel('deg')
legend('measured','true')
grid on;
title('GPS Course Angle Measurements')