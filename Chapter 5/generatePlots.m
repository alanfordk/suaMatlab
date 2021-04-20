%Intermediate calcs
Va = airdata.Data(:,1);
we = airdata.Data(:,5);
wn = airdata.Data(:,4);
psi = states.Data(:,9);
chi = 180./pi.*atan2(Va.*sin(psi)+we, Va.*cos(psi)+wn);

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
    chi]);
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