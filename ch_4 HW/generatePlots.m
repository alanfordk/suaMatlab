scale = 1.0;
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
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot velocity vs. time
figure; plot(states.Time, [states.Data(:,4),...
    states.Data(:,5),...
    states.Data(:,6)]);
xlabel('time(s)');
ylabel('velocity(m/s)');
legend('u','v','w')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot attitude
figure; plot(states.Time, 180/pi*[states.Data(:,7),...
    states.Data(:,8),...
    states.Data(:,9)]);
hold all;
plot(states.Time,...
    chi_calced*180/pi,'--');
xlabel('time(s)');
ylabel('attitude(deg)');
legend('Roll','Pitch','Yaw','\chi')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot angular rates
figure; plot(states.Time, [states.Data(:,10),...
    states.Data(:,11),...
    states.Data(:,12)]);
xlabel('time(s)');
ylabel('angular rates(rad/s)');
legend('p','q','r')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot N vs E position
figure; plot(states.Data(:,2), states.Data(:,1));
xlabel('East Position (m)');
ylabel('North Position (m)');
grid on;
axis equal
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot NED position
figure; plot3(states.Data(:,2), states.Data(:,1), -states.Data(:,3));
xlabel('East Position (m)');
ylabel('North Position (m)');
zlabel('Altitude (m)');
grid on;
axis equal
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot control surfaces
figure; plot(delta.Time, delta.Data(1,:)*180/pi);
xlabel('time(s)')
ylabel('elevator(deg)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

figure; plot(delta.Time, delta.Data(2,:)*180/pi);
xlabel('time(s)')
ylabel('aileron(deg)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

figure; plot(delta.Time, delta.Data(3,:)*180/pi);
xlabel('time(s)')
ylabel('rudder(deg)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

figure; plot(delta.Time, delta.Data(4,:));
xlabel('time(s)')
ylabel('throttle(unitless)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

%Plot airspeed data
figure; plot(airdata.Time, airdata.Data(:,1));
xlabel('time(s)')
ylabel('airspeed(m/s)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

figure; plot(airdata.Time, airdata.Data(:,2)*180/pi);
xlabel('time(s)')
ylabel('angle of attack(deg)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])

figure; plot(airdata.Time, airdata.Data(:,3)*180/pi);
xlabel('time(s)')
ylabel('side slip(deg)')
grid on;
set(gcf,'Position',[100, 100, 560*scale, 420*scale])