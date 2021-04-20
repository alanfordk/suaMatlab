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
    states.Data(:,9)]);
xlabel('time(s)');
ylabel('attitude(deg)');
legend('Roll','Pitch','Yaw')
grid on;

%Plot angular rates
figure; plot(states.Time, [states.Data(:,10),...
    states.Data(:,11),...
    states.Data(:,12)]);
xlabel('time(s)');
ylabel('angular rates(rad/s)');
legend('p','q','r')
grid on;