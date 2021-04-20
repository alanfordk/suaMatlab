%Intermediate calcs
Va_buff = airdata.Data(:,1);
we = airdata.Data(:,5);
wn = airdata.Data(:,4);
psi = states.Data(:,9);
chi_calced = atan2(Va_buff.*sin(psi)+we, Va_buff.*cos(psi)+wn);

%% Plot guidance models vs. truth models
figure; plot(states_guidance.Time, states_guidance.Data(:,1));
hold all;
plot(states.Time, states.Data(:,1),'--r');
xlabel('time(s)');
ylabel('North Position(m)');
grid on;
legend('Guidance','Truth')
title(sprintf('North Position\n Guidance Models vs. Truth'))

figure; plot(states_guidance.Time, states_guidance.Data(:,2));
hold all;
plot(states.Time, states.Data(:,2),'--r');
xlabel('time(s)');
ylabel('East Position(m)');
grid on;
legend('Guidance','Truth')
title(sprintf('East Position\n Guidance Models vs. Truth'))

figure; plot(states_guidance.Data(:,2), states_guidance.Data(:,1));
hold all;
plot(states.Data(:,2), states.Data(:,1),'--r');
xlabel('East Position (m)');
ylabel('North Position (m)');
grid on;
axis equal
legend('Guidance','Truth')
title(sprintf('N-E Position\n Guidance Models vs. Truth'))

figure; plot(states_guidance.Time, states_guidance.Data(:,5));
hold all;
plot(states.Time, -states.Data(:,3),'--r');
xlabel('time(s)');
ylabel('Altitude(m)');
grid on;
legend('Guidance','Truth')
title(sprintf('Altitude\n Guidance Models vs. Truth'))

figure; plot(states_guidance.Time, states_guidance.Data(:,3));
hold all;
plot(states.Time, chi_calced,'--r');
xlabel('time(s)');
ylabel('Course Angle(rad)');
grid on;
legend('Guidance','Truth')
title(sprintf('Course Angle\n Guidance Models vs. Truth'))

figure; plot(states_guidance.Time, states_guidance.Data(:,7));
hold all;
plot(states.Time, Va_buff,'--r');
xlabel('time(s)');
ylabel('Airspeed(m/s)');
grid on;
legend('Guidance','Truth')
title(sprintf('Airspeed\n Guidance Models vs. Truth'))