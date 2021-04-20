if ~skipStatePlots
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
        -states.Data(:,6)]);
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
end
%% Plot path follower performance
figure; plot(crosstrack_error.Time, crosstrack_error.Data,...
    h.Time, h_c.Data-h.Data);
hold on;
plot([0, tsim],[10,10],'k--')
plot([0, tsim],[-10,-10],'k--')
xlabel('time(s)')
ylabel('relative path error(m)')
legend('cross-track','altitude','overshoot requirement','Location','best')
grid on;

%Plot NED position vs. commanded path
figure; plot3(states.Data(:,2), states.Data(:,1), -states.Data(:,3));
hold on;
if strcmp(path_struct.type,'lin')
    path_start = path_struct.origin;
    path_end = path_start + path_struct.direction * norm(states.Data(end,:));
    plot3([path_start(2) path_end(2)],...
        [path_start(1) path_end(1)],...
        [-path_start(3) -path_end(3)],'r--')
elseif strcmp(path_struct.type,'orb')
    lambda = path_struct.orbit_direction;
    rho = path_struct.radius;
    c = path_struct.center;
    phi_temp = 0:0.01:2*pi;
    circle = lambda.*rho.*[cos(phi_temp);
        sin(phi_temp);
        zeros(1,length(phi_temp))];
    r = repmat(c,1,length(phi)) + circle;
    plot3(r(2,:), r(1,:), -r(3,:),'r--');
end
xlabel('East Position (m)');
ylabel('North Position (m)');
zlabel('Altitude (m)');
grid on;
axis equal
legend('actual path', 'desired path');