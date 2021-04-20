function y = takeoff_autopilot(theta_c_in, h, ...
    delta_t_in, P)
%TAKEOFF_AUTOPILOT computes the pitch and throttle commands during takeoff
if h < P.takeoff_altitude
    %If the altitude is below the takeoff altitude, set override the
    %commanded pitch and throttle
    delta_t = P.takeoff_throttle;
    theta_c = P.takeoff_pitch;
else
    %Pass through the commanded pitch and throttle
    delta_t = delta_t_in;
    theta_c = theta_c_in;
end
y = [theta_c, delta_t]';