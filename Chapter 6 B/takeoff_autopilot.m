function output = takeoff_autopilot(theta_c_in, h, delta_t_in, P)
%STATE_MACHINE Summary of this function goes here
%   Detailed explanation goes here

if h < P.takeoff_altitude
    theta_c = P.takeoff_pitch;
    delta_t = P.takeoff_throttle;
else
    theta_c = theta_c_in;
    delta_t = delta_t_in;
end
output = [theta_c, delta_t];
% disp('theta_c');
% disp(theta_c);
% disp(class(theta_c));
end

