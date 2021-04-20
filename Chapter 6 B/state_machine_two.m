function [delta_t] = state_machine_two(h, delta_t_in, P)
%STATE_MACHINE Summary of this function goes here
%   Detailed explanation goes here

if h < P.takeoff_altitude
    delta_t = delta_t_in;

else
    delta_t = 1;
end
% disp('delta_t');
% disp(delta_t);
% disp(class(delta_t));
end

