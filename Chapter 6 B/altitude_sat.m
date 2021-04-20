function [h_c_tilde] = altitude_sat(h, h_c, P)
%ALTITUDE_SAT saturates the altitude
if h_c > (h+P.altitudeHoldZone) && P.stateMachineEnable
    h_c_tilde = h + P.altitudeHoldZone;
elseif h_c < (h-P.altitudeHoldZone) && P.stateMachineEnable
    h_c_tilde = h - P.altitudeHoldZone;
else
    h_c_tilde = h_c;

% disp('h_c_tilde');
% disp(h_c_tilde);
% disp(class(h_c_tilde));
end

