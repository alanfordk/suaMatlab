function h_c_filtered = stateMachine(h_c, h, P)
%STATEMACHINE Implements the simple state machine described in Dr. Beards
%course notes
if (h_c > h + P.altitudeHoldZone) && P.stateMachineEnable
    h_c_filtered = h + P.altitudeHoldZone;
elseif (h_c < h - P.altitudeHoldZone) && P.stateMachineEnable
    h_c_filtered = h - P.altitudeHoldZone;
else
    h_c_filtered = h_c;
end
end

