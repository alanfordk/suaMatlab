function ax = draw_ground_trace(p_n, p_e, p_d)
%DRAW_GROUND_TRACE plots a vertical line from the location of the aircraft
%to the ground
ax = plot3([p_e, p_e], [p_n, p_n], [-p_d,0],'k-.');
end

