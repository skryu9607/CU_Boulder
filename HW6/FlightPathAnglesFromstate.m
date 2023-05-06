function [flight_angles] = FlightPathAnglesFromstate(aircraft_state)
aircraft_state = as;
% v_g : va + wind_vel.
v = [as(7);as(8);as(9)];
R = RotationMatrix321(as(4:6));
V = R' * v + wind_inertial; % Inertial body vel by inertial frame and
% Inertial wind vel by inertial frame.
% Inertial whole vel by inertial frame.
v_g = norm(V);
gam = asin(V(3)/v_g);
chi = atan(V(2)/V(1));
flight_angles = [vg;chi;gam];

end

