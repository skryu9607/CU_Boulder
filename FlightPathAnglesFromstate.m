function [flight_angles] = FlightPathAnglesFromstate(aircraft_state,wind_inertial)
as = aircraft_state;
% v_g : va + wind_vel.
v = [as(7);as(8);as(9)];
R = RotationMatrix321(as(4:6)); % Inertial -> Body
V = R' * v + wind_inertial; % Inertial body vel by inertial frame and
% Inertial wind vel by inertial frame.
% Inertial whole vel by inertial frame.
vg = norm(V);
gam = asin(V(3)/vg);
chi = acos(V(1)/vg*cos(gam));
flight_angles = [vg;chi;gam];

end

