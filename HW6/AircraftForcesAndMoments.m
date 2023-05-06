function [aircraft_forces,aircraft_moments] = AircraftForcesAndMoments(aircraft_state,aircraft_surfaces,wind_inertial, density,aircraft_parameters)
as = aircraft_state;
ap = aircraft_parameters;
% Creat the above function that takes as input the a/c state, the control
% input vector, the inertial wind in inertial coordinates, the air density,
% and the a/c parameters strucutre and returns the total force in body
% coordinates.
% The total force : aerodynamic, propulsive, weight.
% aerodynamic and propulsive moments.
angles = as(4:6);
R = RotationMatrix321(angles);
[aero_force, aero_moments] = AeroF_M_Body_wcoeffs(aircraft_state,aircraft_surfaces,wind_inertial, density,aircraft_parameters);
gravity = [0,0,ap.W]';
aircraft_forces = aero_force + R * gravity;
% MOMENTS : PROPULSIVE + AERODYNAMIC
aircraft_moments = aero_moments;

end

