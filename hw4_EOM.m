close all;clc;clear all;
ttwistor
aircraft_state =[100,200,1000,0.05,-0.02,2.0,15,-2,3,0.01,0.02,0.03]';
aircraft_surfaces = [-.4,.01,.02,.3]';global wind_inertial;
wind_inertial = [-1,-2,-3]';
density = 1.1;
time = 0;
[aero_force, aero_moments] = AeroF_M_Body_wcoeffs(aircraft_state,aircraft_surfaces,wind_inertial, density,aircraft_parameters)
[aircraft_forces,aircraft_moments] = AircraftForcesAndMoments(aircraft_state,aircraft_surfaces,wind_inertial, density,aircraft_parameters)
xdot = AC_EOM(time,aircraft_state,aircraft_surfaces,wind_inertial, aircraft_parameters)
% xdot = [−4.264, 14.478, 3.196, 0.009, 0.019, 0.031, 14.563, 1.835,
% −45.563, 5.240, −15.222, −3.064]T <- These are the answers.
% The only difference is Inertial pos.

