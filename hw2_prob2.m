% Problem 2 of Homework 2
% Plot and Describe the results. When describing the results use
% quantitative terms as much as possible. 
% To let the simulations run long enough to see the behavior reach steady
% stae. 
clc;
ttwistor;
init = 0; final = 150; n = 15001;
time =  linspace(init,final,n);
dt = (final-init) / (n-1);

aircraft_state = [0,0,-1800,deg2rad(15),deg2rad(-12),deg2rad(270),19,3,-2,deg2rad(0.08),deg2rad(-0.2),deg2rad(0)]';
aircraft_surfaces = [deg2rad(5),deg2rad(2),deg2rad(-13),.3]';
wind_inertial = [0,0,0]';
% aircraft_state = [0;0;-1665;0;0;0;18;0;0;0;0;0]
%aircraft_surfaces = [0;0;0;0]
% %wind_inertial = [0,0,0]'
% wind_inertial = [10,10,0]'
aircraft_state_array = [aircraft_state];
control_inputs_array = [aircraft_surfaces];
for i = 1:n-1
    [xdot] = AC_EOM(time,aircraft_state,aircraft_surfaces,wind_inertial, aircraft_parameters);
    aircraft_new_state = aircraft_state + dt*xdot;
    aircraft_state_array = [aircraft_state_array,aircraft_new_state];
    control_inputs_array = [control_inputs_array,aircraft_surfaces];
    aircraft_state =  aircraft_new_state;
end
disp("The update is done")
col = 'b';
PlotSimulation(time, aircraft_state_array,control_inputs_array,col)

