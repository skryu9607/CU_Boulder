% Homework 3 - Problem 3
% Trim conditions -  for the case of straight, wings-level flight.
% 1. A/C could be ascending or descending. 
% This is the only time in the course where we use the expression trim to
% include an equilibrium condition that could be ascending or descending.

% Because the forces and moments acting on an aircraft can be balanced in
% this case the aeronautics community often refers to it as a trim
% condition.

%% Problem 3 - 3) Coordinated Turn.
% Climb orbit h = 1655, va = 18
% gamm0 = 0.
% Zero background wind.
clc;
close all;clear all;
ttwistor
% Time condition
init = 0; final = 500; 
dt = 0.01;
n = (final-init) / dt + 1;
time =  linspace(init,final,n);

% Trim condition

xtd1 = [18,deg2rad(0),1655,-500];
wind_inertial = [0,0,0]';
% Try to implement things.
[as,acs] = coor_min_trim_fun(xtd1,aircraft_parameters);
aircraft_state_array = [as];
control_inputs_array = [acs];
for i = 1:n-1
    [xdot] = AC_EOM(time,as,acs,wind_inertial, aircraft_parameters);
    aircraft_new_state = as + dt * xdot;
    aircraft_state_array = [aircraft_state_array,aircraft_new_state];
    control_inputs_array = [control_inputs_array,acs];
    as = aircraft_new_state;
end
disp("The update is done")
col = 'b';
PlotSimulation(time, aircraft_state_array,control_inputs_array,col)
