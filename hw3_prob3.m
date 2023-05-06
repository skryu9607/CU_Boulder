% Homework 3 - Problem 3
% Trim conditions -  for the case of straight, wings-level flight.
% 1. A/C could be ascending or descending. 
% This is the only time in the course where we use the expression trim to
% include an equilibrium condition that could be ascending or descending.

% Because the forces and moments acting on an aircraft can be balanced in
% this case the aeronautics community often refers to it as a trim
% condition.

%% Problem 3 - 1) SLUF
% Straight, wings-level flight with height h = 1655, va = 18
% gamm0 = 0.
% Zero background wind.
clc;
close all;clear all;
ttwistor;
% Time condition
init = 0; final = 100; n = 15001;
time =  linspace(init,final,n);
dt = (final-init) / (n-1);
% Trim condition
xtd0 = [18,deg2rad(0),1655]';
global wind_inertial
wind_inertial = [10,10,0]';
%[as,acs] = Trim_Expression(xtd0,xtv0);
% Try to implement things.
[as,acs] = min_trim_fun(xtd0,aircraft_parameters);
aircraft_state_array = [as];
as(7:9)
control_inputs_array = [acs];
Va = [xtd0(1)];

for i = 1:n-1
    [xdot] = AC_EOM(time,as,acs,wind_inertial, aircraft_parameters);
    aircraft_new_state = as + dt * xdot;
    aircraft_state_array = [aircraft_state_array,aircraft_new_state];
    control_inputs_array = [control_inputs_array,acs];
    as = aircraft_new_state;
    va_new = sqrt(sum(as(7:9).^2));
    Va = [Va,va_new];
    %[as,acs] = min_trim_fun(xtd0,aircraft_parameters);
end

disp("The update is done")
col = 'b';
PlotSimulation(time, aircraft_state_array,control_inputs_array,col)
figure;
plot(time,Va);
