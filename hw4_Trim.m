close all;clc;clear all;
ttwistor
xtd0 = [18,deg2rad(0),1655]';
global wind_inertial
wind_inertial = [0,0,0]';
% % Try to implement things.
% [xtv,f] = min_trim_fun_1(xtd0,aircraft_parameters)
% xtd1 = [20,deg2rad(0),200,-500]'
% [xtv,f] = coor_min_trim_fun_1(xtd1,aircraft_parameters)
% %xtv = [0.0242,0.1981,0.2057,0.0816,0.0019,0.0133,-0.0091];

%wind_inertial = [-1,-2,-3]';
xtd1 = [18,deg2rad(0),1655,-500]'
% xtv = [alp0,de0,dt0,phi0,beta0,da0,dr0]' 
[xtv,f] = coor_min_trim_fun_1(xtd1,aircraft_parameters)
%trim variables = [0.0504, -0.5125,
%0.1776, -0.0661, -0.0017, -0.0167, 0.0237].
