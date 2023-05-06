%% Problem 1. va = 18 m/s, gamma = 0, h = 1800 m.
% The perturbations
% x_lon = [u_/va_t, alp_t, q_, tha_, h_]';
% x_lat = [beta_, p_, r_, phi_, psi_]';
% u_lon = [de_,dt_]';
% u_lat = [da_,dr_]';
% The result of homework 3.
clc;
close all;clear all;
run("ttwistor.m")
ap = aircraft_parameters;
trim_defi =[18,deg2rad(0),1800];
va_t = trim_defi(1);
h_t = trim_defi(3);
[A_lon,B_lon] = ACLinearLonModels(trim_defi,aircraft_parameters)
[V_lon,D_lon] = eig(A_lon(1:5,1:5))
% np = 2;imp_start = [0,2]; imp_end = [0.5,2.5];amp = 10;
% U = ImpAndDblet(t,np,imp_start,imp_end,amp);
% x_lon = [u_/va_t, alp_t, q_, tha_, h_]';

% Impulse
t = [0 150];
x0 = [0.4729,-0.0003,0.0195,-0.0002,-0.8774,10,0]';
[t,x] = ode45(@(t,x) odeFun(x,A_lon,B_lon),t,x0);

% % Doublet Impulse
% amp = 10;
% x0 = [0.4729,-0.0003,0.0195,-0.0002,-0.8774,amp,0]';
% T1 = [0,10];T2 = [10 150];
% [t1,x_bef] = ode45(@(t,x) odeFun(x,A_lon,B_lon),T1,x0);
% x0_after = x_bef(end,:)' + [0,0,0,0,0,-amp,0]';
% [t2,x_aft] = ode45(@(t,x) odeFun(x,A_lon,B_lon),T2,x0_after);
% t = [t1;t2];x = [x_bef;x_aft];
%%
figure
plot(t,x(:,1)/va_t,t,x(:,2),t,x(:,3)*ap.c/(2*va_t),t,x(:,4),t,x(:,5)/h_t)
grid on;
title("Longitudinal Motion, STATE Vector")
xlabel("Time[s]")
legend('u','alp','q','thata','h')
%%
figure
plot(t,x(:,6),t,x(:,7));title("Longitudinal Motion,Control INPUTS")
xlabel("Time[s]")
legend('Elevator','Throttle')
%%
[A_lat,B_lat] = ACLinearLatModels([18,deg2rad(0),1800],aircraft_parameters)
[V_lat] = eig(A_lat(1:5,1:5))

