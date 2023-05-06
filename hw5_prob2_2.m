clc;
close all;clear all;
run("ttwistor.m")
% x_lat = [beta_, p_, r_, phi_, psi_]';
% u_lat = [da_,dr_]';
ap = aircraft_parameters;
trim_defi =[18,deg2rad(0),1800];
[A_lat,B_lat] = ACLinearLatModels(trim_defi,aircraft_parameters)
[V_lat,D_lat] = eig(A_lat(1:5,1:5))
va_t = trim_defi(1);
tha_t = 0;
beta_t = 10;
phi_t = 0;
da_t = 0;
dr_t = 0;
alp_t = tha_t - trim_defi(2);
u_t = va_t * cos(beta_t) * cos(alp_t);
w_t = va_t * cos(beta_t) * sin(alp_t);
v_t = va_t * sin(beta_t);
q_t = 0;
p_t = 0;
r_t = 0;

% Doublet Impulse
% u_lat = [da_,dr_]'
amp = 1;
x0 = [real(V_lat(:,4))',0,amp]';
t_mid = 0.005;
del = 0.005;
t_final = 5;
T1 = [0,t_mid];T2 = [t_mid t_mid+del];T3 = [t_mid+del t_final];
[t1,x_bef] = ode45(@(t,x) odeFun(x,A_lat,B_lat),T1,x0);
x0_after = x_bef(end,:)';
x0_after(7) = -amp;
[t2,x_aft] = ode45(@(t,x) odeFun(x,A_lat,B_lat),T2,x0_after);
x0_final = x_aft(end,:)';
x0_final(7) = 0;
[t3,x_final] = ode45(@(t,x) odeFun(x,A_lat,B_lat),T3,x0_final);
t = [t1;t2;t3];x = [x_bef;x_aft;x_final];
%%
% x_lat = [beta_, p_, r_, phi_, psi_]'
figure
plot(t,x(:,1),t,x(:,2)*ap.c/(2*va_t),t,x(:,3)*ap.c/(2*va_t),t,x(:,4),t,x(:,5))
grid on;
title("Lateral Motion, STATE Vector")
xlabel("Time[s]")
legend('beta','p','r','phi','psi')
%%
figure
plot(t,x(:,6),t,x(:,7));title("Lateral Motion,Control INPUTS")
xlabel("Time[s]")
legend('Elevator','Throttle')

