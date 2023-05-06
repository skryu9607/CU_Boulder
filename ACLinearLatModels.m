function [Alat,Blat] = ACLinearLatModels(trim_defi,aircraft_parameters)
ap = aircraft_parameters;
% The result of homework 3.
va_t = trim_defi(1);
h_t = trim_defi(3);
% thata and de and dt are derived from homework 3,4.
tha_t = deg2rad(2.9496);

beta_t = 0;
phi_t = 0;
da_t = 0;
dr_t = 0;
S = ap.S;
b = ap.b;
alp_t = tha_t - trim_defi(2);
u_t = va_t * cos(beta_t) * cos(alp_t);
w_t = va_t * cos(beta_t) * sin(alp_t);
v_t = va_t * sin(beta_t);
q_t = 0;
p_t = 0;
r_t = 0;
density = stdatmo(h_t);
%% A_lat_1 
Y_v = density*S*b*v_t/(4*ap.m*va_t) * (ap.CYp*p_t + ap.CYr * r_t) + density*S*v_t/ap.m * (ap.CY0+ap.CYbeta*beta_t + ap.CYda * da_t + ap.CYdr * dr_t) + density*S*ap.CYbeta/(2*ap.m)*sqrt(u_t^2+w_t^2);
Y_p = w_t + density*va_t*S*b/(4*ap.m) * ap.CYp;
Y_r = -u_t+ density*va_t*S*b/(4*ap.m) * ap.CYr;

Alat_1 = [Y_v, Y_p/(va_t*cos(beta_t)),Y_r/(va_t*cos(beta_t)),ap.g*cos(tha_t)*cos(phi_t)/(va_t*cos(beta_t)),0];
%% A_lon_2
L_v = density*S*b^2*v_t/(4*va_t) * (ap.Clp*p_t + ap.Clr * r_t) + density*S*b*v_t * (ap.Cl0+ap.Clbeta*beta_t + ap.Clda * da_t + ap.Cldr * dr_t) + density*S*b*ap.Clbeta/2*sqrt(u_t^2+w_t^2);
L_p = ap.gamma(1)* q_t + density*va_t*S*b^2/4*ap.Clp;
L_r = -ap.gamma(2)*q_t + density*va_t*S*b^2/4*ap.Clr;

Alat_2 = [L_v*va_t*cos(beta_t), L_p, L_r, 0, 0];

%% A_lon_3
N_v = density*S*b^2*v_t/(4*va_t) * (ap.Cnp*p_t + ap.Cnr * r_t) + density*S*b*v_t * (ap.Cn0+ap.Cnbeta*beta_t + ap.Cnda * da_t + ap.Cndr * dr_t) + density*S*b*ap.Cnbeta/2*sqrt(u_t^2+w_t^2);
N_p = ap.gamma(7) * q_t + density*va_t*S*b^2/4*ap.Cnp;
N_r =-ap.gamma(1) * q_t + density*va_t*S*b^2/4*ap.Cnr;
Alat_3 = [N_v*va_t*cos(beta_t), N_p, N_r, 0, 0];
%% A_lon_4
Alat_4 = [0,1,cos(phi_t)*tan(tha_t), q_t*cos(phi_t)*tan(tha_t)-r_t*sin(phi_t)*tan(tha_t),0];

%% A_lon_5
Alat_5 = [0,0,cos(phi_t)*sec(tha_t), p_t*cos(phi_t)*sec(tha_t)-r_t*sin(phi_t)*sec(tha_t),0];
Alat = [Alat_1;Alat_2;Alat_3;Alat_4;Alat_5];

%% B - lon
Y_da = density * va_t^2 * S/(2*ap.m) * ap.CYda;
Y_dr = density * va_t^2 * S/(2*ap.m) * ap.CYdr;
B_lat_1 = [Y_da/(va_t*cos(beta_t)), Y_dr/(va_t*cos(beta_t))];
%%
L_da = density * va_t^2 * S * b/2 * ap.Clda;
L_dr = density * va_t^2 * S * b/2 * ap.Cldr;
B_lat_2 = [L_da, L_dr];
%% 
N_da = density * va_t^2 * S * b/2 * ap.Cnda;
N_dr = density * va_t^2 * S * b/2 * ap.Cndr;
B_lat_3 = [N_da,N_dr];

B_lat_4 = [0,0];

B_lat_5 = [0,0];

Blat = [B_lat_1;B_lat_2;B_lat_3;B_lat_4;B_lat_5];


end

