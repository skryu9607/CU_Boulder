function [Alon,Blon] = ACLinearLonModels(trim_defi,aircraft_parameters)
ap = aircraft_parameters;
% The result of homework 3.
va_t = trim_defi(1);
h_t = trim_defi(3);
% thata and de and dt are derived from homework 3,4.
tha_t = deg2rad(2.9496);
de_t = -0.537017;
dt_t = 0.179151;
alp_t = tha_t - trim_defi(2);
u_t = va_t * cos(alp_t);
w_t = va_t * sin(alp_t);
q_t = 0;

K = ap.K;
S = ap.S;
c = ap.c;
density = stdatmo(h_t);
%% A_lon_1 
CL_t = ap.CL0 + ap.CLalpha * alp_t + ap.CLq * (c/ (2*va_t) * q_t) + ap.CLde * de_t;
CD_t = ap.CDmin + ap.K * (CL_t-ap.CLmin)^2;
C_X_t = -CD_t * cos(alp_t) + CL_t * sin(alp_t);
C_Z_t = -CD_t * sin(alp_t) - CL_t * cos(alp_t);

C_X_alp = -2*K*(CL_t - ap.CLmin) * ap.CLalpha * cos(alp_t) + CD_t * sin(alp_t) + ap.CLalpha * sin(alp_t) + CL_t * cos(alp_t);

C_X_q = -2*K*(CL_t-ap.CLmin) * ap.CLq * cos(alp_t) + ap.CLq * sin(alp_t);

X_u = u_t*density*S/ap.m*C_X_t - density*S*w_t*C_X_alp/(2*ap.m)+ (density*S*c*C_X_q*u_t*q_t)/(4*ap.m*va_t)+density*ap.Sprop*ap.Cprop*de_t/ap.m*(ap.kmotor*u_t/va_t*(1-2*de_t)+2*u_t*(de_t-1)) ;
X_w = -q_t + w_t*density*S/ap.m*(C_X_t) + (density*S*c*C_X_q*w_t*q_t)/(4*ap.m*va_t)+density*S*u_t*C_X_alp/(2*ap.m) + density*ap.Sprop*ap.Cprop*de_t/ap.m*(ap.kmotor*w_t/va_t*(1-2*de_t)+2*w_t*(de_t-1)) ;
X_q = -w_t + density*va_t*S*C_X_q * c/(4*ap.m);

Alon_1 = [X_u, X_w * va_t * cos(alp_t), X_q, -ap.g*cos(alp_t), 0];
%% A_lon_2

C_Z_alp = -2*K*(CL_t - ap.CLmin) * ap.CLalpha * sin(alp_t) - CD_t * cos(alp_t) - ap.CLalpha * cos(alp_t) + CL_t * sin(alp_t); 

C_Z_q = -2*K*(CL_t-ap.CLmin) * ap.CLq * sin(alp_t) - ap.CLq * cos(alp_t);

Z_u = q_t + u_t*density*S/ap.m*C_Z_t-density*S*C_Z_alp*w_t/(2*ap.m) + u_t*density*S*C_Z_q*c*q_t/(4*ap.m*va_t);
Z_w = w_t*density*S/ap.m*C_Z_t+density*S*C_Z_alp*u_t/(2*ap.m) + w_t*density*S*C_Z_q*c*q_t/(4*ap.m*va_t);
Z_q = u_t + density*va_t*S*C_Z_q * c/(4*ap.m);

Alon_2 = [Z_u/(va_t*cos(alp_t)), Z_w, Z_q/(va_t*cos(alp_t)), -ap.g*sin(alp_t)/(va_t*cos(alp_t)),0];

%% A_lon_3

M_u = u_t*density*S*c/ap.Iy*(ap.Cm0+ap.Cmalpha*alp_t+ap.Cmde*de_t) - density*S*c*ap.Cmalpha*w_t/(2*ap.Iy) + density*S*c^2*ap.Cmq*q_t*u_t/(4*ap.Iy*va_t);
M_w = w_t*density*S*c/ap.Iy*(ap.Cm0+ap.Cmalpha*alp_t+ap.Cmde*de_t) + density*S*c*ap.Cmalpha*u_t/(2*ap.Iy) + density*S*c^2*ap.Cmq*q_t*w_t/(4*ap.Iy*va_t);
M_q = density*va_t*S*c^2*ap.Cmq/(4*ap.Iy);

Alon_3 = [M_u, M_w * va_t * cos(alp_t), M_q, 0, 0];
%% A_lon_4
Alon_4 = [0,0,1,0,0];

%% A_lon_5
Alon_5 = [sin(tha_t), -va_t*cos(tha_t)*cos(alp_t),0,u_t*cos(tha_t)+w_t*sin(tha_t),0];
Alon = [Alon_1;Alon_2;Alon_3;Alon_4;Alon_5];
%% B - lon
C_X_de = -2*K*(CL_t-ap.CLmin)*ap.CLde * cos(alp_t) + ap.CLde * sin(alp_t);
X_de = density*va_t^2*S*C_X_de/(2*ap.m);
X_dt = density*ap.Sprop*ap.Cprop/ap.m*(va_t*(ap.kmotor-va_t)+2*dt_t*(ap.kmotor-va_t)^2);
B_lon_1 = [X_de, X_dt];
%%
C_Z_de = -2*K*(CL_t-ap.CLmin)*ap.CLde * sin(alp_t) - ap.CLde * cos(alp_t);
Z_de = density*va_t^2*S*C_Z_de/(2*ap.m);
B_lon_2 = [Z_de/(va_t*cos(alp_t)), 0];
%% 

M_de = density * va_t^2*S*c*ap.Cmde/(2*ap.Iy);
B_lon_3 = [M_de,0];

B_lon_4 = [0,0];

B_lon_5 = [0,0];

Blon = [B_lon_1;B_lon_2;B_lon_3;B_lon_4;B_lon_5];
end

