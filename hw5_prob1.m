%% Problem 1. va = 18 m/s, gamma = 0, h = 1800 m.
% The perturbations
% x_lon = [u_/va_t, alp_t, q_, tha_, h_]';
% x_lot = [beta_, p_, r_, phi_, psi_]';
% u_lon = [de_,dt_]';
% u_lot = [da_,dr_]';
% The result of homework 3.
[A_lon,B_lon] = ACLinearLonModels([18,deg2rad(0),1800],aircraft_parameters)
[V_lon] = eig(A_lon(1:5,1:5))

[A_lat,B_lat] = ACLinearLatModels([18,deg2rad(0),1800],aircraft_parameters)
[V_lat] = eig(A_lat(1:4,1:4))
