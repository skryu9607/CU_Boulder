function [xtv,f] = coor_min_trim_fun_1(trim_defi,aircraft_parameters)
% trim_defi : [va,gamma0,h0,R0]'
% trim_var = [alp0,de0,dt0,phi0,beta0,da0,dr0]'
%xtv0 = [deg2rad(0.5),0.5,0.5,deg2rad(0.5),0.5,0.5,0.5]';
xtv0 = [0,0,0,0,0,0,0]';

f = @(trim_var) coor_trim_fun(trim_var,trim_defi,aircraft_parameters);

xtv = fmincon(f,xtv0,[],[]);

end



