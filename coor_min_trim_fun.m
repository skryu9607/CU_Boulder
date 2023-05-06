function [as,acs] = coor_min_trim_fun(trim_defi,ap)
%COOR_MIN_TRIM_FUN Summary of this function goes here
% trim_var = [alp0,de0,dt0,phi0,beta0,da0,dr0]'
%xtv0 = [deg2rad(0.5),0.5,0.5,deg2rad(0.5),0.5,0.5,0.5]';
xtv0 = [0,0,0,0,0,0,0]';
f =  @(trim_var) coor_trim_fun(trim_var,trim_defi,ap);

xtv = fmincon(f,xtv0,[],[]);

[as,acs] = coor_trim(trim_defi,xtv);

end

