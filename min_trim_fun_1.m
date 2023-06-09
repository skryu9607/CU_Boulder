function [xtv,f] = min_trim_fun_1(trim_defi,aircraft_parameters)
% trim_defi : [va,gamma0,h0]'
% trim_var : [alp0,de0,dt0]'
xtv0 = [0.5,0.5,0.5]';

f = @(trim_var) trim_fun(trim_defi,trim_var,aircraft_parameters);
xtv = fmincon(f,xtv0,[],[]);
end

