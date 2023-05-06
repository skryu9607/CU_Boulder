function [Trim_state,Trim_c_sur] = Trim_Expression(trim_defi,trim_var)
% trim_defi : [va,gamma0,h0]'
% trim_var : [alp0,de0,dt0]'
global wind_inertial

tha0 = trim_var(1) + trim_defi(2);
u0 = trim_defi(1) * cos(trim_var(1));
w0 = trim_defi(1) * sin(trim_var(1));
% Initial state and control
x0 = [0,0,-trim_defi(3),0,tha0,0,u0,0,w0,0,0,0]';
if isequal(wind_inertial,[0,0,0]')
    x0(7:9) = x0(7:9);
else
    x0(7:9) = x0(7:9) + RotationMatrix321(x0(4:6)) * wind_inertial;
end
U0 = [trim_var(2),0,0,trim_var(3)]';

Trim_state = x0;
Trim_c_sur = U0;

end

