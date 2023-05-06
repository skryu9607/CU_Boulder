function [as0,acs0] = coor_trim(trim_defi,trim_var)
% trim_defi = [va,gamma0,height0,R0]'
% trim_var = [alp0,de0,dt0,phi0,beta0,da0,dr0]'
% velocity_body = WindAnglesToAirRelativeVelocityVector([trim_defi(1),trim_var(5),trim_var(1)]);
% u0 = velocity_body(1);v0 = velocity_body(2);w0 = velocity_body(3);
va = trim_defi(1);
u0 = va * cos(trim_var(1)) * cos(trim_var(5));
v0 = va * sin(trim_var(5));
w0 = va * sin(trim_var(1)) * cos(trim_var(5));
tha0 = trim_var(1) + trim_defi(2);
chi_dot = va*cos(trim_defi(2))/trim_defi(4);

p = -sin(tha0) * chi_dot;
q = sin(trim_var(4)) * cos(tha0) * chi_dot;
r = cos(trim_var(4)) * cos(tha0) * chi_dot;

as0 = [0,0,-trim_defi(3),trim_var(4),tha0,0,u0,v0,w0,p,q,r]';

acs0 = [trim_var(2),trim_var(6),trim_var(7),trim_var(3)]';

end
