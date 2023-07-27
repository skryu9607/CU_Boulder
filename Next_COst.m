%% Next cost function
va_dot = 0;
gamma_dot = 0;
q = 0.5 * stdatmo(-h) * ap.S;
L = q * va^2 * ap.Cl0;
D = q * va^2 * (ap.Cd0 + ap.Cl0^2/(pi*ap.AR*ap.e));
T_x = D/ap.m + ap.g * sin(u3);
T_y = 0;
phi = atan(u2*u1/ap.g);
T_z = L- (ap.m * ap.g * cos(u3))/cos(phi);
Thrust = [T_x,T_y,T_z]';
T = nomr(Thrust,2);
e_f = -T*u1/(ap.m*ap.g*eta_ec*eta_p);
e_h = ap.g*(u1*sin(u3)-wind_inertial(3));
costs = e_h + e_f ;
% Bigger is the best