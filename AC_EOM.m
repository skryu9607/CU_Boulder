function [xdot] = AC_EOM(time,aircraft_state,aircraft_surfaces,wind_inertial, aircraft_parameters)
%AC_EOM 이 함수의 요약 설명 위치
% x : state vector which has 12 elements.
% xdot : BY using AC_EOM
% x_next = x + xdot * time_interval.
% X = [x_E,y_E,z_E,phi,tha,psi,u,v,w,p,q,r]' 
% X(1:3) = (X_dot,Y_dot,X_dot)
density = stdatmo(-aircraft_state(3));
ap = aircraft_parameters;
[aircraft_forces,aircraft_moments] = AircraftForcesAndMoments(aircraft_state,aircraft_surfaces,wind_inertial, density,aircraft_parameters);
% Inertial velocities to body coordinates
u = aircraft_state(7);
v = aircraft_state(8);
w = aircraft_state(9);
phi =  aircraft_state(4);tha = aircraft_state(5);psi = aircraft_state(6);
dot_pos = RotationMatrix321(aircraft_state(4:6))' * [u;v;w];
%dot_pos = RotationMatrix321(aircraft_state(4:6))' * [u;v;w] + wind_inertial;

% Angular velocity
p = aircraft_state(10);
q = aircraft_state(11);
r = aircraft_state(12);
ang_rate = [p;q;r];

dot_ang = [1 sin(phi)*tan(tha) cos(phi)*tan(tha);
    0 cos(phi) -sin(phi);% Damn.. I found it.
    0 sin(phi)*sec(tha) cos(phi)*sec(tha)]*ang_rate;

dot_vel = [r * v - q * w;p * w - r * u;q * u - p * v] + aircraft_forces/ap.m;

% Moments
Gamma = ap.Ix*ap.Iz - ap.Ixz^2;
ga1 = (ap.Ixz * (ap.Ix - ap.Iy + ap.Iz))/ Gamma;
ga2 = (ap.Iz * (ap.Iz - ap.Iy) + ap.Ixz^2) / Gamma;
ga3 = ap.Iz/ Gamma;
ga4 = ap.Ixz / Gamma;
ga5 = (ap.Iz - ap.Ix)/ap.Iy;
ga6 = ap.Ixz / ap.Iy;
ga7 = (ap.Ix*(ap.Ix - ap.Iy)+ap.Ixz^2)/ Gamma;
ga8 = ap.Ix / Gamma;
L = aircraft_moments(1);M=aircraft_moments(2);N=aircraft_moments(3);
dot_rate = [ga1*p*q - ga2*q*r;ga5*p*r-ga6*(p^2-r^2);ga7*p*q-ga1*q*r] + [ga3*L + ga4 * N; M/ap.Iy; ga4*L + ga8*N];

xdot = [dot_pos;dot_ang;dot_vel;dot_rate];

end

