function [aero_force,aero_moment] = AeroF_M_Body_wcoeffs(aircraft_state,aircraft_surfaces,wind_inertial,density,aircraft_parameters)
% INPUT : Aircraft state, the control input, the inertial wind_inertial,
% air density, and aircraft_parameters
% OUTPUT : aerodynamic forces and moment_in body coordinates.
ap = aircraft_parameters;
as = aircraft_state;
acs = aircraft_surfaces; % de,da,dr,dt

R = RotationMatrix321(as(4:6)); % Inertial -> Body
if isempty(wind_inertial) == 1
    wind_inertial = [0,0,0]';
end

velocity_body = as(7:9) - R * wind_inertial;
%velocity_body = as(7:9);
[wind_angles] = AirRelativeVelocityToWindAngles(velocity_body);
va = wind_angles(1);
beta = wind_angles(2);
alp = wind_angles(3);
S = ap.S;
dp =  0.5 * density * va^2; % Dynamic pressure
CL = ap.CL0 + ap.CLalpha * alp + ap.CLq * (ap.c / (2*va) * as(11)) + ap.CLde * acs(1);
Lift = dp * S * (CL);
CD = ap.CDmin + ap.K * (CL-ap.CLmin)^2;
Drag = dp * S * CD;

% Longitudinal
X = -Drag * cos(alp) + Lift * sin(alp);
Z = -Drag * sin(alp) - Lift * cos(alp);
M = dp * S * ap.c * (ap.Cm0 + ap.Cmalpha * alp + ap.Cmq * (ap.c/(2*va) * as(11)) + ap.Cmde * acs(1));

% Lateral and Directional
Y = dp * S * (ap.CY0 + ap.CYbeta*beta + ap.CYp * ap.b/(2*va)*as(10) + ap.CYr * ap.b/(2*va) * as(12) + ap.CYda * acs(2) + ap.CYdr *acs(3));
L = dp * S * ap.b * (ap.Cl0 + ap.Clbeta*beta + ap.Clp * ap.b/(2*va)*as(10) + ap.Clr * ap.b/(2*va) * as(12) + ap.Clda * acs(2) + ap.Cldr *acs(3));
N = dp * S * ap.b * (ap.Cn0 + ap.Cnbeta*beta + ap.Cnp * ap.b/(2*va)*as(10) + ap.Cnr * ap.b/(2*va) * as(12) + ap.Cnda * acs(2) + ap.Cndr *acs(3));
CT = 2 * ap.Sprop/S * ap.Cprop * acs(4)/va^2*(va + acs(4)*(ap.kmotor-va))*(ap.kmotor-va);
T = dp * S * CT;

thrust = [T, 0 ,0 ]';
aero_force = [X,Y,Z]' + thrust;
propulsive_moments = [0,0,0]'; % Because it is symmetric.
aero_moment = [L,M,N]' + propulsive_moments;

end


