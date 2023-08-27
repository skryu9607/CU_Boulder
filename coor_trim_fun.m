function cost = coor_trim_fun(trim_var,trim_defi,aircraft_parameters)
% The objective function is more complex in this case,
% since the forces do not sum to zero.
% Instead, the forces must provided acceleration vector 
% a^des to make the turn. 
global wind_inertial
[x0,air_ct] = coor_trim(trim_defi,trim_var);

% cost is the absolute value of force and parameters
% stdatmo : inputs are positive value.
density = stdatmo(-x0(3));
% Y-forces in cost function is aero_force not aircraft_forces
[aero_force,aero_moment] = AeroF_M_Body_wcoeffs(x0,air_ct,wind_inertial,density,aircraft_parameters);
[forces,moments] =  AircraftForcesAndMoments(x0,air_ct,wind_inertial,density,aircraft_parameters);

% acc should be expressed in state vector.
R = trim_defi(4);va = trim_defi(1);tan_speed = va * cos(trim_defi(2));

acc = (tan_speed)^2/(R);
ACC = [0,acc,0]';
Rot = RotationMatrix321(x0(4:6));

Desired_for = forces - aircraft_parameters.m*Rot*ACC;

cost = norm(Desired_for,2) + norm(moments,2) + aero_force(2)^2

end

