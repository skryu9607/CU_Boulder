function cost = trim_fun(trim_defi,trim_var,aircraft_parameters)
% trim_defi : [va,gamma0,h0]'
% trim_var : [alp0,de0,dt0]'
% x : contain trim_definition and trim_variables.
[x0,air_ct] = Trim_Expression(trim_defi,trim_var);
global wind_inertial
density = stdatmo(-x0(3));
% cost is the absolute value of force and parameters
[forces,moments] =  AircraftForcesAndMoments(x0,air_ct,wind_inertial,density,aircraft_parameters);

cost = norm(forces,2) + norm(moments,2);

end

