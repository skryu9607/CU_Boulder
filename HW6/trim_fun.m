function cost = trim_fun(trim_defi,trim_var,aircraft_parameters)
% trim_defi : [va,gamma0,h0]'
% trim_var : [alp0,de0,dt0]'
% x : contain trim_definition and trim_variables.
global wind_inertial
[x0,air_ct] = Trim_Expression(trim_defi,trim_var);
% cost is the absolute value of force and parameters

density = stdatmo(-x0(3));
[forces,moments] =  AircraftForcesAndMoments(x0,air_ct,wind_inertial,density,aircraft_parameters);

cost = norm(forces) + norm(moments);

end

