function dp = PosKinModel(ki,gam,wind_inertial)
wind_inertial(3) = - wind_inertial(3);
dp = va * [cos(ki)*cos(gam);sin(ki)*cos(gam);sin(gam)] + wind_inertial;
end

