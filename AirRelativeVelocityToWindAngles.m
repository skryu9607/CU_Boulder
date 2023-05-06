function [wind_angles] = AirRelativeVelocityToWindAngles(velocity_body)
va = sqrt(sum(velocity_body.^2));
beta = asin(velocity_body(2)/va);
alpha = atan(velocity_body(3)/velocity_body(1));
wind_angles = [va, beta, alpha];
end

