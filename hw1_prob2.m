% Problem 2
phi = deg2rad(-3)
tha = deg2rad(10)
psi = deg2rad(123)
v_b = [15,0,2]'
w_b_e = [1,1,-1]'
% a. AoA
wind_angles = AirRelativeVelocityToWindAngles(v_b)
AoA = rad2deg(wind_angles(3))
% b. Is this a/c ascending?
% z-component.
R = RotationMatrix321([phi,tha,psi])
v_inertial = R'*v_b + R'*w_b_e
% z-component of v_inertial is negative. So, the a/c is descending.

%c. What is the ground speed of aircraft?
v_ground = sqrt(sum(v_inertial(1:2).^2))
