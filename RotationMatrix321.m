function R = RotationMatrix321(euler_angles)
%ROTATIONMATRIX321 
% Inertial frame to Body Frame.
phi = euler_angles(1);
tha = euler_angles(2);
psi = euler_angles(3);
R_3 = [cos(psi) sin(psi) 0;-sin(psi) cos(psi) 0;0 0 1];
R_2 = [cos(tha) 0 -sin(tha);0 1 0;sin(tha) 0 cos(tha)];
R_1 = [1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
R =  R_1 * R_2 * R_3;

end

