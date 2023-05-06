function vector_body = TransformFromInertialToBody(vector_inertial,euler_angles)
R =  RotationMatrix321(euler_angles)
% Rotational matrix : Inertial velocity in Body frame to Inertial vel in
% Inertial frame.
vector_body = R' * vector_inertial
end
