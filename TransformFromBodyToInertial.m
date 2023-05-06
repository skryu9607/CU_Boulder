function vector_inertial = TransformFromBodyToInertial(vector_body,euler_angles)
R = RotationMatrix321(euler_angles)
vector_inertial = R * vector_body
end

