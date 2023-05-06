function gamma = Inertial_gamma(ix,iy,iz,ixz)
I = ix*iz - ixz^2;
gamma = zeros(1,8);
gamma(1) = ixz*(ix-iy+iz)/I;
gamma(2) = (iz*(iz-iy) + ixz^2)/I;
gamma(3) = iz/I;
gamma(4) = ixz/I;
gamma(5) = (iz-ix)/iy;
gamma(6) = ixz/iy;
gamma(7) = (ix*(ix-iy)+ixz^2)/I;
gamma(8) = ix/I;
end

