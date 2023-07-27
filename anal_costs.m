function [del_costs] = anal_costs(as,acs,MP_Input)
%ANAL_COSTS 
% sts : [x_E,y_E,z_E,phi,tha,psi,u,v,w,p,q,r]' 
global va wind_inertial
ttwistor;
ap = aircraft_parameters;
if size(as,2) == 1
    dt = acs(4);
    euler = as(4:6);
    R = RotationMatrix321(euler);
    eta_eh = 0.8;eta_p = 1;
    c_tilda = ap.Sprop*ap.Cprop/(ap.m*ap.g*eta_eh*eta_p)*stdatmo(-as(3));
    ful_com = -c_tilda * va*(dt)*(ap.kmotor-va)*(va + dt * (ap.kmotor-va));
    gamma = MP_Input(2);
    h_dot = -va*sin(gamma) + wind_inertial(3);
    pot_eng = ap.g * h_dot;
    vg = norm([va,0,0]' + R * wind_inertial ,2);
    del_costs = (pot_eng + ful_com) / (vg);
    ful_com_norm = -c_tilda * va*(1)*(ap.kmotor-va)*(va + 1 * (ap.kmotor-va));
    pot_eng_norm = -va*sin(deg2rad(45)) + wind_inertial(3);
    del_costs_norm = (pot_eng_norm + ful_com_norm) / (vg);
    del_costs = del_costs / del_costs_norm;
else %size(sts,2) == 2, splined curves. 끝 점만이 중요하다.
    dt1 = acs(4,1);
    dt2 = acs(4,2);
    euler1 = as(4:6,1);
    euler2 = as(4:6,2);
    R1 = RotationMatrix321(euler1);
    R2 = RotationMatrix321(euler2);
    eta_eh = 0.8;eta_p = 1;
    c_tilda_1 = ap.Sprop*ap.Cprop/(ap.m*ap.g*eta_eh*eta_p)*stdatmo(-as(3,1));
    % c_tilda_2 <- 여기가 좀 애매하긴 하다.
    c_tilda_2 = ap.Sprop*ap.Cprop/(ap.m*ap.g*eta_eh*eta_p)*stdatmo(-as(3,2));
    ful_com1 = -c_tilda_1 * va*(dt1)*(ap.kmotor-va)*(va + dt1 * (ap.kmotor-va));
    ful_com2 = -c_tilda_2 * va*(dt2)*(ap.kmotor-va)*(va + dt2 * (ap.kmotor-va));
    h_dot = +wind_inertial(3); % gamma = 0, NO height change intentionally
    pot_eng = ap.g * h_dot;
    vg1 = norm([va,0,0]' + R1 * wind_inertial ,2);
    vg2 = norm([va,0,0]' + R2 * wind_inertial ,2);
    del_costs1 = (pot_eng+ful_com1) / (vg1);
    del_costs2 = (pot_eng+ful_com2) / (vg2);
    del_costs = (del_costs1 + del_costs2) / 2;
    ful_com_norm1 = -c_tilda_1 * va*(1)*(ap.kmotor-va)*(va + 1 * (ap.kmotor-va));
    ful_com_norm2 = -c_tilda_2 * va*(1)*(ap.kmotor-va)*(va + 1 * (ap.kmotor-va));
    del_costs1_norm = (pot_eng+ful_com_norm1) / (vg1);
    del_costs2_norm = (pot_eng+ful_com_norm2) / (vg2);
    del_costs_norm = (del_costs1_norm + del_costs2_norm) / 2;
    del_costs = del_costs / del_costs_norm;
    
end

end

