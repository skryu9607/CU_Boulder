function [costs] = new_costs(pts,MP_Input,ap)
%% Next cost function
global wind_inertial
eta_ec = 0.8;eta_p = 1;
h = pts(3);
if isa(MP_Input,"cell")
    costs = 0;
    for i = 1:2
        u1 = MP_Input{i}(1);
        u2 = MP_Input{i}(2);
        u3 = MP_Input{i}(3);
        q = 0.5 * stdatmo(-h) * ap.S;
        L = q * u1^2 * ap.CL0;
        D = q * u1^2 * (ap.CD0 + ap.CL0^2/(pi*ap.AR*ap.e));
        T_x = D/ap.m + ap.g * sin(u3);
        T_y = 0;
        phi = atan(u2*u1/ap.g);
        T_z = L- (ap.m * ap.g * cos(u3))/cos(phi);
        Thrust = [T_x,T_y,T_z]';
        T = norm(Thrust,2);
        e_f = -T*u1/(ap.m*ap.g*eta_ec*eta_p);
        %e_h = ap.g*(-u1*sin(u3)+wind_inertial(3));
        % Bigger is the good one.
        e_h = 0;
        costs = costs + e_h - e_f ;
    end
else
    u1 = MP_Input(1);
    u2 = MP_Input(2);
    u3 = MP_Input(3);
    

    % va_dot = 0;
    % gamma_dot = 0;
    q = 0.5 * stdatmo(-h) * ap.S;
    L = q * u1^2 * ap.CL0;
    D = q * u1^2 * (ap.CD0 + ap.CL0^2/(pi*ap.AR*ap.e));
    T_x = D/ap.m + ap.g * sin(u3);
    T_y = 0;
    phi = atan(u2*u1/ap.g);
    T_z = L- (ap.m * ap.g * cos(u3))/cos(phi);
    Thrust = [T_x,T_y,T_z]';
    T = norm(Thrust,2);
    e_f = -T*u1/(ap.m*ap.g*eta_ec*eta_p);
    %e_h = ap.g*(-u1*sin(u3)+wind_inertial(3));
    % Bigger is the good one.
    e_h = 0;
    costs = e_h - e_f ;
    
end

end

