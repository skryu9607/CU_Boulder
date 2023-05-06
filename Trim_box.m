function acs = Trim_box(as,ap,R_t,trim_id,climb_id)
% height is always negative in the state vector.
% if trim_id == 0 % Accelerating Climb
% end
%     
if trim_id == 1 % SLUF
    h_t = +as(3);
    if climb_id == 0 % Steady
        gamma_t = deg2rad(0);
    elseif climb_id == 1 % Climb
        gamma_t = deg2rad(5);
    elseif climb_id == -1 % Descend
        gamma_t = deg2rad(-5);
    end
    va_t = sqrt(sum(as(7:9).^2));
    xtd0 = [va_t,gamma_t,h_t]';

    [as,acs] = min_trim_fun(xtd0,ap);
    
elseif trim_id == 2
    h_t = + as(3);
    if climb_id == 0 % Steady
        gamma_t = deg2rad(0);
    elseif climb_id == 1 % Climb
        gamma_t = deg2rad(5);
    elseif climb_id == -1 % Descend
        gamma_t = deg2rad(-5);
    end
    va_t = sqrt(sum(as(7:9).^2));
    xtd0 = [va_t,gamma_t,h_t,R_t]';
    [as,acs] = coor_min_trim_fun(xtd0,ap);

end

