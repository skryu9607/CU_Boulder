function [as,acs] = Trim_box(x_red,MP_Input,ap)
% height is always negative in the state vector.
% if trim_id == 0 % Accelerating Climb
% end
global va

if isa(MP_Input,"cell") % Splined curves.
    a = MP_Input{1};
    b = MP_Input{2};
    u2 = a(1);
    u3 = a(2);
    h_t = -x_red(5);
    gamma0 = u3;
    R_t = va/u2;
    xtd0 = [va,gamma0,h_t,R_t]';
    [as0,acs0] = coor_min_trim_fun(xtd0,ap);
    u2 = b(1);
    u3 = b(2);
    h_t = -x_red(5); % Because keep the heights.
    gamma1 = u3;
    R_t = va/u2;
    xtd1 = [va,gamma1,h_t,R_t]';
    [as1,acs1] = coor_min_trim_fun(xtd1,ap);
    as = [as0,as1];
    acs = [acs0,acs1];
else
   if MP_Input(1) == 0 % Straight line
       h_t = -x_red(5);
       gamma0 = MP_Input(2);
       xtd0 = [va,gamma0,h_t]';
       [as,acs] = min_trim_fun(xtd0,ap);
   else
       % Curves and Spirals
       h_t = -x_red(5);
       gamma0 = MP_Input(2);
       R_t = va/MP_Input(1);
       xtd0 = [va,gamma0,h_t,R_t]';
       [as,acs] = coor_min_trim_fun(xtd0,ap);
   end
   
end 

end

