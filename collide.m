function flag = collide(pt1,pt2,eps,ind)
%DISTANCE Summary of this function goes here
%   Detailed explanation goes here
if ind == 0 % with obstacles
    d = pt1-pt2;
    d = sqrt(sum(d.^2));
    if d < eps
        flag = true;
    else
        flag = false;
    end
elseif ind == 1 % with thermals
    if abs(pt1(3)) < abs(pt2(3))
        d = pt1(1:2)-pt2(1:2);
        d = sqrt(sum(d.^2));
        if d < eps
            flag = true;
        else
            flag = false;
        end
    else
        flag = false;
    end
end


end

