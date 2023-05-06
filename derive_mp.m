function [mp_idx] = derive_mp(Trajectory,n)
%DERIVE_MP Summary of this function goes here
%   n : the number of MPs
mp_idx = [];
for i = 1 : size(Trajectory,1)
    a = Trajectory(i,:);
    for j = 1 : size(Trajectory,2)-1
        if j == 1
            b(j) = a(j+1) - 1;
        else
            b(j) = a(j+1) - (((a(j)-1)*n)+1);
        end
    end
    mp_idx = [mp_idx;b];
end

       
end

