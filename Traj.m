function NODES = Make_Traj(idx,mynodes)
%The function Traj makes the trajectory from a node in final stage to start
% goal. 
% Input : an index of a certain node
% Output : a series of nodes. to connect with each other. 
% How I know if those connect? -> Using mynodes.
S = mynodes.S;
T = mynodes.T;
prev_idx = idx;
NODES = [idx];
while true
   next_idx = S(find(T == prev_idx));
   prev_idx = next_idx;
   NODES = [next_idx,NODES];
if prev_idx == 1
    break
end
end


end

% end
% 
% %TRAJ Summary of this function goes here
% %   Detailed explanation goes here
% % First, 1 - > length(MP)
% n = length(MP);
% i = 0;
% prev_nodes = x;
% new_nodes = [];
% NODE = [];
% while i < 6
%     NODE = [NODE,prev_nodes];
%     i = i + 1;
%     k = 0;
%     while k < size(prev_nodes,2)
%         k = k + 1;
%         x = prev_nodes(:,k);
%         new_nodes(:,n*(k-1)+1:n*k) = repmat(x,1,n) + MP(x,5,MP_input); 
%     end
% if size(new_nodes,2) ~= n * size(prev_nodes,2)
%     disp("Length가 똑같지 않습니다.")
%     size(new_nodes,2)
%     size(prev_nodes,2)
%     break
% end
% new_nodes = repmat(new_nodes,1,n);
% prev_nodes = new_nodes;
%  
% end
% 
% 
