function [mynodes] = assign_hrst_cost(MP_Inputs,mynodes,nodes_idx,L)
%ASSIGN_HRST_COST : All expanded nodes of each previous node
%   Detailed explanation goes here

% Heuristics
for i = 1:length(nodes_idx)
    mynodes = update_h(mynodes,nodes_idx(i));
end
mynodes.hrsts = mynodes.hrsts / L;

ttwistor;
ap = aircraft_parameters;
% Costs
for i = 1:length(nodes_idx)
    MP_Input = single_MP(MP_Inputs,i);
    del_costs = new_costs(mynodes.sts(:,i),MP_Input,ap);
end
del_costs = del_costs/max(del_costs);
for i = 1:length(nodes_idx)
    mynodes = update_c(mynodes,nodes_idx(i),del_costs);
end
end

