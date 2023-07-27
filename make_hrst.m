function dis = make_hrst(pos_nodes)
%MAKE_HRST : the euclidean distance between a position of a node and 
% the goal.
global goal
pos_goal = [goal(1),goal(2),goal(end)]';
dis = norm(pos_nodes-pos_goal,2);
end

