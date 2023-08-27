classdef node
    properties
        S; % start nodes
        T; % end nodes
        sts; % states
        pts; % pos
        hrsts; % Heurisitics. The euclidean distance to goal.
        costs; 
        mp;
        % cost : gh + 0.5*va^2 + ef
    end
    methods
        function obj = addNode(obj, X, Y,chi,Z)
            obj.sts = [obj.sts,[X, Y, chi,Z]'];
            obj.pts = [obj.pts,[X,Y,Z]'];
            obj.hrsts = [obj.hrsts,0];
            obj.costs = [obj.costs,0];
        end
        function obj = connectNodes(obj, ind1, ind2)
            obj.S = [obj.S, ind1];
            obj.T = [obj.T, ind2];
        end
        function obj = ZeroNodes(obj,idx)
            % 인덱스에 붙은 거 0으로 만들기.
            obj.S(idx) = [0]; % 0 이면 한꺼번에 삭제하기.
            obj.T(idx) = [0];
        end
        function obj = DeleteNodes(obj,idx)
            obj.S(idx) = []; % Node Elimination
            obj.T(idx) = [];
            obj.sts(:,idx) = [];
            obj.pts(:,idx) = [];
            obj.hrsts(idx) = [];
            obj.costs(idx) = [];
            obj.mp(idx) = [];
        end
        function obj = init_hc(obj,idx)
            obj.hrsts(idx) = [];
            obj.costs(idx) = [];
        end
        function obj = update_h(obj,idx)
            obj.hrsts(idx) = make_hrst(obj.pts(idx));
        end
        function obj = update_c(obj,idx,del_costs)
            if length(obj.costs) < idx
                obj.costs(idx) = 0;
            end
            %prv_idx = obj.S(idx);
            prv_idx = idx;
            obj.costs(idx) = obj.costs(prv_idx) + del_costs;

            
        end
    end
end

