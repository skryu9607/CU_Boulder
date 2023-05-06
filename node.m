classdef node
    properties
        S; % start nodes
        T; % end nodes
        sts; % states
        pts; % pos
    end
    methods
        function obj = addNode(obj, X, Y,Ki,gamma,Z)
            obj.sts = [obj.sts,[X,Y,Ki,gamma,Z]'];
            obj.pts = [obj.pts,[X,Y,Z]'];
        end
        function obj = connectNodes(obj, ind1, ind2)
            obj.S = [obj.S, ind1];
            obj.T = [obj.T, ind2];
        end
        function obj = removeNodes(obj)
            obj.S = [];
            obj.T = [];
        end
        
    end
end

