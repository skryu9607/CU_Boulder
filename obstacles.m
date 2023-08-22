classdef obstacles
    % obs : obstacle. there is a source and radius.
    % Radius : the influence of obstacle or certain regulation.
    % -> event horizon.
    properties
        source;
        radius;
    end
    methods
        function obj = add(obj,src,radius)
            obj.source = src';
            obj.radius = radius;
        end
        
    end
end

