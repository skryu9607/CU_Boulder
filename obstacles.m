classdef obstacles
    % obs : obstacle. there is a source and radius.
    % Radius : the influence of obstacle or certain regulation.
    % -> event horizon.
    properties
        source;
        radius;
    end
    methods
        function obj = add(obj,x,y,z,radius)
            obj.source = [x,y,z]';
            obj.radius = radius;
        end
        
    end
end

