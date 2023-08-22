classdef thmals
    %UNTITLED5 Summary of this class goes here
    %   Detailed explanation goes here
    % Making the thermals to wind column. 
    properties
        source;
        radius;
        wind;
    end
    
    methods
        function obj = add(obj,src,radius,wd)
            obj.source = src';
            obj.radius = radius;
            obj.wind = wd;
        end
    end
end

