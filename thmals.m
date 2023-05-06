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
        function obj = add(obj,x,y,z,radius)
            obj.source = [x,y,z]';
            obj.radius = radius;
        end
        function obj = wd(obj,wd_in_th)
            obj.wind = wd_in_th;
        end
    end
end

