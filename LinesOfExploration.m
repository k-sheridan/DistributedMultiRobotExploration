classdef LinesOfExploration
    % A class which describes the area a given robot should explore
    
    properties
        lineMap;
    end
    
    methods
        function obj = LinesOfExploration()
            obj.lineMap = containers.Map();
        end
        
        
    end
end

