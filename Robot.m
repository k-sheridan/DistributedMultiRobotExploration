classdef Robot
    %Generic 2D robot explorer
    
    properties
        % Ranges
        senseRange = 10;
        communicationRange = 10;
        
        communicationRate = 100; % Quadtree nodes per second
        
        forwardVelocity = 0.5; % m/s
    end
    
    methods
        function obj = Robot()
            
        end
    end
end

