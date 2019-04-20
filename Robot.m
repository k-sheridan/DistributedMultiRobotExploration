classdef Robot
    %Generic 2D robot explorer
    
    properties
        % Ranges
        senseRange = 10;
        communicationRange = 10;
        
        communicationRate = 100; % Quadtree nodes per second
        
        forwardVelocity = 0.5; % m/s
        
        state; % stores the robot's estimate of its position in the 'global frame'
        
        localMap; % a local occupancy grid the robot will update as it explores.
        
        explorationTree; % a quad tree used to efficiently communicate map exploration status.
    end
    
    methods
        function obj = Robot()
            
        end
    end
end

