classdef Robot < handle
    %Generic 2D robot explorer
    
    properties
        % ID
        id = -1;
        
        % Ranges
        senseRange = 10;
        communicationRange = 10;
        
        communicationRate = 100; % Quadtree nodes per second
        
        forwardVelocity = 0.5; % m/s
        
        localState; % stores the robot's estimate of its position in the 'local frame'
        startingState; % the transformation between the robot's starting state and the 'global frame'
        
        localMap; % a local occupancy grid the robot will update as it explores.
        localExplorationTree; % a quad tree used to efficiently communicate map exploration status.
        
        linesOfExploration; % a map associating each robot with a line of exploration
    end
    
    methods
        % initialize the robot state estimate
        function obj = Robot(id)
            obj.localState = RobotState([0;0;0], diag([1,1,1] * 1e-24)); % highly certain.
            obj.startingState = RobotState([0;0;0], diag([1, 1, 1] * 1e6)); % highly uncertain.
            
            obj.id = id; % unique id assigned to each robot.
            
            
        end
    end
end

