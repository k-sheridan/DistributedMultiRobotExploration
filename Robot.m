classdef Robot < handle
    %Generic 2D robot explorer
    
    properties
        % ID
        id = -1;
        
        localState; % stores the robot's estimate of its position in the 'local frame'
        startingState; % the transformation between the robot's starting state and the 'global frame'
        
        localMap; % a local occupancy grid the robot will update as it explores.
        localExplorationTree; % a quad tree used to efficiently communicate map exploration status.
        
        linesOfExploration; % a map associating each robot with a line of exploration (local frame)
        
        % waypoints: [[x1;y1], [x2;y2], [x3;y3], ...]
        waypoints;
    end
    
    methods
        % initialize the robot state estimate
        function obj = Robot(id, mapSize, mapResolution)
            obj.localState = RobotState([0;0;0], diag([1,1,1] * 1e-24)); % highly certain.
            obj.startingState = RobotState([0;0;0], diag([1, 1, 1] * 1e6)); % highly uncertain.
            
            obj.id = id; % unique id assigned to each robot.
            
            obj.localMap = Map();
            obj.localMap.initialize(mapSize, mapResolution);
            
            obj.localExplorationTree = ExplorationQuadTree(mapSize*mapResolution, Settings.QUADTREE_LEVELS);
            
            obj.linesOfExploration = LinesOfExploration();
            
        end
        
        % using the current state estimate, drive the vehicle and or plan a
        % new path.
        function [] = motionUpdate()
            
        end
        
        % update the local occupancy grid and exploration tree using the
        % synthetic lidar measurement.
        function [] = senseUpdate(lidarMeasurement)
            
        end
        
        % idk if this is the best way to do this...
        function [] = communicationUpdate()
        end
        
        
    end
end

