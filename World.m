classdef World
    % Stores the ground truth of the world and all robot ground truth
    % positions.
    
    properties
        map % occupancy grid of the world (ground truth in global frame)
        robots % a cell array of all robots.
        robotGroundTruthStates % a cell array which serves as the absolute robot positons.
    end
    
    methods
        function obj = World(pathToMapImage, mapResolution, nRobots)
            obj.map = Map(pathToMapImage, mapResolution);
            
            rng(1); % make deterministic
            
            % create all robots
            obj.robots = {};
            obj.robotGroundTruthStates = {};
            mapSize = length(obj.map.occupancyGrid)*2;
            for id = (1:nRobots)
                obj.robots{id} = Robot(id, mapSize, mapResolution);
                
                % find a valid random starting position
                positionValid = false;
                while (~positionValid)
                    width = length(obj.map.occupancyGrid) * mapResolution;
                    position = [(rand()*2 - 1); (rand()*2 - 1)] * width/2;
                    
                    if obj.map.get(position) == OccupancyState.UNOCCUPIED
                        positionValid = true;
                        
                        obj.robotGroundTruthStates{id} = RobotState([position; rand()*2*pi], diag([0;0;0]));
                    end
                end
            end
        end
        
        
    end
end

