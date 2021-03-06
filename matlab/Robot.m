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
        
        % last lidar measurement
        newestLidarMeasurement;
        
        % waypoints: [[x1;y1], [x2;y2], [x3;y3], ...] meters in local
        % frame.
        waypoints;
        
        pathValid;
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
            
            obj.waypoints = [];
            
            obj.pathValid = false;
            
        end
        
        % using the current state estimate, drive the vehicle and or plan a
        % new path. returns a change in state for the simulation.
        function [dx] = motionUpdate(obj, dt)
            
            % create a new path to a frontier if there is no more to drive.
            [~,nwp] = size(obj.waypoints);
            if ~nwp || (nwp==1 && (norm(obj.waypoints(1:2,1) - obj.localState.pos) < Settings.WAYPOINT_DISTANCE_THRESHOLD))
                % find the best frontier to explore.
                [frontierPos, nFrontiers] = findBestFrontier(obj, [0;0], Settings.DEFAULT_EXPLORATION_RADIUS);
                
                if ~nFrontiers
                    fprintf('No Frontiers in the local area... big search\n');
                    [frontierPos, nFrontiers] = findBestFrontier(obj, obj.localState.pos, -1);
                    
                    if ~nFrontiers
                        fprintf('Exploration Finished!\n');
                        dx = zeros(3, 1);
                        return;
                    end
                end
                
                
                % generate a path to the selected frontier.
                obj.waypoints = obj.generatePath(obj.localState.pos, frontierPos);
            end
            
            
            
            
            % now that there is a path to follow, check how far we have
            % already gone, and move towards next goal...
            stopFlag = false;
            while ~stopFlag && ~isempty(obj.waypoints)
                if norm(obj.waypoints(1:2,1) - obj.localState.pos) < Settings.WAYPOINT_DISTANCE_THRESHOLD
                    obj.waypoints = obj.waypoints(1:2, 2:end); % cut the first waypoint
                else
                    stopFlag = true;
                end
            end
            
            
            if isempty(obj.waypoints)
                dx = zeros(3, 1);
                return;
            end
            
            
            [r, c] = obj.localMap.position2MapIndex(obj.waypoints(1:2, end));
            if obj.localMap.occupancyGrid(r, c) == OccupancyState.OCCUPIED
                obj.waypoints = [];
                dx = zeros(3, 1);
                return;
            end
            
            % move towards the goal
            delta = obj.waypoints(1:2, 1) - obj.localState.pos;
            dist = norm(delta);
            newTheta = atan2(delta(2), delta(1));
            
            newDist = min(Settings.FORWARD_VELOCITY*dt, dist);
            
            if dist < 1e-16
                dPos = [0;0];
            else
                dPos = delta/dist * newDist;
            end
            
            dTheta = newTheta - obj.localState.theta;
            
            if norm(dPos) < 1e-3
                disp('nah');
            end
            
            dx = [dPos; dTheta];
        end
        
        % update the local occupancy grid and exploration tree using the
        % synthetic lidar measurement.
        function [] = senseUpdate(obj, lidarMeasurement)
            % update the local occupancy grid with this measurement
            obj.newestLidarMeasurement = lidarMeasurement;
            
            obj.mapUpdate(lidarMeasurement); % updates the local occupancy grid.
            
        end
        
        function [] = mapUpdate(obj, lidarMeasurement)
            localBearings = lidarMeasurement.bearings + obj.localState.theta; % transform the measurements into the current frame
            [r, c] = obj.localMap.position2MapIndex(obj.localState.pos); % the center point
            
            for idx = (1:length(localBearings))
                if lidarMeasurement.ranges(idx) < 0
                    % set all nodes unoccupied up to sense range.
                    for l = (0:0.5:(Settings.SENSE_RANGE/obj.localMap.mapResolution))
                        col = round(c + cos(localBearings(idx))*l);
                        row = round(r + sin(localBearings(idx))*l);
                        try
                            if obj.localMap.occupancyGrid(row, col) == OccupancyState.UNKOWN
                                obj.localMap.occupancyGrid(row, col) = OccupancyState.UNOCCUPIED;
                            end
                        catch
                            continue;
                        end
                        
                    end
                else
                    for l = (0:0.5:(lidarMeasurement.ranges(idx)/obj.localMap.mapResolution))
                        col = round(c + cos(localBearings(idx))*l);
                        row = round(r + sin(localBearings(idx))*l);
                        try
                            if obj.localMap.occupancyGrid(row, col) == OccupancyState.UNKOWN
                                obj.localMap.occupancyGrid(row, col) = OccupancyState.UNOCCUPIED;
                            end
                        catch
                            continue
                        end
                    end
                    
                    col = round(c + cos(localBearings(idx))*lidarMeasurement.ranges(idx)/obj.localMap.mapResolution);
                    row = round(r + sin(localBearings(idx))*lidarMeasurement.ranges(idx)/obj.localMap.mapResolution);
                    
                    try
                        obj.localMap.occupancyGrid(row, col) = OccupancyState.OCCUPIED;
                    catch
                        continue
                    end
                    
                end
            end
        end
        
        % idk if this is the best way to do this...
        % The world passes in a measurement of the transformation from this
        % robot to the other.
        function [] = communicationUpdate(obj, otherRobot, T_me_them)
            %fprintf('communicated.\n');
            
            % compute a line in the local frame.
            T_local_me = obj.localState.transformation();
            if norm(T_local_me(1:2, 3)) > 1e-9
                center = T_me_them(1:2, 3)/2;
                
                pt = T_local_me(1:2, 1:2)*center + T_local_me(1:2, 3);
                
                normal = -(T_local_me(1:2, 1:2)*center); % vec pointing toward me.
                normal = normal/norm(normal);
                
                
                if ~obj.linesOfExploration.hasLine(otherRobot.id)
                    if otherRobot.linesOfExploration.hasLine(obj.id)
                    % take this robots line of exploration and just negate
                    % it
                    ln = otherRobot.linesOfExploration.lineMap(otherRobot.linesOfExploration.id2key(obj.id));
                    obj.linesOfExploration.addLine(ln.Point, -ln.Normal, otherRobot.id);
                    else
                    % transform to global frame
                    T_global_local = obj.startingState.transformation();
                    
                    obj.linesOfExploration.addLine(T_global_local(1:2, 1:2)*pt + T_global_local(1:2, 3), T_global_local(1:2, 1:2)*normal, otherRobot.id);
                    %obj.linesOfExploration.addLine(pt, normal, otherRobot.id);
                    fprintf('added line\n');
                    end
                end
            end
            
        end
        
        % generate a star path. inputs and outputs are in the local frame
        % in meters.
        function [waypoints] = generatePath(obj, startingPos, goalPos)
            waypoints = obj.generatePathPRM(startingPos, goalPos);
        end
        
        
        % generate a star path. inputs and outputs are in the local frame
        % in meters.
        function [waypoints] = generatePathPRM(obj, startingPos, goalPos)
            og = robotics.OccupancyGrid(double(obj.localMap.occupancyGrid) / double(OccupancyState.OCCUPIED), 1/obj.localMap.mapResolution);
            
            R = [1,0; 0,-1];
            sz = size(obj.localMap.occupancyGrid);
            t = sz(1)*obj.localMap.mapResolution/2;
            
            start = R*startingPos + t;
            goal = R*goalPos + t;
            
            % add buffer
            %inflate(og, 1, 'grid');
            
            setOccupancy(og, start', 0)
            setOccupancy(og, goal', 0)
            
            prm = robotics.PRM;
            prm.Map = og;
            prm.NumNodes = Settings.PRM_NODES;
            prm.ConnectionDistance = Settings.PRM_MAXDIST;
            
            
            path = findpath(prm, start', goal');
            
            if isempty(path)
                % try once more to generate the path.
                prm.NumNodes = Settings.PRM_NODES*10;
                path = findpath(prm, start', goal');
                
                if isempty(path)
                    fprintf('Failed to find valid path.')
                    obj.pathValid = false;
                    path = [start, goal]';
                else 
                    obj.pathValid = true;
                end
            end
            
            waypoints = R*(path' - t);
            
            
        end
        
        function [globalState] = getGlobalStateEstimate(obj)
            R = [cos(obj.startingState.theta), -sin(obj.startingState.theta); sin(obj.startingState.theta), cos(obj.startingState.theta)];
            
            newTheta = obj.startingState.theta + obj.localState.theta;
            
            newPos = R*obj.localState.pos + obj.startingState.pos;
            
            % transform and add the uncertainty.
            T = [R,zeros(2,1); zeros(1,2),1];
            
            globalState = RobotState([newPos; newTheta], obj.startingState.Sigma + T*obj.localState.Sigma*T');
        end
        
    end
end

