classdef World
    % Stores the ground truth of the world and all robot ground truth
    % positions.
    
    properties
        map % occupancy grid of the world (ground truth in global frame)
        robots % a cell array of all robots.
        robotGroundTruthStates % a cell array which serves as the absolute robot positons.
        startingStatesGroundTruth % cell array of initial states.
        t; % timer, seconds
    end
    
    methods
        function obj = World(pathToMapImage, mapResolution, nRobots)
            obj.map = Map(pathToMapImage, mapResolution);
            
            rng(1); % make deterministic
            
            obj.t = 0;
            
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
                    
                    if ~obj.map.isRegionOccupied(position, Settings.ROBOT_RADIUS)
                        positionValid = true;
                        
                        obj.robotGroundTruthStates{id} = RobotState([position; rand()*2*pi], diag([0;0;0]));
                        %obj.robotGroundTruthStates{id}.theta = 0;
                        
                        obj.startingStatesGroundTruth{id} = RobotState([obj.robotGroundTruthStates{id}.pos; obj.robotGroundTruthStates{id}.theta], diag([0;0;0]));
                    end
                end
            end
        end
        
        
        % main simulation running function. handles communication, sensing,
        % control, and simulation.
        function [] = run(obj, dt)
            
            % communicate
            obj.communicate();
            
            % sense
            for idx = (1:length(obj.robots))
                lm = obj.generateLidarMeasurement(obj.robotGroundTruthStates{idx});
                obj.robots{idx}.senseUpdate(lm);
            end
            
            % motion
            for idx = (1:length(obj.robots))
                [dx] = obj.robots{idx}.motionUpdate(dt);
                
                % ground truth update:
                % must transform the delta from local to global.
                globaldx = [obj.startingStatesGroundTruth{idx}.rotation * dx(1:2, 1); dx(3, 1)];
                %globaldx = dx;
                obj.robotGroundTruthStates{idx}.update(globaldx);
                % odom update:
                obj.robots{idx}.localState.update(dx);
                
            end
            
            obj.t = obj.t + dt;
            
        end
        
        % acts as a fake peer to peer communication
        function [] = communicate(obj)
            for recIdx = (1:length(obj.robots))
                for sendIdx = (1:length(obj.robotGroundTruthStates))
                    % check if the two robots can communicate reliably.
                    d = norm(obj.robotGroundTruthStates{recIdx}.pos - obj.robotGroundTruthStates{sendIdx}.pos);
                    
                    % within range.
                    if d < Settings.COMM_RANGE && sendIdx ~= recIdx
                        T_w_rec = obj.robotGroundTruthStates{recIdx}.transformation();
                        T_w_send = obj.robotGroundTruthStates{sendIdx}.transformation();
                        
                        % T_rec_send = T_rec_w * T_w_send
                        T_rec_send = inv(T_w_rec) * T_w_send;
                        
                        obj.robots{recIdx}.communicationUpdate(obj.robots{sendIdx}, T_rec_send);
                        
                        
                        % run the consensus update if the ids are in
                        % ascending order. This ensures the update only
                        % runs once.
                        if recIdx < sendIdx
                            obj.consensusUpdate(recIdx, sendIdx);
                        else
                            obj.consensusUpdate(recIdx, sendIdx);
                        end
                        
                    end
                    
                end
            end
        end
        
        
        % runs the consensus update.
        function [] = consensusUpdate(obj, id1, id2)
            % T_global_global = r1_global_local * r1_local_r1 * T_r1_r2 * inv(r2_global_local * r2_local_r2)
            T_r1_r2 = inv(obj.robotGroundTruthStates{id1}.transformation()) * obj.robotGroundTruthStates{id2}.transformation();
            
            r1_global_local = obj.robots{id1}.startingState.transformation();
            r2_global_local = obj.robots{id2}.startingState.transformation();
            
            r1_local_r1 = obj.robots{id1}.localState.transformation();
            r2_local_r2 = obj.robots{id2}.localState.transformation();
            
            T_globalR1_globalR2 = r1_global_local * r1_local_r1 * T_r1_r2 * inv(r2_global_local * r2_local_r2)
            
            theta = atan2(T_globalR1_globalR2(2, 1), T_globalR1_globalR2(1, 1));
            pos = T_globalR1_globalR2(1:2, 3);
            
            obj.robots{id2}.startingState.update([pos;theta]/2);
            
        end
        
        
        % creates a lidar measurement around the robot state.
        function [lidarMeasurement] = generateLidarMeasurement(obj, robotState)
            % s = theta*r => mapRes = dTheta*senseRange => dTheta =
            % (mapRes/senseRange) * 0.8;
            dTheta = (obj.map.mapResolution/Settings.SENSE_RANGE) * 0.5;
            
            lidarMeasurement = LidarMeasurement();
            lidarMeasurement.bearings = (0:dTheta:2*pi);
            lidarMeasurement.ranges = -ones(1, length(lidarMeasurement.bearings));
            
            [r, c] = obj.map.position2MapIndex(robotState.pos); % center index
            
            for idx = (1:length(lidarMeasurement.bearings))
                for l = (0:0.5:(Settings.SENSE_RANGE/obj.map.mapResolution))
                    col = round(c + cos(lidarMeasurement.bearings(idx))*l);
                    row = round(r + sin(lidarMeasurement.bearings(idx))*l);
                    
                    try
                        if obj.map.occupancyGrid(row, col) == OccupancyState.OCCUPIED
                            lidarMeasurement.ranges(idx) = l*obj.map.mapResolution;
                            break;
                        end
                    catch
                        %fprintf('Wall found.\n');
                        lidarMeasurement.ranges(idx) = l*obj.map.mapResolution;
                        break;
                    end
                end
            end
            
            lidarMeasurement.bearings = lidarMeasurement.bearings - robotState.theta;
            
        end
        
        
    end
end

