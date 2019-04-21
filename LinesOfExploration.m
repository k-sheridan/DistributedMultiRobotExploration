classdef LinesOfExploration
    % A class which describes the area a given robot should explore
    % line: struct: Point, Normal
    properties
        lineMap;
    end
    
    methods
        function obj = LinesOfExploration()
            obj.lineMap = containers.Map();
        end
        
        function [key] = id2key(obj, id)
            key = string(id);
        end
        
        function [] = addLine(obj, pt, normal, id)
            obj.lineMap(obj.id2key(id)) = struct('Point', pt, 'Normal', normal/norm(normal));
        end
        
        % computes the exploration cost for a given point.
        % the goal of this heuristic is to bias the exploration away from
        % other robots. It is inspired by the vornoi based multi-robot
        % exploration.
        function [cost] = computeCost(obj, pos)
            cost = 0;
            for line = obj.lineMap.values
                % compute the distance to the point projected onto the line normal.
                % assumes the normal is of unit length.
                d = line{1}.Normal' * (pos - line{1}.Point);
                
                cost = cost + exp(-d);
            end
            if ~isempty(obj.lineMap.values)
                cost = cost / length(obj.lineMap.values);
            end
        end
        
    end
end

