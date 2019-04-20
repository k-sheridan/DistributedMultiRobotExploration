classdef Map < handle
    %stores occupancy grid and exploration status
    
    properties
        occupancyGrid; % an occupancy grid which is the actual map, origin at center.
        mapResolution; % meters per pixel
    end
    
    methods
        function obj = Map(pathToMapImage, resolution)
            img = imread(pathToMapImage);
            % take the first channel and call it the occupancy grid.
            obj.occupancyGrid = uint8((img(:, :, 1)>0)*OccupancyState.OCCUPIED);
            obj.mapResolution = resolution;
            
            % the map must be square for the quadtree.
            sz = size(obj.occupancyGrid);
            assert(sz(1) == sz(2));
        end
        
        % converts [x;y] coordinate to the occupancy grid index
        function [row, col] = position2MapIndex(obj, pos)
            sz = size(obj.occupancyGrid);
            centerIdx = sz/2;
            temp = round(centerIdx' + pos/obj.mapResolution);
            row = temp(1);
            col = temp(2);
        end
        
        % converts row column to coordinate
        function [pos] = mapIndex2Position(obj, row, col)
            sz = size(obj.occupancyGrid);
            centerIdx = sz/2;
            pos = ([row;col] - centerIdx') * obj.mapResolution;
        end
        
        % sets the entire occupancy grid to unknown
        function [] = reset(obj)
            sz = size(obj.occupancyGrid);
            obj.occupancyGrid = ones(sz(1), sz(2)) * OccupancyState.UNKOWN;
        end
        
        % checks if a given map index is a frontier
        function [frontier] = isFrontier(obj, row, col)
            frontier = (obj.occupancyGrid(row, col) == OccupancyState.UNKOWN) && any(any(obj.occupancyGrid(row-1:row+1, col-1:col+1) == OccupancyState.UNOCCUPIED));
        end
        
        % get the occupancy state of a position
        function [occ] = get(obj, pos)
            [r, c] = obj.position2MapIndex(pos);
            occ = obj.occupancyGrid(r, c);
        end
        
        function [] = set(obj, pos, occ)
            [r, c] = obj.position2MapIndex(pos);
            obj.occupancyGrid(r, c) = occ;
        end
    end
end

