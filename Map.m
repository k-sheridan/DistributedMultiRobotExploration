classdef Map < handle
    %stores occupancy grid and exploration status
    
    properties
        occupancyGrid; % an occupancy grid which is the actual map, origin at center.
        mapResolution; % meters per pixel
    end
    
    methods
        
        function obj = Map(pathToMapImage, resolution)
            if nargin == 0
                return;
            end
            
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
            
            ratio = ((pos + (sz(1)*obj.mapResolution)/2) / (sz(1)*obj.mapResolution));
            
            temp = round([1;1] + (sz(1)-1) * ratio);
            row = temp(2);
            col = temp(1);
        end
        
        % converts row column to coordinate
        function [pos] = mapIndex2Position(obj, row, col)
            sz = size(obj.occupancyGrid)
            
            ratio = (([col;row] - 1) / (sz(1)-1))
            
            pos = ratio * sz(1)*obj.mapResolution - (sz(1)*obj.mapResolution/2);
            
        end
        
        function [] = initialize(obj, size, resolution)
            obj.occupancyGrid = ones(size) * OccupancyState.UNKOWN;
            obj.mapResolution = resolution;
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
        
        % tests if a region surrounding the position is occupied
        function [occupied] = isRegionOccupied(obj, pos, pixelRad)
            [r, c] = obj.position2MapIndex(pos);
            try
                occupied = any(any(obj.occupancyGrid(r-pixelRad:r+pixelRad, c-pixelRad:c+pixelRad)));
            catch
                occupied = true;
            end
        end
        
        function [] = set(obj, pos, occ)
            [r, c] = obj.position2MapIndex(pos);
            obj.occupancyGrid(r, c) = occ;
        end
        
        % tests if a square region of the occupancy grid has been explored
        function [explored] = isRegionExplored(obj, center, width)
            [targetRow, targetCol] = obj.position2MapIndex(center);
            
            % convert the search radius into a pixel/occupancy unit
            pixelSearchRadius = round(width / obj.mapResolution);
            
            sz = size(obj.occupancyGrid);
            
            rowUpper = min(targetRow+pixelSearchRadius, sz(1));
            rowLower = max(targetRow-pixelSearchRadius, 1);
            colUpper = min(targetCol+pixelSearchRadius, sz(2));
            colLower = max(targetCol-pixelSearchRadius, 1);
            
            explored = ~any(any(obj.occupancyGrid(rowLower:rowUpper, colLower:colUpper) == OccupancyState.UNKOWN));
        end
    end
end

