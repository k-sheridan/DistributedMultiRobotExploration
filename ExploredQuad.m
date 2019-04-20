classdef ExploredQuad < handle
    % The nested datatype describing how the map has been explored
    % follows the standard quadrant layout, quadrant angle = (pi/4 +
    % n*pi/2); n = quadrant number
    
    properties
        quadrants; % a set of 4 quadrants. each is the same type as this
        center; % the center position (m) of this quad
        width; % the width of this quad (m)
        depth; % the depth of the quadtree. 0 is the top...
        explored; % boolean stating if this quad has been explored
    end
    
    methods
        function obj = ExploredQuad(center, width, depth, levelsToGo)
            obj.explored = false;
            obj.center = center;
            obj.width = width;
            obj.depth = depth;
            
            % unless this is the last quad add sub quads
            if levelsToGo > 0
                
                levelsToGo = levelsToGo - 1;
                depth = depth + 1;
                
                q1 = ExploredQuad(center + [width/4; width/4], width/2, depth, levelsToGo);
                q2 = ExploredQuad(center + [-width/4; width/4], width/2, depth, levelsToGo);
                q3 = ExploredQuad(center + [-width/4; -width/4], width/2, depth, levelsToGo);
                q4 = ExploredQuad(center + [width/4; -width/4], width/2, depth, levelsToGo);
                
                obj.quadrants = {q1, q2, q3, q4};
                
            end
        end
        
    end
end

