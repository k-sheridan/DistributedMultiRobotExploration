classdef ExplorationQuadTree < handle
    % Efficiently stores the explored areas of the map.
    
    properties
        tree; % the actual tree datastructure
        width;
        levels; % how deep the tree goes
    end
    
    methods
        function obj = ExplorationQuadTree(width, levels)
            obj.tree = ExploredQuad([0;0], width, 0, levels);
            obj.width = width;
            obj.levels = levels;
        end
        
        % given a position, this returns a quadrant number path.
        function [path] = position2Path(obj, pos, level)
            path = ones(1, level);
            
            center = [0;0];
            currentWidth = obj.width;
            for idx = (1:level)
                delta = pos-center;
                q = vec2Quadrant(delta);
                path(idx) = q;
                
                
                switch q
                    case 1
                        delta = [1;1];
                    case 2
                        delta = [-1;1];
                    case 3
                        delta = [-1;-1];
                    case 4
                        delta = [1;-1];
                    otherwise
                        error('wtf');
                end
                
                center = center + delta * currentWidth/4;
                
                currentWidth = currentWidth/2;
                
            end
        end
        
        function [pos] = path2Position(obj, path)
            
            currentWidth = obj.width;
            pos = [0;0];
            
            for q = path
                
                switch q
                    case 1
                        delta = [1;1];
                    case 2
                        delta = [-1;1];
                    case 3
                        delta = [-1;-1];
                    case 4
                        delta = [1;-1];
                    otherwise
                        error('wtf');
                end
                
                
                pos = pos + delta * currentWidth/4;
                
                currentWidth = currentWidth/2;
                
            end
        end
        
        % answers if this position was explored
        function [explored] = get(obj, pos)
            path = obj.position2Path(pos, obj.levels);
            
            currentQuad = obj.tree;
            explored = false;
            for q = path
                if currentQuad.explored
                    explored = true;
                    return;
                end
                
                if isempty(currentQuad.quadrants)
                    return;
                end
                
                currentQuad = currentQuad.quadrants{q};
            end
        end
        
        
        % answers if this position was explored
        function [] = set(obj, pos, explored, level)
            path = obj.position2Path(pos, level);
            
            currentQuad = obj.tree;
            
            for q = path
                if isempty(currentQuad.quadrants)
                    return;
                end
                currentQuad = currentQuad.quadrants{q};
                
                if currentQuad.depth == level
                    if explored
                        currentQuad.setExplored();
                    else
                        currentQuad.explored = false;
                    end
                end
            end
        end
        
        
    end
end

