classdef ExplorationQuadTree
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
    end
end

