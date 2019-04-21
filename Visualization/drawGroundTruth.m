function [] = drawGroundTruth(world)
% draws the ground truth of the map and all robots in their absolute
% position.
hold on

width = length(world.map.occupancyGrid)*world.map.mapResolution;
imshow(world.map.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
set(gca, 'Colormap', [1, 1, 1; 0, 0, 0])

end

