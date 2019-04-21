function [] = drawLocalMap(robot)
% draws a robots local occupancy in its estimated global frame.
width = length(robot.localMap.occupancyGrid)*robot.localMap.mapResolution;
imshow(robot.localMap.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
col = linspace(1, 0, 255)';
set(gca, 'Colormap', [col, col, col]);

end

