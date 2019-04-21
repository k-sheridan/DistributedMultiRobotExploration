function [] = drawGroundTruth(world)
% draws the ground truth of the map and all robots in their absolute
% position.
hold on

width = length(world.map.occupancyGrid)*world.map.mapResolution;
imshow(world.map.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
col = linspace(1, 0, 255)';
set(gca, 'Colormap', [col, col, col]);

hold on

% draw robots
for rbt = world.robotGroundTruthStates
    drawRobot(rbt{1}, [1, 0, 0]);
end

end

