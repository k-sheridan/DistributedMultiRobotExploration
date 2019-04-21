function [] = drawLocalMap(robot)
% draws a robots local occupancy in its estimated global frame.
width = length(robot.localMap.occupancyGrid)*robot.localMap.mapResolution;

% transform local map into estimated global frame.
th = robot.startingState.theta;

tImg = imtranslate(imwarp(robot.localMap.occupancyGrid, affine2d([cos(th), sin(th), 0; -sin(th), cos(th), 0; 0, 0, 1])), robot.startingState.pos);

%imshow(robot.localMap.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
imshow(tImg, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
col = linspace(1, 0, 255)';
set(gca, 'Colormap', [col, col, col]);

end

