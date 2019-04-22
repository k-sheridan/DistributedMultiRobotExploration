function [] = drawLocalMap(robot)
% draws a robots local occupancy in its estimated global frame.
width = length(robot.localMap.occupancyGrid)*robot.localMap.mapResolution;

% transform local map into estimated global frame.
th = robot.startingState.theta;

tImg = imtranslate(imwarp(robot.localMap.occupancyGrid, affine2d([cos(th), sin(th), 0; -sin(th), cos(th), 0; 0, 0, 1])), robot.startingState.pos);

%imshow(robot.localMap.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
imshow(tImg, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED]);
col = linspace(1, 0, 255)';
set(gca, 'Colormap', [col, col, col]);


% TODO after this transformation, the map resolution does not change. so draw
% robots wrt to the center index.

centerIndex = (size(tImg)-1)'/2 + 1;

globalState = robot.getGlobalStateEstimate();

robotRow = centerIndex(1) + globalState.pos(2)/robot.localMap.mapResolution
robotCol = centerIndex(2) + globalState.pos(1)/robot.localMap.mapResolution

hold on

plot(robotCol, robotRow, 'o');


end

