function [] = drawLocalMap(robot)
% draws a robots local occupancy in its estimated global frame.
width = length(robot.localMap.occupancyGrid)*robot.localMap.mapResolution;

% transform local map into estimated global frame.
th = robot.startingState.theta;

tImg = imtranslate(imwarp(robot.localMap.occupancyGrid, affine2d([cos(th), sin(th), 0; -sin(th), cos(th), 0; 0, 0, 1])), robot.startingState.pos/robot.localMap.mapResolution);

sz = size(tImg);
assert(sz(1)==sz(2));

newWidth = sz(1)*robot.localMap.mapResolution;

%imshow(robot.localMap.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
imshow(tImg, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED] , 'XData', [-newWidth/2, newWidth/2], 'Ydata', [-newWidth/2, newWidth/2]);
col = linspace(1, 0, 255)';
set(gca, 'Colormap', [col, col, col]);

globalState = robot.getGlobalStateEstimate();

hold on

plot(globalState.pos(1), globalState.pos(2), 'o');

% transform and draw the path being followed.
path = [robot.localState.pos, robot.waypoints];

pathT = robot.startingState.rotation() * path + robot.startingState.pos;

plot(pathT(1, :), pathT(2, :), 'b-');

end

