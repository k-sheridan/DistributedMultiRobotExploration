function [] = drawLocalMap(robot)
% draws a robots local occupancy in its estimated global frame.
width = length(robot.localMap.occupancyGrid)*robot.localMap.mapResolution;

% transform local map into estimated global frame.
th = robot.startingState.theta;

%tImg = imtranslate(imwarp(robot.localMap.occupancyGrid, affine2d([cos(th), sin(th), 0; -sin(th), cos(th), 0; 0, 0, 1])), robot.startingState.pos/robot.localMap.mapResolution);
tImg = (imwarp(robot.localMap.occupancyGrid, affine2d([cos(th), sin(th), 0; -sin(th), cos(th), 0; 0, 0, 1])));

sz = size(tImg);
%assert(sz(1)==sz(2));

newWidth = sz(1)*robot.localMap.mapResolution;

%imshow(robot.localMap.occupancyGrid, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED], 'XData', [-width/2, width/2], 'Ydata', [-width/2, width/2]);
imshow(tImg, [OccupancyState.UNOCCUPIED, OccupancyState.OCCUPIED] , 'XData', [-newWidth/2, newWidth/2], 'Ydata', [-newWidth/2, newWidth/2]);
col = linspace(1, 0, 255)';
set(gca, 'Colormap', [col, col, col]);

scale = 10;
line(-[robot.startingState.pos(1), robot.startingState.pos(1)+scale], [robot.startingState.pos(2), robot.startingState.pos(2)], 'Color', [1, 0, 0]);
line(-[robot.startingState.pos(1), robot.startingState.pos(1)], [robot.startingState.pos(2), robot.startingState.pos(2)+scale], 'Color', [0, 1, 0]);

globalState = robot.getGlobalStateEstimate();

hold on

plot(globalState.pos(1) - robot.startingState.pos(1), globalState.pos(2) - robot.startingState.pos(2), 'o');

% transform and draw the path being followed.
path = [robot.localState.pos, robot.waypoints];

%pathT = robot.startingState.rotation() * path + robot.startingState.pos;
pathT = robot.startingState.rotation() * path;

plot(pathT(1, :), pathT(2, :), 'b-');


% draw lines of exploration
rot = [0, -1; 1, 0];
scale = 10;
for loe = robot.linesOfExploration.lineMap.values
    pt1 = rot*loe{1}.Normal * scale + loe{1}.Point;
    pt2 = -rot*loe{1}.Normal * scale + loe{1}.Point;
    
    line([pt1(1), pt2(1)], [pt1(2), pt2(2)], 'Color', [0, 1, 0]);
    ptArr = loe{1}.Point + loe{1}.Normal*scale/2;
    line([loe{1}.Point(1), ptArr(1)], [loe{1}.Point(2), ptArr(2)], 'Color', [0, 1, 0]);
    
end


daspect('auto');

end

