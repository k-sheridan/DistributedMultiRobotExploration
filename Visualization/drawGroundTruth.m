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


for idx = (1:length(world.robots))
    if ~isempty(world.robots{idx}.newestLidarMeasurement)
        
        nb = (world.robots{idx}.newestLidarMeasurement.bearings + world.robotGroundTruthStates{idx}.theta);
        %nb = world.robots{idx}.newestLidarMeasurement.bearings;
        
        tempRanges = world.robots{idx}.newestLidarMeasurement.ranges;
        for innerIdx = (1:length(world.robots{idx}.newestLidarMeasurement.ranges))
            if tempRanges(innerIdx) < 0
                tempRanges(innerIdx) = 0;
            end
        end
        
        points = [cos(nb); sin(nb)].*tempRanges + world.robotGroundTruthStates{idx}.pos;
        
%         pixels = zeros(2, length(nb));
%         
%         for idx = (1:length(nb))
%             [r,c] = world.map.position2MapIndex(points(1:2, idx));
%             pixels(1:2, idx) = [c;r];
%         end
        
        %plot(pixels(1, :), pixels(2, :), 'r-')
        plot(points(1, :), points(2, :), 'b-')
        
    end
end

daspect('auto');

end

