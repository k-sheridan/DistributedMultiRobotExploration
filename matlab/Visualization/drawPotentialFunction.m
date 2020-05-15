function [] = drawPotentialFunction(robot)

sz = size(robot.localMap.occupancyGrid);
image = zeros(sz(1), sz(1));

for row = (sz(1)*0.25:sz(1)*0.75)
    for col = (sz(1)*0.25:sz(1)*0.75)
        image(row, col) = exp(-robot.linesOfExploration.computeCost(robot.localMap.mapIndex2Position(row, col)));
        
    end
end

imagesc(image, [0, 1]);

end

