function [bestFrontierPosition, nFrontiers] = findBestFrontier(robot, targetPosition, searchRadius)
% Finds and selects the best frontier position given a map,  target
% location (m), robot position (m), and search radius (m).
% target position should be in local frame.

% weights
kPos = 1;
kTarget = 1;
kLine = 1;

[targetRow, targetCol] = robot.localMap.position2MapIndex(targetPosition);

% convert the search radius into a pixel/occupancy unit
pixelSearchRadius = round(searchRadius / robot.localMap.mapResolution);

sz = size(robot.localMap.occupancyGrid);

rowUpper = min(targetRow+pixelSearchRadius, sz(1));
rowLower = max(targetRow-pixelSearchRadius, 1);
colUpper = min(targetCol+pixelSearchRadius, sz(2));
colLower = max(targetCol-pixelSearchRadius, 1);

% look over all the possible indices for a frontier
nFrontiers = 0;
bestFrontierPosition = targetPosition;
bestFrontierScore = realmax;

for row = (rowLower:rowUpper)
    for col = (colLower:colUpper)
        if (robot.localMap.occupancyGrid(row, col) == OccupancyState.UNKOWN) && any(any(robot.localMap.occupancyGrid(row-1:row+1, col-1:col+1) == OccupancyState.UNOCCUPIED))
            % evaluate the score
            frontierPos = robot.localMap.mapIndex2Position(row, col);
            score = kPos * norm(frontierPos - robot.localState.pos) + kTarget * norm(frontierPos - targetPosition)...
                + kLine * robot.linesOfExploration.computeCost(robot.localState.pos);
           
            if score < bestFrontierScore
                bestFrontierPosition = frontierPos;
                nFrontiers = nFrontiers + 1;
            end
            
        end
    end
end

end

