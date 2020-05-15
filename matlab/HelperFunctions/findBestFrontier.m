function [bestFrontierPosition, nFrontiers] = findBestFrontier(robot, targetPosition, searchRadius)
% Finds and selects the best frontier position given a map,  target
% location (m), robot position (m), and search radius (m).
% target position should be in local frame.

% -1 search radius means search the entire map.

% weights
kPos = 1e-4;
kTarget = 0;
kLine = 1;

[targetRow, targetCol] = robot.localMap.position2MapIndex(targetPosition);

% convert the search radius into a pixel/occupancy unit
pixelSearchRadius = round(searchRadius / robot.localMap.mapResolution);

sz = size(robot.localMap.occupancyGrid);

rowUpper = min(targetRow+pixelSearchRadius, sz(1)-1);
rowLower = max(targetRow-pixelSearchRadius, 2);
colUpper = min(targetCol+pixelSearchRadius, sz(2)-1);
colLower = max(targetCol-pixelSearchRadius, 2);

if searchRadius < 0
    rowUpper = sz(1)-1;
    rowLower = 2;
    colUpper = sz(2)-1;
    colLower = 2;
end

% look over all the possible indices for a frontier
nFrontiers = 0;
bestFrontierPosition = targetPosition;
bestFrontierScore = realmax;

T_G_R = robot.startingState.transformation() * robot.localState.transformation();
%T_G_R = robot.localState.transformation();

for row = (rowLower:rowUpper)
    for col = (colLower:colUpper)
        if (robot.localMap.occupancyGrid(row, col) == OccupancyState.UNOCCUPIED) && any(any(robot.localMap.occupancyGrid(row-1:row+1, col-1:col+1) == OccupancyState.UNKOWN)) && ~any(any(robot.localMap.occupancyGrid(row-1:row+1, col-1:col+1) == OccupancyState.OCCUPIED))
            % evaluate the score
            frontierPos = robot.localMap.mapIndex2Position(row, col);
            score = kPos * norm(frontierPos - robot.localState.pos) + kTarget * norm(frontierPos - targetPosition)...
                + kLine * (robot.linesOfExploration.computeCost(T_G_R(1:2, 3)));
           
            
            if score < bestFrontierScore
                bestFrontierPosition = frontierPos;
                bestFrontierScore = score;
                
            end
            
            nFrontiers = nFrontiers + 1;
            
        end
    end
end

end

