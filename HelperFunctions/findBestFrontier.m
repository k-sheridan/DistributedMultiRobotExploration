function [bestFrontierPosition, nFrontiers] = findBestFrontier(map, robotPosition, targetPosition, searchRadius)
% Finds and selects the best frontier position given a map,  target
% location (m), robot position (m), and search radius (m).

% weights
kPos = 1;
kTarget = 1;

[targetRow, targetCol] = map.position2MapIndex(targetPosition);

% convert the search radius into a pixel/occupancy unit
pixelSearchRadius = round(searchRadius / map.mapResolution);

sz = size(map.occupancyGrid);

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
        if (map.occupancyGrid(row, col) == OccupancyState.UNKOWN) && any(any(map.occupancyGrid(row-1:row+1, col-1:col+1) == OccupancyState.UNOCCUPIED))
            % evaluate the score
            frontierPos = map.mapIndex2Position(row, col);
            score = kPos * norm(frontierPos - robotPosition) + kTarget * norm(frontierPos - targetPosition);
           
            if score < bestFrontierScore
                bestFrontierPosition = frontierPos;
                nFrontiers = nFrontiers + 1;
            end
            
        end
    end
end

end

