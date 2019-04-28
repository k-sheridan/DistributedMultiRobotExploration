function [percentage] = computeExplorationPercentage(world)

freeGT = 0;
freeExplored = 0;
%precompute
TCells = {};
for idx = (1:length(world.robots))
                % compute the row col to check in the robots map.
                T_w_r = world.robotGroundTruthStates{idx}.transformation();
                T_L_r = world.robots{idx}.localState.transformation();
                T_L_w = T_L_r * inv(T_w_r);
                TCells{end+1} = T_L_w;
end


for row = (2:length(world.map.occupancyGrid)-1)
     for col = (2:length(world.map.occupancyGrid)-1)
        if world.map.occupancyGrid(row, col) == OccupancyState.UNOCCUPIED
            freeGT = freeGT + 1;
            
            %check the robots to see if these points were explored
            for idx = (1:length(world.robots))
                
                pt = world.map.mapIndex2Position(row, col);
                T_L_w = TCells{idx};
                pt_local = T_L_w(1:2, 1:2) * pt + T_L_w(1:2, 3);
                
                [r_L, c_L] = world.robots{idx}.localMap.position2MapIndex(pt_local);
                
                try
                    if world.robots{idx}.localMap.occupancyGrid(r_L:r_L, c_L:c_L) == OccupancyState.UNOCCUPIED
                        freeExplored = freeExplored + 1;
                        break;
                    end
                catch
                    
                end
            end
            
            
        end
     end
end

freeGT;
freeExplored;

percentage = freeExplored/freeGT




% % count the number of boundaries expected.
% boundsGT = 0;
% % compute the combined exploration status
% boundsExplored = 0;
% for row = (2:length(world.map.occupancyGrid)-1)
%     for col = (2:length(world.map.occupancyGrid)-1)
%         if (world.map.occupancyGrid(row, col) == OccupancyState.UNOCCUPIED && any(any(world.map.occupancyGrid(row-1:row+1, col-1:col+1) == OccupancyState.OCCUPIED)))
%             boundsGT = boundsGT + 1;
%             
%             % check the robots to see if these points were explored
%             for idx = (1:length(world.robots))
%                 % compute the row col to check in the robots map.
%                 pt = world.map.mapIndex2Position(row, col);
%                 T_w_r = world.robotGroundTruthStates{idx}.transformation();
%                 T_L_r = world.robots{idx}.localState.transformation();
%                 T_L_w = T_L_r * inv(T_w_r);
%                 pt_local = T_L_w(1:2, 1:2) * pt + T_L_w(1:2, 3);
%                 
%                 [r_L, c_L] = world.robots{idx}.localMap.position2MapIndex(pt_local);
%                 
%                 try
%                     if any(any(world.robots{idx}.localMap.occupancyGrid(r_L-1:r_L+1, c_L-1:c_L+1) == OccupancyState.UNOCCUPIED)) && any(any(world.robots{idx}.localMap.occupancyGrid(r_L-1:r_L+1, c_L-1:c_L+1) == OccupancyState.OCCUPIED))
%                         boundsExplored = boundsExplored + 1;
%                         break;
%                     end
%                 catch
%                     
%                 end
%             end
%             
%             
%         end
%     end
% end
% 
% % add the edge bounds
% boundsGT;
% boundsExplored;
% 
% 
% percentage = boundsExplored / boundsGT

end

