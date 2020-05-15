classdef LidarMeasurement
    %LIDARMEASUREMENT represents a robocentric lidar measurement
    
    properties
        bearings; % radians of the measurments. x axis is forward
        ranges; % if the range is < 0, no obstacle was detected.
    end
    
    methods
        function obj = LidarMeasurement()
            
        end
    end
end

