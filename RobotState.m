classdef RobotState < handle
    %  Stores the state estimate of the robot and allows for perfect motion
    %  updates. 
    %  ORDER: [x, y, theta]
    
    properties
        pos % [x;y]
        theta % z axis rotation
        
        Sigma; % order: [x, y, theta]
    end
    
    methods
        %  ORDER: [x, y, theta]
        function obj = RobotState(initialState, initialUncertainty)
            obj.Sigma = initialUncertainty;
            obj.pos = initialState(1:2, 1);
            obj.theta = initialState(3, 1);
        end
        
        %  ORDER: [x, y, theta]
        % applies the general addition operator on the state
        function [] = update(obj, dx)
            obj.pos = obj.pos + dx(1:2, 1);
            obj.theta = obj.theta + dx(3, 1);
        end
        
    end
end

