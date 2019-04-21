function [] = drawRobot(robotState, color)
% draws a basic robot from a circle and line

r = 1; % meters

theta = (0:0.2:2*pi);

% draw cricle

plot(cos(theta)*r + robotState.pos(1), sin(theta)*r + robotState.pos(2), '-', 'Color', color);

line([0, cos(robotState.theta)]*2*r + robotState.pos(1), [0, sin(robotState.theta)]*2*r + robotState.pos(2), 'Color', color);

end

