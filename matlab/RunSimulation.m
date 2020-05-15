% This script will run and visualize a simulated distributed multi-robot
% exploration in a gps-denied environment.
clear all
w = World('map3.png', 0.5, 4);

vw = VideoWriter('test.mp4', 'MPEG-4');
open(vw);

% logged variables
time = [];
% track the estimates of the global pos and theta over time. each row is a
% robot's estimate.
globalX = [];
globalY = [];
globalTheta = [];

% exploration percentage
explored = [];
everyN = 10;


dt = 1;

f1 = figure;

set(gcf, 'Position', [0,0,2000, 1000])

for t = (0:dt:6000)
    
    % log 
    time(1, end+1)=t;
    
    sz = length(globalX);
    for idx = 1:length(w.robots)
        T_L_r = w.robots{idx}.localState.transformation();
        T_C_L = w.robots{idx}.startingState.transformation();
        T_W_r = w.robotGroundTruthStates{idx}.transformation();
        
        T_W_C = T_W_r * inv(T_L_r) * inv(T_C_L);
        
        th = atan2(T_W_C(2, 1), T_W_C(1, 1));
        globalTheta(idx, sz+1) = th;
        globalX(idx, sz+1) = T_W_C(1, 3);
        globalY(idx, sz+1) = T_W_C(2, 3);
        
    end
    if ~mod(t, everyN)
        exPer = computeExplorationPercentage(w);
        explored(1, end+1) = exPer;
    end
    
    if exPer > 0.90
        break;
    end
    
    
     figure(f1);
     clf
     
     
    subplot('Position', [0,0,0.5,1]);
    drawGroundTruth(w);
    subplot('Position', [0.5,0.5,0.25,0.5]);
    drawLocalMap(w.robots{1});
    %drawPotentialFunction(w.robots{1});
    subplot('Position', [0.5,0,0.25,0.5]);
    drawLocalMap(w.robots{2});
    %drawPotentialFunction(w.robots{2});
    subplot('Position', [0.75,0.5,0.25,0.5]);
    drawLocalMap(w.robots{3});
    %drawPotentialFunction(w.robots{3});
    subplot('Position', [0.75,0,0.25,0.5]);
    drawLocalMap(w.robots{4});
    %drawPotentialFunction(w.robots{4});
    
    
    drawnow;
    
    frame = getframe(gcf);
    writeVideo(vw,frame);

w.run(dt);

end

%save the vars.
%save('temp.mat', 'explored', 'time', 'globalTheta', 'globalX', 'globalY')

