% This script will run and visualize a simulated distributed multi-robot
% exploration in a gps-denied environment.
clear all
w = World('mousMaze.png', 0.5, 1);

vw = VideoWriter('test.avi');
open(vw);

f1 = figure;

set(gcf, 'Position', [0,0,1000, 500])

dt = 1;
for t = (1:dt:6000)
    w.run(dt);
    
    figure(f1);
    clf
    
    
%     subplot('Position', [0,0,0.5,1]);
%     drawGroundTruth(w);
    %subplot('Position', [0.5,0.5,0.25,0.5]);
    drawLocalMap(w.robots{1});
%     subplot('Position', [0.5,0,0.25,0.5]);
%     drawLocalMap(w.robots{2});
%     subplot('Position', [0.75,0.5,0.25,0.5]);
%     drawLocalMap(w.robots{3});
%     subplot('Position', [0.75,0,0.25,0.5]);
%     drawLocalMap(w.robots{4});
    
    
    drawnow;
    
    frame = getframe(gcf);
    writeVideo(vw,frame);
end