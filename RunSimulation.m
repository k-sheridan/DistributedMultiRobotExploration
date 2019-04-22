% This script will run and visualize a simulated distributed multi-robot
% exploration in a gps-denied environment.
clear all
w = World('Map1.png', 0.5, 4);

f1 = figure;

dt = 1;
for t = (1:dt:6000)
    w.run(dt);
    
    figure(f1);
    clf
    
    subplot('Position', [0,0,0.5,1]);
    drawGroundTruth(w);
    subplot('Position', [0.5,0.5,0.25,0.5]);
    drawLocalMap(w.robots{1});
    subplot('Position', [0.5,0,0.25,0.5]);
    drawLocalMap(w.robots{2});
    subplot('Position', [0.75,0.5,0.25,0.5]);
    drawLocalMap(w.robots{3});
    subplot('Position', [0.75,0,0.25,0.5]);
    drawLocalMap(w.robots{4});
    
    
    drawnow;
end