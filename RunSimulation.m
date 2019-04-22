% This script will run and visualize a simulated distributed multi-robot
% exploration in a gps-denied environment.

w = World('Map1.png', 0.5, 1);

f1 = figure;
f2 = figure;

dt = 1;
for t = (1:600)
    w.run(dt);
    
    
    figure(f1);
    clf
    drawGroundTruth(w);
    figure(f2);
    clf
    drawLocalMap(w.robots{1});
    drawnow;
end