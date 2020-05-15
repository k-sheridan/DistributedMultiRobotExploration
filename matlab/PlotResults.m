clear
exploreIncr = 10;
% load the raw data
%% simple map
s1 = load('simpleMap_1robot_centerOctantInit.mat');
s2 = load('simpleMap_2robot_centerOctantInit.mat');
s3 = load('simpleMap_4robot_centerOctantInit.mat');
s4 = load('simpleMap_8robot_centerOctantInit.mat');

sCells = {s1, s2, s3, s4};

figure;
hold on;

for s = sCells
    l = length(s{1}.explored);
    maxRat = max(s{1}.explored);
    plot((0:l-1)*exploreIncr, s{1}.explored/maxRat);
end

xlabel('time (s)')
ylabel('Explored ratio');
title('Simple Map Exploration Time');
legend('1 Robot', '2 Robots', '4 Robots', '8 Robots');
grid on


% consensus vis
figure;
hold on;


[r, c] = size(s4.globalX);
for idx = (1:r)
    subplot(1, 2, 1);
    hold on;
    plot(s4.globalX(idx, 1:15));
    
    subplot(1, 2, 2);
    hold on;
    plot(s4.globalY(idx, 1:15));
    
end

subplot(1, 2, 1);
title('Common Global Frame Consensus');
legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5', 'Robot 6', 'Robot 7', 'Robot 8');
xlabel('time (s)')
ylabel('x (m)');
grid on

subplot(1, 2, 2);
title('Common Global Frame Consensus');
legend('1 Robot', '2 Robots', '4 Robots', '8 Robots');
grid on
xlabel('time (s)')
ylabel('y (m)');



%% Hard map now AAE 590

% load the raw data
s1 = load('aae590_1robot_centerOct.mat');
s2 = load('aae590_2robot_centerOct.mat');
s3 = load('aae590_4robot_centerOct.mat');
s4 = load('aae590_8robot_centerOct.mat');

sCells = {s1, s2, s3, s4};

figure;
hold on;

for s = sCells
    l = length(s{1}.explored);
    maxRat = max(s{1}.explored);
    plot((0:l-1)*exploreIncr, s{1}.explored/maxRat);
end

xlabel('time (s)')
ylabel('Explored ratio');
title('Complex Map Exploration Time');
legend('1 Robot', '2 Robots', '4 Robots', '8 Robots');
grid on


%% Hard map now AAE 590 with nocomm

% load the raw data
s1 = load('aae590_1robot_centerOct.mat');
s2 = load('aae590_2robot_centerOct.mat');
s3 = load('aae590_4robot_centerOct.mat');
s4 = load('aae590_8robot_centerOct.mat');
s5 = load('aae590_1robot_centerOct.mat');
s6 = load('aae590_2robot_noComm.mat');
s7 = load('aae590_4robot_noComm.mat');
s8 = load('aae590_8robot_noComm.mat');

sCells = {s1, s2, s3, s4, s5, s6, s7, s8};

figure;
hold on;

for s = sCells(1:4)
    l = length(s{1}.explored);
    maxRat = max(s{1}.explored);
    plot((0:l-1)*exploreIncr, s{1}.explored/maxRat);
end
for s = sCells(5:8)
    l = length(s{1}.explored);
    maxRat = max(s{1}.explored);
    plot((0:l-1)*exploreIncr, s{1}.explored/maxRat, '--');
end

xlabel('time (s)')
ylabel('Explored ratio');
title('Complex Map Exploration Time Compared To No Communication');
legend('1 Robot', '2 Robots', '4 Robots', '8 Robots', '1 Robot No Communication', '2 Robots No Communication', '4 Robots No Communication', '8 Robots No Communication');
grid on

