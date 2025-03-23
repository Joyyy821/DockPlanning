%% File: test_trial.m
% Construct with modules (floating assumption)
clc; clear variables; close all;
addpath('lib');
addpath('lib/display'); addpath('lib/log');
addpath('lib/alg'); addpath('lib/alg/connect');

%% Initialization
str1='shape1-robot15-target12-method2-time';
N_run = 30;
for i = 1:N_run
    str2=num2str(i);
    name=[str1 str2]
    trial = Trial(name);
    
    % Map
    Map_Size = [24, 24]; % length*widthhalf
    
    % Robots
    robot_locs = [6,2;10,2;14,2;2,6;2,10;2,14;18,22;14,22;10,22;22,19;22,16;22,13;22,10;22,7;22,4];
    dock = [0,0,0,1;...
            1,0,0,1;...
            1,1,1,0;...
            0,1,0,1;...
            1,0,0,0;...
            0,1,1,1;...
            1,0,1,1;...
            0,1,0,0;...
            1,0,1,0;...
            1,1,0,1;...
            0,1,1,0;...
            0,0,1,0;...
            0,0,0,1;...
            0,0,0,0;...
            1,1,0,0;...

            ]; % up, down, left, right
    % Targets
     tar_locs = [11,13;12,12;12,13;12,14;13,11;13,12;13,14;13,15;14,12;14,13;14,14;15,13 ];
    try
        trial.run("RobotLocation", robot_locs, ...
            "RobotDock", dock, ...
            "TargetLocation", tar_locs,...
            "MapSize", Map_Size, ...
            "AlgorithmType", 2, ...
            "isLogging", [true, true], ...
            "TargetCenter", 13 ...
        );
    catch e
        disp(e);
    end
end