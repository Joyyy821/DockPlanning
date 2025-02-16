%% File: test_trial.m
% Construct with modules (floating assumption)
clc; clear variables; close all;
addpath('lib');
addpath('lib/display'); addpath('lib/log');
addpath('lib/alg'); addpath('lib/alg/connect');

%% Initialization
str1='shape2-robot28-target28-method2-time';
N_run = 30;
for i = 1:N_run
    str2=num2str(i);
    name=[str1 str2]
    trial = Trial(name);
    
    % Map
    Map_Size = [30,30]; % length*widthhalf
    
    % Robots
    robot_locs = [5,2;8,2;11,2;14,2;17,2;20,2;23,2;2,5;2,8;2,11;2,14;2,17;2,20;2,23;25,28;22,28;19,28;16,28;13,28;10,28;7,28;28,25;28,22;28,19;28,16;28,13;28,10;28,7];
    dock = [1,0,0,1;...
1,1,0,1;...
0,1,0,1;...
1,0,0,1;...
1,1,1,1;...
1,1,1,1;...
1,1,1,0;...
0,1,0,0;...
1,0,0,1;...
1,1,1,1;...
1,1,1,1;...
0,1,1,0;...
1,0,0,0;...
1,1,1,1;...
1,1,1,1;...
0,1,1,1;...
1,0,1,0;...
1,1,1,1;...
1,1,1,1;...
0,1,0,1;...
1,0,1,0;...
1,1,1,1;...
1,1,1,1;...
1,1,0,1;...
0,1,0,0;...
1,0,1,0;...
1,1,1,0;...
0,1,1,0;...
            ]; % up, down, left, right
    % Targets
     tar_locs = [12,15;12,16;12,17;13,14;13,15;13,16;13,17;13,18;14,13;14,14;14,15;14,16;15,12;15,13;15,14;15,15;16,13;16,14;16,15;16,16;17,14;17,15;17,16;17,17;17,18;18,15;18,16;18,17 ];
    try
        trial.run("RobotLocation", robot_locs, ...
            "RobotDock", dock, ...
            "TargetLocation", tar_locs,...
            "MapSize", Map_Size, ...
            "AlgorithmType", 2, ...
            "isLogging", [true, true], ...
            "TargetCenter", 15 ...
        );
    catch e
        disp(e);
    end
end