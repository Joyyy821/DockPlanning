%% File: test_trial.m
% Construct with modules (floating assumption)
clc; clear variables; close all;
Map_Size = [12, 12]; % length*width
% Map_Size = [25, 25]; % length*width

%% Initialization
% Notice: must initialize targets first before initialize robot group,
% otherwise the robot group cannot be located on the extension tree.
trial = Trial();
% Map
trial.gmap = map(Map_Size, 3);

% Robots
robot_locs = [1, 1; 4, 1; 7, 1; 10, 1];
% robot_locs = [1, 23; 5, 23; 9, 23; 13, 23; 17, 23; 21, 23; ...
%     3, 19; 11, 19; 19, 19];
dock = [1,0,0,0;...
        1,0,1,0;...
        0,1,0,0;...
        1,0,1,0
        ]; % up, down, left, right
% dock = [0,1,0,0;...  1
%         0,0,0,1;...  2
%         0,1,1,0;...  3
%         1,1,0,0;...  4
%         0,0,0,1;...  5
%         0,0,1,1;...  6
%         1,0,0,1;...  7
%         1,1,1,0;...  8
%         1,0,1,0;...  9
%         ];
trial.setRobots(robot_locs, dock);

% Set logging if log is required
trial.setLogging(true);

% Targets
tar_locs = [4,4;5,4;5,5;6,5];
% tar_locs = [12, 14; 13, 14; 14, 14;
%             12, 13; 13, 13; 14, 13;
%             12, 12; 13, 12; 14, 12
%         ];
[success, rotated_dock] = trial.setTargets(tar_locs, [4, 4]);
% [success, rotated_dock] = trial.setTargets(tar_locs, [13, 13]);
if ~success
    disp("Program exit ...")
else
    trial.setDisplay(rotated_dock);
    
    %% Main loop
    max_steps = 1e6;
    while ~trial.structure_arrive && trial.step_cnt <= max_steps
        trial.execute();
    end
end
trial.log.endRecording;
