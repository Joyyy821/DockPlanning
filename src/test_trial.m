%% File: test_trial.m
% Construct with modules (floating assumption)
clc; clear variables; close all;
% Map_Size = [15, 15]; % length*width
% a = 2; b = 2;
Map_Size = [15, 15]; % length*width
a = 4; b = 2;

%% Initialization
% Notice: must initialize targets first before initialize robot group,
% otherwise the robot group cannot be located on the extension tree.
trial = Trial();
% Map
trial.gmap = map(Map_Size, 3);

% Robots
robot_locs = [1, 13; 5, 13; 9, 1; 13, 1];
dock = [0,0,0,1;...
        0,1,0,1;...
        0,0,1,0;...
        1,0,1,0];
trial.setRobots(robot_locs, dock);

% Targets
tar_locs = [4,4;5,4;5,5;6,5];
success = trial.setTargets(tar_locs);
if ~success
    disp("Program exit ...")
else
    trial.setDisplay();
    
    %% Main loop
    max_steps = 1e6;
    while ~trial.structure_arrive && trial.step_cnt <= max_steps
        trial.execute();
    end
end
