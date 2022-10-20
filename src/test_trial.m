%% File: test_trial.m
% Construct with modules (floating assumption)
clc; clear variables; close all;
addpath('lib');
addpath('lib/display'); addpath('lib/log');
addpath('lib/alg'); addpath('lib/alg/connect');

%% Initialization
trial = Trial("S-4t-5r-3");

% Map
Map_Size = [12, 12]; % length*width

% Robots
robot_locs = [1, 11; 3, 11; 6, 11; 9, 11; 11, 11];
dock = [1,0,0,0;...
        2,0,1,0;...
        0,1,0,0;...
        1,0,2,0;...
        1,2,0,0
        ]; % up, down, left, right
% Targets
tar_locs = [4,4;5,4;5,5;6,5];

trial.run("RobotLocation", robot_locs, ...
    "RobotDock", dock, ...
    "TargetLocation", tar_locs,...
    "MapSize", Map_Size, ...
    "AlgorithmType", 3, ...
    "isLogging", [true, true] ...
);

