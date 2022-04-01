%% File: test_trial.m
% Construct with modules (floating assumption)
clc; clear variables; close all;
Map_Size = [15, 15]; % length*width
a = 2; b = 2;

%% Initialization
% Notice: must initialize targets first before initialize robot group,
% otherwise the robot group cannot be located on the extension tree.
trial = Trial();
% Map
trial.gmap = map(Map_Size);
% Targets
trial.setTargets(a, b, [5, 5], [5, 6], 3);
% Robots
robot_locs = [1, 1; 5, 1; 9, 1; 13, 1];
trial.setRobotGp(robot_locs);
% Modules
module_locs = [2, 14; 5, 14; 8, 14; 11, 14];
trial.setModules(module_locs);
% Obstacle
obstacle_locs = 10*ones(7, 2);
obstacle_locs(1:end, 1) = 4:10;
trial.obstacles = obstacle(obstacle_locs, trial.gmap);
trial.setDisplay();
pause(2);

%% Main loop
while ~all(trial.target_arrive)
    trial.execute();
end


%% Search testing
% clc; clear variables; close all;
% Map_Size = [15, 15]; % length*width
% trial = Trial();
% % Map
% trial.gmap = map(Map_Size);
% trial.setRobotGp([5, 5]);
% trial.display = display2D(trial.gmap.mapSize,"Robot", trial.getRobots());
% trial.robotGp.initSearch();
% pause(2);
% 
% while true
%     trial.execute();
% end
