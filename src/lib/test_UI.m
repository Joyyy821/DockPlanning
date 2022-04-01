%% test.m
% Directly use GUI2D class
clc; clear all; close all;

Map_Size = [15, 15]; % length*width

Robot_Size = [2, 2]; % length*width

% obstacle
Obstacle = [[10,4];[9,4];[8,4];[7,4]]; %
 
% robot initial
Robot_Current_Position = [[0,0];[13,0]];

% object initial
Object_Current_Position = [[5,0];[10,0]];

% current targets
Robot_Current_Target = [[11, 6]; [5, 6]];
Object_Current_Target = [[7, 5]; [10, 5]];
 
% final target
Robot_Final_Target = [[9,6];[7,6]];
Object_Final_Target = [[9,5];[8,5]];
 
% loop
Robot_Loop = [[0,0];[0,1];[0,2];[1,2];[2,2];[2,1];[2,0];[1,0]];
Robot_Loop2 = [[13,0];[13,1];[13,2];[12,2];[11,2];[11,1];[11,0];[12,0]];

app = GUI2D_exported();
app.initial();

i = 1;
while true
    Robot_Current_Position(2,:) = Robot_Loop2(i,:);
    Robot_Current_Position(1,:) = Robot_Loop(i,:);
%     Robot_Current_Position(2,:) = Robot_Loop2(i,:);
    app.show();
    i = i + 1;
    if i >= 9
        i = 1;
    end
    pause(1);
end

%% Use display2D class to display target
addpath('display');
% Rect
N = 8; a = 2; b = 4;  % Num of blocks; width; length;

Tars = [];
for i = 1:N
    m = ceil(i/b);
    n = i - (m-1) * b;
    Tars = [Tars; targetPoint(i, [m, n])];
end

all_tar = targetGroup(Tars);

display = display2D([15, 15], "FinalTarget", all_tar);
% display.runGUI2D();

for i = 1:N
    m = ceil(i/b);
    n = i - (m-1) * b;
    Tars(i) = targetPoint(i, [m*2, n*2]);
end

all_tar = targetGroup(Tars);
display.updateMap("CurrentTarget", all_tar);

%% Display both targets and robots
clc; clear all; close all;
addpath('display');
Map_Size = [15, 15]; % length*width
% Rect
N = 8; a = 2; b = 4;  % Num of blocks; width; length;

Tars = [];
for i = 1:N
    m = ceil(i/b);
    n = i - (m-1) * b;
    Tars = [Tars; targetPoint(i, [m, n])];
end

all_tar = targetGroup(Tars);

% robots = [];
N = 8;
for i=1:N
    robots(i) = robot(i, [2+2*ceil(i/2), 11+2*(1-rem(i, 2)), 0]);
    robots(i).CognMap = zeros(Map_Size(1), Map_Size(2));
    robots(i).Goal = [all_tar.TargetList(i).Location 0];
end
display = display2D(Map_Size, "FinalTarget", all_tar, ...
                                "Robot", robots);
% robots(1).Astar();
pause(10);
arrive_flag = logical(zeros(1, N));
while true
    for i = 1:N
        if ~arrive_flag(i)
            arrive_flag(i) = ~robots(i).move();
%         else
%             arrive_flag(i) = false;
        end
    end
    display.updateMap("Robot", robots);
    disp(arrive_flag);
    if all(arrive_flag)
        break;
    end
%     pause(1);
end

%% Robot navigation to a target position with carrying module
clc; clear all; close all;
addpath('display');
% Initialization
g_map = map(15, 15);
g_map.setDist("robot", 5, "module", 2);
g_map.obstacleMap(2, 3:6) = 1;
obstacle = [[2, 3]; [2, 4]; [2, 5]; [2, 6]];

tar = targetPoint(1, [4, 5], g_map);
tar_gp = targetGroup(tar);
rob = robot(1, [10, 1, 0], g_map);
mod = module(1, [1, 1, 0], g_map);
display = display2D(g_map.mapSize, "FinalTarget", tar_gp, ...
                                "Robot", rob, ...
                                "Module", mod, ...
                                "Obstacle", obstacle);

pause(10);

% Fetch module
arrive = 0;
if rob.fetchModule(mod) == 2
    while ~arrive
        arrive = ~rob.move();
        display.updateMap("Robot", rob);
    end
end

rob.fetchModule(mod);
mod.dock("robot", rob);

% Navigate to the target point
rob.Goal = [tar.Location + [1, 0], 0];
arrive = 0;
while ~arrive
    arrive = ~rob.move();
    mod.move();
    % Update robot and module position at the same time
    display.updateMap("Robot", rob, "Module", mod);
end
