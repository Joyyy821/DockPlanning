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
N = 1;
robots(1) = robot(1, [10, 11, 0]);
robots(1).CognMap = zeros(Map_Size(1), Map_Size(2));
robots(1).Goal = [all_tar.TargetList(1).Location 0];
display = display2D(Map_Size, "FinalTarget", all_tar, ...
                                "Robot", robots);
% robots(1).Astar();
pause(4);
while robots(1).move()
    display.updateMap("Robot", robots);
    pause(1);
end
