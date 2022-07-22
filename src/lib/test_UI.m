%% test_UI.m
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
clc; clear variables; close all;
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
    robots(i).DockJoint(rem(i, 4)+1) = 1;
    robots(i).CognMap = zeros(Map_Size(1), Map_Size(2));
    robots(i).Goal = [all_tar.TargetList(i).Location 0];
end
display = display2D(Map_Size, "FinalTarget", all_tar, ...
                                "Robot", robots);
% robots(1).Astar();
pause(2);
arrive_flag = logical(zeros(1, N));
while true
    for i = 1:N
        if ~arrive_flag(i)
            arrive_flag(i) = robots(i).move();
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
clc; clear variables; close all;

