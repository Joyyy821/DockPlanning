% Test file: construction w/o objects
clc; clear all; close all;
addpath('display');
Map_Size = [15, 15]; % length*width
% Rect
N = 8; a = 2; b = 4;  % Num of blocks; width; length;
%% Targets & robots initialization
Tars = [];
for i = 1:N
    m = ceil(i/b);
    n = i - (m-1) * b;
    Tars = [Tars; targetPoint(i, [m+4, n+4])];
end

all_tar = targetGroup(Tars);
% Binary tree construction / extension
ext = Extension(all_tar);
ext.TargetToTree([5, 6]);

% Robot initialization
% for i=1:N
%     robots(i) = robot(i, [2+2*ceil(i/2), 11+2*(1-rem(i, 2)), 0]);
%     robots(i).CognMap = zeros(Map_Size(1), Map_Size(2));
% end

for i=1:N
    robots(i) = robot(i, [1+9*floor((i-1)/(N/2)), 4+rem(i-1, N/2), 0]);
    robots(i).CognMap = zeros(Map_Size(1), Map_Size(2));
end


%% Display
display = display2D(Map_Size, "FinalTarget", all_tar, ...
                                "Robot", robots);
pause(10);
disp("BFS iterator:"); disp(ext.BFSit);
i = length(ext.BFSit);
while i > 0
    disp("i: "+string(i));
    cur_node = ext.tarTree.get(ext.BFSit(i));
    new_node = copy(cur_node);
    % Find current targets
    while new_node.Size < N
        i = i-1;
        cur_node = ext.tarTree.get(ext.BFSit(i));
        new_node = new_node.AddTargetGp(cur_node);
    end
    % Update current targets
    display.updateMap("CurrentTarget", new_node);
    % Set goals
    for j = 1:N
        tp = new_node.TargetList(j);
        robots(tp.ID).Goal = [tp.Location 0];
    end
    % Move robots
    arrive_flag = false(1, N);
    while true
        for k = 1:N
            if ~arrive_flag(k)
                arrive_flag(k) = ~robots(k).move();
    %         else
    %             arrive_flag(i) = false;
            end
        end
        display.updateMap("Robot", robots);
%         disp(arrive_flag);
        if all(arrive_flag)
            break;
        end
    %     pause(1);
    end
    i = i - 1;
end
