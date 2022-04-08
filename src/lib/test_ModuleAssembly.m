% File: construction with modules
clc; clear all; close all;
% addpath('display'); addpath('alg');
Map_Size = [15, 15]; % length*width
% Rect
N = 4; a = 2; b = 2;  % Num of blocks; width; length;

% %% Float test
% % Map
% g_map = map(15, 15);
% g_map.setDist("robot", 3, "module", 3);
% 
% mod1 = module(1, [3, 3, 0], g_map);
% mod2 = module(2, [3, 4, 0], g_map);
% mod3 = module(3, [3, 5, 0], g_map);
% mods = [mod1, mod2, mod3];
% display = display2D(g_map.mapSize, "Module", mods);
% 
% while true
%     mod1.walk();
%     mod2.walk();
%     mod3.walk();
%     display.updateMap("Module", mods);
% end

%% Initialization
% Map
g_map = map(15, 15);
g_map.setDist("robot", 3, "module", 3);

% Targets
Tars = [];
for i = 1:N
    m = ceil(i/b);
    n = i - (m-1) * b;
    Tars = [Tars; targetPoint(i, [m+4, n+4], g_map)];
end

all_tar = targetGroup(Tars);
% Binary tree construction / extension
ext = Extension(all_tar, g_map);
ext.TargetToTree([5, 6], 3);

% Robots
robot_locs = [1, 1; 1, 3; 15, 1; 15, 3];
% robot_locs = [1, 14; 4, 14; 9, 14; 12, 14];
for i = 1:N
    robots(i) = robot(i, [robot_locs(i, :), 0], g_map);
    group(i) = AssembleGroup(i, robots(i), g_map);
end

% Modules
module_locs = [2, 14; 5, 14; 8, 14; 11, 14];
for i = 1:N
    modules(i) = module(i, [module_locs(i, :), 0], g_map);
end

% Obstacle
obstacle_locs = zeros(7, 2);
for i = 1:7
    ox = ceil(i);
    oy = i - (ox-1);
    obstacle_locs(i, :) = [ox+3, oy+9];
end
obstacles = obstacle(obstacle_locs, g_map);

%% Display
display = display2D(g_map.mapSize, "FinalTarget", all_tar, ...
                                "Robot", robots, ...
                                "Module", modules, ...
                                "Obstacle", obstacles.Locations);
% pause(10);
% Extension
ext.showExtension(display);
pause(10);
%% Main loop
fetch_arrive = zeros(1, N);
target_arrive = zeros(1, N);
structure_arrive = zeros(1, N);
finish = zeros(1, N);
rob_dirs = [0, -1; -1, 0; 1, 0; 1, 0];
cl = ones(1, 4)*length(ext.GroupLayers);   % current layer in the extension tree

while true
    % Move robots one by one
    for i = 1:N
        % Fetch modules
        if ~fetch_arrive(i)
            isSuccess = group(i).LeadRobot.fetchModule(modules(i), rob_dirs(i, :));
            if isSuccess == 2
                fetch_arrive(i) = group(i).LeadRobot.move();
    %             if arrive(i)
    %                 disp("arrive "+string(i)+":");
    %                 disp(arrive);
    %                 robots(i).fetchModule(modules(i), rob_dirs(i, :));
    %                 modules(i).dock("robot", robots(i));
    %             end
            elseif isSuccess == 1
                group(i).updateGroup("add");
                fetch_arrive(i) = 1;
                group(i).LeadRobot.Goal = [Object_Current_Target(i, :)+rob_dirs(i, :), 0];
            end
        elseif ~target_arrive(i)
            % disp("Robot "+string(i)+" move to target location.");
            rslt = group(i).LeadRobot.nearGoalCheck();
            d_id = group(i).dockGpID;
            [d, ~] = checkDockingGp(ext, d_id, cl(i), g_map);
            if ~isempty(d_id)
                if rslt == 2
%                     [d, locs] = checkDockingGp(ext, d_id, cl(i), g_map);
                    if d
%                         % Set the docking positions to be ignored
%                         d_rob_id = modules(d_id(1)).LeadRobot.ID;
%                         [x, y] = find(g_map.robotMap==d_rob_id);
%                         locs = [locs; x, y];
%                         robots(i).setIngorePos(locs);
                    else
                        continue
                    end
                elseif rslt == 1
%                     d_id = robots(i).carriedModule.DockGpIDs;
%                     [d, locs] = checkDockingGp(ext, d_id, cl(i), g_map);
                    if d
%                         % Set the docking positions to be ignored
%                         robots(i).ignoredPos = locs;
                        for c_did = d_id
                            if any(any(group(c_did).LeadRobot.carriedModule.Boundary))
                                d_gp = group(c_did).LeadRobot.carriedModule;
                                target_arrive(group(d_id(1)).LeadRobot.ID) = true;
                                group(i).modules.dock(d_gp);
                                break
                            end
                        end
                    else
                        target_arrive(i) = true;
                        structure_arrive(i) = true;
                        group(i).updateGroup("del");
                        group(i).LeadRobot.back2startPlace();
                        
                        continue
                    end
                end
            end
            % 先验证是否到达target附近（或者已经在target位置）
            % 如果是在附近，则验证当前对接目标是否已到达：
            % 1）如果到达则完成对接，并离开group返回起始点
            % 2）如果未到达则等待对方group到达，对方到达后再进入对接区域对接，对接完成后继续进行下一步
            % 如果已经在target位置上，则1）直接完成对接后离开；或者2）等对方到达后完成对接并去下一步
            % TODO: 需要在认知地图里把对接目标设为0
            target_arrive(i) = robots(i).move();
%             modules(i).move();
            if target_arrive(i)
                % update target to the next target location
                group(i).dockGpID = ...
                    ext.extractOnce(robots(i).carriedModule, cl(i), display);
                % Update the docking positions to be ignored
                cl(i) = cl(i) - 1;
                d_id = group(i).dockGpID;
                d_rob_id = group(d_id(1)).LeadRobot.ID;
                [x, y] = find(g_map.moduleMap==d_id(1));
                [~, locs] = checkDockingGp(ext, d_id, cl(i), g_map);
                pos_delta = locs(1, :) - [x, y];
                [rx, ry] = find(g_map.robotMap==d_rob_id);
                rob_tar = [rx, ry] + pos_delta;
%                 locs = [locs; rob_tar];
                robots(i).setIgnorePos([locs; rob_tar]);
                
                % robot goal set to next target position
                robots(i).Goal = [Object_Current_Target(i, :)+rob_dirs(i, :), 0];
                if cl(i) > 1
                    target_arrive(i) = false;
                end
            end
%             target_arrive(i) = temp_flag;
        elseif ~structure_arrive(i)
            structure_arrive(i) = robots(i).move();
            if structure_arrive(i)
                robots(i).isCarrying = false;
                robots(i).back2startPlace();
            end
        elseif ~finish(i)
            finish(i) = robots(i).move();
        end
    end
    display.updateMap("Robot", robots, "Module", modules);
%     display.updateMap("Robot", robots);
%     pause(10);
    % Exit
    if all(finish)
        break
    end
end

%% Functions
function robotSearch(robs, mods)
    
end


function [is_arrive, locs] = checkDockingGp(ext, ids, level, gmap)
%     [dock_ids, dock_locs] = ext.getSilibing(ids, level);
%     size = length(dock_ids);
%     for i=1:size
%         if gmap.moduleMap(dock_locs(i,1), dock(i,2)) ~= dock_ids(i)
%             is_arrive = false;
%             return
%         end
%     end
%     is_arrive = true;
    %% checkDockingGp: check whether the pair modules have arrived
    % Input: ext (extension class), ids (the pair module ids), 
    %        level (the current layer in the extension tree),
    %        gmap (global map).
    locs = ext.getLocations(ids, level);
    for i=1:length(ids)
        if gmap.moduleMap(locs(i,1), locs(i,2)) ~= ids(i)
            is_arrive = false;
            return
        end
    end
    is_arrive = true;
end