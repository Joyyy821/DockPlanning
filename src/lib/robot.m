classdef robot < handle
    %ROBOT represents robot that will construct the shape.
    %   The robots:
    %   (1) are identical and labelled;
    %   (2) can locate themselves in the global map;
    %   (3) can sense the neighbouring environment (location of 
    %   objects/modules/robots);
    %   (4) run planning algorithms and move to goal positions.
    
    properties (Access = public)
        % Can be directly modified by the user.
        ID                 (1, 1) int32 {mustBeNonnegative} % ID of the robot
        Status             (1, 1) uint8 % (1) initial; (2) fixed; (3) unfixed; (4) free
        isCarrying         (1, 1) logical % Carrying (1) / Not carrying (0) module
        carriedModule      moduleGroup    % Module
%         moduleToFetch      module % TODO: replace this attribute
        ignoredPos         int32
        Location           (1, 3) double  % Current robot location [x, y, z]
        CognMap                      % Cognitive map of individual robot
        GlobalMap          map       % "pointer" to a global map object
        Goal          (:, 3) double  % Current robot goal [x, y, z]
        priorPlanningID = 0 % The current planning position which has highest priority
        startPlace         (1, 3) double   % The location where the robot is initiated.
        Path = [] % Current planned path 
        stepCount = 1                % Number of total steps
        pauseCmd  % Robot will not move if true.
%         searchDir = [1, 1]
    end
	
    methods
        function obj = robot(id,initLocation, gmap)
            %ROBOT 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                % Default construction function
                obj.ID = 0;
                obj.Location = [0, 0, 0];
                obj.GlobalMap = map(15, 15);
                return
            end
            if nargin == 2
                gmap = map();
            end
            obj.ID = id;
            obj.Location = initLocation;
            obj.startPlace = initLocation;
            obj.GlobalMap = gmap;
            obj.GlobalMap.robotMap(initLocation(1), initLocation(2)) = id;
        end
        
%         function searchModule(obj)
%             % todo: update map
%             nextloc = obj.Location(1:2)+[obj.searchDir(1), 0];
%             % if nextloc not in map: move to the location according to
%             % search dir (2) and change the first dir accordingly.
%             
%         end
        
        
        function success = fetchModule(obj, m, rob_dir)
            if m.Status == 1
                success = 0;
                return
            end
            if obj.checkNeighbour(m)
                obj.isCarrying = 1;
                obj.carriedModule = moduleGroup(m);
                m.dock(obj);
%                 obj.carriedModule.LeadRobot = obj;
                success = 1;
            else
%                 obj.moduleToFetch = m;
                obj.ignoredPos = m.Location(1:2);
                obj.Goal = m.Location + [rob_dir, 0];    % TODO
                success = 2;
            end
        end
        
        function setIgnorePos(obj, locs)
%             if obj.isCarrying
%                 mlst = obj.carriedModule.ModuleList;
%                 for i=1:obj.carriedModule.Size
%                     mlst(i).ignoredPos = locs;
%                 end
%             end
            obj.ignoredPos = locs;
        end
        
        function success = back2startPlace(obj)
            % Set the goal for robot (but robot will not actually move).
            if obj.isCarrying
                disp("Robot "+ string(obj.ID) + " is still carrying modules." +...
                    "It cannot go back to the initial place.");
                success = false;
            else
                obj.Goal = obj.startPlace;
                success = true;
            end
        end
        
        function decision = nearGoalCheck(obj)
            if all(abs(obj.Goal(1:2) - obj.Location(1:2)) <= 1)
                if all(obj.Goal(1:2) == obj.Location(1:2))
                    decision = 1;
                else
                    decision = 2;
                end
            else
                decision = 0;
            end
        end
        
        function decision = canMove(obj, move_dir)
            obj.updateMap();
            % check robot
            if ~obj.checkNextStep(move_dir)
                decision = 0;
                return
            end
            if obj.isCarrying
                % check modules
                mlst = obj.carriedModule.ModuleList;
                for i=1:obj.carriedModule.Size
                    m = mlst(i);
%                     disp("carried module: ");
%                     disp(m);
                    loc = m.Location(1:2);
                    cognmap = obj.updateMap(m);
%                     disp("Module "+string(m.ID)+"'s cognMap:");
%                     disp(flip(cognmap.'));
                    if ~obj.checkNextStep(move_dir, loc, cognmap)
                        decision = -1;
                        return
                    end
                end
            end
            decision = 1;
        end
        
        function decision = checkNextStep(obj,move_dir, loc, cogn_map)
            if nargin == 2
                loc = obj.Location(1:2);
                cogn_map = obj.CognMap;
            end
            next_loc = loc + move_dir;
            if any(next_loc < 1) || any(next_loc > obj.GlobalMap.mapSize)
                decision = false;
                return
            end
            if ~cogn_map(next_loc(1), next_loc(2))
                decision = true;
            else
                decision = false;
            end
%             all_dir = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
%             check_list = [next_loc; next_loc+all_dir(1, :); ...
%                 next_loc+all_dir(2, :); ...
%                 next_loc+all_dir(3, :); ...
%                 next_loc+all_dir(4, :); ...
%                 next_loc+all_dir(5, :); ...
%                 next_loc+all_dir(6, :); ...
%                 next_loc+all_dir(7, :); ...
%                 next_loc+all_dir(8, :)];
%             for i=1:9
%                 if all(check_list(i, :) > 0)
%                     if all(check_list(i, :) <= 15) || all(check_list(i, :) >= 1)
%                         if cogn_map(check_list(i, 1), check_list(i, 2)) == 1
%                             decision = false;
%                             return
%                         end
%                     end
%                 end
%             end
%             decision = true;
        end
        
        function decision = checkNeighbour(obj, element, rob_dir)
%             arguments
%                 obj
%                 element.robot  robot
%                 element.module module
%             end
            loc = element.Location;
            if nargin == 2
                if all(abs(obj.Location - loc) == [0, 1, 0]) || ...
                        all(abs(obj.Location - loc) == [1, 0, 0])
                    decision = true;
                else
                    decision = false;
                end
            end
            if nargin == 3
                if obj.Location(1:2) == loc(1:2) + rob_dir
                    decision = true;
                else
                    decision = false;
                end
            end
        end
        
        function obstacleExpansion(obj)
            % TODO
            % get the module group size and the carring direction
            obj.updateMap();
            map_cp = obj.CognMap;
            m_loc = obj.carriedModule.getBoundary();
            loc = obj.Location(1:2);
            % Extension length (in the extension direction, opposite from 
            % the module carrying direction): 
            % [left, lower; right, upper];
            ext = m_loc - [loc; loc];
            ext(1, :) = max(ext(1, :), [0, 0]);
            ext(2, :) = abs(min(ext(2, :), [0, 0]));
            % Combine the cogn map of robot and modules
            mlst = obj.carriedModule.ModuleList;
            for i=1:obj.carriedModule.Size
                mod_map = obj.updateMap(mlst(i));
                obj.CognMap = obj.CognMap | mod_map;
            end
            % expand the obstacle accordingly
            [M, N] = size(map_cp);
            for i=1:M
                for j=1:N
                    if obj.CognMap(i, j) == 1
                        left = max(i - ext(1, 1), 1);
                        right = min(i + ext(2, 1), M);
                        down = max(j - ext(1, 2), 1);
                        up = min(j + ext(2, 2), N);
                        map_cp(left:right, down:up) = 1;
                    end
                end
            end
            
            % TODO: check the direction. expand the margin
            map_cp(M-ext(1,1)+1:M, :) = 1;
            map_cp(1:ext(2,1), :) = 1;
            map_cp(N-ext(1,2)+1:N, :) = 1;
            map_cp(1:ext(2,2), :) = 1;
            obj.CognMap = map_cp;
        end
        
        function moveModule(obj)
            mlst = obj.carriedModule.ModuleList;
            for i=1:obj.carriedModule.Size
                mlst(i).move(obj.Location);
            end
        end
        
        function isArrive = move(obj)
            % Check whether the robot has arrived at the goal place.
            e = 10e-3;
            [goal_n, ~] = size(obj.Goal);
            for i=1:goal_n
                if norm(obj.Location(1:2) - obj.Goal(i, 1:2)) < e
                    disp("Robot "+string(obj.ID)+" already at the goal position "+...
                        "["+string(obj.Goal(i, 1))+", "+string(obj.Goal(i, 2))+"].");
                    isArrive = true;
                    obj.Goal = obj.Goal(i, :);
                    return
                end
            end
            
            % Planning in turn
            if obj.priorPlanningID == 0 || ~obj.isCarrying
                obj.Astar();
            else
                m = obj.carriedModule.getModule(obj.priorPlanningID);
                obj.Astar(m);
            end
            if obj.pauseCmd
                disp("Robot "+string(obj.ID)+" stuck.");
                isArrive = false;
                return
            end
            disp("Robot "+string(obj.ID)+" at step No. "...
                +string(obj.stepCount)+". Path: ");
%             disp(obj.Path);
            
            while norm(obj.Path(1, 1:2) - obj.Location(1:2)) < e
%                 disp("delete:");
%                 disp(obj.Path(1, :));
                obj.Path(1, :) = [];
            end
            nextLoc = obj.Path(1, 1:2);
            move_dir = nextLoc - obj.Location(1:2);
            cnt = 0;
            while true
                % Find a feasible path for the group
                decision = obj.canMove(move_dir);
                if decision ~= 1 && cnt < obj.carriedModule.Size
                    % Rotate to the next one
                    obj.priorPlanningID = ...
                        obj.carriedModule.getNextModuleID(obj.priorPlanningID);
                    disp("Group "+string(obj.ID)+" is block. Replanning path...");
                    if obj.priorPlanningID == 0 && cnt == 0
                        path = obj.Astar();
                    else
                        cur_m = obj.carriedModule.getModule(obj.priorPlanningID);
                        path = obj.Astar(cur_m);
                    end
                    if obj.pauseCmd
                        disp("Robot "+string(obj.ID)+" stuck.");
                        isArrive = false;
                        return
                    end
                    % Update the move direction and count
                    nextLoc = path(1, 1:2);
                    move_dir = nextLoc - obj.Location(1:2);
                    cnt =cnt + 1;
                elseif decision == 1
                    disp("Replan success from object No. "+...
                        string(obj.priorPlanningID));
%                     disp(obj.Path);
                    break
                elseif cnt == obj.carriedModule.Size
                    % No path can work, consider obstacle expansion
                    obj.obstacleExpansion();
                    obj.AstarAlg(obj.Location(1:2), obj.Goal(1:2));
                    
                    if obj.pauseCmd == true
                        disp("Group "+string(obj.ID)+" stuck.");
                        obj.Path = [];
                        isArrive = false;
                        return
                    else
                        disp("Replanned path by obstacle expansion.");
%                         disp(obj.Path);
                        next_loc = obj.Path(2, 1:2);
                        move_dir = next_loc - obj.Location(1:2);
                        cnt =cnt + 1;
%                         break
                    end
                else
                    obj.pauseCmd = true;
                    disp("Group "+string(obj.ID)+" stuck.");
                    obj.Path = [];
                    isArrive = false;
                    return
                end
            end
            
            % Step 2: Move to the next location
            while norm(obj.Path(1, 1:2) - obj.Location(1:2)) < e
                obj.Path(1, :) = [];
            end
            nextLoc = obj.Path(1, 1:2);
            disp("Robot "+string(obj.ID)+" next loc:");
            disp(nextLoc);
            obj.Path(1, :) = [];
            obj.GlobalMap.robotMap(obj.Location(1), obj.Location(2)) = 0;
            obj.GlobalMap.robotMap(nextLoc(1), nextLoc(2)) = obj.ID;
            obj.Location(1:2) = nextLoc;
            if obj.isCarrying
                obj.moveModule();
            end
            obj.stepCount = obj.stepCount + 1;
%             if obj.Location == obj.Goal
%                 isArrive = true;
%             else
%                 isArrive = false;
%             end
            isArrive = false;
        end
        
        function localmap = updateMap(obj, m)
            if nargin == 2
                [localmap, ~, pr] = obj.GlobalMap.getMap(m.Location, "m");
            else
                [localmap, ~, pr] = obj.GlobalMap.getMap(obj.Location, "r");
            end
            % Assume static objects are known
            localmap = localmap | obj.GlobalMap.obstacleMap;
            % Ignore the carried module group
            if obj.isCarrying
                N = obj.carriedModule.Size;
                module_loc = zeros(N, 3);
                for i=1:N
                    module_loc(i, :) = obj.carriedModule.ModuleList(i).Location;
                    localmap(module_loc(i, 1), module_loc(i, 2)) = 0;
                end
            end
%             % Potential docking groups
%             if ~isempty(obj.ignoredPos)
%                 [n, ~] = size(obj.ignoredPos);
%                 for i=1:n
%                     loc = obj.ignoredPos(i, :);
%                     localmap(loc(1), loc(2)) = 0;
%                 end
%             end
            % The position of the robot itself
            localmap(obj.Location(1), obj.Location(2)) = 0;
            
            % Check the robot priority
            [Nr, ~] = size(pr);
            for i=1:Nr
                if pr(i,3) > obj.ID
                    localmap(pr(i,1), pr(i,2)) = 0;
                end
            end
            % Obstacle expansion
            [allx, ally] = find(localmap);
            all_dir = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
            for i=1:length(allx)
                % Do NOT expand for ignore pos
                toignore = false;
                if ~isempty(obj.ignoredPos)
                    k = find(obj.ignoredPos(:,1)==allx(i));
                    for pos=k
                        if obj.ignoredPos(k, 2)==ally(i)
                            toignore = true;
                            break
                        end
                    end
                    if toignore
                        continue
                    end
                end
                % Expansion for others
                for j = 1:8
                    x = allx(i)+all_dir(j, 1);
                    y = ally(i)+all_dir(j, 2);
                    if x <= 0 || y <= 0 || x > obj.GlobalMap.mapSize(1) ...
                            || y > obj.GlobalMap.mapSize(2)
                        continue
                    end
                    localmap(x, y) = 1;
                end
            end
            if nargin == 1
                obj.CognMap = localmap;
            end
        end
        
        function path = Astar(obj, m)
            if nargin == 1
                loc = obj.Location;
                obj.updateMap();
            else
                loc = m.Location;
                obj.CognMap = obj.updateMap(m);
            end
            pos_shift = obj.Location - loc;
            start = loc(1:2);
            if any(pos_shift ~= 0)
                goal = obj.Goal(1:2) - pos_shift(1:2);
            else
                goal = obj.Goal(:, 1:2);
            end
            obj.AstarAlg(start, goal, pos_shift);
            if obj.Path == Inf
                path = [];
                obj.Path = [];
                obj.pauseCmd = true;
            else
                obj.pauseCmd = false;
                path = obj.Path;
                path(1, :) = [];
%                 obj.Goal = [obj.Path(end, :), 0];
            end
        end
        
        function AstarAlg(obj, start, goal, pos_shift)
            if nargin == 3
                pos_shift = zeros(1, 3);
            end
            Connecting_Distance=0;
            [m, n] = size(obj.CognMap);
            GoalRegister=int8(zeros(m,n));
            [goal_n, ~] = size(goal);
            for i=1:goal_n
                GoalRegister(goal(i, 1),goal(i, 2))=1;
            end
            path = ASTARPATH(start(2), start(1), obj.CognMap, ...
                        GoalRegister, Connecting_Distance);
            path = flip(path);
            obj.Path = path;
            if path == Inf
                % obj.Path = obj.Location(1:2);
                obj.pauseCmd = true;
                return
            else
                obj.pauseCmd = false;
                if goal_n > 1
                    obj.Goal = [obj.Path(end, :), 0];
                end
            end
            
%             obj.modifyPath();
            [n, ~] = size(obj.Path);
            for i = 1:n
                obj.Path(i, :) = obj.Path(i, :) + pos_shift(1:2);
            end
        end
        
        function modifyPath(obj)
            i = 1;
            while true
                n = length(obj.Path);
                if i >= n
                    break;
                end
                loc = obj.Path(i, :);
                nextLoc = obj.Path(i+1, :);
                % delta = 0.1; 
                x_th = 2;
                if all(loc - nextLoc)
                    if norm(loc-nextLoc) > x_th
%                        l = obj.insertPath(i, delta);
                        l = obj.insertPath_re(i);
                        i = i + l;
                        continue;
                    else
                        % check obstacle
%                         nextLoc = [nextLoc(1) loc(2)];
%                         if obj.CognMap(nextLoc) == 1
%                             nextLoc = [loc(1) nextLoc(2)];
%                         end
                        if obj.CognMap(nextLoc(1), loc(2)) == 0
                            nextLoc = [nextLoc(1) loc(2)];
                        elseif obj.CognMap(loc(1), nextLoc(2)) == 0
                            nextLoc = [loc(1) nextLoc(2)];
                        else
                            disp("Wrong path!!");
                        end
                        obj.Path = [obj.Path(1:i, :); nextLoc;...
                            obj.Path(i+1:end, :)];
                        i = i + 1;
                        continue;
                    end
                end
                nextLoc = obj.Path(i+1, :);
                step = abs(loc - nextLoc);
                if step(1) > 1
                    % insert step
                    dist = [(nextLoc(1) - loc(1)) / step(1), 0];
%                     obj.Path = [obj.Path(1:i, :); loc(1)+sign, loc(2);...
%                                 obj.Path(i+1:end, :)];
                elseif step(2) > 1
                    dist = [0, (nextLoc(2) - loc(2)) / step(2)];
                else
                    dist = [];
                end
                if ~isempty(dist)
                    obj.Path = [obj.Path(1:i, :); loc+dist;...
                                obj.Path(i+1:end, :)];
                end
                i = i + 1;
            end
        end
        
        function i_l = insertPath_re(obj, i_s)
            % Find the inserted path recursively
            % i_s: starting index, i_e: ending index
            s = obj.Path(i_s, :); e = obj.Path(i_s+1, :);
            mid = s+(e-s)/2;
            temp_mid = round(mid);
            if obj.CognMap(temp_mid(1), temp_mid(2))
                temp_mid = [floor(mid(1)), floor(mid(2));
                    floor(mid(1)), ceil(mid(2));
                    ceil(mid(1)), floor(mid(2));
                    ceil(mid(1)), ceil(mid(2))];
                mid_check = zeros(1, 4);
                for idx=1:4
                    mid_check(idx) = obj.CognMap(temp_mid(idx, 1), temp_mid(idx, 2));
                end
                idx_checked = find(~mid_check, 1);
                if isempty(idx_checked)
                    disp("Avoid collision failed when inserted path.");
                    mid = round(mid);
                else
                    mid = [temp_mid(idx_checked, 1), temp_mid(idx_checked, 2)];
                end
            else
                mid = temp_mid;
            end
            if all(mid == s) || all(mid == e)
                i_l = 0;
%                 return
            else
                % Insert mid
                obj.Path = [obj.Path(1:i_s, :); mid; obj.Path(i_s+1:end, :)];
                i_l = 1;
            end
            d_s = mid - s; d_e = e - mid;
            if any(abs(d_s) > 1)
                i_ll = obj.insertPath_re(i_s);
            elseif all(abs(d_s) == [1, 1])
                next_loc = [mid(1), s(2)];
                if obj.CognMap(next_loc(1), next_loc(2)) == 1
                    next_loc = [s(1) mid(2)];
                end
                obj.Path = [obj.Path(1:i_s, :); next_loc; ...
                    obj.Path(i_s+1:end, :)];
                i_ll = 1;
            else
                i_ll = 0;
            end
            if any(abs(d_e) > 1)
                i_lr = obj.insertPath_re(i_s+i_ll+1);
            elseif all(abs(d_e) == [1, 1])
                next_loc = [e(1), mid(2)];
                if obj.CognMap(next_loc(1), next_loc(2)) == 1
                    next_loc = [mid(1) e(2)];
                end
                obj.Path = [obj.Path(1:i_s+i_ll+1, :); next_loc; ...
                    obj.Path(i_s+i_ll+2:end, :)];
                i_lr = 1;
            else
                i_lr = 0;
            end
            i_l = i_l + i_ll + i_lr;
        end
        
        function l = insertPath(obj, p_i, dist)
            if nargin <= 2
                dist = 0.1;
            end
            p0 = obj.Path(p_i, :);
            p1 = obj.Path(p_i+1, :);
            total_dist = sqrt(sum((p1 - p0).^2));
            n = floor(total_dist/dist);
            p = zeros(n, 2);
            j = 1;
            for i=1:n
                dx = dist * (p1 - p0) / total_dist;
                px = round(p0(1) + dx(1)*i);
                py = round(p0(2) + dx(2)*i);
                if ((px == p0(1) && py == p0(2)) || ...
                        (j >= 2 && px == p(j-1, 1) && py == p(j-1, 2)))
                    % 如果准备插入的新点在起始点或者与上一个点的位置重合
                    continue;
                elseif px == p1(1) && py == p1(2)
                    continue;
                else
                    p(j, 1) = px;
                    p(j, 2) = py;
                    j = j + 1;
                end
            end
            p(j:end, :) = [];
            [l, ~] = size(p);
            % TODO: 改成递归式查找斜着的路径
%             cur_loc = p(1, :);
%             for i=2:l
%                 next_loc = p(i, :);
%                 
%             end
            obj.Path = [obj.Path(1:p_i, :); p; obj.Path(p_i+1:end, :)];
        end
    end
end
