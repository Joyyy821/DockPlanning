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
%         Status             (1, 1) uint8 % (1) initial; (2) fixed; (3) unfixed; (4) free
        % robot group properties
        % lead robot position = slave robot position + pos shift
        pos_shift          int32    % Position difference from the lead robot
        DockJoint          (1, 4) int32 % [up, down, left, right]
        rotation           double
        ignoredPos         (:, 2) int32
        groupPos           (:, 2) int32
        Location           (1, 3) double  % Current robot location [x, y, z]
        CognMap                      % Cognitive map of individual robot
        GlobalMap          map       % "pointer" to a global map object
        tempGroupMap       int32     % Waiting groups on PT
        Goal          (:, 3) double  % Current robot goal [x, y, z]
        Path = [] % Current planned path 
        stepCount = 1                % Number of total steps
        % True if the group with lower priority should be included in planning
        includeLowerPriGp  logical
        pauseCmd  % Robot will not move if true.
    end
	
    methods
        function obj = robot(id,initLocation, dock, gmap)
            %ROBOT 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                % Default construction function
                obj.ID = 0;
                obj.Location = [0, 0, 0];
                obj.GlobalMap = map([0, 0]);
                return
            end
            if nargin == 3
                gmap = map();
            end
            obj.ID = id;
            obj.Location = initLocation;
            obj.DockJoint = dock;
            obj.GlobalMap = gmap;
            obj.tempGroupMap = zeros(obj.GlobalMap.mapSize);
            obj.GlobalMap.robotMap(initLocation(1), initLocation(2)) = id;
            obj.pos_shift = [0, 0, 0];
            obj.groupPos = [initLocation(1:2)];
        end

        function setIgnorePos(obj, locs)
            obj.ignoredPos = locs;
        end
        
        function addIgnorePos(obj, locs)
            obj.ignoredPos = [obj.ignoredPos; locs(:, 1:2)];
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
        
        function decision = checkNextStep(obj, move_dir, loc, cogn_map)
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
        end
        
        
%         function moveModule(obj)
%             obj.carriedRobot.moveModuleGp(obj.Location);
%         end
% 
%         function walk(obj)
%             % move to a random direction (choose from avaliable directions)
%             dirs = [0, 1; 0, -1; 1, 0; -1, 0];
%             avaliable_dirs = [];
%             for i=1:4
%                 if obj.canMove(dirs(i,:))
%                     avaliable_dirs = [avaliable_dirs; dirs(i,:)];
%                 end
%             end
%             [n,~] = size(avaliable_dirs);
%             if n == 0
%                 obj.stuckSteps = obj.stuckSteps + 1;
%             else
%                 i_dir = randi(n);
%                 nextLoc = obj.Location(1:2) + avaliable_dirs(i_dir, :);
%                 obj.GlobalMap.robotMap(obj.Location(1), obj.Location(2)) = 0;
%                 obj.GlobalMap.robotMap(nextLoc(1), nextLoc(2)) = obj.ID;
%                 obj.Location(1:2) = nextLoc;
%                 if obj.isCarrying
%                     obj.moveModule();
%                 end
%                 obj.stuckSteps = 0;
%             end
%         end
%         
        function move(obj)
            e = 10e-3;
            disp("Robot "+string(obj.ID)+" at step No. "...
                +string(obj.stepCount));
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
            obj.stepCount = obj.stepCount + 1;
%             if obj.Location == obj.Goal
%                 isArrive = true;
%             else
%                 isArrive = false;
%             end
        end
        
        function localmap = updateMap(obj, is_planning, m)
            if nargin == 3
                [localmap, pr] = obj.GlobalMap.getMap(m.Location);
            else
                [localmap, pr] = obj.GlobalMap.getMap(obj.Location);
            end
            if nargin == 1
                is_planning = true;
            end
%             if obj.stuckSteps > 5
%                 is_planning = false;
%             end
            % Assume static objects are known
            localmap = localmap | obj.tempGroupMap;
            % Ignore the group
            [N, ~] = size(obj.groupPos);
            for i=1:N
                localmap(obj.groupPos(i,1), obj.groupPos(i,2)) = 0;
            end
            
            % Check the robot priority
            if is_planning && ~obj.includeLowerPriGp
                [Nr, ~] = size(pr);
                for i=1:Nr
                    if pr(i,3) > obj.ID
                        localmap(pr(i,1), pr(i,2)) = 0;
                    end
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
                    for pos=k.'
                        if obj.ignoredPos(pos, 2)==ally(i)
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

            if nargin <= 2
                obj.CognMap = localmap;
            end

            % Set the goal position as empty
            if is_planning
                obj.CognMap(obj.Goal(1), obj.Goal(2)) = 0;
            end
        end
        
        function path = Astar(obj, m)
            if nargin == 1
                loc = obj.Location;
                obj.updateMap();
            else
                loc = m.Location;
                obj.CognMap = obj.updateMap(true, m);
            end
            posshift = obj.Location - loc;
            start = loc(1:2);
            if any(posshift ~= 0)
                goal = obj.Goal(1:2) - posshift(1:2);
            else
                goal = obj.Goal(:, 1:2);
            end
            obj.AstarAlg(start, goal, posshift);
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