classdef robot < handle
    %ROBOT represents robot that will construct the shape.
    %   The robots:
    %   (1) are identical and labelled;
    %   (2）can locate themselves in the global map;
    %   (3) can sense the neighbouring environment (location of 
    %   objects/modules/robots);
    %   (4) run planning algorithms and move to goal positions.
    
    properties (Access = public)
        % Can be directly modified by the user.
        ID            (1, 1) int32 {mustBeNonnegative} % ID of the robot
        Status        (1, 1) uint8 % (1) initial; (2) fixed; (3) unfixed; (4) free
        isCarrying    (1, 1) logical % Carrying (1) / Not carrying (0) module
        Location           (1, 3) double  % Current robot location [x, y, z]
        localMap                    % Sensing Map
        CognMap                     % Cognitive map of individual robot
        Goal          (1, 3) double  % Current robot goal [x, y, z]
        Path = [] % Current planned path 
        
    end
   
    methods
        function obj = robot(id,initLocation)
            %ROBOT 构造此类的实例
            %   此处显示详细说明
            obj.ID = id;
            obj.Location = initLocation;
        end

        function isMove = move(obj)
            e = 10e-3;
            if isempty(obj.Path)
                if norm(obj.Location(1:2) - obj.Goal(1:2)) < e
                    disp("Robot "+string(obj.ID)+" already at the goal position "+...
                        "["+string(obj.Goal(1))+", "+string(obj.Goal(2))+"].");
                    isMove = false;
                    return
                else
                    p = obj.Astar();
                    disp("path:");
                    disp(p);
                end
            end
            
            while norm(obj.Path(1, 1:2) - obj.Location(1:2)) < e
                obj.Path(1, :) = [];
            end
            % TODO: Step 1: Check obstacle
            
            % Step 2: Move to the next location
            nextLoc = obj.Path(1, 1:2);
%             if all(obj.Location(1:2) - nextLoc)
%                 % TODO: again, check obstacle
%                 nextLoc = [nextLoc(1) obj.Location(2)];
% %                 nextLoc = [obj.Location(1) nextLoc(2)];
%             else
%                 obj.Path(1, :) = [];
%             end
            obj.Path(1, :) = [];
            obj.Location(1:2) = nextLoc;
            isMove = true;
        end
        
        function path = HAstar(obj, distance)
            % default value
            if (nargin<3)
                distance = 0.01;
            end
            % 2D problem
            startPose = [obj.Location(1:2) 0];
            goalPose = [obj.Goal(1:2) 0];
            % Create a state space object
            stateSpace = stateSpaceSE2;
            % Create a state validator object
            validator = validatorOccupancyMap(stateSpace);
            % Create a binary occupancy map and assign the map to the state validator object.
            validator.Map = binaryOccupancyMap(obj.CognMap);
            % Set the validation distance for the validator
            validator.ValidationDistance = distance;
            % Assign the state validator object to the plannerHybridAStar object
            planner = plannerHybridAStar(validator);
            % Compute a path for the given start and goal poses
            pathObj = plan(planner,startPose,goalPose);
            % Extract the path poses from the path object
            disp("origin path: "); disp(pathObj.States);
            obj.Path = floor(pathObj.States);
            path = obj.Path;
        end
        
        function path = Astar(obj)
            Connecting_Distance=8;
            start = obj.Location(1:2);
            goal = obj.Goal(1:2);
            [m, n] = size(obj.CognMap);
            GoalRegister=int8(zeros(m,n));
            GoalRegister(goal(1),goal(2))=1;
            obj.Path = ASTARPATH(start(2), start(1), obj.CognMap, ...
                        GoalRegister, Connecting_Distance);
            obj.Path = flip(obj.Path);
            obj.modifyPath();
            path = obj.Path;
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
                delta = 0.1; x_th = 2;
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
            mid = round(s+(e-s)/2);
            if obj.CognMap(mid(1), mid(2))
                % Avoid collision
                dir = [0, 1; 1, 0; -1, 0; 0, -1; 1, 1; 1, -1; -1, -1; -1, 1];
                mid_lst = zeros(1, length(dir));
                for idx = 1:length(dir)
                    mid_lst(idx) = obj.CognMap(mid(1)+dir(idx, 1), ...
                        mid(2)+dir(idx, 2));
%                     mid_lst = [obj.CognMap(mid(1)+dir(1, 1), mid(2)+dir(1, 2)), ...
%                         obj.CognMap(mid(1)+dir(2, 1), mid(2)+dir(2, 2)), ...
%                         obj.CognMap(mid(1)+dir(3, 1), mid(2)+dir(3, 2)), ...
%                         obj.CognMap(mid(1)+dir(4, 1), mid(2)+dir(4, 2))];
                end
                i_dir = find(~mid_lst);
                if isempty(i_dir)
                    disp("Avoid obstacle failed.");
                else
                    for idx=i_dir
                        temp = mid + dir(idx, :);
                        if ~(all(temp == s) || all(temp == e))
                            mid = temp;
                            break;
                        end
                    end
                end
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
                if obj.CognMap(next_loc) == 1
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
                if obj.CognMap(next_loc) == 1
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
