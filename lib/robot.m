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
        ID                 (1, 1) int32 {mustBeNonnegative} % ID of the robot
        Status             (1, 1) uint8 % (1) initial; (2) fixed; (3) unfixed; (4) free
        isCarrying         (1, 1) logical % Carrying (1) / Not carrying (0) module
        carriedModule      moduleGroup    % Module
        moduleToFetch      module
        Location           (1, 3) double  % Current robot location [x, y, z]
        CognMap                      % Cognitive map of individual robot
        GlobalMap          map       % "pointer" to a global map object
        Goal          (1, 3) double  % Current robot goal [x, y, z]
        startPlace         (1, 3) double   % The location where the robot is initiated.
        Path = [] % Current planned path 
        
    end
	
    methods
        function obj = robot(id,initLocation, gmap)
            %ROBOT 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                % Default construction function
                obj.ID = 0;
                obj.Location = [0, 0, 0];
                obj.GlobalMap = map(0, 0);
                return
            end
            obj.ID = id;
            obj.Location = initLocation;
            obj.startPlace = initLocation;
            obj.GlobalMap = gmap;
            obj.GlobalMap.robotMap(initLocation(1), initLocation(2)) = 1;
        end
        
        function success = fetchModule(obj, m)
            if m.Status == 1
                success = 0;
                return
            end
            if obj.checkNeighbour(m)
                obj.isCarrying = 1;
                obj.carriedModule = moduleGroup(m);
                success = 1;
            else
                obj.moduleToFetch = m.Location;
                obj.Goal = m.Location + [1, 0, 0];    % TODO
                success = 2;
            end
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
                for m = mlst
                    disp("carried module: ");
                    disp(m);
                    loc = m.Location;
                    cognmap = m.updateMap();
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
                loc = obj.Location;
                cogn_map = obj.CognMap;
            end
            next_loc = loc + move_dir;
            all_dir = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
            check_list = [next_loc; next_loc+all_dir(1, :); ...
                next_loc+all_dir(2, :); ...
                next_loc+all_dir(3, :); ...
                next_loc+all_dir(4, :); ...
                next_loc+all_dir(5, :); ...
                next_loc+all_dir(6, :); ...
                next_loc+all_dir(7, :); ...
                next_loc+all_dir(8, :)];
            for i=1:5
                if all(check_list(i, :) > 0)
                    if cogn_map(check_list(i, 1), check_list(i, 2)) == 1
                        decision = false;
                        return
                    end
                end
            end
            decision = true;
        end
        
        function decision = checkNeighbour(obj, element)
%             arguments
%                 obj
%                 element.robot  robot
%                 element.module module
%             end
            loc = element.Location;
            if all(abs(obj.Location - loc) == [0, 1, 0]) || ...
                    all(abs(obj.Location - loc) == [1, 0, 0])
                decision = true;
            else
                decision = false;
            end
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
            
            nextLoc = obj.Path(1, 1:2);
            move_dir = nextLoc - obj.Location(1:2);
            decision = obj.canMove(move_dir);
            if decision == 0
                disp("Robot is blocked.");
                obj.Astar();
                disp("Replanned path:");
                disp(obj.Path);
            elseif decision == -1
                disp("Module group is blocked.");
                mlst = obj.carriedModule.ModuleList;
                for m = mlst
                    m_loc = m.Location;
                    m_path = obj.Astar(m_loc);
                    nextLoc = m_path(1, 1:2);
                    move_dir = nextLoc - obj.Location(1:2);
                    if obj.canMove(move_dir) == 1
                        result = true;
                        disp("Replanned path:");
                        disp(obj.Path);
                        break
                    else
                        result = false;
                    end
                end
                if ~result
                    % TODO: 障碍物增扩
                end
            end
            
            % Step 1: Check obstacle (attempt movement)
            % TODO: 实时避障
            
%             if obj.isCarrying && (~obj.carriedModule.canMove(move_dir))
%                 true_loc = obj.Location;
%                 obj.Location = obj.carriedModule.Location;
%                 obj.Astar();
%                 obj.Path = obj.Path + [1, 0];
%                 obj.Location = true_loc;
%             end
            
            % Step 2: Move to the next location
            while norm(obj.Path(1, 1:2) - obj.Location(1:2)) < e
                disp("delete:");
                disp(obj.Path(1, :));
                obj.Path(1, :) = [];
            end
            nextLoc = obj.Path(1, 1:2);
            disp("next loc:");
            disp(nextLoc);
%             if all(obj.Location(1:2) - nextLoc)
%                 % TODO: again, check obstacle
%                 nextLoc = [nextLoc(1) obj.Location(2)];
% %                 nextLoc = [obj.Location(1) nextLoc(2)];
%             else
%                 obj.Path(1, :) = [];
%             end
            obj.Path(1, :) = [];
            obj.GlobalMap.robotMap(obj.Location(1), obj.Location(2)) = 0;
            obj.GlobalMap.robotMap(nextLoc(1), nextLoc(2)) = 1;
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
        
        function updateMap(obj)
            obj.CognMap = obj.GlobalMap.getMap(obj.Location, "r");
            % Ignore the carried module
            % TODO: change to module group
            if obj.isCarrying
                N = obj.carriedModule.Size;
                module_loc = zeros(N, 3);
                for i=1:N
                    module_loc(i, :) = obj.carriedModule.ModuleList(i).Location;
                    obj.CognMap(module_loc(i, 1), module_loc(i, 2)) = 0;
                end
            end
            if ~isempty(obj.moduleToFetch)
                obj.CognMap(obj.moduleToFetch(1), obj.moduleToFetch(2)) = 0;
            end
            obj.CognMap(obj.Location(1), obj.Location(2)) = 0;
            
            % Obstacle expansion
            [allx, ally] = find(obj.CognMap);
            all_dir = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
            for i=1:length(allx)
                for j = 1:8
                    x = allx(i)+all_dir(j, 1);
                    y = ally(i)+all_dir(j, 2);
                    if x <= 0 || y <= 0
                        continue
                    end
                    obj.CognMap(x, y) = 1;
                end
%                 obj.CognMap(x(i)+1, y(i)) = 1;
%                 obj.CognMap(x(i)-1, y(i)) = 1;
%                 obj.CognMap(x(i), y(i)+1) = 1;
%                 obj.CognMap(x(i), y(i)-1) = 1;
%                 obj.CognMap(x(i)+1, y(i)+1) = 1;
%                 obj.CognMap(x(i)+1, y(i)-1) = 1;
%                 obj.CognMap(x(i)-1, y(i)+1) = 1;
%                 obj.CognMap(x(i)-1, y(i)-1) = 1;
            end
        end
        
        function path = Astar(obj, loc)
            if nargin == 1
                loc = obj.Location;
            end
%             obj.CognMap = obj.GlobalMap.getMap(obj.Location, "r");
            pos_shift = obj.Location - loc;
            obj.updateMap();
            Connecting_Distance=8;
            start = loc(1:2);
            goal = obj.Goal(1:2) - pos_shift(1:2);
            [m, n] = size(obj.CognMap);
            GoalRegister=int8(zeros(m,n));
            GoalRegister(goal(1),goal(2))=1;
            path = ASTARPATH(start(2), start(1), obj.CognMap, ...
                        GoalRegister, Connecting_Distance);
            path = flip(path);
            [n, ~] = size(path);
            for i = 1:n
                path(i, :) = path(i, :) - pos_shift(1:2);
            end
            obj.Path = path;
            obj.modifyPath();
            path = obj.Path;
            path(1, :) = [];
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
                    disp("Avoid collision failed.");
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
