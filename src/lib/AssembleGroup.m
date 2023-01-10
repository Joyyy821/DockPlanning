classdef AssembleGroup < handle
    %ASSEMBLEGROUP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        % Group propereties
        groupID            int32
        LeadRobotID        int32
        RobotList          robot
        Size               int32
        Boundary           (2, 2) int32   % [xmin, ymin; xmax, ymax]

        GlobalMap          map       % "pointer" to a global map object
        status             logical   % 0 = fetch; 1 = construct.
        
        % Planning properties
        stuckSteps    int32
        isStarted          logical   % whether the robot has started assembly
        waiting            logical   % 机器人靠近一个目标，但是需要先等待对接group到达的状态
        priorPlanningID = 1 % The current planning position which has highest priority

        % dock
        dockFlag           logical
        dockGpID           int32
%         changingSite       logical
        dockSiteBlocked    logical
%         delay_access = false

        % Extension
        ext                Extension
        c_tar_i            int32
        
    end
    
    methods
        function obj = AssembleGroup(id, rob, gmap)
            %ASSEMBLEGROUP 构造此类的实例
            %   此处显示详细说明
            if nargin >= 1
                obj.groupID = id;
            end
            if nargin >= 2
                obj.RobotList = rob;
                obj.LeadRobotID = rob.ID;
                obj.Size = 1;
                loc = rob.Location(1:2);
                obj.Boundary = [loc; loc];
            end
            if nargin >= 3
                obj.GlobalMap = gmap;
                obj.GlobalMap.groupMap(loc(1), loc(2)) = obj.groupID;
            end
            obj.stuckSteps = 0;
            obj.waiting = false;
            obj.isStarted = false;
        end
        
        %% Search methods
     
        function decision = isLocInMap(obj, loc)
            if any(loc(1:2) > obj.GlobalMap.mapSize) || ...
                    any(loc(1:2) < 1)
                decision = false;
            else
                decision = true;
            end
        end
        
        %%  Moving methods

        function setGoal(obj, lead_goal)
            for i=1:obj.Size
                obj.RobotList(i).Goal = int32(lead_goal) - obj.RobotList(i).pos_shift;
            end
        end

        function decision = canMove(obj, move_dir)
            for i =1:obj.Size
                obj.RobotList(i).updateMap(false);
                % check robot
                if ~obj.RobotList(i).checkNextStep(move_dir)
                    decision = 0;
                    return
                end
            end
            decision = 1;
        end

        function obstacleExpansion(obj)
            % TODO
            % get the module group size and the carring direction
            obj.RobotList(1).updateMap();
            map_cp = obj.RobotList(1).CognMap;
            gp_loc = obj.Boundary;
            loc = obj.RobotList(1).Location(1:2);
            % Extension length (in the extension direction, opposite from 
            % the module carrying direction): 
            % [left, lower; right, upper];
            ext_l = gp_loc - [loc; loc];
            ext_l(1, :) = max(ext_l(1, :), [0, 0]);
            ext_l(2, :) = abs(min(ext_l(2, :), [0, 0]));
            % Combine the cogn map of robot and modules
            for i=2:obj.Size
                r_map = obj.RobotList(i).updateMap(true);
                obj.RobotList(1).CognMap = obj.RobotList(1).CognMap | r_map;
            end
            % expand the obstacle accordingly
            [M, N] = size(map_cp);
            for i=1:M
                for j=1:N
                    if obj.RobotList(1).CognMap(i, j) == 1
                        left = max(i - ext_l(1, 1), 1);
                        right = min(i + ext_l(2, 1), M);
                        down = max(j - ext_l(1, 2), 1);
                        up = min(j + ext_l(2, 2), N);
                        map_cp(left:right, down:up) = 1;
                    end
                end
            end
            
            % TODO: check the direction. expand the margin
            map_cp(M-ext_l(1,1)+1:M, :) = 1;
            map_cp(1:ext_l(2,1), :) = 1;
            map_cp(N-ext_l(1,2)+1:N, :) = 1;
            map_cp(1:ext_l(2,2), :) = 1;
            obj.RobotList(1).CognMap = map_cp;
        end

        function updatePath(obj, path, rob_i)
            % According to the path proposed by rob_i, update the path for
            % all other robots in this group.
            % if rob_i = 1, update the path for other robots based on the
            % pos_shift; else update lead robot's path first and then
            % repeat.
            if isempty(path)
                for i=1:obj.Size
                    obj.RobotList(i).Path = [];
                end
            else
                % Update path for the lead robot
                if rob_i ~= 1
                    [n, ~] = size(path);
                    pos_shift = obj.RobotList(rob_i).pos_shift(1:2);
                    for i=1:n
                        obj.RobotList(1).Path(i,:) = int32(obj.RobotList(1).Path(i,:)) + pos_shift;
                    end
                else
                    [n, ~] = size(obj.RobotList(1).Path);
                end
                % Update path for the slave robots
                for i=2:obj.Size
                    pos_shift = obj.RobotList(i).pos_shift(1:2);
                    for j=1:n
                        obj.RobotList(i).Path(j,:) = int32(obj.RobotList(1).Path(j,:)) - pos_shift;
                    end
                end
            end
        end

        function [result, path] = plan(obj)
            % move the robot randomly if it has stuck for more than 10
            % steps.
            if obj.waiting
%                 obj.stuckSteps = obj.stuckSteps + 1;
                result = false;
                path = [];
                obj.updatePath([]);
                return
            end
            if obj.stuckSteps > 5
                flag = true;
            else
                flag = false;
            end
            for i=1:obj.Size
                obj.RobotList(i).includeLowerPriGp = flag;
            end
            
            % Check whether the robot has arrived at the goal place.
            e = 10e-3;
            [goal_n, ~] = size(obj.RobotList(1).Goal);
            for i=1:goal_n
                if norm(obj.RobotList(i).Location(1:2) - obj.RobotList(i).Goal(i, 1:2)) < e
                    disp("Group "+string(obj.groupID)+" already at the goal position "+...
                        "["+string(obj.RobotList(i).Goal(i, 1))+", "+...
                        string(obj.RobotList(i).Goal(i, 2))+"].");
                    result = true;
                    obj.RobotList(i).Goal = obj.RobotList(i).Goal(i, :);
                    path = [];
                    obj.updatePath([]);
                    return
                end
            end
            
            % Planning in turn
            if obj.Size == 1
                obj.priorPlanningID = 1;
            end
            obj.RobotList(obj.priorPlanningID).Astar();
            path = obj.RobotList(obj.priorPlanningID).Path;
            loc = obj.RobotList(obj.priorPlanningID).Location(1:2);

            if obj.RobotList(obj.priorPlanningID).pauseCmd
                disp("Group "+string(obj.groupID)+" stuck.");
                result = false;
                obj.stuckSteps = obj.stuckSteps + 1;
                obj.updatePath(path, obj.priorPlanningID);
                return
            end

            while norm(path(1, 1:2) - loc) < e
%                 disp("delete:");
%                 disp(obj.Path(1, :));
                path(1, :) = [];
            end
            nextLoc = path(1, 1:2);
            move_dir = nextLoc - loc;
            cnt = 0;
            while true
                % Find a feasible path for the group
                decision = obj.canMove(move_dir);
                if decision ~= 1 && ...
                        (obj.Size > 1 && cnt < obj.Size-1)
                    % Rotate to the next one
                    obj.priorPlanningID = rem(obj.priorPlanningID, obj.Size) + 1;
                    disp("Group "+string(obj.groupID)+" is blocked. Replanning path...");
                    path = obj.RobotList(obj.priorPlanningID).Path;
                    if obj.RobotList(obj.priorPlanningID).pauseCmd
                        disp("Group "+string(obj.groupID)+" blocked.");
                        result = false;
                        obj.stuckSteps = obj.stuckSteps + 1;
                        obj.updatePath(path, obj.priorPlanningID);
                        return
                    end
                    % Update the move direction and count
                    nextLoc = path(1, 1:2);
                    loc = obj.RobotList(obj.priorPlanningID).Location(1:2);
                    move_dir = nextLoc - loc;
                    cnt =cnt + 1;
                elseif decision == 1
                    disp("Replan success from object No. "+...
                        string(obj.priorPlanningID));
%                     disp(obj.Path);
                    break
                elseif (obj.Size > 1 && cnt == obj.Size-1)
                    % No path can work, consider obstacle expansion
                    obj.obstacleExpansion();
                    obj.RobotList(1).AstarAlg(obj.RobotList(1).Location(1:2), ...
                        obj.RobotList(1).Goal(1:2));
                    
                    if obj.RobotList(1).pauseCmd == true
                        disp("Group "+string(obj.groupID)+" blocked.");
                        path = [];
                        result = false;
                        obj.stuckSteps = obj.stuckSteps + 1;
                        obj.updatePath(path, 1);
                        return
                    else
                        disp("Replanned path by obstacle expansion.");
%                         disp(obj.Path);
                        path = obj.RobotList(1).Path;
                        next_loc = path(2, 1:2);
                        move_dir = next_loc - obj.RobotList(1).Location(1:2);
                        cnt =cnt + 1;
%                         break
                    end
                else
                    for j=1:obj.Size
                        obj.RobotList(j).pauseCmd = true;
                    end
                    disp("Group "+string(obj.groupID)+" stuck.");
                    obj.updatePath([], 1);
                    path = [];
                    result = false;
                    obj.stuckSteps = obj.stuckSteps + 1;
                    return
                end
            end
            obj.updatePath(path, obj.priorPlanningID);
            result = false;
        end

        function is_arrive = move(obj)
            obj.updateMap("del");
            [is_arrive, path] = obj.plan();
            if ~isempty(path)
                for i=1:obj.Size
                    obj.RobotList(i).move();
                end
                obj.stuckSteps = 0;
            end
            obj.setGroupLocForRobot();
            obj.updateMap("add");
        end
        
        function markGroupObstacle(obj, g_id)
            if g_id == 0
                obj.LeadRobot.tempGroupMap = zeros(size(obj.LeadRobot.tempGroupMap));
            else
                [r, c] = find(obj.GlobalMap.workerRobotMap==g_id);
                for i=1:length(r)
                    obj.LeadRobot.tempGroupMap(r(i), c(i)) = g_id;
                end
            end
        end

        function setIgnorePos(obj, locs)
            for i=1:obj.Size
                obj.RobotList(i).setIgnorePos(locs);
            end
        end
        
        function updateMap(obj, options)
            if options == "add"
                val = obj.groupID;
            elseif options == "del"
                val = 0;
            end
            for i=1:obj.Size
                loc = obj.RobotList(i).Location;
                obj.GlobalMap.groupMap(loc(1), loc(2)) = val;
                if obj.status
                    obj.GlobalMap.workerRobotMap(loc(1), loc(2)) = val;
                end
            end
        end
        
        function addGroup(obj, r)
            arguments
                obj   AssembleGroup
                r     AssembleGroup
            end
            obj.RobotList = [obj.RobotList, r.RobotList];
            obj.Size = obj.Size + r.Size;
            % update boundary
            obj.CombineBoundary(r.Boundary);
            % update pos shift 
            r_loc = obj.RobotList(1).Location;
            for i=(obj.Size-r.Size+1):obj.Size
                loc = obj.RobotList(i).Location;
                obj.RobotList(i).pos_shift = r_loc - loc;
                % update map
                obj.GlobalMap.groupMap(loc(1), loc(2)) = obj.groupID;
            end
            % update robot attribute
            obj.setGroupLocForRobot();
        end

        function setGroupLocForRobot(obj)
            locs = obj.getLocation();
            for i=1:obj.Size
                obj.RobotList(i).groupPos = locs;
            end
        end

        function locs = getLocation(obj)
            locs = zeros(obj.Size, 2);
            for i=1:obj.Size
                locs(i, :) = obj.RobotList(i).Location(1:2);
            end
        end
        
%         function docks = getDock(obj)
%             docks = zeros(obj.Size, 4);
%             for i = 1:obj.Size
%                 docks(i, :) = obj.RobotList(i).DockJoint;
%             end
%         end
        
        %%  Auxiliary methods

        function CombineBoundary(obj, bound)
            obj.Boundary(1, :) = min(obj.Boundary(1, :), bound(1, :));
            obj.Boundary(2, :) = max(obj.Boundary(2, :), bound(2, :));
        end
        
        function decision = robotInLocs(obj, locs)
            r_loc = obj.LeadRobot.Location;
            [n,~] = size(locs);
            for i=1:n
                if all(r_loc(1:2) == locs(i, 1:2))
                    decision = true;
                    return
                end
            end
            decision = false;
        end
        
    end
end

