classdef module < handle
    %MODULE Represent the modules that robots will used for construction.
    %   The modules:
    %   (1) are identical and unacuated;
    %   (2) have fixed locations if attached to robot/bank/growing
    %   structure; 
    %   (3) have uncertain locations if not attached (because of the drift
    %   of water);
    %   (4) are able to communicate with nearby modules and robots;
    %   (5) can detect the obstacles in the nearby environment;
    %   (6) can active dock to other modules and robots.
    
    properties (Access = public)
        % Can be directly modified by the user.
        ID            (1, 1) int32 {mustBeNonnegative} % ID of the module
        Status        (1, 1) logical % Attached (1) / Unattached (0)
        Location      (1, 3) double  % Current module location [x, y, z]
        LeadRobot     (1, 1) robot   % leading robot of the module
        % Remark: robot position = module position + PosShift.
        PosShift      (1, 3) double  % Position difference compares with the lead robot
        CognMap
        GlobalMap      map
    end
    
    methods
        function obj = module(id, loc, g_map)
            %MODULE Constructor
            %   1. Initialize module No.1 at location [1, 2, 0]:
            %   m = module(1, [1, 2, 0]);
            if nargin == 0
                % Default construction function
                obj.ID = 0;
                obj.Location = [0, 0, 0];
                obj.GlobalMap = map(0, 0);
                return
            end
            obj.ID = id;
            obj.Location = loc;
            obj.Status = 0;
            obj.GlobalMap = g_map;
            obj.GlobalMap.moduleMap(loc(1), loc(2)) = 1;
        end
        
        % function 查询地图，并返回是否被阻挡
        function decision = canMove(obj, move_dir)
            % move_dir: [0, 1]; [1, 0]; [0, -1]; [-1, 0].
            % update cognition map
            obj.updateMap();
            next_loc = obj.Location(1:2)+move_dir;
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
                    if obj.CognMap(check_list(i, 1), check_list(i, 2)) == 1
                        decision = false;
                        return
                    end
                end
            end
            decision = true;
        end
        
        function cogn_map = updateMap(obj)
            obj.CognMap = obj.GlobalMap.getMap(obj.Location, "m");
            if obj.Status == 0
                error("Function updateMap can only be used after module attachment.");
            else
                % Ignore the leading robot and other modules.
                rob_loc = obj.Location(1:2)+obj.PosShift(1:2);
                obj.CognMap(rob_loc(1), rob_loc(2)) = 0; % robot
                obj.CognMap(obj.Location(1), obj.Location(2)) = 0;
%                 siblings = obj.LeadRobot.carriedModule.ModuleList;
%                 for s = siblings
%                     obj.CognMap(s.Location(1), s.Location(2)) = 0;
%                 end
                
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
                end
            end
            cogn_map = obj.CognMap;
        end
        
        % function move 更新自己的位置，并更改地图上的位置
        function success = move(obj)
            if obj.Status == 0
                disp("Module "+string(obj.ID)+" cannot be moved because it" ...
                    + " has not been attached to any group yet.");
                success = false;
            else
%                 disp("robot: ");
%                 disp(obj.LeadRobot.Location);
%                 disp("shift: ");
%                 disp(obj.PosShift);
                next_loc = obj.LeadRobot.Location - obj.PosShift;
                obj.GlobalMap.moduleMap(obj.Location(1), obj.Location(2)) = 0;
                obj.GlobalMap.moduleMap(next_loc(1), next_loc(2)) = 1;
                obj.Location = next_loc;
                success = true;
            end
        end
        
        function success = dock(obj, options)
            % Active dock to other robot or module (group).
            % TODO 这个应该是moduleGroup的方法
            arguments
                obj
                options.robot  robot
                options.module moduleGroup
            end
%             if ~obj.checkNeighbour(options)
%                 success = false; 
%                 return
%             end
            if isfield(options, "robot")
                if ~obj.checkNeighbour(options.robot)
                    success = false; 
                    return
                end
                obj.LeadRobot = options.robot;
                obj.PosShift = options.robot.Location - obj.Location;
            elseif isfield(options, "module")
                if ~obj.checkNeighbour(options.module)
                    success = false; 
                    return
                end
                % TODO: set moduleGroup properties
            else
                success = false;
                return
            end
            obj.Status = 1;
            success = true;
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
        
    end
end

