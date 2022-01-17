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
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
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
            if all(obj.Location(1:2) - nextLoc)
                % TODO: again, check obstacle
                nextLoc = [nextLoc(1) obj.Location(2)];
%                 nextLoc = [obj.Location(1) nextLoc(2)];
            else
                obj.Path(1, :) = [];
            end
            obj.Location(1:2) = nextLoc;
            isMove = true;
        end
        
        function path = Astar(obj, distance)
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
    end
end
