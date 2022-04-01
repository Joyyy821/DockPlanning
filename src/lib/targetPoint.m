classdef targetPoint < matlab.mixin.Copyable
    % A single target point
    properties (Access = public)
        % Can be directly modified by the user.
        ID            (1, 1) int32 {mustBeNonnegative}  % ID of the target point
        Location      (1, 2) double  % Current target position
        NextPosition  (1, 2) double  % Next position of the target point
        PrevPositions (:, 2) double  % Previous positions of the target point
        GlobalMap      map
    end
    
    properties (Access = private)
        % Cannot be directly modified after assignment
        Type         (1, 1) string  % Robot/Object
    end
    
    methods (Access = public)
        % Constructor function
        function obj = targetPoint(id, loc, g_map, nextPos, prevPos, type)
            if nargin == 0
                % error("User needs to specify the type at least.");
                obj.Type = "Object";
            end
            if nargin >= 6
                % Assign type
                if (type == "Robot") || (type == "robot")
                    obj.Type = "Robot";
                elseif (type == "Object") || (type == "object")
                    obj.Type = "Object";
                else
                    error("Invaild input of type.");
                end
            end
            if nargin >= 1
%                 disp("id: "); disp(id);
                obj.ID = id;  % Assign ID
            end
            if nargin >= 2
%                 disp("loc: "); disp(loc);
                obj.Location = loc;  % Assign location
            end
            if nargin >= 3
                obj.GlobalMap = g_map;
%                 obj.GlobalMap.targetMap(loc(1), loc(2)) = id;
            end
            if nargin >= 4
%                 disp("nextPos: "); disp(nextPos);
                obj.NextPosition = nextPos;  % Assign next position
            end
            if nargin == 5
                obj.PrevPositions = prevPos;  % Assign previous positions
            end
        end
        
        function type = GetType(obj)
            type = obj.Type;
        end

    end
end
