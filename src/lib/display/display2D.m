classdef display2D < handle 
    %DISPLAY2D Pass the input map information to GUI.
    %   TODO: description
    
    properties
        Map_Size                  % length*width
        Robot_Size = [1, 1]       % length*width
        Robot_Current_Position = []
        Robot_Dock = []           % With dimension N_robot * 4
        Robot_Path = []
        Robot_PathL = []
        Current_Target = []
        Final_Target = []
        Target_ID = []
        showInUI = true
        GUI2D % = GUI2D_exported() % GUI object
    end
    
    methods
        function obj = display2D(mapSize,options)
            arguments
                mapSize (1, 2)
                options.FinalTarget  targetGroup
                options.CurrentTarget targetGroup
                options.Robot  robot
                options.ShowUI  logical
            end
            %DISPLAY2D Constructor
            %   version 2 for extension and robot navigation
            obj.Map_Size = mapSize;
            if isfield(options, "FinalTarget")
                loc =  obj.objToLoc(options.FinalTarget);
                obj.getTargetID(options.FinalTarget);
                obj.Final_Target = loc;
            end
            if isfield(options, "CurrentTarget")
                loc =  obj.objToLoc(options.CurrentTarget);
                obj.Current_Target = loc;
                obj.getTargetID(options.CurrentTarget);
            end
            if isfield(options, "Robot")
                loc =  obj.objToLoc(options.Robot);
                obj.Robot_Current_Position = loc;
            end
            if isfield(options, "ShowUI")
                obj.showInUI = options.ShowUI;
            end
            obj.runGUI2D();
        end
        
        function runGUI2D(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj.loadMap();
            if obj.showInUI
                obj.GUI2D = GUI2D_exported();
                obj.GUI2D.initial();
            end
%             disp(obj.GUI2D.Num_robot);
%             obj.GUI2D.show();
        end
        
        function loadMap(obj)
            % Load map info to workspace.
            assignin('base', 'Map_Size', obj.Map_Size);
            assignin('base', 'Robot_Size', obj.Robot_Size);
            assignin('base', 'Robot_Current_Position', obj.Robot_Current_Position);
            assignin('base', 'Robot_Dock', obj.Robot_Dock);
            assignin('base', 'Robot_Path', obj.Robot_Path);
            assignin('base', 'Robot_PathL', obj.Robot_PathL);
            assignin('base', 'Current_Target', obj.Current_Target);
            assignin('base', 'Final_Target', obj.Final_Target);
            assignin('base', 'Target_ID', obj.Target_ID);
        end

        function rotateRobot(obj, dock)
            obj.Robot_Dock = dock;
            assignin('base', 'Robot_Dock', obj.Robot_Dock);
            if obj.showInUI
                obj.GUI2D.updateRobotDock();
            end
        end
        
%         function updateMap(obj, objTar)
%             % TODO: update the properties and ws variables
% %             disp(obj.GUI2D.Num_robot);
%             obj.Object_Current_Target = obj.objToLoc(objTar);
%             assignin('base', 'Object_Current_Target', obj.Object_Current_Target);
% %             disp(obj.GUI2D.Num_robot);
%             obj.GUI2D.show();
%         end
        function updateMap(obj, options)
            arguments
                obj
                options.CurrentTarget  targetGroup
                options.Robot  robot
                options.TargetID targetGroup
                options.PartialTargets
            end
            % TODO: update the properties and ws variables
%             disp(obj.GUI2D.Num_robot);
            if isfield(options, "CurrentTarget")
                obj.Current_Target = obj.objToLoc(options.CurrentTarget);
%                 obj.getTargetID(options.CurrentTarget);
                assignin('base', 'Current_Target', obj.Current_Target);
            end
            if isfield(options, "Robot")
                obj.Robot_Current_Position = obj.objToLoc(options.Robot);
                assignin('base', 'Robot_Current_Position', obj.Robot_Current_Position);
                assignin('base', 'Robot_Path', obj.Robot_Path);
                assignin('base', 'Robot_PathL', obj.Robot_PathL);
            end
            if isfield(options, "TargetID")
                obj.getTargetID(options.TargetID);
                assignin('base', 'Target_ID', obj.Target_ID);
                if obj.showInUI
                    obj.GUI2D.updateTargetID = true;
                end
            end
            if isfield(options, "PartialTargets")
%                 idxs = options.PartialTargetIDs;
                locs = options.PartialTargets;
                [n, ~] = size(locs);
                for i=1:n
                    if any(locs(i, :))
                        obj.Current_Target(i, :) = locs(i, :);
                    end
                end
                assignin('base', 'Current_Target', obj.Current_Target);
            end
%             if isfield(options, "Obstacle")
%                 obj.Obstacle
%             disp(obj.GUI2D.Num_robot);
            if obj.showInUI
                obj.GUI2D.show();
                if isfield(options, "TargetID")
                    obj.GUI2D.updateTargetID = false;
                end
            end
        end
        
        function getTargetID(obj, t)
            N = t.Size;
            obj.Target_ID = zeros(1, N);
            for i=1:N
                idx = t.TargetList(i).ID;
                id = t.TargetList(i).displayID;
                obj.Target_ID(idx) = id;
            end
        end
        
        function loc = objToLoc(obj, e)
            %
            
            if class(e) == "targetGroup"
                N = e.Size;
                loc = zeros(N, 2);
                for i=1:N
                    tar = e.TargetList(i);
                    id = tar.ID;
                    loc(id, :) = tar.Location;
                end
            elseif class(e) == "robot"
                % Load positions
                N = length(e);
                loc = zeros(N, 2);
                path_l = zeros(1, N);
                for r = e
                    id = r.ID;
                    loc(id, :) = r.Location(1:2);
                    [path_l(id), ~] = size(r.Path);
                    path_l(id) = path_l(id) + 1;
                end
                % Load dock
                obj.Robot_Dock = zeros(N, 4);
                for i=1:N
                    obj.Robot_Dock(i, 1:4) = e(i).DockJoint;
                end
                % Load paths
                paths = zeros(sum(path_l), 2);
                for r = e
                    id = r.ID;
                    s_idx = sum(path_l(1:id-1));
                    if ~isempty(r.Path)
                        paths(s_idx+1:s_idx+path_l(id), :) = ...
                            [r.Location(1:2); r.Path];
                    else
                        paths(s_idx+1:s_idx+path_l(id), :) = r.Location(1:2);
                    end
                end
                obj.Robot_Path = paths;
                
                obj.Robot_PathL = path_l;
            end
        end
        
    end
end

