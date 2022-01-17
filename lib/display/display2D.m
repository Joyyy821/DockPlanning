classdef display2D < handle 
    %DISPLAY2D Pass the input map information to GUI.
    %   TODO: description
    
    properties
        Map_Size                  % length*width
        Robot_Size = [1, 1]       % length*width
        Obstacle = []
        Robot_Current_Position = []
        Object_Current_Position = []
        Robot_Current_Target = []
        Object_Current_Target = []
        Robot_Final_Target = []
        Object_Final_Target = []
        GUI2D % = GUI2D_exported() % GUI object
    end
    
    methods
        function obj = display2D(mapSize,options)
            arguments
                mapSize (1, 2)
                options.FinalTarget  targetGroup
                options.CurrentTarget targetGroup
                options.Robot  robot
            end
            %DISPLAY2D Constructor
            %   version 2 for extension and robot navigation
            obj.Map_Size = mapSize;
            if isfield(options, "FinalTarget")
                loc =  obj.objToLoc(options.FinalTarget);
                obj.Object_Final_Target = loc;
            end
            if isfield(options, "CurrentTarget")
                loc =  obj.objToLoc(options.CurrentTarget);
                obj.Object_Current_Target = loc;
            end
            if isfield(options, "Robot")
                loc =  obj.objToLoc(options.Robot);
                obj.Robot_Current_Position = loc;
            end
            obj.runGUI2D();
        end
        
        function runGUI2D(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj.loadMap();
            obj.GUI2D = GUI2D_exported();
            obj.GUI2D.initial();
%             disp(obj.GUI2D.Num_robot);
%             obj.GUI2D.show();
        end
        
        function loadMap(obj)
            % Load map info to workspace.
            assignin('base', 'Map_Size', obj.Map_Size);
            assignin('base', 'Robot_Size', obj.Robot_Size);
            assignin('base', 'Obstacle', obj.Obstacle);
            assignin('base', 'Robot_Current_Position', obj.Robot_Current_Position);
            assignin('base', 'Object_Current_Position', obj.Object_Current_Position);
            assignin('base', 'Robot_Current_Target', obj.Robot_Current_Target);
            assignin('base', 'Object_Current_Target', obj.Object_Current_Target);
            assignin('base', 'Robot_Final_Target', obj.Robot_Final_Target);
            assignin('base', 'Object_Final_Target', obj.Object_Final_Target);
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
            end
            % TODO: update the properties and ws variables
%             disp(obj.GUI2D.Num_robot);
            if isfield(options, "CurrentTarget")
                obj.Object_Current_Target = obj.objToLoc(options.CurrentTarget);
                assignin('base', 'Object_Current_Target', obj.Object_Current_Target);
            end
            if isfield(options, "Robot")
                obj.Robot_Current_Position = obj.objToLoc(options.Robot);
                assignin('base', 'Robot_Current_Position', obj.Robot_Current_Position);
            end
%             disp(obj.GUI2D.Num_robot);
            obj.GUI2D.show();
        end
        
        function loc = objToLoc(~, e)
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
                N = length(e);
                loc = zeros(N, 2);
                for r = e
                    id = r.ID;
                    loc(id, :) = r.Location(1:2);
                end
            end
        end
        
    end
end

