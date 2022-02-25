classdef GUI2D_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        GridLayout                 matlab.ui.container.GridLayout
        LeftPanel                  matlab.ui.container.Panel
        MapInformationLabel        matlab.ui.control.Label
        NumberofrobotsLabel        matlab.ui.control.Label
        NumberofobjectsLabel       matlab.ui.control.Label
        PauseButton                matlab.ui.control.Button
        StepsLabel                 matlab.ui.control.Label
        ItemstodisplayLabel        matlab.ui.control.Label
        PathSwitch                 matlab.ui.control.Switch
        DisplaypathLabel_2         matlab.ui.control.Label
        CTargetSwitch              matlab.ui.control.Switch
        DisplaycurrenttargetLabel  matlab.ui.control.Label
        ObjectSwitch               matlab.ui.control.Switch
        DisplayobjectLabel         matlab.ui.control.Label
        RobotSwitch                matlab.ui.control.Switch
        DisplayrobotLabel          matlab.ui.control.Label
        GridSwitch                 matlab.ui.control.Switch
        DisplaygridLabel           matlab.ui.control.Label
        FTargetSwitch              matlab.ui.control.Switch
        DisplayfinaltargetLabel    matlab.ui.control.Label
        ObstacleSwitch             matlab.ui.control.Switch
        DisplayobstacleLabel       matlab.ui.control.Label
        NumberofobstaclesLabel     matlab.ui.control.Label
        RightPanel                 matlab.ui.container.Panel
        UIAxes                     matlab.ui.control.UIAxes
        UIAxesLegend               matlab.ui.control.UIAxes
        AdjustthemovingvelocityoftheitemsSliderLabel  matlab.ui.control.Label
        AdjustthemovingvelocityoftheitemsSlider  matlab.ui.control.Slider
        assigntargetSwitchLabel    matlab.ui.control.Label
        assigntargetSwitch         matlab.ui.control.Switch
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    
    properties (Access = private)
        
        Map_x = 0 % Length x
        Map_y = 0 % Length y
        
        % Robot size
        Robot_size = [1, 1]
        
        % Struct of moving items and targets
        Robot = struct( ...
                       'position', {}, ... % Current position of the robot
                       'nextPos', {}, ... % Next position of the robot
                       'handlers', {}, ... % Handler of the robot figure and the text label (2D)
                       'target', {}, ... % Label of the current target
                       ...'targetPosition', {}, ... % Position of the current target
                       'path', {},... % Planed path for the items to move
                       'pathHandler', {} ... % Handler of the path line (1D)
                       )

        Object = struct(...
                        'position', {}, ... 
                        'nextPos', {}, ...
                        'handlers', {}, ...
                        'target', {} ...
                        )

        Robot_targets = struct(... Current target
                         'assignedLabel', {}, ... Label after assignment
                         'position', {}, ...
                         'nextPos', {}, ...
                         'handlers', {} ...
                         )
        
        Object_targets = struct(... Current target
                         'assignedLabel', {}, ... Label after assignment
                         'position', {}, ...
                         'nextPos', {}, ...
                         'handlers', {} ...
                         )
        
        % Static items
        Obstacle = []  % Position of obstacle(s)
        Obstacle_h = []  % Handler of obstacle (1D, w/o label)
        Robot_final_target = []  % Final target of robot(s)
        Robot_final_target_h = []  % Handler of robot final target (2D, w label)
        Obj_final_target = []  % Final target of object(s)
        Obj_final_target_h = []  % Handler of object final target (2D, w label)
        
        % Representation of items
        Robot_fig = imread("robot.jpg")
        Color_list = ["r", "#0072BD", "[0.5 0.5 0.5]", "#D95319", 'w'] % Color used for display
        % (1) Robot (2) Object (3) Obstacle (4) Path (5) white (for text)
        Target_alpha = 0.5
        Path_shape = '-'
        Path_width = 1
        
        % Map information
        Num_robot = 0  % Number of robot(s)
        Num_obj = 0  % Number of object(s)
        Num_obj_target = 0;
        Num_robot_target = 0;
        Num_obstacle = 0  % Number of obstacle(s)
        step = 0  % Number of step(s)
        
        % Controlling parameters
        V_move = 10  % The velocity for moving items
        Stop_event = false  % Whether the moving process has been paused.
        CTarget_created = false; % Whether the current targets have been decided.
        Target_assigned = false  % Whether the current targets have been assigned to robots or objects.

    end
    
    methods (Access = public)
        
        % initialization
        function initial(app)
            hold(app.UIAxes)
            hold(app.UIAxesLegend)
            % display the legend
            app.displayLegend();
            
            % load robot and object list
            app.loadData();

            % display 
            app.displayItems();
        end
        
        % Main function: show one step movement in the map
        function show(app)
            % load robot and object list
            app.updateData();

            % move all items
            app.moveItems();                        
        end
        
    end
    
    methods (Access = private)
        
        %% functions
        % load variables from workspace
        function loadData(app)
            % Load data from workspace
            % map
            MapDimension = evalin('base', 'Map_Size');
            app.Map_x = MapDimension(1);
            app.Map_y = MapDimension(2);
            disp("Map dimension has been loaded.");
            
            % robot
            app.Robot_size = evalin('base', 'Robot_Size');
            Robot_pos = evalin('base', 'Robot_Current_Position');
            [app.Num_robot, ~] = size(Robot_pos);
            for i = 1:app.Num_robot
                app.Robot(i).position = Robot_pos(i, :);
                app.Robot(i).nextPos = app.Robot(i).position;
            end
            disp('Robot position(s) has/have been loaded.');

            % object
            Obj_pos = evalin('base', 'Object_Current_Position');
            [app.Num_obj, ~] = size(Obj_pos);
            for i = 1:app.Num_obj
                app.Object(i).position = Obj_pos(i, :);
                app.Object(i).nextPos = app.Object(i).position;
            end
            disp('Object position(s) has/have been loaded.');
                        
            % final target
            app.Robot_final_target = evalin('base', 'Robot_Final_Target');
            app.Obj_final_target = evalin('base', 'Object_Final_Target');
            disp('Final target(s) has/have been loaded.');
            [app.Num_robot_target, ~] = size(app.Robot_final_target);
            [app.Num_obj_target, ~] = size(app.Obj_final_target);
%             disp(app.Num_obj_target);
            
            % current target
            robot_target = evalin('base', 'Robot_Current_Target');
            obj_target = evalin('base', 'Object_Current_Target');
            %%% TODO: ÿÿÿÿÿÿcurrent target ÿÿ %%%
            
            for i = 1:length(robot_target)
                app.Robot_targets(i).position = robot_target(i, :);
                app.Robot_targets(i).nextPos = app.Robot_targets(i).position;
            end
            
            for i = 1:length(obj_target)
                app.Object_targets(i).position = obj_target(i, :);
                app.Object_targets(i).nextPos = app.Object_targets(i).position;
            end
            if isempty(robot_target)
%                 for i = 1:app.Num_robot_target
%                     app.Robot_targets(i).position = app.Robot_final_target(i, :);
%                 end
                app.CTarget_created = false;
            end
            if isempty(obj_target)
%                 for i = 1:app.Num_obj_target
%                     app.Object_targets(i).position = app.Obj_final_target(i, :);
%                 end
                app.CTarget_created = false;
            else
                app.CTarget_created = true;
            end
            disp('Current target(s) has/have been loaded.');

            
            % obstacle
            app.Obstacle = evalin("base", 'Obstacle');
            app.Num_obstacle = length(app.Obstacle);
            disp('Obstacle(s) has/have been loaded.');
            
            % path
            % TODO
            
            % Update map info
            app.NumberofrobotsLabel.Text = "Number of robot(s): " + int2str(app.Num_robot);
            app.NumberofobjectsLabel.Text = "Number of object(s): " + int2str(app.Num_obj);
            app.NumberofobstaclesLabel.Text = "Number of obstacle(s): " + int2str(app.Num_obstacle);
            app.StepsLabel.Text = "Step(s): " + int2str(app.step);
            
%             % Check whether the number of position(s) and the number of
%             % target(s) are in accordance.
%             if app.Num_robot == length(robot_target) && app.Num_obj == length(obj_target)
%                 success = true;
%             else
%                 success = false;
%                 disp('Numbers of the loaded position(s) and target(s) are not in accordance.');
%             end
        end
        
        function updateData(app)
            % next position
            Robot_pos_next = evalin('base', 'Robot_Current_Position');
            Robot_target_next = evalin('base', 'Robot_Current_Target');
            Obj_pos_next = evalin('base', 'Object_Current_Position');
            Obj_target_next = evalin('base', 'Object_Current_Target');
            
            for i = 1:app.Num_robot
                app.Robot(i).nextPos = Robot_pos_next(i, :);
            end
            for i=1:app.Num_robot_target
                app.Robot_targets(i).nextPos = Robot_target_next(i, :);
            end
            
            for i = 1:app.Num_obj
                app.Object(i).nextPos = Obj_pos_next(i, :);
            end
            if ~isempty(Obj_target_next)
                for i=1:app.Num_obj_target
                    app.Object_targets(i).nextPos = Obj_target_next(i, :);
                end
%             else
%                 app.Object_targets(i).nextPos = app.Object_targets(i).position;
            end
            
            % If target has been assigned and target need to be switched
            %% TODO
            
            % steps
%             app.step = evalin('base', 'steps');
            app.step = app.step + 1;
            app.StepsLabel.Text = "Step(s): " + int2str(app.step);
        end
        
        
        % display all items
        function displayItems(app)
            
            % Display current object
            for i=1:app.Num_obj
                % draw object with labelled id.
                obj_pos = [app.Object(i).position(1), app.Object(i).position(2)];
                app.Object(i).handlers = app.drawItem(app.UIAxes, "Object", obj_pos, i);
            end
            for i=1:app.Num_obj_target
                % Display current object target
                if app.CTarget_created
                    obj_target_pos = [app.Object_targets(i).position(1), app.Object_targets(i).position(2)];
                    app.Object_targets(i).handlers = app.drawItem(app.UIAxes, "Object_target", ...
                        obj_target_pos, i);
                end
                % Display final object target
                obj_h = app.drawItem(app.UIAxes, "Object_final_target", ...
                    app.Obj_final_target(i, :));
                app.Obj_final_target_h = [app.Obj_final_target_h; obj_h];
                
            end
            
            % Display current robot
            for i=1:app.Num_robot
                % draw robot with labeled id.
                robot_pos = [app.Robot(i).position(1), app.Robot(i).position(2)];
                app.Robot(i).handlers = app.drawItem(app.UIAxes, "Robot", robot_pos, i);
            end
            for i=1:app.Num_robot_target
                % Display current robot target
                if app.CTarget_created
                    robot_target_pos = [app.Robot_targets(i).position(1), app.Robot_targets(i).position(2)];
                    app.Robot_targets(i).handlers = app.drawItem(app.UIAxes, "Robot_target", ...
                        robot_target_pos, i);
                end
                % Display final robot target
                robot_h = app.drawItem(app.UIAxes, "Robot_final_target", ...
                    app.Robot_final_target(i, :));
                app.Robot_final_target_h = [app.Robot_final_target_h; robot_h];
                
                %%%% !!!TODO!!! 20210511 Edit from HERE %%%%
                % Display path
                %% TODO
                
            end
            
            
            % Display obstacles
            for i=1:app.Num_obstacle
                app.Obstacle_h = [app.Obstacle_h, app.drawItem(app.UIAxes, "Obstacle", ...
                    app.Obstacle(i, :))];
            end
                
%             uistack(app.Robot.handlers, "top");
            % coordinate system
            axis(app.UIAxes, [0, app.Map_x, 0, app.Map_y]);
            set(app.UIAxes, 'XTick', 0:app.Map_x, ...
                'YTick', 0:app.Map_y, ...
                'XTickMode', 'manual', 'YTickMode', 'manual');
        end
        
        % move all Items to the current positions
        function success = moveItems(app)
            accuracy = 0.001;
            isMoveFinished = false;
            while ~(app.Stop_event || isMoveFinished)
%                 disp('in loop')

                % stopping criterion
                isMoveFinished = true;
%                 if (all(all(abs(app.Robot.position - app.Robot.nextPos) < accuracy)) && ...
%                         all(all(abs(app.Object.position - app.Object.nextPos) < accuracy)) && ...
%                         all(all(abs(app.Robot_targets.position - app.Robot_targets.nextPos) < accuracy)) && ...
%                         all(all(abs(app.Object_targets.position - app.Object_targets.nextPos) < accuracy)))
%                     disp('end')
%                     success = true;
%                     return;
%                 else
                    % move robot
                for i=1:app.Num_robot
                    if ~(norm(app.Robot(i).position - app.Robot(i).nextPos) < accuracy)
                        isMoveFinished = false;
%                         disp('Robot move: from');
%                         disp(app.Robot(i).position);
                        for j=1:2
                            sign = app.isASmallerThanB(app.Robot(i).position(j), ...
                                app.Robot(i).nextPos(j));
                            if j == 1
                            app.Robot(i).handlers(1).XData = ...
                                app.Robot(i).handlers(1).XData + sign * 0.1;
                            else
                                app.Robot(i).handlers(1).YData = ...
                                    app.Robot(i).handlers(1).YData + sign * 0.1;
                            end
                            app.Robot(i).handlers(2).Position(j) = ...
                                app.Robot(i).handlers(2).Position(j) + sign * 0.1;
                            % Update position
                            app.Robot(i).position(j) = app.Robot(i).handlers(2).Position(j)...
                                - app.Robot_size(j)/2;
                        end
%                         disp('to');
%                         disp(app.Robot(i).nextPos);
                    end
                end
                
                % Move robot target
                for i=1:app.Num_robot_target
                    if app.CTarget_created
                        if ~(norm(app.Robot_targets(i).position - app.Robot_targets(i).nextPos) < accuracy)
                            isMoveFinished = false;
    %                         disp('Robot target move: from');
    %                         disp(app.Robot_targets(i).position);
                            for j=1:2
                                sign = app.isASmallerThanB(app.Robot_targets(i).position(j), ...
                                    app.Robot_targets(i).nextPos(j));
                                
                                app.Robot_targets(i).handlers(1).Position(j) = ...
                                    app.Robot_targets(i).handlers(1).Position(j) + sign * 0.1;
                                app.Robot_targets(i).handlers(2).Position(j) = ...
                                    app.Robot_targets(i).handlers(2).Position(j) + sign * 0.1;
                                % Update position
                                app.Robot_targets(i).position(j) = app.Robot_targets(i).handlers(1).Position(j);
                            end
    %                         disp('to');
    %                         disp(app.Robot_targets(i).nextPos);
                        end
                    end 
                end
                
                % move object
                for i=1:app.Num_obj
                    if ~(norm(app.Object(i).position - app.Object(i).nextPos) < accuracy)
                        isMoveFinished = false;
%                         disp('Object move: from');
%                         disp(app.Object(i).position);
                        for j=1:2
                            sign = app.isASmallerThanB(app.Object(i).position(j), ...
                                app.Object(i).nextPos(j));
                            
                            app.Object(i).handlers(1).Position(j) = ...
                                app.Object(i).handlers(1).Position(j) + sign * 0.1;
                            app.Object(i).handlers(2).Position(j) = ...
                                app.Object(i).handlers(2).Position(j) + sign * 0.1;
                            % Update position
                            app.Object(i).position(j) = app.Object(i).handlers(1).Position(j);
                        end
%                         disp('to');
%                         disp(app.Object(i).nextPos);
                    end
                end
                
                % move object target
                for i=1:app.Num_obj_target
                    if app.CTarget_created
                        if ~isempty([app.Object_targets(i).nextPos])
                            if ~(norm(app.Object_targets(i).position - app.Object_targets(i).nextPos) < accuracy)
                                isMoveFinished = false;
    %                             disp('Object target move: from');
    %                             disp(app.Object_targets(i).position);
                                for j=1:2
                                    sign = app.isASmallerThanB(app.Object_targets(i).position(j), ...
                                        app.Object_targets(i).nextPos(j));
                                    
                                    app.Object_targets(i).handlers(1).Position(j) = ...
                                        app.Object_targets(i).handlers(1).Position(j) + sign * 0.1;
                                    app.Object_targets(i).handlers(2).Position(j) = ...
                                        app.Object_targets(i).handlers(2).Position(j) + sign * 0.1;
                                    % Update position
                                    app.Object_targets(i).position(j) = app.Object_targets(i).handlers(1).Position(j);
                                end
    %                             disp('to');
    %                             disp(app.Object_targets(i).nextPos);
                            end
                        end
                    end
%                         pause(1/app.V_move);
                end
                
                    %% TODO: Move path(?) %%
                    
                    
                pause(1/app.V_move);
%                 end
            end
%             if all(all(abs(item_lst - item_lst_next) < 0.01))
%                 success = true;
%             else
%                 success = false;
%             end
            success = true;
        end

        function displayLegend(app)
            
%             imagesc(app.UIAxesLegend, app.Robot_fig, "XData", [0.5, 2.5], "YData", [19.5, 17.5]);
%             app.UIAxesLegend.YDir = "normal";
            % Show text
            legend_text = ["Robot", "Object", "Robot/Object final target", ...
                "Robot/Object current target",  "(before assignment)", ...
                "Robot/Object current target", "(after assignment)", "Path"];
            l = length(legend_text);
            for i = 1:l
                text(app.UIAxesLegend, 3.5, 20.5-2*i, legend_text(i), ...
                    "HorizontalAlignment","left", "FontSize", 14);
            end
            
            % Robot
%             axis(app.UIAxesLegend, [0, 1, 18, 19]);
%             app.drawItem(app.UIAxesLegend, "Robot", [1, 18]);
%             axis(app.UIAxesLegend, [0, 20, 0, 20]);
            

%             set(gcf, "Position", [1, 18, 1, 1]);
%             disp(get(gcf));
            app.drawItem(app.UIAxesLegend, "Robot", [1, 18]);
%             rectangle(app.UIAxesLegend,"EdgeColor",app.Color_list(1),...
%                 "FaceColor","none", "Position", [1, 18, 1, 1], "LineWidth",1);
            
            % Object
            app.drawItem(app.UIAxesLegend, "Object", [1, 16]);
%             rectangle(app.UIAxesLegend,"EdgeColor",app.Color_list(2),...
%                 "FaceColor",app.Color_list(2), "Position", [1, 16, 1, 1]);
            
            % Final target
            app.drawItem(app.UIAxesLegend, "Robot_final_target", [0.5, 14]);
            app.drawItem(app.UIAxesLegend, "Object_final_target", [2, 14]);
%             rectangle(app.UIAxesLegend, "FaceColor", "none", "EdgeColor", app.Color_list(1), ...
%                 "Position", [0.5, 14, 1, 1], "LineWidth", 1);
%             rectangle(app.UIAxesLegend, "FaceColor", "none", "EdgeColor", app.Color_list(2), ...
%                 "Position", [2, 14, 1, 1], "LineWidth", 1);
            
            %% TODO: Current target before assignment
%             rectangle(app.UIAxesLegend,"EdgeColor",app.Color_list(1),...
%                 "LineStyle", "--",...
%                 "FaceColor",app.Color_list(1), "Position", [1, 12, 1, 1]);        
%             for i = 1:2
%                 dohatch(app.UIAxesLegend, [0.5, 0.5, 1.5, 1.5]+1.5*(i-1),...
%                     [14, 15, 15, 14], 30, app.Color_list(i), "-", 4, 1);
%             end

            app.drawItem(app.UIAxesLegend, "Robot_target", [0.5, 11]);
            app.drawItem(app.UIAxesLegend, "Object_target", [2, 11]);
            
            % Current target after assignment
            for i = 0:1
               rectangle(app.UIAxesLegend,"FaceColor", app.Color_list(i+1), ...
                   "EdgeColor",app.Color_list(i+1),"Curvature",1,...
                   "Position",[0.5+1.5*i, 7, 1, 1]);
            end
            
            % Path
            line(app.UIAxesLegend, [0.5, 3], [4.5, 4.5], "LineWidth", 1, "Color", app.Color_list(4));

            % coordinate system
            axis(app.UIAxesLegend, [0, 20, 0, 20]);
%             set(app.UIAxesLegend, 'XTick', 0:app.Map_x, ...
%                 'YTick', 0:app.Map_y, ...
%                 'XTickMode', 'manual', 'YTickMode', 'manual');
        end
        
        
        function sign = isASmallerThanB(~, a, b)
            if abs(a - b) < 0.001
                sign = 0;
            elseif a < b
                sign = 1;
            else
                sign = -1;
            end
%             disp(['a:',num2str(a),'   b:',num2str(b),'   sign:',num2str(sign)])
        end
        
        function h = drawItem(app, axis, type, pos, id)
            % ÿÿÿÿÿÿÿÿÿÿÿÿaxisÿÿÿÿÿÿÿÿÿÿRobot/Object/Targetÿ
            % ÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿÿhandler
            text_c = 'k';
            if axis == app.UIAxes
                rob_x = app.Robot_size(1);
                rob_y = app.Robot_size(2);
            elseif axis == app.UIAxesLegend
                rob_x = 1;
                rob_y = 1;
            else
                disp("Wrong input of axis.");
            end
            switch(type)
                case "Robot"
%                     item_h = rectangle(axis, "EdgeColor","none", ...
%                             "LineWidth", 0.5, ...
%                             "FaceColor",app.Color_list(1), ...
%                             "Position",[pos, rob_x, rob_y]);
                    if axis == app.UIAxes                    
                        d = 0.1;
                    else
                        d = -0.5;
                    end
                    item_h = imagesc(axis, app.Robot_fig, "XData", [pos(1)+d, pos(1)+rob_x-d], ...
                        "YData", [pos(2)+rob_y-d, pos(2)+d]);
                        
                    % Display robot ID
                    delta_pos_text = app.Robot_size / 2;
                    text_c = 'w';
                    
                case "Object"
                    item_h = rectangle(axis, "EdgeColor","none",...
                            "LineWidth", 0.5,...
                            "FaceColor",app.Color_list(2), ...
                            "Position",[pos, 1, 1]);
                    % Display object ID
                    delta_pos_text = [0.5 0.5];
                    
                case "Robot_target"
%                     X = [pos(1), pos(1), pos(1)+rob_x, pos(1)+rob_x];
%                     Y = [pos(2), pos(2)+rob_y, pos(2)+rob_y, pos(2)];
%                     v = [X(1), Y(1); X(2), Y(2); X(3), Y(3); X(4), Y(4)];
%                     f = [1 2 3 4];
%                     item_h = patch(axis, 'Faces', f, 'Vertices', v, ...
%                         'FaceColor', app.Color_list(1), ...
%                         "EdgeColor","none",...
%                         'FaceAlpha', app.Target_alpha);
                    
                    item_h = rectangle(axis, "FaceColor", app.Color_list(1), ...
                        "EdgeColor", "none", ...
                        "Position", [pos, rob_x, rob_y]);
                    % item_h.FaceColor = [app.Color_list(1), app.Target_alpha];
                    item_h.FaceColor = app.Color_list(1);
                    % Display target ID
                    delta_pos_text = app.Robot_size / 2;
                    
                case "Object_target"
%                     X = [pos(1), pos(1), pos(1)+1, pos(1)+1];
%                     Y = [pos(2), pos(2)+1, pos(2)+1, pos(2)];
%                     v = [X(1), Y(1); X(2), Y(2); X(3), Y(3); X(4), Y(4)];
%                     f = [1 2 3 4];
%                     item_h = patch(axis, 'Faces', f, 'Vertices', v, ...
%                         'FaceColor', app.Color_list(2), ...
%                         "EdgeColor","none",...
%                         'FaceAlpha', app.Target_alpha);
                    
                    item_h = rectangle(axis, "FaceColor", app.Color_list(2), ...
                        "EdgeColor", "none", ...
                        "Position", [pos, 1, 1]);
                    % item_h.FaceColor = [app.Color_list(2), app.Target_alpha];
                    item_h.FaceColor = app.Color_list(2);
                    % Display target ID
                    delta_pos_text = [0.5 0.5];
                    
                case "Robot_final_target"
                    item_h = rectangle(axis, "FaceColor", "none", ...
                        "EdgeColor", app.Color_list(1), ...
                        "LineWidth", 1, ...
                        "Position", [pos, rob_x, rob_y]);
                    delta_pos_text = app.Robot_size / 2;
                    
                case "Object_final_target"
                    item_h = rectangle(axis, "FaceColor", "none", ...
                        "EdgeColor", app.Color_list(2), ...
                        "LineWidth", 1, ...
                        "Position", [pos, 1, 1]);
                    delta_pos_text = [0.5 0.5];
                    
                case "Obstacle"
                    item_h = rectangle(axis, "FaceColor", app.Color_list(3), ...
                        "EdgeColor", app.Color_list(3), ...
                        "LineWidth", 1, ...
                        "Position", [pos, 1, 1]);
                    
                case "Path"
                    if axis == app.UIAxes
                       % draw path according to the algorithm 
                    elseif axis == app.UIAxesLegend
                        % draw a representive line
                    else
                        disp("Wrong input of axis.");
                    end
                otherwise
                    disp("Wrong input of type for drawItem request.");
            end
            
            % display text if there is a input of text and return the
            % handler
            if nargin == 5
                text_h = text(axis, pos(1)+delta_pos_text(1), pos(2)+delta_pos_text(2), int2str(id), ...
                    "HorizontalAlignment","center", ...
                    "Color", text_c);
                h = [item_h, text_h];
            elseif nargin == 4
                h = item_h;
            else
                disp("Wrong number of input parameters.");
            end
        end
        
%         function updataItem(app, type, pos, id)
%             switch(type)
%                 case "Robot"
%                     
%                 case "Object"
%                     
%                 case "Robot_target"
%                     
%                 case "Object_target"
%                     
%                 case "Path"
%                     
%                 otherwise
%                     disp("Wrong input of type for updataItem request.");
%             end
%         end
        
        function success = doTargetAssignment(app)
            %% TODO: Check whether the target label has been assgined.
            % If not, print a message and return false
%             if app.Robot_targets

            % Change target shape + update the label 
            % to corresponding robot/object labels
            assign_step = 10;  % number of steps
            time = 1;  % time required for the assignment process (s)
            for j = 1:assign_step
                % Robot
                for i = 1:app.Num_robot
                % Robot_targets(i).assignLabel
                    app.Robot_targets(i).handlers(1).Curvature = ...
                        app.Robot_targets(i).handlers(1).Curvature + 1/assign_step;
                    % int to string?
                    app.Robot_targets(i).handlers(2).Text = app.Robot_targets(i).assignedLabel;
                end
                
                % Object
                for i = 1:app.Num_obj
                % Robot_targets(i).assignLabel
                    app.Object_targets(i).handlers(1).Curvature = ...
                        app.Object_targets(i).handlers(1).Curvature + 0.1;
                    % int to string?
                    app.Object_targets(i).handlers(2).Text = app.Object_targets(i).assignedLabel;
                end
                
                pause(time/assign_step);
            end
            
            success = true;
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: PauseButton
        function PauseButtonPushed(app, event)
%             hold(app.UIAxesLegend)
%             app.displayLegend();
%             imagesc(app.UIAxes, app.Robot_fig, "XData", [1 2], "YData", [18 19]);

            if app.PauseButton.Text == "Pause"
                app.Stop_event = true;
                app.PauseButton.Text = "Resume";
                app.PauseButton.BackgroundColor = [0.8, 0.8, 0.8];
            elseif app.PauseButton.Text == "Resume"
                app.Stop_event = false;
                app.PauseButton.Text = "Pause";
                app.PauseButton.BackgroundColor = [1.0, 1.0, 1.0];
            end
        end

        % Value changed function: RobotSwitch
        function RobotSwitchValueChanged(app, event)
            value = app.RobotSwitch.Value;
            if value == "On"
                % robot(s) set to visible
                for i = 1:app.Num_robot
                    set(app.Robot(i).handlers, 'Visible', 'on');
                end
            else
                % value == "Off", robot(s) set to invisible
                for i = 1:app.Num_robot
                    set(app.Robot(i).handlers, 'Visible', 'off');
                end
            end
        end

        % Value changed function: ObjectSwitch
        function ObjectSwitchValueChanged(app, event)
            value = app.ObjectSwitch.Value;
            if value == "On"
                % object(s) set to visible
                for i = 1:app.Num_obj
                    set(app.Object(i).handlers, 'visible', 'on');
                end
            else
                % value == "Off", object(s) set to invisible
                for i = 1:app.Num_obj
                    set(app.Object(i).handlers, 'visible', 'off');
                end
            end
        end

        % Value changed function: ObstacleSwitch
        function ObstacleSwitchValueChanged(app, event)
            value = app.ObstacleSwitch.Value;
            
            if value == "On"
                % obstacle sets to visible
                set(app.Obstacle_h, 'visible', 'on');
            else
                % obstacle sets to invisble
                set(app.Obstacle_h, 'visible', 'off');
            end
        end

        % Value changed function: CTargetSwitch
        function CTargetSwitchValueChanged(app, event)
            value = app.CTargetSwitch.Value;
            if value == "On"
                disp("On");
                % set visible on
                for i = 1:app.Num_robot
                    set(app.Robot_targets(i).handlers, 'visible', 'on');
                end
                
                for i = 1:app.Num_obj
                    set(app.Object_targets(i).handlers, 'visible', 'on');
                end
                
            elseif value == "Off"
                disp("Off");
                % set visible off
                for i = 1:app.Num_robot
                    set(app.Robot_targets(i).handlers, 'visible', 'off');
                end
                
                for i = 1:app.Num_obj
                    set(app.Object_targets(i).handlers, 'visible', 'off');
                end
                
            end
        end

        % Value changed function: FTargetSwitch
        function FTargetSwitchValueChanged(app, event)
            value = app.FTargetSwitch.Value;
            
            if value == "On"
                % final target sets to visible
                set(app.Robot_final_target_h, 'visible', 'on');
                set(app.Obj_final_target_h, 'visible', 'on');
            else
                % final target sets to invisble
                set(app.Robot_final_target_h, 'visible', 'off');
                set(app.Obj_final_target_h, 'visible', 'off');
            end
        end

        % Value changed function: PathSwitch
        function PathSwitchValueChanged(app, event)
            value = app.PathSwitch.Value;
            if value == "On"
                % path(s) set to visible
%                 set(app.GridSwitch, 'BackgroundColor', 'g');
                disp("Path on");
                
            else
                % value == "Off", path(s) set to invisible
                
            end
        end

        % Value changed function: GridSwitch
        function GridSwitchValueChanged(app, event)
            value = app.GridSwitch.Value;
            if value == "On"
                % grid set to visible
                disp("Grid on");
                app.UIAxes.GridLineStyle = "-";
            elseif value == "Off"
                % value == "Off", grid set to invisible
                disp("Grid off");
                app.UIAxes.GridLineStyle = "none";
            end
        end

        % Callback function
        function AdjustthemovingvelocityofitemsSliderValueChanged(app, event)
            value = app.AdjustthemovingvelocityoftheitemsSlider.Value;
            app.V_move = int32(value * 10);
        end

        % Value changing function: 
        % AdjustthemovingvelocityoftheitemsSlider
        function AdjustthemovingvelocityoftheitemsSliderValueChanging(app, event)
            changingValue = event.Value;
            app.V_move = int32(changingValue * 10);
        end

        % Value changed function: assigntargetSwitch
        function assigntargetSwitchValueChanged(app, event)
            value = app.assigntargetSwitch.Value;
            if value == "On"
                app.doTargetAssignment();
            end
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 2x1 grid
                app.GridLayout.RowHeight = {669, 669};
                app.GridLayout.ColumnWidth = {'1x'};
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 1;
            else
                % Change to a 1x2 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {267, '1x'};
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 2;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 1039 669];
            app.UIFigure.Name = 'UI Figure';
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {267, '1x'};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create MapInformationLabel
            app.MapInformationLabel = uilabel(app.LeftPanel);
            app.MapInformationLabel.FontSize = 16;
            app.MapInformationLabel.FontWeight = 'bold';
            app.MapInformationLabel.Position = [69 180 130 22];
            app.MapInformationLabel.Text = 'Map Information';

            % Create NumberofrobotsLabel
            app.NumberofrobotsLabel = uilabel(app.LeftPanel);
            app.NumberofrobotsLabel.FontSize = 14;
            app.NumberofrobotsLabel.Position = [34 140 154 22];
            app.NumberofrobotsLabel.Text = 'Number of robot(s): ';

            % Create NumberofobjectsLabel
            app.NumberofobjectsLabel = uilabel(app.LeftPanel);
            app.NumberofobjectsLabel.FontSize = 14;
            app.NumberofobjectsLabel.Position = [34 100 154 22];
            app.NumberofobjectsLabel.Text = 'Number of object(s): ';

            % Create PauseButton
            app.PauseButton = uibutton(app.LeftPanel, 'push');
            app.PauseButton.ButtonPushedFcn = createCallbackFcn(app, @PauseButtonPushed, true);
            app.PauseButton.BackgroundColor = [1 1 1];
            app.PauseButton.FontSize = 20;
            app.PauseButton.Position = [69 583 130 62];
            app.PauseButton.Text = 'Pause';

            % Create StepsLabel
            app.StepsLabel = uilabel(app.LeftPanel);
            app.StepsLabel.FontSize = 14;
            app.StepsLabel.Position = [34 20 120 28];
            app.StepsLabel.Text = 'Step(s): ';

            % Create ItemstodisplayLabel
            app.ItemstodisplayLabel = uilabel(app.LeftPanel);
            app.ItemstodisplayLabel.FontSize = 16;
            app.ItemstodisplayLabel.FontWeight = 'bold';
            app.ItemstodisplayLabel.Position = [71 526 137 22];
            app.ItemstodisplayLabel.Text = 'Item(s) to display';

            % Create PathSwitch
            app.PathSwitch = uiswitch(app.LeftPanel, 'slider');
            app.PathSwitch.ValueChangedFcn = createCallbackFcn(app, @PathSwitchValueChanged, true);
            app.PathSwitch.FontSize = 1;
            app.PathSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.PathSwitch.Position = [34 284 45 20];
            app.PathSwitch.Value = 'On';

            % Create DisplaypathLabel_2
            app.DisplaypathLabel_2 = uilabel(app.LeftPanel);
            app.DisplaypathLabel_2.FontSize = 14;
            app.DisplaypathLabel_2.Position = [98 284 83 22];
            app.DisplaypathLabel_2.Text = 'Display path';

            % Create CTargetSwitch
            app.CTargetSwitch = uiswitch(app.LeftPanel, 'slider');
            app.CTargetSwitch.ValueChangedFcn = createCallbackFcn(app, @CTargetSwitchValueChanged, true);
            app.CTargetSwitch.FontSize = 1;
            app.CTargetSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.CTargetSwitch.Position = [34 364 45 20];
            app.CTargetSwitch.Value = 'On';

            % Create DisplaycurrenttargetLabel
            app.DisplaycurrenttargetLabel = uilabel(app.LeftPanel);
            app.DisplaycurrenttargetLabel.FontSize = 14;
            app.DisplaycurrenttargetLabel.Position = [98 364 155 22];
            app.DisplaycurrenttargetLabel.Text = 'Display current target';

            % Create ObjectSwitch
            app.ObjectSwitch = uiswitch(app.LeftPanel, 'slider');
            app.ObjectSwitch.ValueChangedFcn = createCallbackFcn(app, @ObjectSwitchValueChanged, true);
            app.ObjectSwitch.FontSize = 1;
            app.ObjectSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.ObjectSwitch.Position = [34 444 45 20];
            app.ObjectSwitch.Value = 'On';

            % Create DisplayobjectLabel
            app.DisplayobjectLabel = uilabel(app.LeftPanel);
            app.DisplayobjectLabel.FontSize = 14;
            app.DisplayobjectLabel.Position = [98 444 109 22];
            app.DisplayobjectLabel.Text = 'Display object';

            % Create RobotSwitch
            app.RobotSwitch = uiswitch(app.LeftPanel, 'slider');
            app.RobotSwitch.ValueChangedFcn = createCallbackFcn(app, @RobotSwitchValueChanged, true);
            app.RobotSwitch.FontSize = 1;
            app.RobotSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.RobotSwitch.Position = [34 484 45 20];
            app.RobotSwitch.Value = 'On';

            % Create DisplayrobotLabel
            app.DisplayrobotLabel = uilabel(app.LeftPanel);
            app.DisplayrobotLabel.FontSize = 14;
            app.DisplayrobotLabel.Position = [98 484 103 22];
            app.DisplayrobotLabel.Text = 'Display robot';

            % Create GridSwitch
            app.GridSwitch = uiswitch(app.LeftPanel, 'slider');
            app.GridSwitch.ValueChangedFcn = createCallbackFcn(app, @GridSwitchValueChanged, true);
            app.GridSwitch.FontSize = 1;
            app.GridSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.GridSwitch.Position = [34 244 45 20];
            app.GridSwitch.Value = 'On';

            % Create DisplaygridLabel
            app.DisplaygridLabel = uilabel(app.LeftPanel);
            app.DisplaygridLabel.FontSize = 14;
            app.DisplaygridLabel.Position = [98 244 99 22];
            app.DisplaygridLabel.Text = 'Display grid';

            % Create FTargetSwitch
            app.FTargetSwitch = uiswitch(app.LeftPanel, 'slider');
            app.FTargetSwitch.ValueChangedFcn = createCallbackFcn(app, @FTargetSwitchValueChanged, true);
            app.FTargetSwitch.FontSize = 1;
            app.FTargetSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.FTargetSwitch.Position = [34 324 45 20];
            app.FTargetSwitch.Value = 'On';

            % Create DisplayfinaltargetLabel
            app.DisplayfinaltargetLabel = uilabel(app.LeftPanel);
            app.DisplayfinaltargetLabel.FontSize = 14;
            app.DisplayfinaltargetLabel.Position = [98 324 120 22];
            app.DisplayfinaltargetLabel.Text = 'Display final target';

            % Create ObstacleSwitch
            app.ObstacleSwitch = uiswitch(app.LeftPanel, 'slider');
            app.ObstacleSwitch.ValueChangedFcn = createCallbackFcn(app, @ObstacleSwitchValueChanged, true);
            app.ObstacleSwitch.FontSize = 1;
            app.ObstacleSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.ObstacleSwitch.Position = [34 404 45 20];
            app.ObstacleSwitch.Value = 'On';

            % Create DisplayobstacleLabel
            app.DisplayobstacleLabel = uilabel(app.LeftPanel);
            app.DisplayobstacleLabel.FontSize = 14;
            app.DisplayobstacleLabel.Position = [98 404 107 22];
            app.DisplayobstacleLabel.Text = 'Display obstacle';

            % Create NumberofobstaclesLabel
            app.NumberofobstaclesLabel = uilabel(app.LeftPanel);
            app.NumberofobstaclesLabel.FontSize = 14;
            app.NumberofobstaclesLabel.Position = [34 60 174 22];
            app.NumberofobstaclesLabel.Text = 'Number of obstacle(s): ';

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create UIAxes
            app.UIAxes = uiaxes(app.RightPanel);
            title(app.UIAxes, '')
            xlabel(app.UIAxes, '')
            ylabel(app.UIAxes, '')
            app.UIAxes.PlotBoxAspectRatio = [1.01748251748252 1 1];
            app.UIAxes.TickLength = [0 0];
            app.UIAxes.GridColor = [0.902 0.902 0.902];
            app.UIAxes.GridAlpha = 1;
            app.UIAxes.Box = 'on';
            app.UIAxes.XAxisLocation = 'origin';
            app.UIAxes.XTick = [0 1];
            app.UIAxes.YTick = [0 1];
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.TitleFontWeight = 'bold';
            app.UIAxes.Position = [5 145 500 500];

            % Create UIAxesLegend
            app.UIAxesLegend = uiaxes(app.RightPanel);
            title(app.UIAxesLegend, '')
            xlabel(app.UIAxesLegend, '')
            ylabel(app.UIAxesLegend, '')
            app.UIAxesLegend.DataAspectRatio = [1 1 1];
            app.UIAxesLegend.PlotBoxAspectRatio = [1 1 1];
            app.UIAxesLegend.FontSize = 1;
            app.UIAxesLegend.TickLength = [0 0];
            app.UIAxesLegend.XColor = [0.9412 0.9412 0.9412];
            app.UIAxesLegend.YAxisLocation = 'right';
            app.UIAxesLegend.YColor = [0.9412 0.9412 0.9412];
            app.UIAxesLegend.TitleFontWeight = 'bold';
            app.UIAxesLegend.Position = [504 403 243 242];

            % Create AdjustthemovingvelocityoftheitemsSliderLabel
            app.AdjustthemovingvelocityoftheitemsSliderLabel = uilabel(app.RightPanel);
            app.AdjustthemovingvelocityoftheitemsSliderLabel.HorizontalAlignment = 'right';
            app.AdjustthemovingvelocityoftheitemsSliderLabel.FontSize = 16;
            app.AdjustthemovingvelocityoftheitemsSliderLabel.FontWeight = 'bold';
            app.AdjustthemovingvelocityoftheitemsSliderLabel.Position = [101 108 307 22];
            app.AdjustthemovingvelocityoftheitemsSliderLabel.Text = 'Adjust the moving velocity of the items';

            % Create AdjustthemovingvelocityoftheitemsSlider
            app.AdjustthemovingvelocityoftheitemsSlider = uislider(app.RightPanel);
            app.AdjustthemovingvelocityoftheitemsSlider.Limits = [1 10];
            app.AdjustthemovingvelocityoftheitemsSlider.MajorTicks = [1 2 3 4 5 6 7 8 9 10];
            app.AdjustthemovingvelocityoftheitemsSlider.ValueChangingFcn = createCallbackFcn(app, @AdjustthemovingvelocityoftheitemsSliderValueChanging, true);
            app.AdjustthemovingvelocityoftheitemsSlider.Position = [126 87 256 3];
            app.AdjustthemovingvelocityoftheitemsSlider.Value = 1;

            % Create assigntargetSwitchLabel
            app.assigntargetSwitchLabel = uilabel(app.RightPanel);
            app.assigntargetSwitchLabel.HorizontalAlignment = 'center';
            app.assigntargetSwitchLabel.Position = [613.5 266 74 22];
            app.assigntargetSwitchLabel.Text = 'assign target';

            % Create assigntargetSwitch
            app.assigntargetSwitch = uiswitch(app.RightPanel, 'slider');
            app.assigntargetSwitch.ValueChangedFcn = createCallbackFcn(app, @assigntargetSwitchValueChanged, true);
            app.assigntargetSwitch.Position = [627 303 45 20];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = GUI2D_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end