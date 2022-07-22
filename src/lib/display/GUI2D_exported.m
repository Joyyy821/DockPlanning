classdef GUI2D_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        GridLayout                 matlab.ui.container.GridLayout
        LeftPanel                  matlab.ui.container.Panel
        DisplayfinaltargetLabel    matlab.ui.control.Label
        FTargetSwitch              matlab.ui.control.Switch
        DisplaygridLabel           matlab.ui.control.Label
        GridSwitch                 matlab.ui.control.Switch
        DisplayrobotLabel          matlab.ui.control.Label
        RobotSwitch                matlab.ui.control.Switch
        DisplaycurrenttargetLabel  matlab.ui.control.Label
        CTargetSwitch              matlab.ui.control.Switch
        DisplaypathLabel_2         matlab.ui.control.Label
        PathSwitch                 matlab.ui.control.Switch
        ItemstodisplayLabel        matlab.ui.control.Label
        StepsLabel                 matlab.ui.control.Label
        PauseButton                matlab.ui.control.Button
        NumberofrobotsLabel        matlab.ui.control.Label
        MapInformationLabel        matlab.ui.control.Label
        RightPanel                 matlab.ui.container.Panel
        assigntargetSwitch         matlab.ui.control.Switch
        assigntargetSwitchLabel    matlab.ui.control.Label
        AdjustthemovingvelocityoftheitemsSlider  matlab.ui.control.Slider
        AdjustthemovingvelocityoftheitemsSliderLabel  matlab.ui.control.Label
        UIAxesLegend               matlab.ui.control.UIAxes
        UIAxes                     matlab.ui.control.UIAxes
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
                       'dockers', {}, ...  % Dock joints [up, down, left, right]
                       'dockHandler', {}, ...  % Handler for the dock joints.
                       'target', {}, ... % Label of the current target
                       ...'targetPosition', {}, ... % Position of the current target
                       'path', {},... % Planed path for the items to move
                       ...'pathL', {}, ... % Path length
                       'pathHandler', {} ... % Handler of the path line (1D)
                       )
        
        targets = struct(... Current target
                         'assignedLabel', {}, ... Label after assignment
                         'position', {}, ...
                         'nextPos', {}, ...
                         'handlers', {} ...
                         )
        
        % Static items
        final_target = []  % Final target of object(s)
        final_target_h = []  % Handler of object final target (2D, w label)
        
        % Representation of items
        Color_list = ["#0072BD", "#D95319", 'w', '#4DBEEE'] % Color used for display
        % (1) Robot (2) Path (3) Text (white) (4) Target
        Path_shape = '-'
        Path_width = 1
        
        % Map information
        Num_robot = 0  % Number of robot(s)
        Num_target = 0
        step = 0  % Number of step(s)
        
        % Controlling parameters
        V_move = 10  % The velocity for moving items
        Stop_event = false  % Whether the moving process has been paused.
        CTarget_created = false  % Whether the current targets have been decided.
        Target_assigned = false  % Whether the current targets have been assigned to robots or objects.
    end
    
    properties (Access = public)
        updateTargetID = false   % whether to update target id when calling the show method.
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
            Robot_dock = evalin('base', 'Robot_Dock');
            pathL = evalin('base', 'Robot_PathL');
            paths = evalin('base', 'Robot_Path');
            [app.Num_robot, ~] = size(Robot_pos);
            for i = 1:app.Num_robot
                app.Robot(i).position = Robot_pos(i, :);
                app.Robot(i).nextPos = app.Robot(i).position;
                app.Robot(i).dockers = Robot_dock(i, :);
                p_idx = sum(pathL(1:i-1));
                app.Robot(i).path = paths(p_idx+1:p_idx+pathL(i), :);
                disp("p_idx: "+string(p_idx));
                disp("app robbot path: ");
                disp(app.Robot(i).path);
            end
            disp('Robot position(s), dock sites, and path(s) have been loaded.');
                        
            % final target
            app.final_target = evalin('base', 'Final_Target');
            disp('Final target(s) has/have been loaded.');
            [app.Num_target, ~] = size(app.final_target);
%             disp(app.Num_obj_target);
            
            % current target
            target = evalin('base', 'Current_Target');
            target_id = evalin('base', 'Target_ID');

            for i = 1:length(target)
                app.targets(i).position = target(i, :);
                app.targets(i).nextPos = app.targets(i).position;
                app.targets(i).assignedLabel = target_id(i);
            end

            if isempty(target)
                for i = 1:app.Num_target
                    app.targets(i).position = app.final_target(i, :);
                end
                app.CTarget_created = false;
            else
                app.CTarget_created = true;
            end
            disp('Current target(s) has/have been loaded.');

            % Update map info
            app.NumberofrobotsLabel.Text = "Number of robot(s): " + int2str(app.Num_robot);
            app.StepsLabel.Text = "Step(s): " + int2str(app.step);
        end
        
        function updateData(app)
            % next position
            Robot_pos_next = evalin('base', 'Robot_Current_Position');
            pathL = evalin('base', 'Robot_PathL');
            paths = evalin('base', 'Robot_Path');
            target_next = evalin('base', 'Current_Target');
            target_id_next = evalin('base', 'Target_ID');
            
            for i = 1:app.Num_robot
                app.Robot(i).nextPos = Robot_pos_next(i, :);
                p_idx = sum(pathL(1:i-1));
                app.Robot(i).path = paths(p_idx+1:p_idx+pathL(i), :);
            end
            if ~isempty(target_next)
                for i=1:app.Num_target
                    app.targets(i).nextPos = target_next(i, :);
                    app.CTarget_created = true;
%                     if isempty(app.Object_targets(i).position)
%                         app.Object_targets(i).position = app.Obj_final_target(i, :);
%                     end
                end
%             else
%                 app.Object_targets(i).nextPos = app.Object_targets(i).position;
            end

            if ~isempty(target_id_next)
                for i=1:app.Num_target
                    app.targets(i).assignedLabel = target_id_next(i);
                end
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
            for i=1:app.Num_target
                % Display current object target
                obj_target_pos = [app.targets(i).position(1), app.targets(i).position(2)];
                app.targets(i).handlers = app.drawItem(app.UIAxes, "target", ...
                    obj_target_pos, i);
                if ~app.CTarget_created
                    set(app.targets(i).handlers, 'visible', 'off');
                
                end
                % Display final object target
                obj_h = app.drawItem(app.UIAxes, "final_target", ...
                    app.final_target(i, :));
                app.final_target_h = [app.final_target_h; obj_h];
                
            end
            
            % Display current robot
            for i=1:app.Num_robot
                % draw robot with labeled id.
                robot_pos = [app.Robot(i).position(1), app.Robot(i).position(2)];
                app.Robot(i).handlers = app.drawItem(app.UIAxes, "Robot", robot_pos, i);
                app.Robot(i).pathHandler = app.drawItem(app.UIAxes, "Path", robot_pos, i);
            end
            
                
%             uistack(app.Robot.handlers, "top");
            % coordinate system
            axis(app.UIAxes, [1, app.Map_x+1, 1, app.Map_y+1]);
            set(app.UIAxes, 'XTick', 1:app.Map_x, ...
                'YTick', 1:app.Map_y, ...
                'XTickMode', 'manual', 'YTickMode', 'manual');
        end
        
        % move all Items to the current positions
        function success = moveItems(app)
            accuracy = 0.001;
            isMoveFinished = false;

            % update target id
            if app.updateTargetID
                for i=1:app.Num_target
                    app.targets(i).handlers(2).String = app.targets(i).assignedLabel;
                    set(app.targets(i).handlers(1), 'Visible', 'on');
                    set(app.targets(i).handlers(2), 'Visible', 'on');
                end
            end
            % Draw the new path
            for i=1:app.Num_robot
                if ~isempty(app.Robot(i).path)
                    app.Robot(i).pathHandler.XData = ...
                        [app.Robot(i).position(1); app.Robot(i).path(:, 1)]+0.5;
                    app.Robot(i).pathHandler.YData = ...
                        [app.Robot(i).position(2); app.Robot(i).path(:, 2)]+0.5;
                end
            end
            
            while ~(app.Stop_event || isMoveFinished)
%                 disp('in loop')

                % stopping criterion
                isMoveFinished = true;
                    % move robot
                for i=1:app.Num_robot
                    if ~(norm(app.Robot(i).position - app.Robot(i).nextPos) < accuracy)
                        isMoveFinished = false;
                        for j=1:2
                            sign = app.isASmallerThanB(app.Robot(i).position(j), ...
                                app.Robot(i).nextPos(j));
                            % move robot (rect)
                            app.Robot(i).handlers(1).Position(j) = ...
                                app.Robot(i).handlers(1).Position(j) + sign * 0.1;
                            % move path
                            if j == 1
                                app.Robot(i).pathHandler.XData(1) = ...
                                    app.Robot(i).pathHandler.XData(1) + sign * 0.1;
                            elseif j == 2
                                app.Robot(i).pathHandler.YData(1) = ...
                                    app.Robot(i).pathHandler.YData(1) + sign * 0.1;
                            end
                            % move robot (text)
                            app.Robot(i).handlers(2).Position(j) = ...
                                app.Robot(i).handlers(2).Position(j) + sign * 0.1;
                            % move docker
                            for k=1:4
                                if j == 1
                                    app.Robot(i).dockHandler(k).XData = ...
                                        app.Robot(i).dockHandler(k).XData + sign * 0.1;
                                elseif j == 2
                                    app.Robot(i).dockHandler(k).YData = ...
                                        app.Robot(i).dockHandler(k).YData + sign * 0.1;
                                end
                            end
                            % Update position
                            app.Robot(i).position(j) = app.Robot(i).handlers(2).Position(j)...
                                - app.Robot_size(j)/2;
                        end
                    end
                end

                % move target
                for i=1:app.Num_target
                    if app.CTarget_created
                        set(app.targets(i).handlers(1), 'Visible', 'on');
                        id = app.targets(i).assignedLabel;
                        if id ~= 0
                            set(app.targets(i).handlers(2), 'string', int2str(id));
                            set(app.targets(i).handlers(2), 'Visible', 'on');
                        end
                        if ~isempty([app.targets(i).nextPos])
                            pos_delta = app.targets(i).nextPos - app.targets(i).position;
                            for j=1:2
                                app.targets(i).handlers(1).Position(j) = ...
                                    app.targets(i).handlers(1).Position(j) + pos_delta(j);
                                app.targets(i).handlers(2).Position(j) = ...
                                    app.targets(i).handlers(2).Position(j) + pos_delta(j);                                
                            end
                            app.targets(i).position = app.targets(i).nextPos;
                        end
                    end
                end

%                 drawnow;
                pause(1/app.V_move);
            end
            success = true;
        end

        function displayLegend(app)
            
%             imagesc(app.UIAxesLegend, app.Robot_fig, "XData", [0.5, 2.5], "YData", [19.5, 17.5]);
%             app.UIAxesLegend.YDir = "normal";
            % Show text
%             legend_text = ["Robot", "Object", "Robot/Object final target", ...
%                 "Robot/Object current target",  "(before assignment)", ...
%                 "Robot/Object current target", "(after assignment)", "Path"];
            legend_text = ["Robot", "Final target", ...
                "Current target", "Path"];
            l = length(legend_text);
            for i = 1:l
                text(app.UIAxesLegend, 3.5, 10.5-2*i, legend_text(i), ...
                    "HorizontalAlignment","left", "FontSize", 14);
            end
            
            app.drawItem(app.UIAxesLegend, "Robot", [1, 8]);

            % Final target
            app.drawItem(app.UIAxesLegend, "final_target", [1, 6]);        
            %% TODO: Current target before assignment
            app.drawItem(app.UIAxesLegend, "target", [1, 4]);

            % Path
            line(app.UIAxesLegend, [0.5, 3], [2.5, 2.5], "LineWidth", 1, "Color", app.Color_list(2));

            % coordinate system
            axis(app.UIAxesLegend, [0, 10, 0, 10]);
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
            % 对画图命令进行封装，输入axis，左下角坐标，类型（Robot/Object/Target）
            % 用于第一次创建图形，显示相应图像，并返回图形相应的handler
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
                    % robot
                    item_h = rectangle(axis, "EdgeColor","none",...
                            "LineWidth", 0.5,...
                            "FaceColor",app.Color_list(1), ...
                            "Position",[pos, rob_x, rob_y]);
                    % dock
                    dock_x = [0, 1; 0, 1; 0, 0; 1, 1] * app.Robot_size(1);
                    dock_y = [1, 1; 0, 0; 0, 1; 0, 1] * app.Robot_size(2);
                    if axis == app.UIAxes
                        for i=1:4
                            temp_h = line(axis, pos(1)+dock_x(i,:), pos(2)+dock_y(i,:), ...
                                "LineWidth", 1, ...
                                "Color", 'r');
                            app.Robot(id).dockHandler(i) = temp_h;
                            if ~app.Robot(id).dockers(i)
                                set(app.Robot(id).dockHandler(i), 'visible', 'off');
                            end
                        end
                    end
                    % Display robot ID
                    delta_pos_text = app.Robot_size / 2;
                    text_c = 'k'; % '#7E2F8E';

                case "target"
                    item_h = rectangle(axis, "FaceColor", app.Color_list(4), ...
                        "EdgeColor", "none", ...
                        "Position", [pos, 1, 1]);
                    % item_h.FaceColor = [app.Color_list(2), app.Target_alpha];
                    item_h.FaceColor = app.Color_list(4);
                    % Display target ID
                    delta_pos_text = [0.5 0.5];
                    if nargin == 5
                        id = app.targets(id).assignedLabel;
                    end

                case "final_target"
                    item_h = rectangle(axis, "FaceColor", "none", ...
                        "EdgeColor", app.Color_list(4), ...
                        "LineWidth", 1, ...
                        "Position", [pos, 1, 1]);
                    delta_pos_text = [0.5 0.5];

                case "Path"
                    if axis == app.UIAxes
                        % draw path according to the algorithm 
                        if ~isempty(app.Robot(id).path)
                            item_h = plot(axis, app.Robot(id).path(:, 1),...
                                app.Robot(id).path(:, 2), "Color", app.Color_list(2));
                        else
                            item_h = plot(axis, pos(1)+0.5, pos(2)+0.5, "Color", app.Color_list(4));
                        end
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
            if nargin == 5 && type ~= "Path"
                text_h = text(axis, pos(1)+delta_pos_text(1), pos(2)+delta_pos_text(2), int2str(id), ...
                    "HorizontalAlignment","center", ...
                    "Color", text_c, ...
                    "FontSize", 18, ...
                    "FontWeight", "bold");
                h = [item_h, text_h];
                if id == 0
                    set(text_h, 'Visible', 'off'); % 
                end
            elseif nargin == 4 || type == "Path"
                h = item_h;
            else
                disp("Wrong number of input parameters.");
            end
        end
        
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
                    app.targets(i).handlers(1).Curvature = ...
                        app.targets(i).handlers(1).Curvature + 0.1;
                    % int to string?
                    app.targets(i).handlers(2).Text = app.targets(i).assignedLabel;
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
                    set(app.targets(i).handlers, 'visible', 'on');
                end
                
            elseif value == "Off"
                disp("Off");
                % set visible off
                for i = 1:app.Num_robot
                    set(app.Robot_targets(i).handlers, 'visible', 'off');
                end
                
                for i = 1:app.Num_obj
                    set(app.targets(i).handlers, 'visible', 'off');
                end
                
            end
        end

        % Value changed function: FTargetSwitch
        function FTargetSwitchValueChanged(app, event)
            value = app.FTargetSwitch.Value;
            
            if value == "On"
                % final target sets to visible
                set(app.Robot_final_target_h, 'visible', 'on');
                set(app.final_target_h, 'visible', 'on');
            else
                % final target sets to invisble
                set(app.Robot_final_target_h, 'visible', 'off');
                set(app.final_target_h, 'visible', 'off');
            end
        end

        % Value changed function: PathSwitch
        function PathSwitchValueChanged(app, event)
            value = app.PathSwitch.Value;
            if value == "On"
                % path(s) set to visible pathHandler
                for i=1:app.Num_robot
                    set(app.Robot(i).pathHandler, 'visible', 'on');
                end
                disp("Path on");
                
            else
                % value == "Off", path(s) set to invisible
                for i=1:app.Num_robot
                    set(app.Robot(i).pathHandler, 'visible', 'off');
                end
                disp("Path off");
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

        % Value changing function: AdjustthemovingvelocityoftheitemsSlider
        function AdjustthemovingvelocityoftheitemsSliderValueChanging(app, event)
            changingValue = event.Value;
            app.V_move = int32(changingValue * 10);
            disp("changing value: "+string(app.V_move));
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
                app.GridLayout.RowHeight = {635, 635};
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
            app.UIFigure.Position = [100 100 1039 635];
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
            app.MapInformationLabel.Position = [69 236 130 22];
            app.MapInformationLabel.Text = 'Map Information';

            % Create NumberofrobotsLabel
            app.NumberofrobotsLabel = uilabel(app.LeftPanel);
            app.NumberofrobotsLabel.FontSize = 14;
            app.NumberofrobotsLabel.Position = [34 196 154 22];
            app.NumberofrobotsLabel.Text = 'Number of robot(s): ';

            % Create PauseButton
            app.PauseButton = uibutton(app.LeftPanel, 'push');
            app.PauseButton.ButtonPushedFcn = createCallbackFcn(app, @PauseButtonPushed, true);
            app.PauseButton.BackgroundColor = [1 1 1];
            app.PauseButton.FontSize = 20;
            app.PauseButton.Position = [69 549 130 62];
            app.PauseButton.Text = 'Pause';

            % Create StepsLabel
            app.StepsLabel = uilabel(app.LeftPanel);
            app.StepsLabel.FontSize = 14;
            app.StepsLabel.Position = [34 156 120 28];
            app.StepsLabel.Text = 'Step(s): ';

            % Create ItemstodisplayLabel
            app.ItemstodisplayLabel = uilabel(app.LeftPanel);
            app.ItemstodisplayLabel.FontSize = 16;
            app.ItemstodisplayLabel.FontWeight = 'bold';
            app.ItemstodisplayLabel.Position = [71 492 137 22];
            app.ItemstodisplayLabel.Text = 'Item(s) to display';

            % Create PathSwitch
            app.PathSwitch = uiswitch(app.LeftPanel, 'slider');
            app.PathSwitch.ValueChangedFcn = createCallbackFcn(app, @PathSwitchValueChanged, true);
            app.PathSwitch.FontSize = 1;
            app.PathSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.PathSwitch.Position = [35 331 45 20];
            app.PathSwitch.Value = 'On';

            % Create DisplaypathLabel_2
            app.DisplaypathLabel_2 = uilabel(app.LeftPanel);
            app.DisplaypathLabel_2.FontSize = 14;
            app.DisplaypathLabel_2.Position = [98 331 83 22];
            app.DisplaypathLabel_2.Text = 'Display path';

            % Create CTargetSwitch
            app.CTargetSwitch = uiswitch(app.LeftPanel, 'slider');
            app.CTargetSwitch.ValueChangedFcn = createCallbackFcn(app, @CTargetSwitchValueChanged, true);
            app.CTargetSwitch.FontSize = 1;
            app.CTargetSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.CTargetSwitch.Position = [35 411 45 20];
            app.CTargetSwitch.Value = 'On';

            % Create DisplaycurrenttargetLabel
            app.DisplaycurrenttargetLabel = uilabel(app.LeftPanel);
            app.DisplaycurrenttargetLabel.FontSize = 14;
            app.DisplaycurrenttargetLabel.Position = [98 411 155 22];
            app.DisplaycurrenttargetLabel.Text = 'Display current target';

            % Create RobotSwitch
            app.RobotSwitch = uiswitch(app.LeftPanel, 'slider');
            app.RobotSwitch.ValueChangedFcn = createCallbackFcn(app, @RobotSwitchValueChanged, true);
            app.RobotSwitch.FontSize = 1;
            app.RobotSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.RobotSwitch.Position = [35 451 45 20];
            app.RobotSwitch.Value = 'On';

            % Create DisplayrobotLabel
            app.DisplayrobotLabel = uilabel(app.LeftPanel);
            app.DisplayrobotLabel.FontSize = 14;
            app.DisplayrobotLabel.Position = [98 451 103 22];
            app.DisplayrobotLabel.Text = 'Display robot';

            % Create GridSwitch
            app.GridSwitch = uiswitch(app.LeftPanel, 'slider');
            app.GridSwitch.ValueChangedFcn = createCallbackFcn(app, @GridSwitchValueChanged, true);
            app.GridSwitch.FontSize = 1;
            app.GridSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.GridSwitch.Position = [35 291 45 20];
            app.GridSwitch.Value = 'On';

            % Create DisplaygridLabel
            app.DisplaygridLabel = uilabel(app.LeftPanel);
            app.DisplaygridLabel.FontSize = 14;
            app.DisplaygridLabel.Position = [98 291 99 22];
            app.DisplaygridLabel.Text = 'Display grid';

            % Create FTargetSwitch
            app.FTargetSwitch = uiswitch(app.LeftPanel, 'slider');
            app.FTargetSwitch.ValueChangedFcn = createCallbackFcn(app, @FTargetSwitchValueChanged, true);
            app.FTargetSwitch.FontSize = 1;
            app.FTargetSwitch.FontColor = [0.9412 0.9412 0.9412];
            app.FTargetSwitch.Position = [35 371 45 20];
            app.FTargetSwitch.Value = 'On';

            % Create DisplayfinaltargetLabel
            app.DisplayfinaltargetLabel = uilabel(app.LeftPanel);
            app.DisplayfinaltargetLabel.FontSize = 14;
            app.DisplayfinaltargetLabel.Position = [98 371 120 22];
            app.DisplayfinaltargetLabel.Text = 'Display final target';

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 2;

            % Create UIAxes
            app.UIAxes = uiaxes(app.RightPanel);
            app.UIAxes.PlotBoxAspectRatio = [1.01748251748252 1 1];
            app.UIAxes.TickLength = [0 0];
            app.UIAxes.XAxisLocation = 'origin';
            app.UIAxes.XTick = [0 1];
            app.UIAxes.XTickLabelRotation = 0;
            app.UIAxes.YTick = [0 1];
            app.UIAxes.YTickLabelRotation = 0;
            app.UIAxes.ZTickLabelRotation = 0;
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.FontSize = 14;
            app.UIAxes.GridColor = [0.902 0.902 0.902];
            app.UIAxes.GridAlpha = 1;
            app.UIAxes.Box = 'on';
            app.UIAxes.Position = [5 111 518 518];

            % Create UIAxesLegend
            app.UIAxesLegend = uiaxes(app.RightPanel);
            app.UIAxesLegend.DataAspectRatio = [1 1 1];
            app.UIAxesLegend.PlotBoxAspectRatio = [1 1 1];
            app.UIAxesLegend.TickLength = [0 0];
            app.UIAxesLegend.XColor = [0.9412 0.9412 0.9412];
            app.UIAxesLegend.XTickLabelRotation = 0;
            app.UIAxesLegend.YAxisLocation = 'right';
            app.UIAxesLegend.YColor = [0.9412 0.9412 0.9412];
            app.UIAxesLegend.YTickLabelRotation = 0;
            app.UIAxesLegend.ZTickLabelRotation = 0;
            app.UIAxesLegend.FontSize = 1;
            app.UIAxesLegend.Position = [556 369 191 190];

            % Create AdjustthemovingvelocityoftheitemsSliderLabel
            app.AdjustthemovingvelocityoftheitemsSliderLabel = uilabel(app.RightPanel);
            app.AdjustthemovingvelocityoftheitemsSliderLabel.HorizontalAlignment = 'right';
            app.AdjustthemovingvelocityoftheitemsSliderLabel.FontSize = 16;
            app.AdjustthemovingvelocityoftheitemsSliderLabel.FontWeight = 'bold';
            app.AdjustthemovingvelocityoftheitemsSliderLabel.Position = [101 74 307 22];
            app.AdjustthemovingvelocityoftheitemsSliderLabel.Text = 'Adjust the moving velocity of the items';

            % Create AdjustthemovingvelocityoftheitemsSlider
            app.AdjustthemovingvelocityoftheitemsSlider = uislider(app.RightPanel);
            app.AdjustthemovingvelocityoftheitemsSlider.Limits = [1 10];
            app.AdjustthemovingvelocityoftheitemsSlider.MajorTicks = [1 2 3 4 5 6 7 8 9 10];
            app.AdjustthemovingvelocityoftheitemsSlider.ValueChangingFcn = createCallbackFcn(app, @AdjustthemovingvelocityoftheitemsSliderValueChanging, true);
            app.AdjustthemovingvelocityoftheitemsSlider.BusyAction = 'cancel';
            app.AdjustthemovingvelocityoftheitemsSlider.Position = [126 53 256 3];
            app.AdjustthemovingvelocityoftheitemsSlider.Value = 1;

            % Create assigntargetSwitchLabel
            app.assigntargetSwitchLabel = uilabel(app.RightPanel);
            app.assigntargetSwitchLabel.HorizontalAlignment = 'center';
            app.assigntargetSwitchLabel.Position = [614 232 74 22];
            app.assigntargetSwitchLabel.Text = 'assign target';

            % Create assigntargetSwitch
            app.assigntargetSwitch = uiswitch(app.RightPanel, 'slider');
            app.assigntargetSwitch.ValueChangedFcn = createCallbackFcn(app, @assigntargetSwitchValueChanged, true);
            app.assigntargetSwitch.Position = [627 269 45 20];

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