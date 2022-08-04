classdef Logging < handle
    %LOGGING Record the simulation procedure
    %   此处显示详细说明
    
    % TODO: (1) record1step: write mat and write exp txt
    %           mat: robot initial pose + dock change + target expansion +
    %           robot position and path update
    %           txt: position (include initial rotation), happended dock
    %       (2) update the UI (a) for heterogenous dock joints (different
    %           color to distinguish (b) current target update (c) in the
    %           display2D class, set an option to indicate whether to use 
    %           to GUI or only run the simulation without display.
    %       (3) write a wrapped file to initial several simulations with
    %           one click (an episode).
    
    properties
        filemat       string
        filetxt       string
        % For hardware implementation
        recordType    logical
        filerobots    string
        frids
        rotations     double
    end
    
    methods
        function obj = Logging(options)
            %LOGGING Record the simulation
            %   Optional input arguments:
            %   File: the file name to store the logging data
            %   RecordOption: whether to generate the moving procedure for
            %   hardware experiment (true/false).
            arguments
                options.File           string
                options.Header         string
                options.RecordOption   int32
            end
            % (1) create file (2) write header (3) set attribute values
            % Log file
            clk = strjoin(string(int32(clock)),'-');
            if isfield(options, "File")
                dirname = options.File;
            else
                dirname = "log-"+clk;
            end
            if isfield(options, "Header")
                header = options.Header;
            else
                header = "Simulation trial ran on: "+clk;
            end
            mkdir("./lib/log/", dirname);
            logdir = "./lib/log/"+dirname+"/";
            % data file (for simulation play back)
            obj.filemat = logdir+"data.mat";
            save(obj.filemat, "header");
            % standard output in the command line window (for user to
            % check)
            obj.filetxt = logdir+"stdout.txt";
            diary(obj.filetxt);
            diary on;
            % Experiment files
            if isfield(options, "RecordOption")
                obj.recordType = logical(options.RecordOption);
            else
                obj.recordType = false;
            end
            obj.filerobots = "";
            obj.frids = zeros(1, options.RecordOption);
            if obj.recordType
                for i=1:options.RecordOption
                    obj.filerobots(i) = logdir+num2str(i)+".txt";
                    obj.frids(i) = fopen(obj.filerobots(i), "a+");
                end
            end
        end
        
        function recordInit(obj)
            % record to mat file
            Map_Size = evalin('base', 'Map_Size');
            Robot_Size = evalin('base', 'Robot_Size');
            Final_Target = evalin('base', 'Final_Target');
            Robot_Current_Position0 = evalin('base', 'Robot_Current_Position');
            Robot_Dock0 = evalin('base', 'Robot_Dock');
            save(obj.filemat, "Map_Size", "Robot_Size", "Final_Target",...
                "Robot_Current_Position0", "Robot_Dock0", "-append");
            % record to txt file
            for i=1:length(obj.frids)
                % position
                fwrite(obj.frids(i), "position: [");
                fwrite(obj.frids(i), strjoin(string(Robot_Current_Position0(i,:)), ","));
                fprintf(obj.frids(i), ",0]\n");
                % dock
                fprintf(obj.frids(i), "dock: [0, 0, 0, 0]\n");
            end
        end
        
        function recordRotation(obj, rob_rotation)
            % record dock to mat
            Robot_Dock1 = evalin('base', 'Robot_Dock');
            save(obj.filemat, "Robot_Dock1", "-append");
            % record rotation radian to txt
            obj.rotations = rob_rotation;
            rlocs = evalin('base', 'Robot_Current_Position');
            for i=1:length(obj.frids)
                % position
                fwrite(obj.frids(i), "position: [");
                fwrite(obj.frids(i), strjoin(string(rlocs(i,:)), ","));
                fwrite(obj.frids(i), ","+string(rob_rotation(i)));
                fprintf(obj.frids(i), "]\n");
                % dock
                fprintf(obj.frids(i), "dock: [0, 0, 0, 0]\n");
            end
        end
        
        function recordTargetID(obj)
            Target_ID = evalin('base', 'Target_ID');
            save(obj.filemat, "Target_ID", "-append");
        end
        
        function recordExtension(obj, c_layer)
            Current_Target = evalin('base', 'Current_Target');
            eval("Current_Target"+string(c_layer)+" = Current_Target;");
            save(obj.filemat, "Current_Target"+string(c_layer), "-append");
        end
        
        function record1step(obj, step_num, happened_docks)
            Robot_Current_Position = evalin('base', 'Robot_Current_Position');
            Robot_Path = evalin('base', 'Robot_Path');
            Robot_PathL = evalin('base', 'Robot_PathL');
            eval("Robot_Current_Position"+string(step_num)+" = Robot_Current_Position;");
            eval("Robot_Path"+string(step_num)+" = Robot_Path;");
            eval("Robot_PathL"+string(step_num)+" = Robot_PathL;");
            save(obj.filemat, "Robot_Current_Position"+string(step_num), ...
                "Robot_Path"+string(step_num), ...
                "Robot_PathL"+string(step_num), "-append");
            % text file
            for i=1:length(obj.frids)
                % position
                fwrite(obj.frids(i), "position: [");
                fwrite(obj.frids(i), strjoin(string(Robot_Current_Position(i,:)), ","));
                fwrite(obj.frids(i), ","+string(obj.rotations(i)));
                fprintf(obj.frids(i), "]\n");
                % dock
                fprintf(obj.frids(i), "dock: ["+strjoin(string(happened_docks(i,:)), ",")+...
                    "]\n");
            end
        end
        
        function endRecording(~)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            diary off;
            fclose('all');
%             if obj.recordType
%                 for i=1:length(obj.frids)
%                     fclose(obj.frids(i));
%                 end
%             end
        end
    end
end

