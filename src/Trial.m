classdef Trial < handle
    %TRIAL 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        % Classes
        gmap               map
        tars               targetGroup
        ext                Extension
        robGp              AssembleGroup
        display            display2D
        % tabu search
        ts                 TabuSearch
        alg_type           int32
        % logging
        islogging          logical
        log                Logging
        % Flags
        structure_arrive   logical
        start              logical
        % params
        name               string
        assignment         int32    % robot - target assignment
        N_rob              int32
        ci
        step_cnt           int32
    end
    
    methods
        
        %%  Trial methods
        function obj = Trial(name)
            %TRIAL 构造此类的实例
            %   此处显示详细说明
            obj.step_cnt = 0;
            obj.structure_arrive = false;
            if nargin == 1
                obj.name = name;
            else
                obj.name = "";
            end
%             obj.log = Logging();
        end
        
        function run(obj, options)
            arguments
                %% Required arguments
                obj                      Trial
                % required
                options.RobotLocation    int32
                % required
                options.TargetLocation   int32
                % required
                % 1: w/o rotation; 2: with rotation; 3: with rotation +
                % male/female dock ports
                options.ALgorithmType    int32
                %% Recommended arguments
                % optional (set map size to [15, 15] if the argument is not
                % provided)
                % Still, user is recommended to manually set up the map
                % size in case that the default map is too small.
                options.MapSize          int32
                % optional (set the robot dock joints to homogeneous if the 
                % argument is not provided)
                options.RobotDock        int32
                % optional (default value is [false, false])
                % [whether to record log, whether to generate data for hardware experiment]
                options.isLogging        (1, 2) logical
                % optional (set the robot cognitive distance to 3 if the
                % argument is not provided)
                %% Optional arguments
                options.RobotCognDist    int32
                % optional (set the target extension center to the first
                % target location if the argument is not provided)
                options.TargetCenter     int32
                % optional (default value equals 3)
                options.ExtensionLength  int32
            end
            %% Initialization
            
            % Initialize map object
            if isfield(options, "MapSize")
                map_size = options.MapSize;
            else
                map_size = [15, 15];
            end
            if isfield(options, "RobotCognDist")
                cogn_dist = options.RobotCognDist;
            else
                cogn_dist = 3;
            end
            obj.gmap = map(map_size, cogn_dist);
            % Set robots
            if isfield(options, "RobotLocation")
                rob_loc = options.RobotLocation;
            else
                error("Required input argument RobotLocation is not provided by the user.");
            end
            if isfield(options, "RobotDock")
                d = options.RobotDock;
            else
                [nrob, ~] = size(rob_loc);
                d = ones(nrob, 4);
            end
            obj.setRobots(rob_loc, d);
            % Set logging
            if isfield(options, "isLogging")
                obj.islogging = options.isLogging;
            else
                obj.islogging = [false, false];
            end
            if obj.islogging(1)
                obj.setLogging(obj.islogging(2));
            end
            % Set target and run tabu search
            if isfield(options, "TargetLocation")
                t = options.TargetLocation;
            else
                error("Required input argument TargetLocation is not provided by the user.");
            end
            if isfield(options, "ALgorithmType")
                obj.alg_type = options.ALgorithmType;
            else
                error("Required input argument ALgorithmType is not provided by the user.");
            end
            if isfield(options, "TargetCenter")
                c = options.TargetCenter;
            else
                c = t(1, :);
            end
            if isfield(options, "ExtensionLength")
                extl = options.ExtensionLength;
            else
                extl = 3;
            end
            [success, rotated_dock] = obj.setTargets(obj.alg_type, t, c, extl);
            %% Robot navigation
            if ~success
                disp("Program exit ...")
            else
                obj.setDisplay(rotated_dock, 3);
                % Main loop
                max_steps = 1e6;
                while ~obj.structure_arrive && ...
                        obj.step_cnt <= max_steps
                    obj.execute();
                end
            end
            obj.endSim;
        end
        
        function setRobots(obj, locs, docks)
            [l, ~] = size(locs);
            disp("The initial dock joint distributions for robots:");
            for i = 1:l
                r = robot(i, [locs(i, :), 0], docks(i,:), obj.gmap);
                obj.robGp(i) = AssembleGroup(i, r, obj.gmap);
                disp("Robot No. "+string(i)+": "+strjoin(string(docks(i,:))));
            end
%             if all(obj.islogging)
%                 obj.log.recordType = true;
%             end
            obj.start = false(1, l);
%             obj.structure_arrive = false(1, l);
            obj.N_rob = l;
        end

        function rs = getRobots(obj)
            rs = robot();
            for i=1:length(obj.robGp)
                for j=1:obj.robGp(i).Size
                    k = obj.robGp(i).RobotList(j).ID;
                    rs(k) = obj.robGp(i).RobotList(j);
                end
            end
        end

        function idx = findRobGp(obj, g_id)
            for i=1:length(obj.robGp)
                if obj.robGp(i).groupID == g_id
                    idx = i;
                    return
                end
            end
            error("Cannot find the desired group id: "+string(g_id));
        end

        function docks = getDock(obj)
            docks = zeros(obj.N_rob, 4);
            rs = obj.getRobots();
            for i=1:obj.N_rob
                docks(i,:) = rs(i).DockJoint;
            end
        end

        function rots = setDock(obj, docks)
            rs = obj.getRobots();
            rots = zeros(1, obj.N_rob);
            for i=1:obj.N_rob
                % check the degree of rotation first
                rotate_matrix = [1 2 3 4;
                                 3 4 2 1;
                                 2 1 4 3;
                                 4 3 1 2
                                ];
                for j=1:4
                    if all(rs(i).DockJoint(rotate_matrix(j,:)) == docks(i,:))
                        rs(i).rotation = (j-1)*pi/2;
                        rots(i) = rs(i).rotation;
                        break
                    end
                end
                if isempty(rs(i).rotation)
                    error("Invalid rotation for robot "+num2str(i)+...
                        ", where the origin shape is "+num2str(rs(i).DockJoint)+...
                        " and the desired shape is "+num2str(docks(i,:))+".");
                end
                rs(i).DockJoint = docks(i,:);
            end
        end

        function locs = getAllRobotLoc(obj)
            locs = zeros(obj.N_rob, 2);
            rs = obj.getRobots();
            for i=1:obj.N_rob
                locs(i,:) = rs(i).Location(1:2);
            end
        end

        function [ts_flag, dock] = setTargets(obj, alg_type, locs, ext_c, ext_l)
            if nargin == 3
                ext_c = locs(1,:);
                ext_l = 3;
            elseif nargin == 4
                ext_l = 3;
            end
            [l, ~] = size(locs);
            Tars = [];
            for i = 1:l
                Tars =[Tars; targetPoint(i, locs(i, :), obj.gmap)];
            end
            obj.tars = targetGroup(Tars);
            [ts_flag, dock] = obj.runTS(alg_type);
            if ts_flag == 1
                obj.tars.setDisplayIDandDock(obj.assignment, dock);
                % Binary tree construction / extension
                obj.ext = Extension(obj.tars, obj.gmap);
                obj.ext.TargetToTree(ext_c, ext_l);
            end
        end

        function [ts_flag, dock] = runTS(obj, alg_type)
            if obj.N_rob
                % Set tabu search
                obj.ts = TabuSearch(obj.tars.getLocs, obj.getDock, ...
                    obj.getAllRobotLoc, alg_type);
                disp("Search for valid target assignment...");
                [sol, dock, cost] = obj.ts.search();
                disp("sol:");disp(sol);
                disp("dock: "); disp(dock);
%                 obj.setDock(dock);
                if cost > 1
                    disp("Invalid solution with smallest number of connected"+...
                        "graph = "+string(cost));
                    ts_flag = 0;
                    return
                elseif cost == 1
                    disp("Best solution valid.");
                    ts_flag = 1;
                    obj.assignment = sol;
                    return
                end
            end
            ts_flag = -1;   % Robot has not been set yet.
            dock = [];
        end
        
        function setDisplay(obj, dock, pause_t)
            if nargin == 2
                pause_t = 0;
            end
            % Display
            obj.display = display2D(obj.gmap.mapSize, ...
                                    "FinalTarget", obj.tars, ...
                                    "Robot", obj.getRobots, ...
                                    "ShowUI", true);
            if obj.islogging(1)
                obj.log.recordInit();
            end
            
            % Wait for the UI initialization
            pause(pause_t);

            % show assignment result
            obj.display.updateMap("TargetID", obj.tars);
            
            % Rotation robots
            rots = obj.setDock(dock);
            obj.display.rotateRobot(dock);
            
            if obj.islogging(1)
                obj.log.recordTargetID();
                obj.log.recordRotation(rots);
            end
            
            % Extension
            if ~obj.islogging(1)
                obj.ext.showExtension(obj.display, "PauseT", 1);
            else
                obj.ext.showExtension(obj.display, "Log", obj.log);
            end
            
            cl = length(obj.ext.GroupLayers);
            for i=1:length(obj.robGp)
                id = obj.robGp(i).RobotList(1).ID;
                obj.robGp(i).c_tar_i = obj.ext.getTreeIdx(id, cl);
                if isempty(obj.robGp(i).c_tar_i)
                    obj.robGp(i).RobotList(1).Goal = obj.robGp(i).RobotList(1).Location;
                end
            end
            
        end
        
        function setLogging(obj, to_exp)
            if to_exp
                ro = obj.N_rob;
            else
                ro = 0;
            end
            if obj.name == ""
                obj.log = Logging("RecordOption", ro);
            else
                obj.log = Logging("File", obj.name, "RecordOption", ro);
            end
            clk = clock;
            disp("Program starts ...");
            disp("Trial executed on: "+strjoin(string(int32(clk(1:3))),'-')+...
                " "+strjoin(string(int32(clk(4:6))),':'));
            disp("Trial: "+obj.name);
        end
        
        function setRobotTarget(obj)
            i = obj.ci;
            if isempty(obj.robGp(i).c_tar_i)
                return
            end
            if ~obj.robGp(i).isStarted
                % Go to the leaf node
                goal_tar = obj.ext.getTargetByIdx(obj.robGp(i).c_tar_i);
                goal_loc = goal_tar.TargetList(1).Location;
                obj.robGp(i).isStarted = true;
            else
                % Go to the parent node
                parent_idx = obj.ext.tarTree.getparent(obj.robGp(i).c_tar_i);
                if parent_idx == 0
                    obj.structure_arrive = true;
                    disp("Robot "+string(i)+" finished.");
                    return
                end
                parent_tar = obj.ext.getTargetByIdx(parent_idx);
                ignore_loc = zeros(parent_tar.Size-1, 2);
                k = 1;
                for j=1:parent_tar.Size
                    loc = parent_tar.TargetList(j).Location;
                    if obj.robGp(i).groupID==parent_tar.TargetList(j).displayID
                        goal_loc = loc;
                    else
                        ignore_loc(k,:) = loc;
                        k = k + 1;
                    end
                end
                obj.robGp(i).c_tar_i = parent_idx;
                obj.robGp(i).setIgnorePos(ignore_loc);
                obj.robGp(i).status = true;
                % TODO: update current target
                
%                 obj.robGp(i).dock_loc = obj.getSilibingTargetLocs();
            end
            disp("Robot "+string(obj.robGp(i).groupID)+"'s goal: ["+...
                string(goal_loc(1))+", "+string(goal_loc(2))+"].");
            obj.robGp(i).setGoal([goal_loc, 0]);
        end

        function execute(obj)
            %EXECUTE Run the docking procedure for one step
            %   Execute once

            % Move the robot groups (search or construct)
            dock_lst = [];
            for i=1:length(obj.robGp)
                obj.ci = i;
                if ~obj.robGp(i).isStarted
                    obj.setRobotTarget();
%                     obj.start(i) = true;
                end
                if isempty(obj.robGp(i).c_tar_i)
                    continue
                end
                if ~obj.structure_arrive
                    if ~obj.ext.tarTree.isleaf(obj.robGp(i).c_tar_i)
                        [occupied, observed] = obj.checkTargetStatus();
                        if observed && ~occupied(2)
                            obj.robGp(i).waiting = true;
                        elseif observed && occupied(2)
                            obj.robGp(i).waiting = false;
                        end
                    end
                    is_arrive = obj.robGp(i).move();
                    if is_arrive && ~obj.ext.tarTree.isleaf(obj.robGp(i).c_tar_i)
                        [flag, gp] = obj.isTargetCompleted();
                        if ~flag
                            obj.robGp(i).waiting = true;
                        else
                            obj.robGp(i).waiting = false;
                            % dock
                            if length(gp) == 2
                                [nr, ~] = size(dock_lst);
                                add_gp = true;
                                for j=1:nr
                                    if dock_lst(j,1) == obj.robGp(i).groupID ...
                                            || dock_lst(j,2) == obj.robGp(i).groupID
                                        add_gp = false;
                                        break
                                    end
                                end
                                if add_gp
                                    dock_lst = [dock_lst; gp];
                                end
                            end
                            obj.setRobotTarget();
                        end
                    elseif obj.ext.tarTree.isleaf(obj.robGp(i).c_tar_i)
                        % Check the pair target
                        [is_occupied, observed] = obj.checkTargetStatus();
                        if is_arrive && is_occupied(2)
                            obj.setRobotTarget();
                        elseif ~is_arrive && observed
                            dock_tar = obj.ext.tarTree.getsiblings(obj.robGp(i).c_tar_i);
                            t_locs = [];
                            for j=1:length(dock_tar)
                                tar = obj.ext.getTargetByIdx(dock_tar(j));
                                tar = tar.getLocs();
                                t_locs = [t_locs; tar];
                            end
                            obj.robGp(i).setIgnorePos(t_locs);
                        end
                    end
%                     % TODO: check if the structure has finished
%                     if obj.rob(i).c_tar_root == 0
%                         obj.structure_arrive(i) = true;
%                     else
%                         obj.moveRob();
%                     end
                end
            end
            % dock
            if ~isempty(dock_lst)
                [n, ~] = size(dock_lst);
                for i=1:n
                    % dock
                    g1 = obj.findRobGp(dock_lst(i,1));
                    g2 = obj.findRobGp(dock_lst(i,2));
                    obj.dock(g1, g2);
                end
            end
            % update the display
            obj.display.updateMap("Robot", obj.getRobots);
            obj.step_cnt = obj.step_cnt + 1;
            if ~isempty(obj.log)
                happended_dock = obj.findHappendedDock();
                obj.log.record1step(obj.step_cnt, happended_dock);
            end
        end
        
        function happended_dock = findHappendedDock(obj)
            happended_dock = zeros(obj.N_rob, 4);
%             if naargin == 2 && isempty(dock_lst)
%                 happended_dock = [];
%                 return
%             end
%             cnt = 1;
            for i = 1:length(obj.robGp)
                if obj.robGp(i).Size > 1
                    for j = 1:obj.robGp(i).Size
                        % check the docking status
                        gid = obj.robGp(i).groupID;
                        dirs = [0, 1; 0, -1; -1, 0; 1, 0];  % up, down, left, right
                        rid = obj.robGp(i).RobotList(j).ID;
                        for k=1:4
                            d = obj.robGp(i).RobotList(j).DockJoint(k);
                            loc = obj.robGp(i).RobotList(j).Location(1:2)+dirs(k,:);
                            if d && obj.gmap.groupMap(loc(1), loc(2)) == gid
                                happended_dock(rid, k) = d;
                            end
                        end
%                         cnt = cnt + 1;
                    end
%                 else
%                     cnt = cnt + 1;
                end
            end
        end

        function dock(obj, g_idx1, g_idx2)
            obj.robGp(g_idx1).addGroup(obj.robGp(g_idx2));
            obj.robGp(g_idx2) = [];
            obj.robGp(g_idx1).waiting = false;
        end

        function endSim(obj)
            % Simulation summary
            disp("-------------- End of Trial: Simulation Summary -----------------");
            disp("Target extension steps: "+string(obj.ext.depth));
            disp("Total time steps: "+string(obj.step_cnt));
            disp("--------");
            disp("Total moved steps for each robot:");
            rs = obj.getRobots;
            for i=1:obj.N_rob
                s = rs(i).stepCount;
                disp("Robot No. "+string(rs(i).ID)+": "+string(s));
            end
            if ~isempty(obj.log)
                obj.log.endRecording;
            end
        end

        %% Extension/Target methods
        
        function [decision, gp] = isTargetCompleted(obj, locs)
            gp = [];
            i = obj.ci;
            if nargin == 1
                tar = obj.ext.getTargetByIdx(obj.robGp(i).c_tar_i);
                locs = tar.getLocs();
%                 locs = obj.getSilibingTargetLocs();
            end
            [n, ~] = size(locs);
            for j = 1:n
                loc = locs(j, 1:2);
                g_id = obj.gmap.groupMap(loc(1), loc(2));
                if ~g_id
                    decision = false;
                    return
                else
                    gp = [gp, g_id];
                end
            end
            gp = unique(gp);
            decision = true;
        end
        
        function [is_occupied, observed] = checkTargetStatus(obj)
            % ci->robotGp->c_tar_i->loc
            % if loc in robot's sight and loc is occupied
            % find the occupied group and record each other
            % remark: the target is regarded as occupied only if all 
            % modules arrive at the target, if only half of the modules are
            % presented, then change the target (to index 2) and set
            % ignored places for the robot.
            % check the assembly group id to find the corresponding assembly
            % group and compare the size with the target size (or
            % alternative: set a flag)
            observed = false;
            is_occupied = [0, 0];
            i = obj.ci;
            tar = obj.ext.getTargetByIdx(obj.robGp(i).c_tar_i);
            is_leaf = obj.ext.tarTree.isleaf(obj.robGp(i).c_tar_i);
            if is_leaf
                t_loc = tar.getLocs();
                r_loc =  obj.robGp(i).getLocation();
                dist = sqrt((t_loc(1)-r_loc(1))^2 + (t_loc(2)-r_loc(2))^2);
                if dist <= obj.gmap.robotDist + 2
                    observed = true;
                end
                dock_tar = obj.ext.tarTree.getsiblings(obj.robGp(i).c_tar_i);
                for i_dt = 1:length(dock_tar)
                    dt = obj.ext.getTargetByIdx(dock_tar(i_dt));
                    dockt_loc = dt.getLocs(); % should only contain one loc because it's a leaf node
                    if obj.gmap.workerRobotMap(dockt_loc(1), dockt_loc(2))
                        if dockt_loc(1) == obj.robGp(i).RobotList(1).Location(1) && ...
                                dockt_loc(2) == obj.robGp(i).RobotList(1).Location(2)
                            is_occupied(1) = true;
                        else
                            disp("Robot "+string(i)+" detects its dock pair.");
                            is_occupied(2) = true;
                        end
                    end
                end
                if length(dock_tar) == 1
                    is_occupied(2) = true;
                end
                return
            end
                
%                 dock_tar = obj.ext.tarTree.getsiblings(obj.robGp(i).c_tar_i);
%                 if  length(dock_tar) == 2
%                     if dock_tar(1) == obj.robGp(i).c_tar_i
%                         dock_tar = dock_tar(2);
%                     else
%                         dock_tar = dock_tar(1);
%                     end
%                 end
%             end
            
            group_locs = obj.robGp(i).getLocation();
            dist = ceil(sqrt(double(obj.gmap.mapSize(1)^2+obj.gmap.mapSize(2)^2)));
            t_locs = tar.getLocs();
            for j=1:obj.robGp(i).Size
                for k = 1:tar.Size
                    temp_dist = sqrt(sum((t_locs(k,1:2)-group_locs(j, 1:2)).^2));
                    if temp_dist < dist
                        dist = temp_dist;
                    end
                    if dist <= obj.gmap.robotDist
                        observed = true;
                        break
                    end
                end
                if observed
                    break
                end
            end
%             if (nearest_idx == 1 && dist <= obj.gmap.robotDist) || ...
%                     (nearest_idx ~= 1 && dist <= obj.gmap.moduleDist)
%                 observed = true;
%             end

            for j = 1:tar.Size
                if observed
                    groupid = obj.gmap.groupMap(t_locs(j,1), t_locs(j,2));
                    if groupid
                        [g_size, ~] = size(find(obj.gmap.groupMap==groupid));
                        if g_size == tar.Size  % TODO: 不可以这样判断
                            is_occupied = [groupid, groupid];
                            return
                        else
                            % check target position
                            r_goal = obj.robGp(i).RobotList(1).Goal;
                            if obj.gmap.groupMap(r_goal(1), r_goal(2)) == groupid
                                is_occupied(1) = groupid;
                            else
                                if g_size == tar.Size - obj.robGp(i).Size
                                    is_occupied(2) = groupid;
                                end
%                                 obj.robotGp(i).ignoreDockPair(tar);
                            end
                        end
                    end
                end
            end
%             if tar.Size == 2
%                 tl = tar.TargetList(1).Location;
%                 if obj.gmap.moduleMap(tl(1), tl(2))
%                     tl = tar.TargetList(2).Location;
%                 end
%                 obj.robotGp(i).LeadRobot.Goal = ...
%                     [tl+obj.robotGp(i).attachdir, 0];
%                 obj.changeDockSite();
%             end
        end

    end
end

