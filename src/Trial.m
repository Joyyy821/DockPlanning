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
        ts                 TabuSearch
        % Flags
        structure_arrive   logical
        start              logical
        % params
        assignment         int32    % robot - target assignment
        N_rob              int32
        ci
        step_cnt           int32
    end
    
    methods
        
        %%  Trial methods
        function obj = Trial()
            %TRIAL 构造此类的实例
            %   此处显示详细说明
            obj.step_cnt = 0;
            obj.structure_arrive = false;
            addpath('lib');
            addpath('lib/display'); addpath('lib/alg');
            addpath('lib/alg/connect');
        end
        
        function setRobots(obj, locs, docks)
            [l, ~] = size(locs);
            for i = 1:l
                r = robot(i, [locs(i, :), 0], docks(i,:), obj.gmap);
                obj.robGp(i) = AssembleGroup(i, r, obj.gmap);
            end
            obj.start = false(1, l);
%             obj.structure_arrive = false(1, l);
            obj.N_rob = l;
        end

        function rs = getRobots(obj)
            rs = [];
            for i=1:length(obj.robGp)
                rs = [rs, obj.robGp(i).RobotList];
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

        function ts_flag = setTargets(obj, locs, ext_c, ext_l)
            if nargin == 2
                ext_c = locs(1,:);
                ext_l = 3;
            end
            [l, ~] = size(locs);
            Tars = [];
            for i = 1:l
                Tars =[Tars; targetPoint(i, locs(i, :), obj.gmap)];
            end
            obj.tars = targetGroup(Tars);
            ts_flag = obj.runTS();
            if ts_flag
                obj.tars.setDisplayID(obj.assignment);
                % Binary tree construction / extension
                obj.ext = Extension(obj.tars, obj.gmap);
                obj.ext.TargetToTree(ext_c, ext_l);
            end
        end

        function ts_flag = runTS(obj)
            if obj.N_rob == obj.tars.Size
                % Set tabu search
                obj.ts = TabuSearch(obj.tars.getLocs, obj.getDock);
                [sol, cost] = obj.ts.search();
                if cost > 1
                    disp("Invalid solution with smallest number of connected"+...
                        "graph = "+str(cost));
                    ts_flag = 0;
                    return
                elseif cost == 1
                    disp("Best solution valid.")
                    ts_flag = 1;
                    obj.assignment = sol;
                    return
                end
            end
            ts_flag = -1;   % Robot has not been set yet.
        end
        
        function setDisplay(obj, pause_t)
            if nargin == 1
                pause_t = 0;
            end
            % Display
            obj.display = display2D(obj.gmap.mapSize, ...
                                    "FinalTarget", obj.tars, ...
                                    "Robot", obj.getRobots);

            % Wait for the UI initialization
            pause(pause_t);

            % show assignment result
            obj.display.updateMap("TargetID", obj.tars);

            % Extension
            obj.ext.showExtension(obj.display);
            cl = length(obj.ext.GroupLayers);
            for i=1:length(obj.robGp)
                obj.robGp(i).c_tar_i = obj.ext.getTreeIdx(i, cl);
            end
        end
        
        function setRobotTarget(obj)
            i = obj.ci;
            if ~obj.start(i)
                % Go to the leaf node
                goal_tar = obj.ext.getTargetByIdx(obj.robGp(i).c_tar_i);
                goal_loc = goal_tar.TargetList(1).Location;
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
                if ~obj.start(i)
                    obj.setRobotTarget();
                    obj.start(i) = true;
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
                    elseif is_arrive
                        obj.setRobotTarget();
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
        end

        function dock(obj, g_idx1, g_idx2)
            obj.robGp(g_idx1).addGroup(obj.robGp(g_idx2));
            obj.robGp(g_idx2) = [];
            obj.robGp(g_idx1).waiting = false;
        end

        %% Extension/Target methods
        
        function locs = getSilibingTargetLocs(obj)
            % TODO!!
            i = obj.ci;
            m_id = sort(obj.robotGp(i).modules.getIDs());
            c_tar_is = obj.ext.tarTree.getchildren(obj.robotGp(i).c_tar_i);
            if length(c_tar_is) == 1
                locs = [];
            else
                s_id = [];
                for j=1:2
                    t = obj.ext.getTargetByIdx(c_tar_is(j));
                    t_id = sort(t.getDisplayIDs());
                    if length(t_id) == length(m_id) && all(m_id == t_id)
                        s = obj.ext.getTargetByIdx(c_tar_is(3-j));
                        s_id = s.getIDs();
                    end
                end
                locs = [];
                tar = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
                for j=1:length(s_id)
                    loc = tar.getTarLoc(s_id(j));
                    locs = [locs; loc];
                end
            end
        end
        
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
            % TODO: define a function to check whether it is PT with depth
            % -1
%             is_leaf = obj.ext.isTargetPair(obj.robotGp(i).c_tar_i);
%             if is_leaf
%                 dock_tar = obj.ext.tarTree.getparent(obj.robotGp(i).c_tar_i);
%                 dock_tar = obj.ext.getTargetByIdx(dock_tar);
%                 dockt_locs = dock_tar.getLocs();
%             else
%                 
%                 dock_tar = obj.ext.tarTree.getsiblings(obj.robotGp(i).c_tar_i);
%                 if  length(dock_tar) == 2
%                     if dock_tar(1) == obj.robotGp(i).c_tar_i
%                         dock_tar = dock_tar(2);
%                     else
%                         dock_tar = dock_tar(1);
%                     end
%                 end
%             end
%             
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

