classdef Trial < handle
    %TRIAL 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        % Classes
        gmap               map
        tars               targetGroup
        ext                Extension
        robotGp            AssembleGroup
        floatGp            moduleGroup
        mods               module      % Only save the floating modules
        fixed_mods         moduleGroup
        obstacles          obstacle
        display         %   display2D
        % Flags
%         fetch_arrive       logical
%         target_arrive      logical
        structure_arrive   logical
        finish             logical
        % params
        N_rob              int32
        N_mod              int32
%         rob_dirs           int32
        % Construction order: matrix shape, each row from index 1 to the
        % last non-zero entry is a subtree represented by the tree index of
        % the second last layer in the extension tree.
        con_order          int32
        sub_root           int32
        % Setting params
        enable3D           logical   % true if 3D construction is required.
        % TODO: 不应该存robdir 应该在extension
        % class导入之后就把哪个编号的模块可以对接哪些面告诉所有机器人group
        ci
        step_cnt           int32
    end
    
    methods
        
        %%  Trial methods
        function obj = Trial(dim)
            %TRIAL 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                dim = 2;
            end
            if dim == 2
                obj.enable3D = false;
            elseif dim == 3
                obj.enable3D = true;
            else
                error("Wrong input of dimension for a trial object.");
            end
            obj.obstacles = obstacle();
            obj.step_cnt = 0;
            addpath('lib');
            addpath('lib/display'); addpath('lib/alg');
        end
                
        function robs = getRobots(obj)
            robs = robot();
            for i=1:obj.N_rob
                robs(i) = obj.robotGp(i).LeadRobot;
            end
        end
        
        function setRobotGp(obj, locs)
            [l, ~] = size(locs);
            for i = 1:l
                rob = robot(i, [locs(i, :), 0], obj.gmap);
                obj.robotGp(i) = AssembleGroup(i, rob, obj.gmap);
                if ~isempty(obj.ext)
                    obj.robotGp(i).ext = obj.ext;
                else
                    warning("Initialize robot groups before targets."+...
                        " The extension object cannot be linked to the robots.")
                end
                obj.robotGp(i).initSearch();
            end
            obj.structure_arrive = false(1, l);
            obj.finish = false(1, l);
            obj.N_rob = l;
        end
        
        function setConOrder(obj)
            if ~isempty(obj.ext)
                [obj.con_order, obj.sub_root] = ...
                    obj.ext.genConstructOrder(obj.N_rob);
                for i=1:obj.N_rob
                    obj.robotGp(i).c_tar_idx = [1, 1];
                    obj.robotGp(i).c_tar_i = obj.con_order(1, 1);
                    obj.robotGp(i).c_tar_root = obj.sub_root(1);
                end
                disp("Construction order generated.")
                disp("Principle targets indexes:")
                disp(obj.con_order)
                disp("Indexes of the subtree roots:")
                disp(obj.sub_root);
            end
        end
        
        function [m, idx] = getModule(obj, id, l, r)
            % 如果只有1个id -> 返回对应的module/找不到模块报错
            % 如果有多个id -> 返回对应的module group/找不到group报错
            if nargin == 2
                l = 1; r = length(obj.mods);
            end
            for i=l:r
                m = obj.mods(i);
                if m.ID == id
                    idx = i;
                    return
                end
            end
            error("Required float module ID does not exist.");
        end
        
        function ms = getAllMods(obj)
            ms = obj.mods;
            for i=1:obj.N_rob
                if ~isempty(obj.robotGp(i).attachModuleID)
                    ms = [ms, obj.robotGp(i).modules.ModuleList.'];
                end
            end
            for i=1:length(obj.floatGp)
                ms = [ms, obj.floatGp(i).ModuleList.'];
            end
            for i=1:length(obj.fixed_mods)
                ms = [ms, obj.fixed_mods(i).ModuleList.'];
            end
        end
        
        function setModules(obj, locs)
            [obj.N_mod, ~] = size(locs);
            for i = 1:obj.N_mod
                obj.mods(i) = module(i, [locs(i, :), 0], obj.gmap);
            end
%             obj.N_mod = length(modules);
        end
        
        function setTargets(obj, w, l, c, ext_c, ext_l)
            % Input: width, length, coordinate of lower left corner,
            % extension center, extension length
            N = w*l;
            Tars = [];
            for i = 1:N
                m = ceil(i/l);
                n = i - (m-1) * l;
                Tars = [Tars; targetPoint(i, [m+c(1)-1, n+c(2)-1], obj.gmap)];
            end
            obj.tars = targetGroup(Tars);
            
            % Binary tree construction / extension
            obj.ext = Extension(obj.tars, obj.gmap);
            obj.ext.TargetToTree(ext_c, ext_l);
            
        end
        
        function setDisplay(obj)
            % Display
%             robs = obj.getRobots();
            if ~obj.enable3D
                obj.display = display2D(obj.gmap.mapSize, ...
                                        "FinalTarget", obj.tars, ...
                                        "Robot", obj.getRobots(), ...
                                        "Module", obj.mods, ...
                                        "Obstacle", obj.obstacles.Locations);
            else
                % TODO: 3D display
            end
            % Extension
            obj.ext.showExtension(obj.display);
            cl = length(obj.ext.GroupLayers);
            for i=1:obj.N_rob
                obj.robotGp(i).cl = cl;
            end
        end
        
        function execute(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明

            % Walk the modules
            for i=1:length(obj.mods)
                obj.mods(i).walk();
            end
            % TODO: move the floating module groups
            for i=1:length(obj.floatGp)
                % module group floating to other places.
                obj.floatGp(i).walk();
            end
            
            % Move the robot groups (search or construct)
            for i=1:obj.N_rob
                obj.ci = i;
                if ~obj.structure_arrive(i)
                    % TODO: check if the structure has finished
                    if obj.robotGp(i).c_tar_root == 0
                        obj.structure_arrive(i) = true;
                        obj.robotGp(i).LeadRobot.back2startPlace();
                    else
                        obj.moveRobGp();
                    end
                elseif ~obj.finish(i)
                    % move to the initial place
                    obj.finish(i) = obj.robotGp(i).move();
                end
            end
            % update the display
            obj.display.updateMap("Robot", obj.getRobots(), ...
                "Module", obj.getAllMods());
            obj.step_cnt = obj.step_cnt + 1;
        end
        
        %%  Assembly group methods
        % Some methods of assembly group methods require access to other
        % classes, which are implemented in the trial class.
        
        function moveRobGp(obj)
            i = obj.ci;
            % If an assembly group is changing its dock site, then this
            % function will use the changeDockSite method to move.
            if obj.robotGp(i).changingSite
                % 只要调用了move函数就算move了一次
                [is_changed, is_moved] = obj.robotGp(i).changeDockSite();
                if is_changed
                    obj.robotGp(i).changingSite = false;
                    obj.robotGp(i).dockSiteBlocked = false;
                end
                if is_moved
                    return
                end
            end
            if ~obj.robotGp(i).status
                % search and fetch
                if ~obj.robotGp(i).LeadRobot.isCarrying
                    % Search
                    id_m = obj.robotGp(i).search();
                    if ~isempty(id_m)
                        [float_m, m_idx] = obj.getModule(id_m);
                        obj.robotGp(i).updateGroup("add", float_m);
                        if class(float_m) == "module"
                            obj.mods(m_idx) = [];
                            obj.robotGp(i).robot2target(1);
                        elseif class(float_m) == "moduleGroup"
                            obj.floatGp(m_idx) = [];
                            % 需要根据获得的模块形状判断target位置
                        end
                    end
                    if obj.robotGp(i).delay_access
                        obj.robotGp(i).LeadRobot.setIgnorePos([]);
                        obj.robotGp(i).delay_access = false;
                    end
                else
                    % Navigate to the target
                    is_arrive = obj.robotGp(i).move();
%                     if ~obj.robotGp(i).waiting
%                         is_arrive = obj.robotGp(i).move();
%                     else
%                         is_arrive = false;
%                     end
                    if is_arrive
                        % 如果是static target，attach module然后回去继续搜索
                        % 如果是float target：
                        % （1）切换到construct status
                        % （2）检查是否需要更改对界面
                        obj.robotGp(i).markGroupObstacle(0);
                        t = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
                        if t.is2static
                            obj.fixed_mods = [obj.fixed_mods, ...
                                obj.robotGp(i).modules];
                            mlocs = obj.robotGp(i).modules.getLocations();
                            obj.gmap.setStructureMap(mlocs);
                            obj.robotGp(i).LeadRobot.addIgnorePos(mlocs);
                            obj.robotGp(i).updateGroup("del");
                            obj.robotGp(i).c_tar_root = 0;
                        else
%                             obj.robotGp(i).changingSite = true;
%                             if change_finish
                            if obj.ext.isTargetPair(obj.robotGp(i).c_tar_i)
                                m_id = obj.robotGp(i).modules.ModuleList(1).ID;
                                tar_opt = obj.robotGp(i).target_option;
%                                 m_loc = obj.robotGp(i).modules.ModuleList(1).Location(1:2);
                                obj.ext.assignID(obj.robotGp(i).c_tar_i, m_id, tar_opt);
                            end
%                                 obj.robotGp(i).c_tar_i = ...
%                                     obj.ext.tarTree.getparent(obj.robotGp(i).c_tar_i);
%                                 obj.robotGp(i).robot2target();
%                             else
                            % if the other part does not arrive
                            % switch to construct state and wait
                            % else
                            % dock, update target index, and leave
                            if obj.isTargetCompleted()
                                obj.dockandRecord();
                                obj.toNextPT();
                                obj.robotGp(i).delay_access = true;
                            else
                                obj.robotGp(i).changeStatus(true);
                                obj.robotGp(i).changingSite = true;
                                obj.robotGp(i).toParentTarget();
                            end
%                             end
%                             end
                            % 检查对接的group有没有到 到了则对接并离开
                            % 问题：如果有部分已经完成对接但是其他的机器人还没记完meeting？
                        end
%                         m_id = obj.robotGp(i).modules.getIDs();
%                         if length(m_id) == 1
%                             obj.markTargetID(m_id);
%                         end
                    else  % have not arrived yet
                        if length(obj.robotGp(i).meetingList) == obj.N_rob - 1
                            % TODO: robot go to the next subtree and clear
                            % meeting list
                            obj.toNextSubtree();
                        else
                            % If the current target has already occupied
                            % (1) record meeting if there is a robot
                            % (2) change target position
                            [occupied, observed] = obj.checkTargetStatus();
                            t = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
                            
%                             if isempty(find(obj.con_order==obj.robotGp(i).c_tar_i, 1))
%                                 % Not PT
                            if observed && ~any(occupied)
                                if t.is2static
                                    m_id = obj.robotGp(i).modules.getIDs();
                                    if isempty(find(t.dockerPoints==m_id(1), 1))
                                        obj.robotGp(i).LeadRobot.waiting = true;
                                        disp("Group "+string(i)+...
                                            " waiting for docking.");                            % 问题：如果有部分已经完成对接但是其他的机器人还没记完meeting？

                                        return
%                                         else

                                    end
                                end
                            end
%                             end
                            tonext_flag = false;
                            if occupied(2)
                                obj.robotGp(i).LeadRobot.waiting = false;
                                % ignore dock pair
                                obj.robotGp(i).ignoreDockPair(t);
                            end
                            if occupied(1) && ~occupied(2)
                                % robot need to go to next target
                                disp("Robot "+string(i)+" reports target occupied.");
%                                 obj.toNextPT();
                                obj.robotGp(i).robot2target(2);
%                             elseif occupied(2)
                            elseif all(occupied)
                                if occupied(1) == occupied(2) && occupied(1) > 0
                                    
                                elseif occupied(1) ~= occupied(2)
                                    error("FIXME!!");
                                end
                                tonext_flag = true;
                            elseif observed % target avaliable
                                groups = obj.gmap.getGroupAroundTarget(t);
                                for j=1:length(groups)
                                    if occupied(2) && groups(j) ~= occupied(2) && ...
                                            groups(j) < obj.robotGp(i).groupID
                                        tonext_flag = true;
                                    end
                                end
                                obj.robotGp(i).changingSite = true;
                            end
                            
                            if tonext_flag
                                obj.recordMeeting(occupied(2));
                                obj.robotGp(i).LeadRobot.setIgnorePos([]);
%                                 obj.robotGp(i).LeadRobot.addIgnorePos()
%                                 obj.robotGp(i).delay_access = true;
                                obj.robotGp(i).markGroupObstacle(occupied(1));
                                obj.toNextPT();
                                obj.robotGp(i).robot2target(1);
                            end
                        end
                    end
                end
            else
                % construct
                if length(obj.robotGp(i).meetingList) == obj.N_rob - 1
%                 if true
                    % construct until leave for next subtree
                    % if finish current node
                    %     if current node is not the root of the current
                    %     subtree
                    %         go to the parent target;
                    %     else
                    %         detach and leave for next subtree;
                    %         check whether the module group is floating or
                    %         static and add to corresponding data struct;
                    % else
                    %     if the other part arrives
                    %         dock and leave for next subtree;
                    % TODO: implement docking?
                    [target_complete, s_id] = obj.toCurrentTarget();
                    if target_complete
                        if ~s_id
                            obj.fixed_mods = [obj.fixed_mods, ...
                                    obj.robotGp(i).modules];
                            obj.robotGp(i).updateGroup("del");
                            obj.toNextSubtree();
                            return
                        end
                        if length(obj.robotGp(s_id).meetingList) < obj.N_rob-1
                            return
                        end
                        obj.dockandRecord(i, s_id, false);
                        extract_flag = obj.robotGp(i).toParentTarget();
                        obj.toNextSubtree(s_id);
                        if ~extract_flag   % arrived the subtree root
                            t = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
                            if t.is2static
                                obj.fixed_mods = [obj.fixed_mods, ...
                                    obj.robotGp(i).modules];
                            else
                                obj.floatGp = [obj.floatGp, ...
                                    obj.robotGp(i).modules];
                            end
                            obj.robotGp(i).updateGroup("del");
                            obj.toNextSubtree();
                        end
                    end
                    
                end
            end
        end
        
        function dockandRecord(obj, id1, id2, recordflag)
            % TODO 检查这个函数和相关的子函数
            if nargin == 1
                % dock at the current target
                id2 = obj.ci;
                locs = obj.getSilibingTargetLocs();
                id1 = obj.gmap.groupMap(locs(1,1), locs(1,2));
                recordflag = true;
            elseif nargin == 2
                id2 = obj.ci;
                recordflag = true;
            elseif nargin == 3
                recordflag = true;
            end
            if recordflag
                obj.recordMeeting(id1, id2);
            end
            gp1 = obj.robotGp(id1);
            gp2 = obj.robotGp(id2);
            % dock between assembly groups, gp2 will be free
            rloc = gp1.LeadRobot.Location;
            for i=1:gp2.modules.Size
                m = gp2.modules.ModuleList(i);
                mloc = m.Location;
                pos_shift = rloc-mloc;
                gp2.modules.ModuleList(i).PosShift = pos_shift;
            end
            gp1.modules.AddModuleGp(gp2.modules);
            gp1.LeadRobot.carriedModule = gp1.modules;
            
            gp2.modules = moduleGroup();
            gp2.LeadRobot.isCarrying = false;
            gp2.LeadRobot.carriedModule = moduleGroup();
            gp2.changeStatus(false);
%             id2 = gp2.groupID;
%             obj.toNextSubtree(id);
            gp2.updateGroup("del");
            gp1.updateGroup("add");
            obj.robotGp(id1) = gp1;
            obj.robotGp(id2) = gp2;
        end
        
        function recordMeeting(obj, id2, id1)
            if nargin == 2
                id1=obj.ci;
            end
            l1 = obj.robotGp(id1).meetingList;
            obj.robotGp(id1).meetingList = [l1, id2];
            l2 = obj.robotGp(id2).meetingList;
            obj.robotGp(id2).meetingList = [l2, id1];
        end
        
        %% Extension/Target methods
        
        function toNextPT(obj, delta_i)
            if nargin == 1
                delta_i = 1;
            end
            i = obj.ci;
            r = obj.robotGp(i).c_tar_idx(1);
            c = obj.robotGp(i).c_tar_idx(2)+delta_i;
            [~, Nc] = size(obj.con_order);
            if c > Nc || obj.con_order(r, c) == 0
                obj.toNextSubtree();
            else
                obj.robotGp(i).c_tar_i = obj.con_order(r, c);
                obj.robotGp(i).c_tar_idx = [r, c];
            end
        end
        
        function [target_complete, s_id] = toCurrentTarget(obj)
            % 检查当前target位置有没有另外一边的target在
            s_id = 0;
            i = obj.ci;
            is_arrive = obj.robotGp(i).move();
            
            locs = obj.getSilibingTargetLocs();
%             s_id = obj.gmap.workerRobotMap(locs(1,1), locs(1,2));
            if ~is_arrive
                target_complete = false;
                [occupied, observed] = obj.checkTargetStatus();
                if observed && ~obj.isTargetCompleted(locs)
                    obj.robotGp(i).LeadRobot.waiting = true;
                else
                    obj.robotGp(i).LeadRobot.waiting = false;
                end
                if occupied(2)
                    s_id = occupied(2);
                    [r, c] = find(obj.gmap.robotMap==s_id);
                    obj.robotGp(i).LeadRobot.setIgnorePos([r, c; locs]);
                end
                % dock site blocked
                r_loc = obj.robotGp(i).LeadRobot.Location(1:2);
                if observed && obj.gmap.workerRobotMap(r_loc(1), r_loc(2))
                    obj.robotGp(i).dockSiteBlocked = true;
                    obj.robotGp(i).changingSite = true;
                end
%                 s_id = 0;
%                 obj.robotGp(i).LeadRobot.setIgnorePos(locs);
            else
                target_complete = obj.isTargetCompleted(locs);
                s_id = obj.gmap.workerRobotMap(locs(1,1), locs(1,2));
            end
            
        end
        
        function locs = getSilibingTargetLocs(obj)
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
        
        function toNextSubtree(obj, i)
            % ci->robotGp->c_subtree, modify this attribute to next, and
            % change gp.c_tar_i as well. Change robotGp status to 0. Clear
            % the meeting list. if current root is 1 change to 0
            if nargin == 1
                i = obj.ci;
            end
            r = obj.robotGp(i).c_tar_idx(1)+1;
            c = 1;
            if r <= length(obj.sub_root)
                obj.robotGp(i).c_tar_i = obj.con_order(r, c);
                obj.robotGp(i).c_tar_idx = [r, c];
                obj.robotGp(i).c_tar_root = obj.sub_root(r);
            else
                obj.robotGp(i).c_tar_root = 0;
            end
            obj.robotGp(i).changeStatus(false);
            obj.robotGp(i).meetingList = [];
        end
        
        function decision = isTargetCompleted(obj, locs)
            i = obj.ci;
            if nargin == 1
                tar = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
                locs = tar.getLocs();
%                 locs = obj.getSilibingTargetLocs();
            end
            [n, ~] = size(locs);
            s_id = 0;
            for j = 1:n
                loc = locs(j, 1:2);
                if ~obj.gmap.structureMap(loc(1), loc(2)) && ...
                        ~obj.gmap.groupMap(loc(1), loc(2))
                    decision = false;
                    return
                elseif obj.gmap.groupMap(loc(1), loc(2))
                    temp_id = obj.gmap.groupMap(loc(1), loc(2));
                    if temp_id ~= i
                        s_id = temp_id;
                    end
                end
            end
            if s_id
                s_tar_idx = obj.robotGp(s_id).c_tar_i;
                if obj.ext.isTargetPair(obj.robotGp(i).c_tar_i)
                    p_tar_idx = obj.ext.tarTree.getparent(obj.robotGp(i).c_tar_i);
                    if s_tar_idx ~= p_tar_idx
                        decision = false;
                        return
                    end
                else
                    if s_tar_idx ~= obj.robotGp(i).c_tar_i
                        decision = false;
                        return
                    end
                end
            end
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
            tar = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
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
            r_loc = obj.robotGp(i).LeadRobot.Location;
            m_locs = obj.robotGp(i).modules.getLocations();
            group_locs = [r_loc; m_locs];
            dist = ceil(sqrt(double(obj.gmap.mapSize(1)^2+obj.gmap.mapSize(2)^2)));
            t_locs = tar.getLocs();
            for j=1:obj.robotGp(i).modules.Size+1
                for k = 1:tar.Size
                    temp_dist = sqrt(sum((t_locs(k,1:2)-group_locs(j, 1:2)).^2));
                    if temp_dist < dist
                        dist = temp_dist; nearest_idx = j;
                    end
                    if (nearest_idx == 1 && dist <= obj.gmap.robotDist) || ...
                            (nearest_idx ~= 1 && dist <= obj.gmap.moduleDist)
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
                    if (groupid && obj.robotGp(groupid).status)
                        g_size = obj.robotGp(groupid).modules.Size;
                        if g_size == tar.Size  % TODO: 不可以这样判断
                            is_occupied = [groupid, groupid];
                            return
                        else
                            % check target position
                            r_goal = obj.robotGp(i).LeadRobot.Goal;
                            m_goal = int32(r_goal(1:2)) - obj.robotGp(i).attachdir;
                            if obj.gmap.groupMap(m_goal(1), m_goal(2)) == groupid
                                is_occupied(1) = groupid;
                            else
                                is_occupied(2) = groupid;
%                                 obj.robotGp(i).ignoreDockPair(tar);
                            end
                        end
                    end
                    if obj.gmap.structureMap(t_locs(j,1), t_locs(j,2))
                        % check target position
                        r_goal = obj.robotGp(i).LeadRobot.Goal;
                        m_goal = int32(r_goal(1:2)) - obj.robotGp(i).attachdir;
                        if obj.gmap.structureMap(m_goal(1), m_goal(2))
                            is_occupied(1) = -1;
                        end
                        if ~all(m_goal(1:2)==t_locs(j,:))
                            is_occupied(2) = -1;
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
        
        function markTargetID(obj, m_id)
            i = obj.ci;
            t_idx = obj.robotGp(i).c_tar_i;
            m_loc = obj.robotGp(i).modules.ModuleList(1).Location;
            tar = obj.ext.getTargetByIdx(t_idx);
            for i=1:2
                t = tar.targetList;
                if t(i).Location(1:2) == m_loc(1:2)
                    t(i).displayID = m_id;
                    t_id = t(i).ID;
                    tar.targetList(i) = t(i);
                    break
                end
            end
            c_idx = t_idx;
            while c_idx ~= 1
                c_idx = obj.ext.tarTree.getparent(c_idx);
                tar = obj.ext.getTargetByIdx(c_idx);
                tar.setDisplayID(t_id, m_id);
                obj.ext.tarTree.set(c_idx, tar);
            end
        end

    end
end

