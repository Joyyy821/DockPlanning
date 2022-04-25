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
            obj.step_cnt = 0;
            addpath('lib');
            addpath('lib/display'); addpath('lib/alg');
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
            end
            
            % Move the robot groups (search or construct)
            for i=1:obj.N_rob
                obj.ci = i;
                if ~obj.structure_arrive(i)
                    obj.moveRobGp();
                    % TODO: check if the structure has finished
                    if obj.robotGp(i).c_tar_root == 0
                        obj.structure_arrive(i) = true;
                        obj.robotGp(i).LeadRobot.back2startPlace();
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
        
        function moveRobGp(obj)
            i = obj.ci;
            if obj.robotGp(i).changingSite
                [is_changed, is_moved] = obj.changeDockSite();
                if is_changed
                    obj.robotGp(i).changingSite = false;
                end
                if is_moved
                    return
                end
            end
            if ~obj.robotGp(i).status
                % search and fetch
                if ~obj.robotGp(i).LeadRobot.isCarrying
                    id_m = obj.robotGp(i).search();
                    if ~isempty(id_m)
                        [float_m, m_idx] = obj.getModule(id_m);
                        obj.robotGp(i).updateGroup("add", float_m);
                        obj.mods(m_idx) = [];
%                         obj.fetch_arrive(i) = true;
%                         target = int32(evalin('base', 'Object_Current_Target'));
                        obj.robot2target();
                    end
                else
                    is_arrive = obj.robotGp(i).move();
                    if is_arrive
                        % 如果是static target，attach module然后回去继续搜索
                        % 如果是float target：
                        % （1）切换到construct status
                        % （2）检查是否需要更改对界面
                        t = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
                        if t.is2static
                            obj.fixed_mods = [obj.fixed_mods, ...
                                obj.robotGp(i).modules];
                            obj.robotGp(i).updateGroup("del");
                        else
%                             obj.robotGp(i).changingSite = true;
%                             if change_finish
                            if obj.ext.tarTree.isleaf(obj.robotGp(i).c_tar_i)
                                obj.robotGp(i).c_tar_i = ...
                                    obj.ext.tarTree.getparent(obj.robotGp(i).c_tar_i);
                                m_id = obj.robotGp(i).modules.ModuleList(1).ID;
                                obj.ext.assignID(obj.robotGp(i).c_tar_i, m_id);
                                obj.robot2target();
                            else
                                % if the other part does not arrive
                                % switch to construct state and wait
                                % else
                                % dock, update target index, and leave
                                if obj.isTargetCompleted()
                                    obj.dockandRecord();
                                    obj.changeTarget();
                                else
                                    obj.robotGp(i).status = true;
                                end
                            end
%                             end
                            % 检查对接的group有没有到 到了则对接并离开
                            % 问题：如果有部分已经完成对接但是其他的机器人还没记完meeting？
                        end
%                         m_id = obj.robotGp(i).modules.getIDs();
%                         if length(m_id) == 1
%                             obj.markTargetID(m_id);
%                         end
                    else
                        if length(obj.robotGp(i).meetingList) == obj.N_rob - 1
                            % TODO: robot go to the next subtree and clear
                            % meeting list
                            obj.toNextSubtree();
                        else
                            % If the current target has already occupied
                            % (1) record meeting if there is a robot
                            % (2) change target position
                            [occupied, observed] = obj.checkTargetStatus();
                            if occupied
                                % robot need to go to next target
                                obj.changeTarget();
                                obj.robot2target();
%                             elseif occupied == 1
                                
                            elseif observed % target avaliable
                                obj.robotGp(i).changingSite = true;
                            end
                        end
                    end
                end
            else
                % construct
                if length(obj.robotGp(i).meetingList) == obj.N_rob - 1
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
                    [silibing_arrive, s_id] = obj.checkSilibingTarget();
                    if silibing_arrive
                        obj.dockandRecord(i, s_id, false);
                        if obj.robotGp(i).c_tar_i ~= obj.robotGp(i).c_tar_root
                            obj.robotGp(i).c_tar_i = ...
                                obj.ext.tarTree.getparent(obj.robotGp(i).c_tar_i);
                            obj.robot2target();
                            obj.robotGp(i).changingSite = true;
                        else
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
        
        function dockandRecord(obj, id1, id2, record)
            if nargin == 1
                % dock at the current target
                id2 = obj.ci;
                locs = obj.getSilibingTargetLocs();
                id1 = obj.gmap.groupMap(locs(1,1), locs(1,2));
            end
            if nargin <= 3
                record = true;
            end
            gp1 = obj.robotGp(id1);
            gp2 = obj.robotGp(id2);
            % dock between assembly groups, gp2 will be free
            gp1.modules.AddModule(gp2.modules);
            gp1.LeadRobot.carriedModule = gp1.modules;

            gp2.modules = moduleGroup();
            gp2.LeadRobot.isCarrying = false;
            gp2.LeadRobot.carriedModule = moduleGroup();
            gp2.status = false;
            id = gp2.groupID;
            obj.toNextSubtree(id);
            gp2.updateGroup("del");
            gp1.updateGroup("add");
            obj.robotGp(id1) = gp1;
            obj.robotGp(id2) = gp2;
            if record
                obj.robotRecord(id1, id2);
            end
        end
        
        function robotRecord(obj, id1, id2)
            l1 = obj.robotGp(id1).meetingList;
            obj.robotGp(id1).meetingList = [l1, id2];
            l2 = obj.robotGp(id2).meetingList;
            obj.robotGp(id2).meetingList = [l2, id1];
%             id1 = gp1.groupID;
%             id2 = gp2.groupID;
%             gp1.meetingList = [gp1.meetingList, id2];
%             gp2.meetingList = [gp2.meetingList, id1];
        end
        
        function changeTarget(obj, delta_i)
            if nargin == 1
                delta_i = 1;
            end
            i = obj.ci;
            r = obj.robotGp(i).c_tar_idx(1);
            c = obj.robotGp(i).c_tar_idx(2)+delta_i;
            obj.robotGp(i).c_tar_i = obj.con_order(r, c);
            obj.robotGp(i).c_tar_idx = [r, c];
        end
        
        function robot2target(obj)
            i = obj.ci;
            target = obj.ext.tarTree.get(obj.robotGp(i).c_tar_i);
            if target.Size == 1
                t_loc = target.TargetList(1).Location;
            else
                t_loc = target.getTarLoc(obj.robotGp(i).attachModuleID, "display");
            % TODO: assembleGroup的attachmoduleID属性改成记录
            % attach module在group中的相对位置，同时查找target
            % 中对应位置的target location，然后向robot发布goal。
            % Alternative method: 这个时候相当于已经确定模块和target
            % display id相对应，直接查找与attachmoduleID有相同display
            % id的target位置，得到t_loc
            end
            if target.is2static
                obj.robotGp(i).LeadRobot.isDockerIgnored = true;
            end
            obj.robotGp(i).LeadRobot.Goal = ...
                [int32(t_loc)+obj.robotGp(i).attachdir, 0];
        end
        
        function [is_finish, is_moved] = changeDockSite(obj)
            % 根据当前target检查机器人对接方向是否在允许的方向中，如果不在则
            % 发布可行的goal并更换对接面，每调用一次函数移动一步，直到完成对接
            % 面更换返回true TODO
            i = obj.ci;
            locs = obj.robotGp(i).getDockSites();
            [n,~] = size(locs);
            if obj.robotInLocs(locs)
                is_finish = true;
                is_moved = false;
                return
            else
                obj.robotGp(i).LeadRobot.isCarrying = false;
                obj.robotGp(i).LeadRobot.Goal(1:n,1:2) = locs;
                is_finish = obj.robotGp(i).move();
                is_moved = true;
            end
            if is_finish
                obj.robotGp(i).LeadRobot.isCarrying = true;
                obj.robotGp(i).findAttachment();
            end
        end
        
        function decision = robotInLocs(obj, locs)
            r_loc = obj.robotGp(obj.ci).LeadRobot.Location;
            [n,~] = size(locs);
            for i=1:n
                if all(r_loc(1:2) == locs(i, 1:2))
                    decision = true;
                    return
                end
            end
            decision = false;
        end
        
        function [silibing_arrive, s_id] = checkSilibingTarget(obj)
            % 检查当前target位置有没有另外一边的target在
            i = obj.ci;
            is_arrive = obj.robotGp(i).move();
            locs = obj.getSilibingTargetLocs();
            if ~is_arrive
                silibing_arrive = false;
                obj.robotGp(i).LeadRobot.setIgnorePos(locs);
            else
                silibing_arrive = obj.isTargetCompleted(locs);
                s_id = obj.gmap.groupMap(locs(1,1), locs(1,2));
            end
        end
        
        function locs = getSilibingTargetLocs(obj)
            i = obj.ci;
            m_id = sort(obj.robotGp(i).modules.getIDs());
            c_tar_is = obj.ext.tarTree.getchildren(obj.robotGp(i).c_tar_i);
            if length(c_tar_is) == 1
                locs = [];
            else
                for j=1:2
                    t = obj.ext.getTargetByIdx(c_tar_is(j));
                    t_id = sort(t.getDisplayIDs());
                    if length(t_id) == length(m_id) && all(m_id == t_id)
                        s = obj.ext.getTargetByIdx(c_tar_is(3-i));
                        s_id = s.getIDs();
                    end
                end
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
            obj.robotGp(i).status = false;
            obj.robotGp(i).meetingList = [];
        end
        
        function decision = isTargetCompleted(obj, locs)
            if nargin == 1
                locs = obj.getSilibingTargetLocs();
            end
            [n, ~] = size(locs);
            for j = 1:n
                loc = locs(j, 1:2);
                if ~obj.gmap.groupMap(loc(1), loc(2))
                    decision = false;
                    return
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
            is_occupied = false;
            i = obj.ci;
            tar = obj.ext.getTargetByIdx(obj.robotGp(i).c_tar_i);
            t_locs = tar.getLocs();
            r_loc = obj.robotGp(i).LeadRobot.Location;
            for j=1:tar.Size
                if sum((t_locs(j,1:2)-r_loc(1:2)).^2) < obj.gmap.robotDist
                    observed = true;
                    groupid = obj.gmap.groupMap(t_locs(j,1), t_locs(j,2));
                    if groupid && obj.robotGp(i).status
                        g_size = obj.robotGp(groupid).modules.Size;
                        if g_size == tar.Size
                            is_occupied = true;
                            obj.robotRecord(i, groupid);
                            return
                        else
                            % set ignore place, update target location 
                            [r, c] = find(obj.gmap.groupMap==groupid);
                            locs = [r, c];
                            obj.robotGp(i).LeadRobot.setIgnorePos(locs);
                            % check target position
                            r_goal = obj.robotGp(i).LeadRobot.Goal;
                            m_goal = r_goal(1:2) - obj.robotGp(i).attachdir;
                            if obj.gmap.groupMap(m_goal(1), m_goal(2)) == groupid
                                is_occupied = true;
                            else
                                is_occupied = false;
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
%                     cl = length(obj.ext.GroupLayers);
%                     obj.robotGp(i).cl = cl;
                    obj.robotGp(i).ext = obj.ext;
                end
                obj.robotGp(i).initSearch();
            end
%             obj.fetch_arrive = false(1, l);
%             obj.target_arrive = false(1, l);
            obj.structure_arrive = false(1, l);
%             obj.structure_arrive = false;
            obj.finish = false(1, l);
%             obj.rob_dirs = zeros(l, 2);
            obj.N_rob = l;
            if ~isempty(obj.ext)
                [obj.con_order, obj.sub_root] = ...
                    obj.ext.genConstructOrder(obj.N_rob);
                for i=1:obj.N_rob
                    obj.robotGp(i).c_tar_idx = [1, 1];
                    obj.robotGp(i).c_tar_i = obj.con_order(1, 1);
                    obj.robotGp(i).c_tar_root = obj.sub_root(1);
                end
            end
        end
        
        function [m, idx] = getModule(obj, id, l, r)
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
%             mid = int32((r-l)/2+l);
%             if obj.mods(mid).ID == id
%                 m = obj.mods(mid);
%                 idx = mid;
%                 return
%             elseif r-l == 1
%                 if obj.mods(r).ID == id
%                     idx = r;
%                 elseif obj.mods(l).ID == id
%                     idx = l;
%                 end
%                 m = obj.mods(idx);
%                 return
%             elseif obj.mods(mid).ID > id
%                 r = mid;
%             else
%                 l = mid;
%             end
%             [m, idx] = obj.getModule(id, l, r);
        end
        
        function ms = getAllMods(obj)
            ms = obj.mods;
            for i=1:obj.N_rob
                if ~isempty(obj.robotGp(i).attachModuleID)
                    ms = [ms, obj.robotGp(i).modules.ModuleList.'];
                end
            end
            for i=1:length(obj.floatGp)
                ms = [ms, obj.floatGp.ModuleList.'];
            end
            for i=1:length(obj.fixed_mods)
                ms = [ms, obj.fixed_mods.ModuleList.'];
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
    end
end

