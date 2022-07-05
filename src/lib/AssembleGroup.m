classdef AssembleGroup < handle
    %ASSEMBLEGROUP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        groupID            int32
        LeadRobot          robot
        modules            moduleGroup
        attachModuleID     int32
        attachdir          int32     % robot location - attached module location
        GlobalMap          map       % "pointer" to a global map object
        status             logical   % 0 = fetch; 1 = construct.
        waiting            logical
        meetingList        int32     % record all the robots that have met
        % dock
        dockFlag           logical
        dockGpID           int32
        changingSite       logical
        % extension
        cl                 int32     % current layer in the extension tree
        ext                Extension
        c_tar_i            int32
        c_tar_idx          int32
        c_tar_root         int32   % current root of the subtree
        % search
        searchPath         int32
        search_var         int32  % [cur search idx, pause counts, start place idxs]
        
    end
    
    methods
        function obj = AssembleGroup(id, rob, gmap)
            %ASSEMBLEGROUP 构造此类的实例
            %   此处显示详细说明
            if nargin >= 1
                obj.groupID = id;
            end
            if nargin >= 2
                obj.LeadRobot = rob;
            end
            if nargin >= 3
                obj.GlobalMap = gmap;
                loc = obj.LeadRobot.Location;
                obj.GlobalMap.groupMap(loc(1), loc(2)) = obj.groupID;
            end
            obj.status = false;
            obj.waiting = false;
            obj.changingSite = false;
        end
        
        function initSearch(obj)
            loc = obj.LeadRobot.Location(1:2);
            [ld, sd] = obj.setSearchDir(loc);
            p1 = obj.genPath(ld, sd, loc);
            [n1, ~] = size(p1);
            ld2 = -ld; sd2 = -sd;
            if ~obj.isLocInMap(loc+ld2) && ~obj.isLocInMap(loc+sd2)
                obj.searchPath = [p1; loc];
            else
                p2 = obj.genPath(ld2, sd2, loc);
                obj.searchPath = [p1; loc; p2; loc];
            end
            [n2, ~] = size(obj.searchPath);
            obj.search_var = [0, 0, n1+1, n2];
        end
        
        function [ld, sd] = setSearchDir(obj, loc)
            % random a direction
%             long_dirs = [1, 0; -1, 0; 0, 1; 0, -1];
%             del_i = [];
%             for i=1:4
%                 if ~obj.isLocInMap(loc+long_dirs(i,:))
%                     del_i = [del_i, i];
%                 end
%             end
%             long_dirs(del_i, :) = [];
%             [l_n, ~] = size(long_dirs);
%             ld_idx = randi(l_n);
%             ld = long_dirs(ld_idx, :);
            ld = [0, 1];
            if ld(1)
                short_dirs = [0, 1; 0, -1];
            elseif ld(2)
                short_dirs = [1, 0; -1, 0];
            end
            if ~obj.isLocInMap(loc+short_dirs(1, :))
                sd = short_dirs(2, :);
            elseif ~obj.isLocInMap(loc+short_dirs(2, :))
                sd = short_dirs(1, :);
            else
                sd_idx = randi(2);
                sd = short_dirs(sd_idx, :);
            end
        end
        
        function decision = isLocInMap(obj, loc)
            if any(loc(1:2) > obj.GlobalMap.mapSize) || ...
                    any(loc(1:2) < 1)
                decision = false;
            else
                decision = true;
            end
        end
        
        function expan_map = getExpandedMap(obj)
            % return a map with obstacle expaned.
            expan_map = obj.GlobalMap.obstacleMap | obj.GlobalMap.dockerMap;
            [allx, ally] = find(expan_map);
            all_dir = [0 1; 1 0; 0 -1; -1 0; 1 1; 1 -1; -1 1; -1 -1];
            for i=1:length(allx)
                for j = 1:8
                    x = allx(i)+all_dir(j, 1);
                    y = ally(i)+all_dir(j, 2);
                    if x <= 0 || y <= 0 || x > obj.GlobalMap.mapSize(1) ...
                            || y > obj.GlobalMap.mapSize(2)
                        continue
                    end
                    expan_map(x, y) = 1;
                end
            end
        end
        
        function p = genPath(obj, ld, sd, loc)
            cur_loc = loc; cur_d = ld;
            p = [];
            mapsize = obj.GlobalMap.mapSize;
            nstep = 2;
            while true
                for i=1:2
                    if cur_d(i) == 1
%                         len = mapsize(i) - cur_loc(i);
                        temp_lst = (cur_loc(i)+1):nstep:mapsize(i);
%                         len = length(temp_lst);
%                         temp_p = ones(len, 2) * cur_loc(3-i);
%                         temp_p(:, i) = temp_lst;
                    elseif cur_d(i) == -1
%                         len = cur_loc(i)-1;
                        temp_lst = flip(1:nstep:(cur_loc(i)-1));
                    end
                end
                len = length(temp_lst);
                temp_p = ones(len, 2) * cur_loc(3-i);
                temp_p(:, i) = temp_lst;
                % Delete the positions with static obstacles (or surround).
                del_i = [];
                exp_map = obj.getExpandedMap();
                for i=1:length(temp_p)
                    cur_p = temp_p(i, :);
                    if exp_map(cur_p(1), cur_p(2))
                        del_i = [del_i, i];
                    end
                end
                temp_p(del_i, :) = [];
                % TODO: have NOT handle the situation when a certain
                % row/col is all occupied by obstacles.
                if ~isempty(temp_p)
                    p = [p; temp_p; temp_p(end, :)+sd];
                else
                    p = [p; cur_loc+sd];
                end
                cur_d = -cur_d;
                cur_loc = p(end, :);
                if ~obj.isLocInMap(p(end, :))
                    p(end, :) = [];
                    break
                end
            end
        end
        
        function id_m = search(obj)
            id_m = [];
            loc = obj.LeadRobot.Location;
            % Step 1: update map to see if float module has been found
            [~, m_loc] = obj.GlobalMap.getMap(loc, "search");
            del_m = [];
            if ~isempty(m_loc)
                [mn, ~] = size(m_loc);
                robgoals = zeros(1, 3);
                cnt = 1;
                for i=1:mn
                    if obj.GlobalMap.structureMap(m_loc(i,1), m_loc(i,2))
                        del_m = [del_m, i];
                        continue
                    end
%                     temp = obj.getAttachDirs(m_loc(i, 1:2), m_loc(i,3));
% TODO: m_loc目前只支持单个浮动模块抓取，改成兼容抓取group
                    temp = obj.getAttachDirs(m_loc(i, 1:2));
                    [temp_n, ~] = size(temp);
                    for j=1:temp_n
%                         robgoals((i-1)*temp_n+j, 1:3) = ...
%                             [m_loc(i, 1:2)+temp(j, :), 0];
                    robgoals(cnt, 1:3) = ...
                            [m_loc(i, 1:2)+temp(j, :), 0];
                    cnt = cnt + 1;
                    end
%                     robgoals = [robgoals; temp];
                end
                % Set goal according to the attach dir and module location
                if ~isempty(find(robgoals,1))
                    obj.LeadRobot.Goal = robgoals;
                end
                % Set the ignore pos
                obj.LeadRobot.setIgnorePos(m_loc(1, 1:2));
            end
            m_loc(del_m, :) = [];
            if isempty(m_loc)
                obj.LeadRobot.setIgnorePos([]);
                % Step 2: set searching goal
                c_idx = obj.search_var(1);
                if ~c_idx || all(obj.LeadRobot.Location(1:2) ==...
                        obj.LeadRobot.Goal(1:2)) || ...
                        (obj.search_var(2) > 2 && ...
                        isempty(find(obj.search_var(3:4)==c_idx, 1)))
                    [n, ~] = size(obj.searchPath);
                    obj.search_var(1) = c_idx+1;
                    if obj.search_var(1) > n
                        obj.initSearch();
                        obj.search_var(1) = 1;
                    end
                    obj.search_var(2) = 0;
                    obj.LeadRobot.Goal = [obj.searchPath(...
                        obj.search_var(1), :), 0];
                end
            end
            % Step 3: move the robot and check goal
            is_arrive = obj.move();
            if obj.LeadRobot.pauseCmd
                obj.search_var(2) = obj.search_var(2) + 1;
            end
            [goal_n, ~] = size(obj.LeadRobot.Goal);
            e = 10e-3;
            if ~is_arrive
                for i=1:goal_n
                    if norm(obj.LeadRobot.Location(1:2) - ...
                            obj.LeadRobot.Goal(i, 1:2)) < e
                        is_arrive = true;
                        obj.LeadRobot.Goal = obj.LeadRobot.Goal(i,:);
                        break
                    end
                end
            end
            if is_arrive && (~isempty(m_loc))
                id_m = m_loc(1, 3);
                return
            end
        end
        
        function locs = getDockSites(obj)
            % find avaliable dock sites from current target and module
            % locations.
            if obj.groupID == 4
                dirs = [1, 0];
                m = obj.modules.getModule(obj.attachModuleID);
                mloc = m.Location(1:2);
                locs = mloc+dirs;
                return
            elseif obj.groupID == 2
                dirs = [-1, 0];
                m = obj.modules.getModule(obj.attachModuleID);
                mloc = m.Location(1:2);
                locs = mloc+dirs;
                return
            end
            tar = obj.ext.getTargetByIdx(obj.c_tar_i);
            locs = obj.getAllSites();
            [n, ~] = size(locs);
            pos_shift = obj.LeadRobot.Goal - obj.LeadRobot.Location;
            pos_shift = pos_shift(1:2);
            del_i = [];
            om = obj.GlobalMap.groupMap | obj.GlobalMap.structureMap | ...
                    obj.GlobalMap.dockerMap | obj.GlobalMap.obstacleMap;
            [r, c] = find(obj.GlobalMap.groupMap==obj.groupID);
            for i=1:length(r)
                om(r(i),c(i)) = 0;
            end
            for i = 1:n
                if tar.isLocInTar(locs(i,:), pos_shift)
                    del_i = [del_i, i];
                    continue
                end
                loc = locs(i,:)+pos_shift;
                
                if om(loc(1), loc(2))
%                     if any(int32(obj.LeadRobot.Location(1:2)) ~= loc)
                    del_i = [del_i, i];
%                     end
                end
            end
            locs(del_i, :) = [];
%             tar_ch = obj.ext.tarTree.getchildren(obj.c_tar_i);
%             if length(tar_ch) == 1
%                 locs = obj.getAllSites();
%             else
%                 for i=1:2
%                     t = tar_ch(i);
%                     for j = 1:t.Size
%                         id = t.TargetList(j).displayID;
%                         if id
%                             m_id = obj.modules.getIDs();
%                             if ~isempty(find(m_id==id, 1))
%                                 break
%                             end
%                         else
%                             break
%                         end
%                     end
%                 end
%                 
%             end
        end
        
        function locs = getAllSites(obj)
            mod_bound = obj.modules.Boundary;
            [locs, ~] = obj.ext.nearLocs(mod_bound);
        end
        
%         function dirs = getAttachDirs(obj, m_loc, m_id)
        function dirs = getAttachDirs(obj, m_loc)
            % TODO: m_loc -> m_bound
%             tar_ch = obj.ext.tarTree.getchildren(obj.c_tar_i);
            targp = obj.ext.tarTree.get(obj.c_tar_i);
            tar_bound = targp.Boundary;
            dock_idx = obj.ext.tarTree.getsiblings(obj.c_tar_i);
            while true
                if dock_idx(1) == obj.c_tar_i && length(dock_idx) == 2
                    dock_idx = dock_idx(2);
                    break
                elseif length(dock_idx) == 2 && dock_idx(2) == obj.c_tar_i
                    dock_idx = dock_idx(1);
                    break
                elseif length(dock_idx) == 1
                    % TODO: if reach the root?
                    p = obj.ext.tarTree.getparent(dock_idx);
                    dock_idx = obj.ext.tarTree.getsiblings(p);
                end
            end
            targp_dock = obj.ext.tarTree.get(dock_idx);
            tar_dock_bound = targp_dock.Boundary;
            
%             tar_loc = obj.ext.getLocations(m_id, obj.cl);
%             [~, tar_attach_loc] = obj.ext.getSilibing(m_id, obj.cl);
            dirs = [1, 0; -1, 0; 0, 1; 0, -1];
%             tar_dir = tar_attach_loc - tar_loc;
            % [delta_xmin, delta_ymin; delta_xmax, delta_ymax];
            tar_dir = tar_dock_bound - tar_bound;
            if all(tar_dir(1, :) > 0)
                dirs(1, :) = [];
            elseif all(tar_dir(1, :) < 0)
                dirs(2, :) = [];
            elseif all(tar_dir(2, :) > 0)
                dirs(3, :) = [];
            elseif all(tar_dir(2, :) < 0)
                dirs(4, :) = [];
            end
            del_i = [];
            for i=1:3
                if ~obj.isLocInMap(m_loc+dirs(i, :))
                    del_i = [del_i, i];
                end
                att_loc = dirs(i,:)+targp.TargetList(1).Location;
                omap = obj.GlobalMap.obstacleMap | obj.GlobalMap.dockerMap;
                if omap(att_loc(1), att_loc(2))
                    del_i = [del_i, i];
                end
            end
            dirs(del_i, :) = [];
        end
        
        function is_arrive = move(obj)
            obj.updateMap("del");
            is_arrive = obj.LeadRobot.move();
            obj.updateMap("add");
        end
        
        function updateMap(obj, options)
            if options == "add"
                val = obj.groupID;
            elseif options == "del"
                val = 0;
            end
            rob_loc = obj.LeadRobot.Location;
            if obj.status
                obj.GlobalMap.workerRobotMap(rob_loc(1), rob_loc(2)) = val;
            end
            obj.GlobalMap.groupMap(rob_loc(1), rob_loc(2)) = val;
            if obj.LeadRobot.isCarrying
                for i=1:obj.modules.Size
                    mod_loc = obj.modules.ModuleList(i).Location;
                    obj.GlobalMap.groupMap(mod_loc(1), mod_loc(2)) = val;
                    if obj.status
                        obj.GlobalMap.workerRobotMap(mod_loc(1), ...
                            mod_loc(2)) = val;
                    end
                end
            end
        end
        
        function updateGroup(obj, options, m)
%             arguments
%                 obj
%                 options.add
%             end
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            % TODO: change to optional argument (add or delete, ect)
            if options == "add"
                if nargin == 3
                    obj.LeadRobot.fetchModule(m);
                end
                val = obj.groupID;
                
            elseif options == "del"
                val = 0;
                obj.attachModuleID = [];
            end
            % update the group map
            for i=1:obj.LeadRobot.carriedModule.Size
                loc = obj.LeadRobot.carriedModule.ModuleList(i).Location;
                obj.GlobalMap.groupMap(loc(1), loc(2)) = val;
            end
            % update the group attributes
            if options == "del"
                obj.LeadRobot.isCarrying = false;
                obj.LeadRobot.carriedModule = moduleGroup();
            end
            obj.modules = obj.LeadRobot.carriedModule;
            if options == "add"
                obj.findAttachment();
            end
        end
        
        function findAttachment(obj)
            % find attach pos
            obj.attachdir = [];
            obj.attachModuleID = [];
            loc = obj.LeadRobot.Location;
            dirs = [1, 0; -1, 0; 0, 1; 0, -1];
            for i=1:4
                m_pos = loc(1:2) + dirs(i, :);
                if ~obj.isLocInMap(m_pos)
                    continue
                end
                m_id = obj.GlobalMap.moduleMap(loc(1)+dirs(i,1), ...
                        loc(2)+dirs(i,2));
                all_mod_ids = obj.modules.getIDs();
                if m_id && ~isempty(find(all_mod_ids==m_id, 1))
                    obj.attachModuleID = [obj.attachModuleID, m_id];
                    obj.attachdir = [obj.attachdir; -dirs(i, :)];
                    obj.modules.ModuleList(1).PosShift = [obj.attachdir, 0];
                end
            end
        end
    end
end

