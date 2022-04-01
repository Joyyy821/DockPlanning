classdef AssembleGroup < handle
    %ASSEMBLEGROUP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        groupID            int32
        LeadRobot          robot
        modules            moduleGroup
        attachModuleID     int32
        attachdir          int32
        GlobalMap          map       % "pointer" to a global map object
        dockFlag           logical
        dockGpID           int32
        cl                 int32     % current layer in the extension tree
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
%             obj.d_search = [1, 1, 2];
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
            long_dirs = [1, 0; -1, 0; 0, 1; 0, -1];
            del_i = [];
            for i=1:4
                if ~obj.isLocInMap(loc+long_dirs(i,:))
                    del_i = [del_i, i];
                end
            end
            long_dirs(del_i, :) = [];
            [l_n, ~] = size(long_dirs);
            ld_idx = randi(l_n);
            ld = long_dirs(ld_idx, :);
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
        
        function p = genPath(obj, ld, sd, loc)
            cur_loc = loc; cur_d = ld;
            p = [];
            mapsize = obj.GlobalMap.mapSize;
            while true
                for i=1:2
                    if cur_d(i) == 1
                        len = mapsize(i) - cur_loc(i);
                        temp_p = ones(len, 2) * cur_loc(3-i);
                        temp_p(:, i) = (cur_loc(i)+1):mapsize(i);
                    elseif cur_d(i) == -1
                        len = cur_loc(i)-1;
                        temp_p = ones(len, 2) * cur_loc(3-i);
                        temp_p(:, i) = flip(1:(cur_loc(i)-1));
                    end
                end
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
            if ~isempty(m_loc)
                robdir = [1, 0];
                % Set goal according to the attach dir and module location
                obj.LeadRobot.Goal = [m_loc(1, 1:2)+robdir, 0];
                obj.attachdir = robdir;
                % Set the ignore pos
                obj.LeadRobot.setIgnorePos(m_loc(1, 1:2));
            else
                obj.LeadRobot.setIgnorePos([]);
                % Step 2: set searching goal
                c_idx = obj.search_var(1);
                if all(obj.LeadRobot.Location(1:2) ==...
                        obj.LeadRobot.Goal(1:2)) || ~c_idx ||...
                        (obj.search_var(2) > 2 && ...
                        isempty(find(obj.search_var(3:4)==c_idx, 1)))
                    [n, ~] = size(obj.searchPath);
                    obj.search_var(1) = c_idx+1;
                    if obj.search_var(1) > n
                        obj.initSearch();
                        obj.search_var(1) = 1;
                    end
                    obj.search_var(2) = 0;
                    obj.LeadRobot.Goal(1:2) = obj.searchPath(...
                        obj.search_var(1), :);
                end
            end
            % Step 3: move the robot and check goal
            is_arrive = obj.move();
            if obj.LeadRobot.pauseCmd
                obj.search_var(2) = obj.search_var(2) + 1;
            end
            if (~is_arrive) && all(obj.LeadRobot.Location == obj.LeadRobot.Goal)
                is_arrive = true;
            end
            if is_arrive && (~isempty(m_loc))
                id_m = m_loc(1, 3);
                return
            end
        end
        
%         function id_m = search(obj)
%             id_m = [];
%             loc = obj.LeadRobot.Location;
%             % Step 1: update map to see if float module has been found
%             [~, m_loc] = obj.GlobalMap.getMap(loc, "search");
%             if ~isempty(m_loc)
%                 % TODO
%                 % From id find possible attach dir
% %                 if m_loc(1, 3) == 1 || m_loc(1, 3) == 3
% %                     robdir = [0, -1];
% %                 elseif m_loc(1, 3) == 2
% %                     robdir = [-1, 0];
% %                 else
% %                     robdir = [1, 0];
% %                 end
%                 robdir = [1, 0];
%                 % Set goal according to the attach dir and module location
%                 obj.LeadRobot.Goal = [m_loc(1, 1:2)+robdir, 0];
%                 obj.attachdir = robdir;
%                 % Set the ignore pos
%                 obj.LeadRobot.setIgnorePos(m_loc(1, 1:2));
%             else
%                 obj.LeadRobot.setIgnorePos([]);
%                 % Step 2: set searching goal
%                 if obj.d_search(3) == 2 && obj.d_search(2) == 1
%                     d_max = obj.GlobalMap.mapSize(2);
%                     if loc(2) < d_max
%                         obj.LeadRobot.Goal(1:2) = [loc(1), d_max];
%                     else
%                         obj.d_search(2:3) = [-1, 1];
%                     end
%                 end
%                 if obj.d_search(3) == 2 && obj.d_search(2) == -1
%                     if loc(2) > 1
%                         obj.LeadRobot.Goal(1:2) = [loc(1), 1];
%                     else
%                         obj.d_search(2:3) = [1, 1];
%                     end
%                 end
%                 if obj.d_search(3) == 1 && obj.d_search(1) == 1
%                     d_max = obj.GlobalMap.mapSize(1);
%                     if loc(1) < d_max
%                         obj.LeadRobot.Goal(1:2) = ...
%                             [loc(1)+obj.d_search(1), loc(2)];
%                     else
%                         obj.d_search(1) = -1;
%                     end
%                 end
%                 if obj.d_search(3) == 1 && obj.d_search(1) == -1
%                     if loc(1) > 1
%                         obj.LeadRobot.Goal(1:2) = ...
%                             [loc(1)+obj.d_search(1), loc(2)];
%                     else
%                         obj.d_search(1) = 1;
%                         obj.LeadRobot.Goal(1:2) = ...
%                             [loc(1)+obj.d_search(1), loc(2)];
%                     end
%                 end
%             end
%             % Step 3: move the robot and check goal
%             is_arrive = obj.move();
%             if (~is_arrive) && all(obj.LeadRobot.Location == obj.LeadRobot.Goal)
%                 is_arrive = true;
%             end
%             if is_arrive && (~isempty(m_loc))
%                 id_m = m_loc(1, 3);
%                 return
%             end
%             if is_arrive && obj.d_search(3) == 1
%                 obj.d_search(3) = 2;
%             end
%         end
        
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
            obj.GlobalMap.groupMap(rob_loc(1), rob_loc(2)) = val;
            if obj.LeadRobot.isCarrying
                for i=1:obj.modules.Size
                    mod_loc = obj.modules.ModuleList(i).Location;
                    obj.GlobalMap.groupMap(mod_loc(1), mod_loc(2)) = val;
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
                % find attach pos
                loc = obj.LeadRobot.Location;
                dirs = [1, 0; -1, 0; 0, 1; 0, -1];
                for i=1:4
                    m_pos = loc(1:2) + dirs(i, :);
                    if ~obj.isLocInMap(m_pos)
                        continue
                    end
                    m_id = obj.GlobalMap.moduleMap(loc(1)+dirs(i,1), ...
                            loc(2)+dirs(i,2));
                    if m_id
                        obj.attachModuleID = [obj.attachModuleID, m_id];
                    end
                end
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
        end
    end
end

