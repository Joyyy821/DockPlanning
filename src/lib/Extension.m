classdef Extension < handle
    %EXTENSION Target extension
    %   此处显示详细说明
    
    properties
        finTar targetGroup   % Object of final targets
        tarTree tree         % Object of extension tree
        depthTree tree       % 查询节点深度（用于检查一个节点是否在倒数第二层）
        depth                % tarTree depth
        curTree tree         % target tree modified during construction ordering
        cur_dt  tree         % depth tree
        curBFSit int32
        curIdxT tree         % Index tree map to the original index
%         curIdxTwLeaf  tree
        center (1, 2) double % Extension center
        BFSit  int32         % Breadth first search iterator of tarTree
        Size                 % Number of targets
        extL                 % Extension length
        GroupLayers    targetGroup   % Each group represents a layer in the tree
        Nnode % Number of node up to the i_th layer. Length equals to the depth of the extension tree
        GlobalMap          map       % "pointer" to a global map object
    end
    
    methods (Access = public)
        function obj = Extension(tars, gmap)
            %EXTENSION 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                obj.finTar = targetGroup();
                obj.Size = 0;
            elseif nargin == 1
                obj.finTar = tars;
                obj.Size = tars.Size;
            else
                obj.finTar = tars;
                obj.Size = tars.Size;
                obj.GlobalMap = gmap;
%                 obj.setStructureMap();
            end
            obj.extL = 1;
            
        end
        
        %% Class attributes enquiry
        
        function decision = isTarStatic(obj, options)
            arguments
                obj              Extension
                options.index    int32
                options.location int32
            end
            decision = false;
            if isfield(options, "location")
                loc = options.location;
                bound = [loc; loc];
                [locs, n] = obj.nearLocs(bound);
                for i=1:n
                    if obj.GlobalMap.dockerMap(locs(i,1), locs(i,2))
                        decision = true;
                        break;
                    end
                end
            end
        end
        
        function flag = isTargetPair(obj, ID)
            % 根据target在assembly tree中的index判断是否为倒数第二层的节点
            d = obj.depthTree.get(ID);
            if obj.depth - d == 1
                flag = true;
            else
                flag = false;
            end
        end

        function t = getTargetByIdx(obj, id)
            t = obj.tarTree.get(id);
        end

        function [locs, n] = nearLocs(obj, bound)
            % Rectangular bound [xmin, ymin; xmax, ymax].
            locs = zeros(1, 2);
            x1 = bound(1, 1) - 1; x2 = bound(2, 1) + 1;
            y1 = bound(1, 2) - 1; y2 = bound(2, 2) + 1;
            Nx = bound(2, 1) - bound(1, 1) + 1;
            Ny = bound(2, 2) - bound(1, 2) + 1;
            n = 0;
%             while i < (2*Nx+2*Ny)
            if x1 >= 1
                for j = (n+1):(n+Ny)
                    locs(j, :) = [x1, j-n+y1];
                end
                n = n + Ny;
            end
            if x2 <= obj.GlobalMap.mapSize(1)
                for j = (n+1):(n+Ny)
                    locs(j, :) = [x2, j-n+y1];
                end
                n = n + Ny;
            end
            if y1 >= 1
                for j = (n+1):(n+Nx)
                    locs(j, :) = [j-n+x1, y1];
                end
                n = n + Nx;
            end
            if y2 <= obj.GlobalMap.mapSize(2)
                for j = (n+1):(n+Nx)
                    locs(j, :) = [j-n+x1, y2];
                end
                n = n + Nx;
            end
%             end
        end
        
        function locs = getLocations(obj, ids, layer)
            if isempty(ids)
                locs = [];
                return
            end
            tree_idx = obj.getTreeIdx(ids, layer);
            tar = obj.tarTree.get(tree_idx);
            node_locs = tar.getLocs();
            node_ids = tar.getIDs();
            n = 0;
            locs = zeros(length(ids), 2);
            for i=1:tar.Size
                if ~isempty(find(ids==node_ids(i), 1))
                    n = n + 1;
                    locs(n, :) = node_locs(i, :);
                end
            end
        end
        
        function tree_id = getTreeIdx(obj, ids, layer)
            if layer == 1
                % TODO: verify this relation
                tree_id = 1;
                return
            end
            for i=obj.Nnode(layer-1)+1:obj.Nnode(layer)
                temp_id = obj.BFSit(i);
                targets = obj.tarTree.get(temp_id);
                disp("ids: "); disp(ids);
                disp("target ids: "); disp(targets.getIDs);
                if all(ismember(ids, targets.getIDs))
                    tree_id = temp_id;
                end
            end
        end
        
        function [s_ids, s_locs] = getSilibing(obj, ids, layer)
            s_ids = [];
            if layer == 1
                return
            end
%             N_before = sum(obj.Nnode(layer-1:1));
            tree_id = obj.getTreeIdx(ids, layer);
            siblings = obj.tarTree.getsiblings(tree_id);
            for s=siblings
                if s ~= tree_id
                    s_tar = obj.tarTree.get(s);
                    s_ids = s_tar.getIDs();
                    s_locs = s_tar.getLocs();
                end
            end
        end
        
        %% Setters
        
        function setTargetMap(obj)
            tlst = obj.GroupLayers(end).TargetList;
            for i=1:obj.Size
                loc = tlst(i).Location;
                obj.GlobalMap.targetMap(loc(1), loc(2)) = tlst(i).ID;
            end
        end
        
        function markBankTargets(obj)
            % TODO: read the obstacle map and tranverse through the
            % extension tree, mark all target group that are near the bank.
            for i=obj.BFSit
                node = obj.tarTree.get(i);
                bound = node.Boundary;
                [locs, n] = obj.nearLocs(bound);
%                 [n, ~] = size(locs);
                for j=1:n
                    if obj.GlobalMap.dockerMap(locs(j,1), locs(j,2))
                        node.is2static = true;
                        break;
                    end
                end
            end
        end
        
        %% Display & Example
        
        function showTree(obj, options)
            % options: target (final extension tree); current (current
            % extension tree during the construction order generation.
            if nargin == 1
                options = "target";
            end
            if options == "target"
                t = obj.tarTree;
            elseif options == "current"
                t = obj.curTree;
            else
                error("Wrong input of show tree options!");
            end
            figure("Name", "Extension Tree"); plot(t);
            figure("Name", "Index Tree"); 
            if options == "target"
                [~, index] = t.subtree(1);
                plot(index);
            elseif options == "current"
                plot(obj.curIdxT);
            end
        end
        
        function showExtension(obj, display, pause_t)
            if nargin == 1
                mapsize = obj.GlobalMap.mapSize;
                display = display2D(mapsize, "FinalTarget", obj.finTar);
                pause_t = 0;
            elseif nargin == 2
                pause_t = 0;
            end
            
            % is_fin = false;
            i = 1;
            c_layer = 1;
%             L = length(obj.BFSit);
            L = length(obj.BFSit) - obj.finTar.Size;
            while i < L
                cur_node = obj.tarTree.get(obj.BFSit(i));
                new_node = copy(cur_node);
                while new_node.Size < obj.Size
                    i = i+1;
                    cur_node = obj.tarTree.get(obj.BFSit(i));
                    new_node = new_node.AddTargetGp(cur_node);
                end
                obj.GroupLayers = [obj.GroupLayers, new_node];
%                 if i == 1
%                     obj.Nnode(c_layer) = i;
%                 else
%                     obj.Nnode(c_layer) = i - sum(obj.Nnode(c_layer-1:1));
%                 end
                obj.Nnode(c_layer) = i;
                i = i+1;
                c_layer = c_layer + 1;
                display.updateMap("CurrentTarget", new_node);
                pause(pause_t);
            %     if i >= Size
            %         is_fin = true;
            %     end
            end
            obj.setTargetMap();
%             display.updateMap("CurrentTarget", new_node);
        end
        
        function example(obj)
%             addpath('display');
            % Example of the target expansion procedure
            % Initialization of a square of targets
            % Rect
            obj.Size = 9; a = 3; b = 3;  % Num of blocks; width; length;

            Tars = [];
            for i = 1:obj.Size
                m = ceil(i/a);
                n = i - (m-1) * b;
                Tars = [Tars; targetPoint(i, [m, n])];
            end

            obj.finTar = targetGroup(Tars);
            
            % Fit tree
            obj.TargetToTree([1, 1]);
            
            % plot
            obj.showTree();
            obj.showExtension();
        end
        
        %% Construction procedure
        
        function assignID(obj, idx, id, tar_opt)
            tar = obj.getTargetByIdx(idx);
            if tar.Size > 2
                error("Invoke assignID failed because the assigned target"+ ...
                    " is not located at a leaf node.");
            end
            childs = obj.tarTree.getchildren(idx);
            child = childs(tar_opt);
%             for i=1:tar.Size
%                 t_id = tar.TargetList(i).ID;
%                 if all(tar.TargetList(i).Location == loc)
%                     break
%                 end
%             end
            tar = obj.getTargetByIdx(child);
            t_id = tar.TargetList(1).ID;
            
            c_i = child;
            while true
                success = tar.setDisplayID(t_id, id);
                % set docker point index
                if success && tar.is2static
                    loc = tar.getTarLoc(t_id);
                    if obj.isTarStatic("location", loc)
                        tar.dockerPoints = [tar.dockerPoints, id];
                    end
                end
                obj.tarTree.set(c_i, tar);
                if c_i == 1
                    break
                end
                c_i = obj.tarTree.getparent(c_i);
                tar = obj.getTargetByIdx(c_i);
            end
        end
        
        function dock_ids = extractOnce(obj, modules, layer, display)
            % Input: module group object, the group's current layer, the UI
            % to display
            % Output: the module ids which the input group will dock with
            % at the parent tree layer
            
            % Step 1: find the parent node and update current target
            % accordingly
            N = modules.Size;
            ids = zeros(1, N);
            for i=1:N
                ids(i) = modules.ModuleList(i).ID;
            end
            next_targets = obj.GroupLayers(layer-1).TargetList;
            locs = zeros(obj.Size, 2);
            for i=1:obj.Size
                t = next_targets(i);
                if ~isempty(find(ids==t.ID, 1))
                    locs(t.ID, :) = t.Location;
                    % Any problem if there are switching targets?
                    [row, col] = find(obj.GlobalMap.targetMap==t.ID);
                    obj.GlobalMap.targetMap(row, col) = 0;
                    obj.GlobalMap.targetMap(t.Location(1), t.Location(2)) = t.ID;
                end
            end
            display.updateMap("PartialTargets", locs);
            % Step 2: find the modules to dock with at the next step.
            [dock_ids, ~] = obj.getSilibing(ids, layer);
        end
        
        %% Construction order
        function [order, sub_root] = genConstructOrder(obj, N_rob)
            % TODO: tranverse the second last layer of the extension tree,
            % determine each subtree size, and update the near-static-
            % strucuture targets.
            order = []; sub_root = [];
            [obj.curTree, obj.curIdxT] = obj.tarTree.subtree(1);
            obj.cur_dt = obj.curTree.depthtree();
            obj.curBFSit = obj.curTree.breadthfirstiterator(false);
            obj.markBankTargets();
%             pair_idx = obj.BFSit(obj.Nnode(end-2)+1:obj.Nnode(end-1));
%             [sub_ord, parents] = obj.genOrderOnce(pair_idx, N_rob);
%             [nr, nc] = size(sub_ord);
%             order(1:nr, 1:nc) = sub_ord;
%             row = nr;
            row = 0; parents = [];
            while isempty(parents) || any(parents ~= 1)
                if isempty(parents)
                    pair_idx = obj.updateCurTree();
                else
                    pair_idx = obj.updateCurTree(parents);
                end
%                 disp("pair idx");
%                 disp(pair_idx);
                [sub_ord, parents] = obj.genOrderOnce(pair_idx, N_rob);
                [nr, nc] = size(sub_ord);
                order(row+1:row+nr, 1:nc) = sub_ord;
                sr = obj.curIdxT.get(parents);
                if length(sr) > 1
                    sr = cell2mat(sr);
                end
                sub_root(row+1:row+nr) = sr;
                row = row + nr;
%                 disp("parents");
%                 disp(parents);
            end
%             sub_root = sub_root.';
        end
        
        function pair_idx = findPTidx(obj, t)
            % use curTree
            if nargin == 1
                d = obj.curTree.depth();
                pair_idx = find(obj.cur_dt==d-1);
                Nn = length(pair_idx);
                Nl = length(obj.curTree.findleaves());
                pair_idx = obj.curBFSit(end-Nn-Nl+1:end-Nl);
            else
                d = t.depth();
                dt = t.depthtree();
                pair_idx = find(dt==d-1);
                Nn = length(pair_idx);
                Nl = length(t.findleaves());
                BFS_it = t.breadthfirstiterator(false);
                pair_idx = BFS_it(end-Nn-Nl+1:end-Nl);
            end
        end
        
        function pair_idx = updateCurTree(obj, parents)
            if nargin == 1
                pair_idx = obj.chopLeave();
%                 pair_idx = obj.findPTidx();
                return
            end
            % curTree, curIdxT, curBFSit, cur_st
            temp = obj.curTree;
            j = 1; Np = length(parents);
            while j <= Np
                p = parents(j);
                cd = temp.getchildren(p);
                if isempty(cd)
                    j = j + 1;
                    continue
                end
                [~, index] = temp.subtree(p);
                if length(cd) >= 1
                    temp = temp.chop(cd(1));
                    obj.curIdxT = obj.curIdxT.chop(cd(1));
                end
                if length(cd) == 2
                    cd = temp.getchildren(p);  % update the child index
                    temp = temp.chop(cd);
                    obj.curIdxT = obj.curIdxT.chop(cd);
                end
                ds = index.depthfirstiterator();
%                 disp(ds);
                ds(1) = [];
                ds = index.get(ds);
%                 disp(ds);
                if length(ds) > 1
                    ds = cell2mat(ds);
                end
%                 disp("ds"); disp(ds);
%                 disp(isempty(ds));
%                 if ~isempty(ds)
                for i = 1:length(parents)
                    op = parents(i);
                    parents(i) = op - length(find(ds<op));
                end
%                 end
                j = j + 1;
%                 parents(1) = [];
%                 for c = chds
%                     [st, index] = obj.curTree(p);
%                     temp = temp.chop(c);
%                 end
            end
            obj.curTree = obj.getBalanceTree(temp);
            obj.curIdxT = obj.getBalanceTree(obj.curIdxT);
%             obj.showTree("current");
            pair_idx = obj.chopLeave();
            obj.cur_dt = obj.curTree.depthtree();
            obj.curBFSit = obj.curTree.breadthfirstiterator(false);
%             pair_idx = obj.findPTidx();
        end
        
        function [sub_ord, parents] = genOrderOnce(obj, pair_idx, N_rob)
            sub_ord = zeros(1, 1);
            parents = [];
            Nn = length(pair_idx);
%             Nl = length(obj.curTree.findleaves());
%             pair_idx = obj.curBFSit(end-Nn-Nl+1:end-Nl);
            static_cnt = 0;
            i = 1; sub_i = 1; row = 1;
            while true
                if i > Nn
                    if sub_i > 1
%                         sub_ord(row, 1:sub_i-1) = pair_idx(i-sub_i+1:end);% end = i-1;
                        len = 0;
                        while len < sub_i - 1
                            [cp, l] = obj.findSubtree(i, sub_i, pair_idx);
                            parents = [parents, cp];
%                             col = 1;
                            for j=1:l
                                cpi = pair_idx(i-sub_i+j);
% %                                 tp = obj.curIdxT.get(cpi);
%                                 child_idx = obj.curIdxT.getchildren(cpi);
%                                 cl = length(child_idx);
%                                 if cl == 1
%                                     sub_ord(row, col) = obj.curIdxT.get(child_idx);
%                                 elseif cl == 2
%                                     child = cell2mat(obj.curIdxT.get(child_idx));
%                                     sub_ord(row, col:col+1) = child;
%                                 end
%                                 col = col + cl;
                                sub_ord(row, j) = obj.curIdxT.get(cpi);
                            end
%                             sub_ord(row, 1:l) = pair_idx(i-sub_i+1:(i-sub_i+l));
                            len = len + l;
                            sub_i = sub_i - l;
                            row = row + 1;
                        end
                    end
                    break
                end
                cnode = obj.curTree.get(pair_idx(i));
                if cnode.is2static
                    static_cnt = static_cnt + 1;
                end
                if sub_i > (N_rob + static_cnt - 1)
%                     isn = obj.curTree.get(pair_idx(i-sub_i));
%                     ien = obj.curTree.get(pair_idx(i-1));
                    [cp, len] = obj.findSubtree(i, sub_i, pair_idx);
                    parents = [parents, cp];
%                     col = 1;
                    for j=1:len
                        cpi = pair_idx(i-sub_i+j);
                        sub_ord(row, j) = obj.curIdxT.get(cpi);
% %                         tp = obj.curIdxT.get(cpi);
%                         child_idx = obj.curIdxT.getchildren(cpi);
%                         l = length(child_idx);
%                         if l == 1
%                             sub_ord(row, col) = obj.curIdxT.get(child_idx);
%                         elseif l == 2
%                             child = cell2mat(obj.curIdxT.get(child_idx));
%                             sub_ord(row, col:col+1) = child;
%                         end
%                         col = col + l;
                    end
%                     sub_ord(row, 1:len) = pair_idx(i-sub_i+1:(i-sub_i+len));
                    i = i-sub_i+len+1;
                    static_cnt = 0;
                    sub_i = 1;
                    row = row + 1;
                    continue;
                end
                i = i + 1;
                sub_i = sub_i + 1;
            end
        end
        
        function [cp, len] = findSubtree(obj, i, sub_i, pair_idx)
            isn = pair_idx(i-sub_i+1);
            ien = pair_idx(i-1);
            p = obj.curTree.findpath(isn, ien);
            d = obj.cur_dt.get(p);
            if length(d) > 1
                d = cell2mat(d);
            end
            [~, icp] = min(d);  % common parent index
            cp = p(icp);
            while true
                [st, st_idx] = obj.curTree.subtree(cp);
%                 sl = obj.findPTidx(st);
                sl = st.findleaves();
                sls = st_idx.get(sl(1)); sle = st_idx.get(sl(end));
                len = length(sl);
                i_sls = find(pair_idx == sls);
                i_sle = find(pair_idx == sle);
                if i-1 >= i_sle && i-sub_i+1 == i_sls
                    break
                elseif i-1 < i_sle
                    cp = obj.curTree.getchildren(cp);
                    if length(cp) > 1
                        cp = cp(1);
                    end
%                             st = obj.curTree.subtree(cp);
%                             len = length(st.findleaves());
                else
                    cp = obj.curTree.getchildren(cp);
                    if length(cp) > 1
                        cp = cp(2);
                    end
                end
            end
        end
        
        function newleave = chopLeave(obj)
            lIDs = obj.curTree.findleaves();
%             disp("leave: ");
%             disp(lIDs);
            if isempty(lIDs)
                error("Attempt to chop leaf from an empty current tree!");
            end
            tempID = lIDs;
%             obj.curIdxTwLeaf = obj.curIdxT;
            while ~isempty(tempID)
                id = tempID(1);
                obj.curTree = obj.curTree.chop(id);
                obj.curIdxT = obj.curIdxT.chop(id);
                for i=2:length(tempID)
                    node = tempID(i);
                    if node > id
                        node = node - 1;
                    end
                    tempID(i) = node;
                end
                tempID(1) = [];
            end
            newleave = obj.curTree.findleaves();
            obj.cur_dt = obj.curTree.depthtree();
            obj.curBFSit = obj.curTree.breadthfirstiterator(false);
        end
        
        %% Tree generation
        
        function t = TargetToTree(obj, c, exl)
            %METHOD TargetToTree
            %   Extension procedure
            if nargin == 3
                obj.extL = exl;
            end
            t = tree(obj.finTar);
            obj.center = c;
            t = obj.TreeSplitting(obj.finTar, t, 1, 0);
            t = obj.getBalanceTree(t);
%             d = t.depth();
            obj.cur_dt = t.depthtree();
%             IDs = t.findleaves();
%             for i=IDs
%                 cur_d = obj.cur_dt.get(i);
%                 c_i = i;
%                 while cur_d < d
%                     [t, c_i] = t.addnode(c_i, t.get(c_i));
%                     cur_d = cur_d + 1;
%                 end
%             end
            obj.tarTree = t;
            % Extension BFS
            obj.BFSit = obj.tarTree.breadthfirstiterator(false);
            obj.depthTree = obj.tarTree.depthtree;
            obj.depth = obj.tarTree.depth;
        end
        
        function t = getBalanceTree(~, prev_t)
            d = prev_t.depth();
            dt = prev_t.depthtree();
            IDs = prev_t.findleaves();
            for i=IDs
                cur_d = dt.get(i);
                c_i = i;
                while cur_d < d
                    [prev_t, c_i] = prev_t.addnode(c_i, prev_t.get(c_i));
                    cur_d = cur_d + 1;
                end
            end
            t = prev_t;
        end

    end
    
    methods (Access = private)
        % option = 0 means cut vertically
        % option = 1 means cut horizontally
        function tree = TreeSplitting(obj, cur_root, tree, node_idx, option)
            if cur_root.Size == 1
            else
                tar_child(2) = targetGroup();
                [tar_child(1), tar_child(2)] = cur_root.TargetSplitting(option, obj.center, obj.extL);
                id = [];
                [tree, id(1)] = tree.addnode(node_idx, tar_child(1));
                [tree, id(2)] = tree.addnode(node_idx, tar_child(2));
                change_options = [tar_child(1).CanBeSplit(); tar_child(2).CanBeSplit()];

                for i = 1:2
                    if all(change_options(i, :))
                        tree = obj.TreeSplitting(tar_child(i), tree, id(i), ~option);
                    elseif change_options(i, 1)
                        tree = obj.TreeSplitting(tar_child(i), tree, id(i), 0);
                    elseif change_options(i, 2)
                        tree = obj.TreeSplitting(tar_child(i), tree, id(i), 1);
                    end
                end
            end
        end
        
        
    end
end

