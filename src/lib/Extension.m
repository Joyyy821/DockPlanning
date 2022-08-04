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

        function t = getTargetByIdx(obj, id)
            t = obj.tarTree.get(id);
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
            tree_id = [];
            for i=obj.Nnode(layer-1)+1:obj.Nnode(layer)
                temp_id = obj.BFSit(i);
                targets = obj.tarTree.get(temp_id);
%                 disp("ids: "); disp(ids);
%                 disp("target ids: "); disp(targets.getDisplayIDs);
                if all(ismember(ids, targets.getDisplayIDs))
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
        
        function showExtension(obj, display, options)
            arguments
                obj             Extension
                display         display2D
                options.Pause   double
                options.Log     Logging
            end
            if isfield(options, "Pause")
                pause_t = options.Pause;
            else
                pause_t = 0;
            end
            if isfield(options, "Log")
                log = options.Log;
            else
                log = [];
            end
            if nargin == 1
                mapsize = obj.GlobalMap.mapSize;
                display = display2D(mapsize, "FinalTarget", obj.finTar);
                pause_t = 0;
%             elseif nargin == 2
%                 pause_t = 0;
            end
            
            % is_fin = false;
            i = 1;
            c_layer = 1;
            L = length(obj.BFSit);
%             L = length(obj.BFSit) - obj.finTar.Size;
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
                if ~isempty(log)
                    log.recordExtension(c_layer);
                end
                pause(pause_t);
            %     if i >= Size
            %         is_fin = true;
            %     end
            end
            obj.setTargetMap();
%             display.updateMap("CurrentTarget", new_node);
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
            t = obj.TreeSplitting(obj.finTar, t, 1, 1);
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

