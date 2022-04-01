classdef Extension < handle
    %EXTENSION Target extension
    %   此处显示详细说明
    
    properties
        finTar targetGroup   % Object of final targets
        tarTree tree         % Object of extension tree
        center (1, 2) int32  % Extension center
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
                obj.setStructureMap();
            end
            obj.extL = 1;
            
        end
        
        function setStructureMap(obj)
            tlst = obj.finTar.TargetList;
            for i=1:obj.Size
                loc = tlst(i).Location;
                obj.GlobalMap.structureMap(loc(1), loc(2)) = 1;
            end
        end
        
        function setTargetMap(obj)
            tlst = obj.GroupLayers(end).TargetList;
            for i=1:obj.Size
                loc = tlst(i).Location;
                obj.GlobalMap.targetMap(loc(1), loc(2)) = tlst(i).ID;
            end
        end
        
        function t = TargetToTree(obj, c, exl)
            %METHOD TargetToTree
            %   Extension procedure
            if nargin == 3
                obj.extL = exl;
            end
            t = tree(obj.finTar);
            obj.center = c;
            t = obj.TreeSplitting(obj.finTar, t, 1, 0);
            d = t.depth();
            dt = t.depthtree();
            IDs = t.findleaves();
            for i=IDs
                cur_d = dt.get(i);
                c_i = i;
                while cur_d < d
                    [t, c_i] = t.addnode(c_i, t.get(c_i));
                    cur_d = cur_d + 1;
                end
            end
            obj.tarTree = t;
            % Extension BFS
            obj.BFSit = obj.tarTree.breadthfirstiterator(false);
        end
        
%         function dock_ids = getDockPair(obj, level, ids)
%             all_ctars = obj.GroupLayers(level).TargetList;
%             
%         end
        
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
        
        function showTree(obj)
            plot(obj.tarTree);
        end
        
        function showExtension(obj, display, pause_t)
            if nargin == 1
                display = display2D([15, 15], "FinalTarget", obj.finTar);
                pause_t = 0;
            elseif nargin == 2
                pause_t = 0;
            end
            
            % is_fin = false;
            i = 1;
            c_layer = 1;
            L = length(obj.BFSit);
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
%                 display.updateMap("CurrentTarget", new_node);
                pause(pause_t);
            %     if i >= Size
            %         is_fin = true;
            %     end
            end
            obj.setTargetMap();
            display.updateMap("CurrentTarget", new_node);
        end
        
        function example(obj)
            addpath('display');
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

