classdef Extension < handle
    %EXTENSION Target extension
    %   此处显示详细说明
    
    properties
        finTar targetGroup   % Object of final targets
        tarTree tree         % Object of extension tree
        center (1, 2) int32  % Extension center
        BFSit  int32         % Breadth first search iterator of tarTree
        Size                 % Number of targets
    end
    
    methods (Access = public)
        function obj = Extension(tars)
            %EXTENSION 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                obj.finTar = targetGroup();
                obj.Size = 0;
            else
                obj.finTar = tars;
                obj.Size = tars.Size;
            end
            addpath('display');
        end
        
        function t = TargetToTree(obj, c)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
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
        
        function showTree(obj)
            plot(obj.tarTree);
        end
        
        function showExtension(obj, display, pause_t)
            if nargin == 1
                display = display2D([15, 15], "FinalTarget", obj.finTar);
                pause_t = 1;
            elseif nargin == 2
                pause_t = 1;
            end
            
            % is_fin = false;
            i = 1;
            L = length(obj.BFSit);
            while i < L
                cur_node = obj.tarTree.get(obj.BFSit(i));
                new_node = copy(cur_node);
                while new_node.Size < obj.Size
                    i = i+1;
                    cur_node = obj.tarTree.get(obj.BFSit(i));
                    new_node = new_node.AddTargetGp(cur_node);
                end
                i = i+1;
                display.updateMap("CurrentTarget", new_node);
                pause(pause_t);
            %     if i >= Size
            %         is_fin = true;
            %     end
            end
        end
        
        function example(obj)
            % Example of the target expansion procedure
            % Initialization of a square of targets
            % Rect
            obj.Size = 9; a = 3; b = 3;  % Num of blocks; width; length;

            Tars = [];
            for i = 1:obj.Size
                m = ceil(i/b);
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
                [tar_child(1), tar_child(2)] = cur_root.TargetSplitting(option, obj.center);
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

