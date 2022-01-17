clc; clear all; close all;
addpath('display');
%% Initialization of a square of targets
% Rect
N = 9; a = 3; b = 3;  % Num of blocks; width; length;

Tars = [];
for i = 1:N
    m = ceil(i/b);
    n = i - (m-1) * b;
    Tars = [Tars; targetPoint(i, [m, n])];
end

all_tar = targetGroup(Tars);
% dim = all_tar.Boundary;
% tar_tree = tree(all_tar);

%% Fit tree
c = [1, 1];
% tar_tree = TreeSplitting(all_tar, tar_tree, 1, 0, c);
tar_tree = TargetToTree(all_tar, c);
plot(tar_tree);

%% Display the extension result on GUI
% Initialize
% display = display2D([15, 15], all_tar);
root = tar_tree.Node(1);
display = display2D([15, 15], "FinalTarget", root{1});
% display.runGUI2D();
pause(10);
% Extension
it = tar_tree.breadthfirstiterator(false);
% is_fin = false;
i = 1;
while i < length(it)
    cur_node = tar_tree.get(it(i));
    new_node = copy(cur_node);
    while new_node.Size < N
        i = i+1;
        cur_node = tar_tree.get(it(i));
        new_node = new_node.AddTargetGp(cur_node);
    end
    i = i+1;
    display.updateMap("CurrentTarget", new_node);
    pause(1);
%     if i >= N
%         is_fin = true;
%     end
end

%% 
% option = 0 means cut vertically
% option = 1 means cut horizontally
function tree = TreeSplitting(cur_root, tree, node_idx, option, c)
    if cur_root.Size == 1
    else
        tar_child(2) = targetGroup();
        [tar_child(1), tar_child(2)] = cur_root.TargetSplitting(option, c);
        id = [];
        [tree, id(1)] = tree.addnode(node_idx, tar_child(1));
        [tree, id(2)] = tree.addnode(node_idx, tar_child(2));
        change_options = [tar_child(1).CanBeSplit(); tar_child(2).CanBeSplit()];

        for i = 1:2
            if all(change_options(i, :))
                tree = TreeSplitting(tar_child(i), tree, id(i), ~option, c);
            elseif change_options(i, 1)
                tree = TreeSplitting(tar_child(i), tree, id(i), 0, c);
            elseif change_options(i, 2)
                tree = TreeSplitting(tar_child(i), tree, id(i), 1, c);
            end
        end
    end
end

function t = TargetToTree(target, c)
    t = tree(target);
    t = TreeSplitting(target, t, 1, 0, c);
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
end
