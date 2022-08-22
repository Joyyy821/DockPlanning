%
% Copyright (c) 2015, Mostapha Kalami Heris & Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "LICENSE" file for license terms.
%
% Project Code: YPEA116
% Project Title: Implementation of Tabu Search for n-Queens Problem
% Publisher: Yarpiz (www.yarpiz.com)
% 

% Developer: Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Cite as:
% Mostapha Kalami Heris, Tabu Search (TS) in MATLAB (URL: https://yarpiz.com/243/ypea116-tabu-search), Yarpiz, 2015.
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

% ts;

%% TS 4-module test
diary("test_ts_4r_5s_v2.txt");
diary on;

rob_loc = [1, 12; 12, 12; 1, 1; 12, 1];
shape_list = ["S", "T", "L", "1", "q"];
disp("-------------------------------------------------");
disp("Tabu search test episodes for 4-robot experiments");
for i=1:3
    runTrial(rob_loc, shape_list, i);
end

diary off;

% %% A test script
% 
% % Initialization with default target and robot settings.
% point = [4,4;5,4;5,5;6,5]; % x, y
% dock = [1,0,0,0;...
%         1,0,1,0;...
%         0,1,0,0;...
%         1,0,1,0
%         ]; % up, down, left, right
% rob_loc = [1, 1; 4, 1; 7, 1; 10, 1];
% myts = TabuSearch(point, dock, rob_loc, 2);
% % Searching
% [sol, dock, cost] = myts.search();
% disp("Assignment: "); disp(sol);
% disp("Dock: "); disp(dock);
% disp("Cost: "); disp(cost);

%% Trial methods

function runTrial(rob_loc, shape_list, type)
    disp("--------------------------");
    disp("Test episode for type: "+string(type));
    
    N = length(shape_list);
    myts = cell(1, N);
    for i=1:N
        disp("--------------------------");
        disp("Trial No. "+string(i)+" - shape type: "+shape_list(i));
        p = getTargets(shape_list(i));
        d = getDocks(shape_list(i), type);
        disp("Target locations: "); disp(p);
        disp("Dock joints: "); disp(d);
        myts{i} = TabuSearch(p, d, rob_loc, type);
%         % Set candLen
%         if type == 1 && i >= 4
%             myts{i}.CandLen = 2;
%         elseif type >= 2
%             myts{i}.CandLen = 10;
%         end
%         if type >= 2 && i <= 3
%             myts{i}.CandLen = 4;
%         elseif type >= 2 && i == 4
%             myts{i}.CandLen = 10;
%         end
%         disp("Array length for chosing candidates: "+string(myts{i}.CandLen));
        % Searching and show search result
        [sol, dock, cost] = myts{i}.search();
        disp("Assignment: "); disp(sol);
        disp("Dock: "); disp(dock);
        disp("Cost: "); disp(cost);
    end
end

function t = getTargets(shape)
    switch shape
        case "S"   % S-shape
            t = [4, 4; 5, 4; 5, 5; 6, 5];
        case "T"   % T-shape
            t = [4, 5; 5, 5; 6, 5; 5, 4];
        case "L"   % L-shape
            t = [4, 4; 4, 5; 5, 5; 6, 5];
        case "1"   % stick (one) - shape
            t = [4, 4; 5, 4; 6, 4; 7, 4];
        case "q"   % squared shape
            t = [4, 5; 5, 5; 4, 4; 5, 4];
    end
end

function d = getDocks(shape, type)
    shape_list = ["S", "T", "L", "1", "q"];
    N = 5;
%     shape_list = ["S", "L", "1", "q"];
    dock = cell(1, N);
    if type <= 2
        dock(1) = {[0, 1, 0, 1; 0, 0, 1, 0; 0, 0, 0, 1; 1, 0, 1, 0]};
        dock(2) = {[0, 1, 1, 1; 0, 0, 1, 0; 0, 0, 0, 1; 1, 0, 0, 0]};
        dock(3) = {[0, 0, 1, 1; 0, 0, 1, 0; 0, 1, 0, 1; 1, 0, 0, 0]};
        dock(4) = {[0, 0, 1, 1; 0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 1, 1]};
        dock(5) = {[0, 1, 0, 1; 1, 0, 0, 1; 0, 0, 1, 0; 0, 0, 1, 0]};
    end
    
    if type == 3
        % assign 2 types of dock joints
        dock(1) = {[0, 1, 0, 1; 0, 0, 2, 0; 0, 0, 0, 1; 2, 0, 2, 0]};
        dock(2) = {[0, 1, 2, 1; 0, 0, 2, 0; 0, 0, 0, 1; 2, 0, 0, 0]};
        dock(3) = {[0, 0, 1, 1; 0, 0, 2, 0; 0, 2, 0, 2; 1, 0, 0, 0]};
        dock(4) = {[0, 0, 2, 1; 0, 0, 2, 0; 0, 0, 0, 1; 0, 0, 2, 1]};
        dock(5) = {[0, 1, 0, 2; 2, 0, 0, 2; 0, 0, 1, 0; 0, 0, 1, 0]};
%         for i=1:N
%             [r, c] = find(dock{i});
%             p = randperm(length(r), length(r)/2);
%             for j=1:length(p)
%                 dock{i}(r(p(j)), c(p(j))) = 2;
%             end
%         end
    end
    if type >= 2
        % random shuttle
        dock = rand_rotate(dock);
%         for i = 1:N
%             for j = 1:N
%                 dock{i} = DoRotation(dock{i}, j
    end
    for i = 1:N
        if shape == shape_list(i)
            d = dock{i};
            return
        end
    end
    error("The input shape is "+shape+", which is not in the shape list.");
end

function new_dock = rand_rotate(cell_dock)
    N_exp = length(cell_dock);
    [N_rob, Nd] = size(cell_dock{1});
    new_dock = cell(N_exp);
    for i=1:N_exp
        new_dock{i} = zeros(N_rob, Nd);
        for j=1:N_rob
            new_dock{i}(j,:) = rotate(cell_dock{i}(j,:), randi([0,3],1));
        end
    end
end

function new_d = rotate(d, num)
    switch num
        case 0
            new_d = d;
        case 1
            new_d = d([3 4 2 1]);
        case 2
            new_d = d([2 1 4 3]);
        case 3
            new_d = d([4 3 1 2]);
    end
end
